# Lesson 09: iBSS and the Handoff

**Goal:** Understand what iBSS is and why it stops, then perform the
**iBSS → iBoot handoff** — the step a real USB/DFU host would do — and get
the real iBoot running under a **real ARMv5 MMU**. This is the hardest
technical lesson in the course; take it slow.

**Prerequisites:** Lessons 05–08. You can read the iBSS image and you have a
working debug loop.

---

## 1. What iBSS actually is

"iBSS" = **iBoot Stub (S)mall**. It is the first real Apple image after the
ROM. Its entire job:

1. Run a small runtime (clocks, watchdog, USB).
2. **Receive the next image (iBEC) over USB DFU.**
3. Verify it (SHA-1), decrypt it (AES-128-ECB), load it to its target
   address, and jump to it.

It is deliberately tiny. It has no filesystem, no console, no kernel. It is a
**loader that waits for a host.** On real hardware the "host" is your Mac
running iTunes/Apple's restore. **In QEMU there is no USB host, so iBSS runs
its runtime, then parks forever waiting for an image that never arrives.**
That parking point is `0x22001360` (`b .`).

## 2. The iBSS image, end to end

From lesson 05 you know the `.dfu` file is a 0x800-byte `89001.0` wrapper +
plaintext payload. The payload, once loaded at `0x09000000`:

1. **Entry** — a short ARM/Thumb prologue (`mov r0, pc`-style PC-relative
   setup) that establishes where the code is.
2. **Self-copy** — iBSS copies itself from `0x09000000` to **SRAM
   `0x22000000`** and jumps there. It runs from SRAM for the rest of its
   life. (This is why all the runtime addresses you see are `0x2200xxxx`.)
3. **Hardware init** — program CLOCK1 (PLL), enable the **watchdog**
   (`0x400000` → `0x3e300000`), set up USB.
4. **Park** — `b .` at `0x22001360`, waiting for the USB DFU host.

The watchdog step is important: iBSS *arms the WDT right before parking*. On
real hardware that's fine (the host activity or the next stage resets it). In
QEMU our WDT stub just **absorbs the kick** (lesson 07, D-table) so the CPU
isn't reset. If you "fix" the WDT to actually reset, your run dies the
moment iBSS parks.

> **Verify it yourself:** in the loaded image, `0x22001360` is a `b .`. Map
> it back to the file: runtime base `0x22000000`, so file offset
> `= 0x22001360 − 0x22000000 + 0x800 (wrapper) = 0x1B60`.
> `xxd -s 0x1B60 -l 2 "work/.../iBSS.n45ap.RELEASE.dfu"` → `fe e7`. That's the
> Thumb halfword `0xE7FE` (`B .`, little-endian) — iBSS's halt is **Thumb**
> code. Do it and confirm.

## 3. Why the handoff must be done by QEMU

On real hardware, after iBSS parks, the host:
1. Speaks the USB DFU protocol to iBSS.
2. Sends **iBEC**; iBSS verifies + decrypts it, loads it to `0x0A000000`, jumps.
3. iBEC then receives **iBoot**, verifies + decrypts it, loads it, jumps.

We are not emulating USB at the wire level (that's the "most authentic"
option in lesson 12). So **QEMU plays the host**: when it sees iBSS parked,
it does the host's job — stage the next images, set up the environment, and
redirect the CPU to iBoot. The periodic timer (lesson 07) is the trigger:

```c
/* in s5l8900_periodic_cb */
if (!handoff_done && pc == 0x22001360) {
    handoff_done = 1;
    s5l8900_handoff(cpu);     /* lesson 09 */
}
```

## 4. The iBoot linking problem (why we need an MMU)

iBoot is **linked** so that:
- its **code** is referenced at **VA `0x18000000`**, and
- its **data** (globals, literal pools, the big constant tables) is
  referenced at **VA `0x28000000`** — exactly `0x10000000` higher.

Both halves are backed by the **same physical image**. On real hardware the
SoC's MMU maps both virtual windows onto one physical block. This is the
classic ARM "split code/data virtual mapping."

The consequence for us: if we just copy iBoot to physical `0x18000000` and
run it with the MMU **off**, every code access works (VA `0x18xxxxxx` ==
PA), but **every data access to `0x28xxxxxx` faults** (nothing is mapped
there). iBoot immediately data-aborts on its first global load.

> This is exactly the bug the project hit: "force-disabled the MMU, which
> broke iBoot's virtual data references and caused a cascaded data abort."
> The fix was not "disable the MMU" — it was **build a real page table that
> maps both windows to the same physical image** and **turn the MMU on.**

## 5. ARMv5 MMU refresher (the minimum you need)

The ARM1176 is ARMv5. It uses the **short descriptor** page format:
- A **4096-entry L1 table** (16 KB), each entry = 32 bits.
- We only use **1 MB sections** (no L2 page tables — simpler, and enough).
- **L1 index = `VA >> 20`** (the top 10 bits of the virtual address).
- A 1 MB section entry = `physical_base | attribute_byte`, where the base
  occupies the high bits and the low byte holds the attributes.

### The attribute byte (the whole lesson in one byte)

For a **1 MB section** entry the low bits are what matter:

 Here is the layout **as QEMU actually parses it** — the authoritative source
 is `get_phys_addr_v5()` in `target/arm/ptw.c`:

 ```c
 type   = desc & 3;               /* [1:0]   */
 domain = (desc >> 5)  & 0xf;     /* [8:5]   */
 ap     = (desc >> 10) & 0x3;     /* [11:10] */
 phys   = (desc & 0xfff00000) | (address & 0x000fffff);   /* PA[31:20] */
 ```

 ```
 [31:20] -> PA[31:20]          (physical base)
 [19:12] -> 0
 [11:10] -> AP                 (2-bit simple AP; 0b00 = RW, user+supervisor)
 [9]     -> 0
 [8:5]   -> domain             (4 bits; with DACR=0xFFFFFFFF all are Client)
 [7:4]   -> cache type         (0x3 = Normal cacheable; 0x8 = Device n-cache)
 [3:2]   -> 0
 [1:0]   -> type               *** the critical bits: 0b10 = valid 1MB section ***
 ```

 Two attribute bytes the project uses (both have `type = 0b10`, `AP = 0b00`
 = RW, `domain = 0`):

 ```c
 #define SECT(pa)  ((uint32_t)(pa) | 0x32)   /* cache 0x3: Normal, RW, cacheable  */
 #define DEV(pa)   ((uint32_t)(pa) | 0x82)   /* cache 0x8: Device, RW, no-cache   */
 ```

- `0x32` = `0x30 | 0x2` → cache type `0x3` (Normal, cacheable) + `type 0b10`.
  Use for RAM/code.
- `0x82` = `0x80 | 0x2` → cache type `0x8` (Device, non-cacheable) + `type 0b10`.
  Use for MMIO peripherals. You **must** map peripherals as Device, or the
  cache will swallow/break their side-effects.

> **The bug that cost a session:** the low bits were first written as
> `0x30` / `0x80` (i.e. `[1:0] = 0b00`). In the short-descriptor format
> `[1:0] = 0b00` means **the section is not present** → **every single access
> is a translation fault.** The entire iBoot run was one giant data-abort
> storm. Flipping the byte from `0x30`→`0x32` (and `0x80`→`0x82`) — i.e.
> **setting `[1:0] = 2`** — fixed it. One bit. This is your lesson-08
> "hypothesize → instrument → fix" in its purest form: DFSR said
> "translation fault," you inspected the page-table entry, you saw `[1:0]=0`.

## 6. Build the page table

The table is an array of 4096 `uint32_t`. Place it in **SRAM** (RAM, not a
device region) so the MMU can walk it: base `0x22038000`.

```c
#define S5L8900_PT_BASE (S5L8900_RAM_BASE + 0x38000)   /* 0x22038000 */
static uint32_t pt[4096];
memset(pt, 0, sizeof(pt));

/* ---- RAM / code (Normal) ---- */
pt[0x180] = SECT(0x18000000);   /* VA 0x18xxxxxx -> code, PA 0x18000000 */
pt[0x280] = SECT(0x18000000);   /* VA 0x28xxxxxx -> DATA, SAME PA!      */
pt[0x220] = SECT(S5L8900_RAM_BASE); /* VA 0x22xxxxxx -> SRAM */
pt[0x230] = SECT(S5L8900_IBOOT_BASE);   /* staging */
pt[0x000] = SECT(0x00000000);   /* evec */
pt[0x090] = SECT(0x09000000);   /* iBSS */
pt[0x0A0] = SECT(0x0A000000);   /* iBEC */
pt[0x600] = SECT(0x60000000);   /* kernelcache (for bootx) */

/* ---- Device (peripherals) ---- */
pt[0xE00] = DEV(0xE0000000);    /* UART window */
/* Fill the whole 128MB peripheral window (0x38000000..0x3FFFFFFF) as Device
 * so any peripheral iBoot touches translates instead of faulting. */
for (int sec = 0x380; sec <= 0x3FF; sec++)
    if (pt[sec] == 0)
        pt[sec] = DEV(0x38000000 + (uint32_t)(sec - 0x380) * 0x100000);

/* Write the table into SRAM, then point the MMU at it */
cpu_physical_memory_write(S5L8900_PT_BASE, pt, sizeof(pt));
```

The **one line that solves the linking problem** is `pt[0x280] = SECT(0x18000000);`
— it maps the data window (`0x28xxxxxx`) onto the same physical image as the
code window. Now iBoot's `0x28xxxxxx` data references resolve.

## 7. Turn the MMU on

```c
cpu->env.cp15.ttbr0_s  = S5L8900_PT_BASE;   /* table base */
cpu->env.cp15.ttbr1_s  = 0;
cpu->env.cp15.dacr_s   = 0xFFFFFFFF;        /* all 16 domains = Client (RW) */
/* SCTLR: M(0)=1 A(1)=1 C(2)=1 I(12)=1 -> MMU + alignment + D-cache + I-cache */
cpu->env.cp15.sctlr_s  = 0x1007;
arm_rebuild_hflags(&cpu->env);              /* *** required after touching cp15 *** */
```

- `TTBR0` = where the L1 table lives.
- `DACR = 0xFFFFFFFF` → every domain is **Client** (fully RW). If a domain
  were Manager/No-access you'd get permission faults.
- `SCTLR = 0x1007`:
  - bit 0 `M` = **MMU on**
  - bit 1 `A` = alignment check
  - bit 2 `C` = **data cache on**
  - bit 12 `I` = **instruction cache on**
- `arm_rebuild_hflags()` recomputes the CPU's internal mode flags. **You must
  call it after changing any CP15 control register** or the change won't take
  effect. (Forgetting this is a classic "I set SCTLR but the MMU didn't turn
  on" bug.)

## 8. Set up the CPU for iBoot's entry

iBoot starts in **ARM mode** at its **reset handler** (offset `0x40` in the
payload, which lives at `0x18000000`). The reset handler:
1. Zeroes BSS (`0x18021980`–`0x18026000`).
2. Sets up the SVC/IRQ/FIQ/ABT/UND stacks.
3. Switches to SVC mode, masks interrupts.
4. `BXes` to the entry dispatcher, which calls **`main_init`** (Thumb).

So QEMU's job is to park the CPU in a *safe pre-boot state*; the reset
handler takes it from there:

```c
cpu->env.regs[13]  = S5L8900_RAM_BASE + 0x20000;  /* SP (SRAM, outside BSS) */
cpu->env.regs[14]  = S5L8900_RAM_BASE + 0xF900;   /* LR (handler overrides) */
cpu->env.thumb     = 0;                           /* ARM mode */
cpu->env.regs[15]  = S5L8900_IBOOT_RUNTIME + 0x40;/* PC = reset handler */
/* SVC (0x13), ARM (T=0), IRQ+FIQ masked (I=F=1) */
cpu->env.uncached_cpsr = (cpu->env.uncached_cpsr & ~0x3F) | 0x13 | (1<<7) | (1<<6);
```

The stack **must** be outside the BSS range, or the BSS-zeroing step would
wipe the stack mid-boot. `0x22020000` (top of the 512 KB SRAM) is safe.

## 9. Staging vs runtime — copy iBoot to its link address

Remember iBoot must run at its **link address `0x18000000`** so every baked
absolute literal (timer tables, string pointers, the dispatch table) is
correct. The flow:

1. At machine init, load the iBoot **payload** (the `0x22000` bytes *after*
   the `0x400` img2 header — **not** the header) into a **staging** region
   (`0x23000000`).
2. In the handoff, **copy staging → runtime `0x18000000`** (the USB-OTG
   region), now that iBEC/iBSS are done with it.
3. Because the payload base now == the link base, **no per-literal surgery is
   needed** — the "Path A" insight: run it at its true address and every
   pointer is already right.

```c
size_t sz = S5L8900_IBOOT_PAYLOAD;              /* 0x22000, no header */
uint8_t *buf = g_malloc(sz);
cpu_physical_memory_read(S5L8900_IBOOT_BASE, buf, sz);       /* staging */
cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME, buf, sz);   /* runtime */
```

> The header is **not** copied. If you copy the 0x400 header, the payload is
> shifted by 0x400 and *every* absolute literal is wrong. This is the mirror
> image of lesson 04's offset formula: `file F (>= 0x400) -> base + (F - 0x400)`.

## 10. Invalidate the translation cache and go

The CPU has already translated blocks for the old (identity, MMU-off) state.
After all patches + page table + CP15 changes, flush:

```c
queue_tb_flush(cs);                 /* drop ALL translated blocks */
cs->exception_index = -1;
cs->exit_request = 1;
cpu_reset_interrupt(cs, CPU_INTERRUPT_EXITTB);
```

`queue_tb_flush` (not an inline `tb_flush`) is the main-loop-safe way to say
"the code/memory changed, stop using cached translations." Then the periodic
callback returns and the vCPU resumes — now at iBoot's reset handler, under a
real MMU.

## 11. (Optional, for bootx later) Preload the kernelcache

```c
/* load work/.../kernelcache.release.s5l8900xrb -> PA 0x60000000 (A-bit RAM) */
```
`pt[0x600]` maps it. This is where `bootx` will find the kernel. It's staged
now so it's available when you get to lesson 12.

## 12. Build, run, observe

```sh
ninja -C build qemu-system-arm
./build/qemu-system-arm -M s5l8900 -bios "$ROM" -kernel "$IBSS" -nographic
```

Watch the `PERIODIC`/`>>>` lines:
1. iBSS runs and parks at `0x22001360`.
2. Your callback fires: "iBSS at halt — performing handoff."
3. iBoot payload is copied to `0x18000000`, the page table is written, SCTLR
   is set, CPU is parked at the reset handler.
4. The PC now walks through `0x18000xxx` (reset handler) and into `0x18004xxx`
   (`main_init`). **If you see the PC executing iBoot code with no data-abort
   storm, the MMU is working.**

If you instead see an instant abort storm back at the reset handler:
- check `[1:0] == 2` on every non-zero page-table entry (the 0x30/0x80 bug);
- check `arm_rebuild_hflags` is called;
- check `DACR` is all-client;
- use the fault-capture handler (lesson 08) to read the DFSR — it will say
  "translation fault" if the entry is missing, "permission fault" if the
  domain/AP is wrong.

## 13. Checkpoint

You can now answer:
1. Why does iBSS park at `0x22001360`, and who on real hardware is it waiting for?
2. Why must the WDT stub *not* reset the CPU?
3. Why does iBoot need an MMU at all? What exactly faults without it?
4. What does `pt[0x280] = SECT(0x18000000)` do, and why is it the key line?
5. What do the `[1:0]` bits of a page-table entry mean, and what's the 0x30/0x32 bug?
6. Why must the stack be outside the BSS range?
7. Why is `arm_rebuild_hflags` required after setting SCTLR?

## 14. Exercises

- Build the page table by hand for a *different* split (code at `0x10000000`,
  data at `0x20000000`) and confirm the MMU resolves both.
- Introduce the 0x30/0x80 bug on purpose, run it, and use the fault-capture
  handler + DFSR to diagnose it — without looking at the fix. Then fix it.
- Use the P2 step timer to print the first 30 instructions iBoot executes
  after the handoff, and label each (reset handler / BSS zero / stack setup /
  BX to dispatcher).

Next: [Lesson 10: Crack iBoot](10-crack-iboot.md)
