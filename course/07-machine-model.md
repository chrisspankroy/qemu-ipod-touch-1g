# Lesson 07: Build The Machine

**Goal:** Write `hw/arm/s5l8900.c` from an empty file until the **real
SecureROM** boots the **real, unmodified iBSS**, which runs, self-copies into
SRAM, and halts at its natural completion point `0x22001360`. That is the
first honest milestone: firmware doing firmware things, QEMU just being the
silicon.

**Prerequisites:** Lessons 03–06. You will be editing C and rebuilding QEMU
many times.

**The honest note:** the real `hw/arm/s5l8900.c` in this repo is 6,000+ lines
because it accumulated every debugging trick and half-finished path over many
sessions. This lesson builds the *clean* ~400-line version that produces the
same first milestone. You will recognize where the big file diverges (it has
more: the iBoot handoff, MMU, console, a dozen timers).

---

## 1. Design decisions (read before writing code)

Five decisions shape everything. Each one was learned the hard way; the repo
history (and `CONCEPTS.md` section 12) explains them in detail.

### D1. Make the SecureROM a RAM region, not a ROM.

`memory_region_init_ram`, not a `rom_add_file`. Two reasons:
- QEMU lets you **patch the ROM bytes before the CPU runs** (see D3).
- In-place patching of read-only ROM regions fights TCG's translation cache
  (translated blocks go stale). Loading into RAM, patching the buffer, then
  *writing* memory is cache-safe.

### D2. Add a global catch-all region.

The ROM and iBSS poke **dozens** of peripherals (clocks, PMU, GPIO, USB PHY,
watchdog, …). You will not model all of them on day one. A catch-all over the
whole 4 GB space, **added first with priority 0**, makes every unmapped read
return 0 and every write a logged no-op. Specific devices are added after it
with priority ≥ 1 and automatically shadow it. Without it, the first unmapped
access is a guest data abort and you're chasing ghosts.

### D3. Surgically patch the ROM so it hands off to iBSS.

The ROM's job on real hardware is to receive iBSS **over USB DFU**, verify
and decrypt it, then jump to it. We are not emulating USB at the wire level
(lesson 12 explains the option). Instead: QEMU loads iBSS to its real
hardware address (`0x09000000`) from `-kernel`, and the ROM is patched so its
reset vector jumps straight there. The ROM's USB/DFU code is bypassed, and a
few other patches neutralize loops/handlers that would otherwise hang or
cascade. **Rule: patch as little as possible, prefer emulating hardware, and
comment every patch with a *why*.**

### D4. Drive the machine with a 100 ms REALTIME timer.

A single periodic callback samples the vCPU's PC, logs it, and later (lesson
09) detects the iBSS halt to perform the handoff. REALTIME, because the guest
spends its life in tight `b .` loops where VIRTUAL time stops (lesson 06.5).

### D5. Make status registers "all ready."

The universal stubbing rule, learned from a dozen hangs:

> **Status/lock/ready registers read as all-ones (`0xFFFFFFFF`).
> Busy/active/flag registers read as 0.**

Firmware polls with single-bit tests (`tst r0, #2; bne .`). If the register
reads all-ones, *every* "wait for bit set" poll passes. If you guess a
specific value (`0x3`) you will hang on the first poll for bit 3. (The flip
side — a register that must read 0 so a "wait until clear" loop exits — is
covered by the same rule.)

---

## 2. Wire the build

Three files (find them in the repo to compare):

1. `hw/arm/Kconfig` — add:
   ```
   config S5L8900
       bool
   ```
2. `configs/devices/arm-softmmu/default.mak` — add:
   ```
   CONFIG_S5L8900=y
   ```
3. `hw/arm/meson.build` — add:
   ```
   arm_common_ss.add(when: 'CONFIG_S5L8900', if_true: files('s5l8900.c'))
   ```

Create `hw/arm/s5l8900.c`. Run `ninja -C build qemu-system-arm`. It should
compile (an empty-but-valid file) and `-M s5l8900` should be unknown until
you finish this lesson.

## 3. The address table

Start the file with the map. These are the real values (from
`work/notes.txt` and the ROM's own constants):

```c
#define S5L8900_EVEC_BASE   0x00000000
#define S5L8900_EVEC_SIZE   0x1000
#define S5L8900_VROM_BASE   0x20000000
#define S5L8900_VROM_SIZE   0x100000
#define S5L8900_RAM_BASE    0x22000000
#define S5L8900_RAM_SIZE    (512 * KiB)
#define S5L8900_IBSS_BASE   0x09000000
#define S5L8900_IBSS_SIZE   (256 * KiB)
#define S5L8900_IBEC_BASE   0x0A000000
#define S5L8900_IBEC_SIZE   (256 * KiB)
#define S5L8900_IBOOT_BASE  0x23000000
#define S5L8900_IBOOT_SIZE  (1 * MiB)
#define S5L8900_IBOOT_RUNTIME 0x18000000
#define S5L8900_USBOTG_BASE 0x18000000
#define S5L8900_USBOTG_SIZE (2 * MiB)
#define S5L8900_PERIPH_BASE 0x38000000
#define S5L8900_PERIPH_SIZE 0x08000000
#define S5L8900_CLOCK0_BASE 0x38100000
#define S5L8900_VIC0_BASE   0x38e00000
#define S5L8900_VIC1_BASE   0x38e01000
#define S5L8900_USB_BASE    0x38c00000
#define S5L8900_CLOCK1_BASE 0x3c500000
#define S5L8900_WDT_BASE    0x3e300000
#define S5L8900_PMU_BASE    0x3e500000
#define S5L8900_UART_BASE   0xE0002000
#define S5L8900_UART_SIZE   0x1000
#define IMG2_HDR_SIZE       0x800   /* .dfu wrapper header skipped on load */
```

## 4. Machine skeleton

```c
static void s5l8900_cpu_reset(void *opaque)
{
    ARMCPU *cpu = opaque;
    /* ARM1176 resets to 0x0; S5L8900 boots from SecureROM at 0x20000000 */
    cpu->env.regs[15] = S5L8900_VROM_BASE;
}

static void s5l8900_machine_init(MachineClass *mc)
{
    mc->desc             = "Apple iPod Touch 1G (S5L8900)";
    mc->init             = s5l8900_init;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("arm1176");
    mc->default_ram_size = S5L8900_RAM_SIZE;
}
DEFINE_MACHINE_ARM("s5l8900", s5l8900_machine_init)
```

## 5. Memory regions, in creation order

Order matters: **catch-all first** (priority 0), everything else after.

```c
static void s5l8900_init(MachineState *machine)
{
    MemoryRegion *sysmem = get_system_memory();

    /* Catch-all: added FIRST so specific devices shadow it */
    MemoryRegion *catchall = g_new0(MemoryRegion, 1);
    memory_region_init_io(catchall, NULL, &s5l8900_catchall_ops, NULL,
                          "s5l8900.catchall", UINT64_MAX);
    memory_region_add_subregion_overlap(sysmem, 0, catchall, 0);

    /* Exception vectors (the ROM copies its vector table here) */
    memory_region_init_ram(evec, NULL, "s5l8900.evec", S5L8900_EVEC_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, S5L8900_EVEC_BASE, evec);

    /* SRAM — where iBSS self-copies and runs */
    memory_region_init_ram(sram, NULL, "s5l8900.sram", S5L8900_RAM_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, S5L8900_RAM_BASE, sram);

    /* SecureROM as RAM (design decision D1) */
    memory_region_init_ram(vrom, NULL, "s5l8900.vrom", S5L8900_VROM_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, S5L8900_VROM_BASE, vrom);

    /* iBSS / iBEC load regions, iBoot staging, USB-OTG (iBoot runtime), A-bit RAM */
    ... /* same pattern: init_ram + add_subregion for each */
}
```

The catch-all ops (log writes so you get a roadmap of unmodeled peripherals):

```c
static uint64_t s5l8900_catchall_read(void *o, hwaddr off, unsigned size) { return 0; }
static void s5l8900_catchall_write(void *o, hwaddr off, uint64_t v, unsigned size)
{
    qemu_log_mask(LOG_UNIMP, "s5l8900.catchall: w%u 0x%08lx <= 0x%08x\n",
                  size, off, (unsigned)v);
}
```

## 6. The UART — your print primitive

```c
static Chardev *s5l8900_serial_chr = NULL;

static uint64_t s5l8900_uart_read(void *o, hwaddr offset, unsigned size)
{
    if (offset == 5) return 0x60;   /* line status: TH=1 TEMT=1, no errors */
    return 0;
}
static void s5l8900_uart_write(void *o, hwaddr offset, uint64_t value, unsigned size)
{
    if (offset == 0 && s5l8900_serial_chr) {
        uint8_t ch = value & 0xff;
        qemu_chr_write(s5l8900_serial_chr, &ch, 1, false);
    }
}
```

In `s5l8900_init`: `s5l8900_serial_chr = serial_hd(0);` then attach an IO
region of size `0x1000` at `0xE0002000`. (Why 0x1000 and not 0x40: the ROM
may reference the whole register block; a generous region avoids faults.)

## 7. The peripheral stub menu

Model each of these as a tiny "store registers, read per the rule" device.
The table is what the firmware actually does with each one:

| Base | Name | Firmware behavior | Stub behavior |
|---|---|---|---|
| `0x38100000` | CLOCK0 | programs PLLs | store regs; read 0 |
| `0x3c500000` | CLOCK1 | **polls `+0x40` for PLL lock** | `+0x40` reads `0xFFFFFFFF` (D5); else stored reg |
| `0x38e00000` | VIC0 | sets vector addr, enables IRQs | store; `VICADDRESS` reads 0 (nothing pending) |
| `0x38e01000` | VIC1 | same | same |
| `0x38c00000` | USB OTG | full DFU engine | store; `GRSTCTL` reads `(1<<31)` (AHBIdle) |
| `0x3e300000` | WDT | `iBSS` enables+kicks it (`0x400000`) before halting | **absorb writes, never reset the CPU** |
| `0x3e500000` | PMU | polls `+0x04` for power status | `+0x04` reads `0x8` |

The WDT row is critical: a real watchdog reset would kill your run. Absorbing
the kick is the entire "watchdog emulation."

The CLOCK1 device is the first taste of the "make it ready" rule:

```c
static uint64_t s5l8900_clock_read(void *o, hwaddr offset, unsigned size)
{
    if (offset == 0x40) return 0xFFFFFFFF; /* PLL lock: every bit set */
    if (offset < sizeof(s->regs)) return s->regs[offset/4];
    return 0;
}
```

## 8. Load the firmware

```c
    /* SecureROM from -bios */
    if (machine->firmware) {
        gsize sz; guint8 *rom; GError *err = NULL;
        if (g_file_get_contents(machine->firmware, (gchar **)&rom, &sz, &err)) {
            if (sz > S5L8900_VROM_SIZE) sz = S5L8900_VROM_SIZE;
            s5l8900_rom_surgery(rom, sz);        /* next section */
            cpu_physical_memory_write(S5L8900_VROM_BASE, rom, sz);
            g_free(rom);
        }
    }
    /* iBSS from -kernel: skip the 0x800 img2 wrapper */
    if (machine->kernel_filename) {
        gsize sz; guint8 *ibss; GError *err = NULL;
        if (g_file_get_contents(machine->kernel_filename, (gchar **)&ibss, &sz, &err)) {
            cpu_physical_memory_write(S5L8900_IBSS_BASE,
                                      ibss + IMG2_HDR_SIZE, sz - IMG2_HDR_SIZE);
            g_free(ibss);
        }
    }
```

Note `-bios` maps to `machine->firmware` and `-kernel` to
`machine->kernel_filename` — no `rom_add_file` needed.

## 9. ROM surgery

Apply these to the ROM buffer **before** writing it to memory. Each has a
one-line *why* (keep these comments; they are the lesson's core):

```c
static void s5l8900_rom_surgery(uint8_t *rom, gsize sz)
{
    /* 1. Reset vector -> iBSS. Real flow: ROM receives iBSS over USB DFU,
     *    verifies + decrypts it, jumps. We have no USB, so jump directly.
     *    LDR r0,[pc,#0x3C] at 0x00 reads constant at 0x00+8+0x3C = 0x44. */
    stl_le_p(rom + 0x00, 0xE59F003C);   /* LDR r0, [pc, #0x3C] */
    stl_le_p(rom + 0x04, 0xE12FFF10);   /* BX r0 */
    stl_le_p(rom + 0x44, S5L8900_IBSS_BASE);

    /* 2. Vector table: the ROM copies 0x20000000 -> 0x0 during init. Any
     *    exception would run ROM handlers that fall into a self-overwriting
     *    copy loop. Make every vector a safe self-loop (reset stays ours). */
    stl_le_p(rom + 0x0C, 0xEAFFFFFE);   /* Prefetch abort: B . */
    stl_le_p(rom + 0x10, 0xEAFFFFFE);   /* Data abort:       B . */
    stl_le_p(rom + 0x18, 0xEAFFFFFE);   /* IRQ:              B . */
    stl_le_p(rom + 0x40, 0xEAFFFFFE);   /* the copy loop itself: B . */

    /* 3. CLOCK1 PLL poll (0x208-0x214): LDR r0,[r1]; CMP r0,#1; BNE loop.
     *    The ROM wants the value to be EXACTLY 1, but iBSS later polls the
     *    same register with single-bit tst's needing bits 1,3,.... One
     *    constant can't satisfy both, so NOP the ROM's BNE and let the stub
     *    return 0xFFFFFFFF for iBSS. */
    stl_le_p(rom + 0x214, 0xE1A00000);  /* NOP */
}
```

> Verify these offsets yourself with `xxd -s 0x208 -l 0x10 "work/ROM BOOT,
> S5L8900 Rev.2"` and decode by hand (lesson 03). The bytes are
> `24 14 9f e5 | 00 00 91 e5 | 01 00 50 e3 | fb ff ff 1a` =
> `LDR r1,[pc,#0x24]; LDR r0,[r1]; CMP r0,#1; B -4`.

## 10. The periodic timer — the machine's conscience

```c
static QEMUTimer *s5l8900_periodic_timer = NULL;
static void s5l8900_periodic_cb(void *opaque)
{
    ARMCPU *cpu = ARM_CPU(qemu_get_cpu(0));
    static int n = 0;
    n++;
    uint32_t pc = cpu->env.regs[15];
    if (n <= 10 || n % 50 == 0) {
        fprintf(stderr, "PERIODIC[%d]: pc=0x%08x thumb=%d cpsr=0x%08x\n",
                n, pc, cpu->env.thumb, (unsigned)cpu->env.uncached_cpsr);
        fflush(stderr);
    }
    /* Lesson 09 adds: if (pc == 0x22001360) s5l8900_handoff(cpu); */
    timer_mod(s5l8900_periodic_timer,
              qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + 100 * 1000 * 1000);
}
```

Create it at the end of `s5l8900_init` with `timer_new_ns(QEMU_CLOCK_REALTIME,
s5l8900_periodic_cb, NULL)` and fire it 100 ms out. Register the CPU reset
callback with `qemu_register_reset(s5l8900_cpu_reset, cpu)`.

## 11. Build, run, observe

```sh
ninja -C build qemu-system-arm
./build/qemu-system-arm -M s5l8900 \
  -bios "work/ROM BOOT, S5L8900 Rev.2" \
  -kernel "work/iPod1,1_1.1_3A101a_Restore/Firmware/dfu/iBSS.n45ap.RELEASE.dfu" \
  -nographic
```

**What you should see:** `PERIODIC[n]` lines walking through the ROM's init
(`0x20000xxx`), jumping to `0x09000xxx` (iBSS entry), then into SRAM
`0x2200xxxx` as iBSS self-copies and runs. Within a few hundred ms the PC
settles at `0x22001360` and stays there — iBSS has enabled its watchdog and
is waiting for a USB host that will never come. The run is alive, but the
guest is parked by design.

If you instead see the PC stuck at `0x20000214`, your CLOCK1 stub (or the
ROM NOP) is wrong. If it aborts immediately at `0x0`, your evec region or
vector patch is wrong. If `qemu_chr_write` never fires, the UART region
isn't shadowing the catch-all (check creation order and size).

## 12. Checkpoint

You can now answer:
1. Why is the ROM a RAM region, and what breaks if you `rom_add_file` it?
2. Why must the catch-all be added first, with priority 0?
3. Why can't one constant satisfy both the ROM's PLL poll and iBSS's?
4. Why does the WDT stub *never* reset the CPU?
5. Why is the periodic timer REALTIME?
6. What does iBSS do with its life between `0x09000000` and `0x22001360`?

## 13. Exercises

- Add a write to the UART in the ROM's `BEGIN_HARDWARE_INIT` path by planting
  a small ARM sequence that writes `0x48` ('H') to `0xE0002000`, and confirm
  you see it. (You are now patching firmware to make it talk to your stub.)
- Give the CLOCK1 stub a register array and print every read/write; run the
  ROM and produce a list of the exact register programming sequence the ROM
  performs. This is your first hardware "spec sheet" reverse-engineered from
  firmware.
- Extend the periodic callback to detect "PC unchanged for 10 samples" and
  print a distinct `STUCK` line. That detector is the seed of the halt
  detection in lesson 09.

**Pitfalls:**
- Stale binary: you edited `.c`, QEMU ran old behavior. Rebuild *every time*.
- Forgetting `fflush(stderr)` — with `-nographic`, stderr is piped and your
  `>>>` lines appear late or not at all.
- Catch-all added *after* a device → the device is shadowed.
- `0xEAFFFFFE` is the ARM `B .` self-loop. The **Thumb** self-loop is the
  halfword `0xE7FE` (stored little-endian as `FE E7`). They are *different*
  encodings — pasting an ARM loop into Thumb code (or vice-versa) gives you a
  "loop" that decodes as garbage and walks off into the void. (See
  `appendix.md` C for the full table.)

Next: [Lesson 08: The Debug Loop](08-debug-loop.md)
