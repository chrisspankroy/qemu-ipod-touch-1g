# Progress

## Current Goal
Run actual, unmodified iBoot on emulated iPod Touch 1G in QEMU and capture its serial output.

## Constraints & Preferences
- QEMU fork at `/Users/chris/dev/ipod-touch-1g`, supporting files at `/Users/chris/dev/ipod-touch-1g/work`
- Main device file: `hw/arm/s5l8900.c`
- Stop once capturing real serial output from iBoot
- Do not ask questions unless truly blocked
- Work on unfinished TODOs in order, mark completed with [x]

## LATEST STATUS (2026-08-20) — FULL MMU REWORK; iBoot runs in virtual mode, data-aborts at VA 0x00000805

**Strategy shift (this session):** Abandoned the old "MMU disabled + patch the command-dispatch table" path. Now doing the **real MMU rework**: QEMU builds a genuine ARMv5 (short-descriptor, 1MB-section) page table, loads it at `TTBR0`, enables MMU+align+cache (`SCTLR=0x1007`), and lets iBoot run in **virtual mode** so its `0x28xxxxxx` data refs and `0x18xxxxxx` code refs resolve to the same physical image at `0x18000000`. iBoot now executes real code deep past `main_init` and faults **cleanly** (CPU parks, no crash loop). The remaining blocker is a single, unexplained **data abort**.

**⚠️ RESUME FIRST ACTION — the built binary is STALE.** `hw/arm/s5l8900.c` mtime is 23:49:31 but `build/qemu-system-arm` is 23:42:14 (built 7 min earlier, before the last source edit — the DFSR revert). **Rebuild before running:** `ninja -C build qemu-system-arm`. Do not trust any run output produced before a rebuild.

**Page-table / MMU setup** (`hw/arm/s5l8900.c:2508-2543`):
- `#define S5L8900_PT_BASE (S5L8900_RAM_BASE + 0x38000)` → `0x22038000` (in SRAM); `static uint32_t pt[4096]`, memset 0, then written to `S5L8900_PT_BASE` via `cpu_physical_memory_write`.
- `#define SECT(pa) ((uint32_t)(pa) | 0x32)` (Normal RW cacheable section) and `#define DEV(pa) ((uint32_t)(pa) | 0x82)` (Device RW non-cacheable). The `|0x32` sets `[1:0]=0b10` (present 1MB section) which `get_phys_addr_v5` requires — the old `0x30/0x80` left `[1:0]=0b00` → translation fault on every access.
- Entries: `pt[0x180]=pt[0x280]=SECT(0x18000000)` (code+data same phys), `pt[0x220]=SECT(0x22000000)` (SRAM), `pt[0x230]=SECT(0x23000000)` (staging), `pt[0x000]=SECT(0x00000000)` (low RAM), `pt[0x090]=SECT(0x09000000)` (iBSS), `pt[0x0A0]=SECT(0x0A000000)` (iBEC), `pt[0x600]=SECT(0x60000000)` (kernelcache), `pt[0xE00]=DEV(0xE0000000)` (UART), `pt[0x380]=DEV(0x38000000)`, `pt[0x3E0]=DEV(0x3E000000)`.
- `ttbr0_s = S5L8900_PT_BASE`; `ttbr1_s = 0`; `dacr_s = 0xFFFFFFFF` (all domains Super); `sctlr_s = 0x1007`; `arm_rebuild_hflags`.
- CPU is `arm1176` (ARMv6 core, but walking the **v5 short-descriptor** tables).
- Memory: evec RAM at `0x00000000` size `0x1000` (added ~line 6807, non-overlap `memory_region_add_subregion`); a global catch-all IO region at `0x0`/4GB priority 0 added first (~line 4947). Low vectors fetched from `0x0` successfully (code fetch works), so evec RAM *is* the region serving `0x0`.

**Exception handler** (`hw/arm/s5l8900.c:1951-1965`, at `0x2200F800`) — the **working** 8-instruction "capture-then-park" version (verified with capstone):
```
0xE10F0000  MRS r0, CPSR
0xE58F0020  STR r0, [pc,#0x20] -> [0x2200F82C] CPSR
0xEE150F10  MRC p15,#0,r0,c5,c0,#0 (DFAR)
0xE58F0014  STR r0, [pc,#0x14] -> [0x2200F828] DFAR
0xEE160F10  MRC p15,#0,r0,c6,c0,#0 (IFAR)
0xE58F0008  STR r0, [pc,#8]    -> [0x2200F824] IFAR
0xE58FE010  STR lr, [pc,#0x10] -> [0x2200F830] LR
0xEAFFFFFE  B #-4 (park at 0x2200F81C)
```
Storage layout: `0xF824`=IFAR, `0xF828`=DFAR, `0xF82C`=CPSR, `0xF830`=LR. **Do NOT add an in-handler `MRC p15,#0,?,c5,c0,#3` (DFSR)** — it caused a nested-exception loop on arm1176 and had to be reverted. If you need the fault *type*, read it C-side (see below).

**FAULT-CTX dump** (`hw/arm/s5l8900.c:1207-1211`): reads 5 words from `0x2200F820` and prints `IFAR / DFAR / CPSR / LR / pad`.

**C-side DFAR/DFSR capture (ALREADY EXISTS)** (`hw/arm/s5l8900.c:1084-1099`, in `s5l8900_step_trace_cb`): one-shot, when `cpsr & 0x1F` is `0x12`(ABT) or `0x10`(PABT) it prints `DFAR-CAP: ... dfar_s dfar_ns ifar_s dfsr_s` read straight from `cpu->env.cp15.{dfar_s,dfar_ns,ifar_s,dfsr_s}`. **This is the right way to get the fault type** (no MRC-in-handler). Verify on next run that `dfsr_s` is populated (bits [3:0]=fault status: 0b0010=translation, 0b0011=domain, 0b0110=access-permission, 0b1111=external/other). Note the `static dfar_dumped` latch — it fires only on the FIRST abort.

**The observed fault (from last good run, `/tmp/irqfix_run.txt:152`):**
```
FAULT-CTX: IFAR=0x4101b6c4 DFAR=0x00000805 CPSR=0x800001d7 LR=0x18007d34 pad=0x00000000
```
- **DFAR = `0x00000805`** = the faulting **data** VA. **LR = `0x18007d34`** → faulting PC = `0x18007d32` (Thumb, LR=PC+2). **CPSR = `0x800001d7`** → mode `0x17` (Abort), T=1 (Thumb), E=1 (big-endian bit!). IFAR `0x4101b6c4` is stale garbage (this is a data abort, not prefetch).
- **Faulting instruction** (capstone on `work/iBoot.decrypted` @ `0x18007d32`, Thumb): `str r6, [r0, #4]` → effective addr = `r0+4 = 0x805`, so **r0 = `0x801`**. Surrounding: `adds r3,#1; str r2,[r0,r1]; adds r0,r0,r1; movs r5,#2; str r6,[r0,#4]` (loop storing into a low-address structure).

**Why the fault is UNEXPLAINED (the mystery to solve):**
- `pt[0x000] = SECT(0) = 0x32`. Per `target/arm/ptw.c:1056` `get_phys_addr_v5`: `type=(0x32&3)=2` (1MB section); `phys_addr=(0x32 & 0xfff00000)|(0x805 & 0xfffff)=0x805`; `domain=(0x32>>5)&0xf=1`; `ap=(0x32>>10)&3=0`.
- `dacr_s=0xFFFFFFFF` → domain 1 = Super (no domain fault). AP=0 = RW privileged+user (no permission fault). So the walk should **succeed** with RW.
- PA `0x805` is inside evec RAM (`0x0`–`0xFFF`) and that RAM is confirmed reachable (vector table at `0x0` fetches fine). So the store to PA `0x805` **should not fault** at the memory level.
- Yet a data abort fires on the store. Root cause not yet found.

**Next steps (in order):**
1. **Rebuild** (`ninja -C build qemu-system-arm`) — binary is stale (see ⚠️ above).
2. **Re-run** (clean, see Repro) and capture `DFAR-CAP` (the C-side `dfsr_s`) **and** the `FAULT-CTX` line. Confirm the faulting PC/VA are the same (`0x18007d32` / `0x00000805`). Read `dfsr_s` to get the fault *type*.
3. **Branch on fault type:**
   - If **translation/domain/permission** (bits[3:0] in {2,3,6}) → the walk is faulting in QEMU; re-examine `pt[0x000]`, DACR, and whether the walker actually uses `get_phys_addr_v5` vs `v6` for arm1176 (check `get_phys_addr()` regime selection in `target/arm/ptw.c`). Possibly the `|0x32` section encoding is subtly wrong for this walker.
   - If **external/other (0b1111)** → it's a memory-system abort, not a translation fault. Then check whether evec RAM truly backs PA `0x805` (region priority vs the global catch-all), or whether r0=`0x801` is a corrupted pointer (iBoot read a bad value earlier).
4. **Check the E=1 (big-endian) bit in CPSR `0x800001d7`**: if iBoot switched to big-endian mode, all subsequent multi-byte data accesses are byte-swapped and could corrupt pointers / cause spurious faults. Investigate whether iBoot (or our setup) sets E in CPSR, and whether arm1176 here should be little-endian.
5. **Apply fix**, rebuild, re-run; iterate until iBoot gets past `0x18007d32` (reaches console / next fault).

**Also uncommitted (keep unless it breaks):** `target/arm/tcg/translate.c:6636` — replaced `assert((pc_next & 1)==0)` for Thumb with a strip-LSB fixup (`pc_next &= ~1`) for externally-redirected CPUs. This is a QEMU TCG workaround for our device-model redirects.

**Other force-disable-MMU sites still present** (cleanup later, not current blocker): `hw/arm/s5l8900.c` ~lines 879, 2852-2853, 3072-3073, 3580-3584, 4494.

**Repro (after rebuild):**
```
S5L8900_IBSS_CLEAN=1 timeout 25 ./build/qemu-system-arm -M s5l8900 \
  -bios "work/ROM BOOT, S5L8900 Rev.2" \
  -kernel "work/iPod1,1_1.1_3A101a_Restore/Firmware/dfu/iBSS.n45ap.RELEASE.dfu" \
  -nographic > /tmp/irqfix_run.txt 2>&1
```
Then inspect: `grep -E "DFAR-CAP|FAULT-CTX|MMU REWORK" /tmp/irqfix_run.txt`.
- **Build**: `ninja -C build qemu-system-arm`
- **Disassembly**: capstone against `work/iBoot.decrypted` (140288 bytes; runtime addr = `0x18000000 + file_offset`).
- iBoot entry: `pc=0x18000440` (ARM reset handler), `cpsr=0xd3` (SVC/ARM/LE), `lr=0x2200f900`, `sp=0x22020000`. Preseeded console input `"help\nbootx 60000000\n"` at `0x22011100`; dispatch `[0x18006000]`; `bootx` handler `0x18006011`.

---

## LATEST STATUS (2026-08-18) — iBSS→iBoot handoff works; iBoot prints console prompt

**Milestone (building on the clean baseline):** wired the iBSS→iBoot handoff into the clean path, so a clean run now goes: **real iBSS → natural halt → iBoot loads → `iBoot` console prompt printed.**

**Change** (`hw/arm/s5l8900.c`, clean-mode periodic block ~line 2049): when the clean run observes iBSS parked at its halt `pc==0x22001360`, call `s5l8900_config_board_trigger(cpu)` once (same handoff the real `config_board\0` DFU signal would trigger). This copies iBEC→0x18000000, applies iBEC patches, then `s5l8900_jump_to_tramp` copies iBoot→0x18000000 + preloads the kernelcache (3,324,650 bytes) at `0x60000000` and jumps to iBoot's ARM reset handler `0x18000400`. iBoot prints the `iBoot` prompt. Timer stays re-armed to keep monitoring iBoot's PC.

**Verified in clean run** (`S5L8900_IBSS_CLEAN=1`):
- `CLEAN: iBSS at halt 0x22001360 - triggering iBEC/iBoot handoff`
- `JUMP CB: preloaded kernelcache 3324650 bytes @ 0x60000000`
- Serial prints `Hello from iPod!` then the `iBoot` prompt.
- Preseeded console input `help\nbootx 60000000\n` is injected via the stateful getchar stub (0x49C0) + wrapper (0x17D6A); strcmp stubbed to return 0 (0x17DE0).

**Next blocker — iBoot command dispatch crash:** after the prompt, iBoot parses `help` and the dispatch at `0x18005ed6` jumps to garbage (`0x10`) → exception → SRAM safe-handler loop (`0x2200f81c`). The dispatch:
```
0x18005ed6: ldr r3, [pc, #0x128]   ; r3 = [0x18006000]  (table base)
0x18005ed8: lsls r2, r4, #2         ; r2 = index*4
0x18005eda: ldr r3, [r2, r3]        ; r3 = handler
0x18005edc: mov pc, r3
```
- **Pristine** image `[0x18006000] = 0x18005ae0` (which is CODE, not a table). At **runtime** it's overwritten to `0xd2842201` (garbage). So iBoot init is meant to write a runtime table base here, but in QEMU it lands on garbage.
- Command name/handler table (decoded from image @ `0x1f780`): name ptrs at `0x1801bxxx`, handlers at `0x18004xxx`–`0x18008xxx`. e.g. `poweroff`→`0x18004805`, `echo`→near `0x18005fd1`, **`bootx`→`0x18006011`** (usage "boot a kernel cache at specified address" @ `0x1b174`).
- Candidate 8-entry Thumb jump tables found in image (all-even/odd, contiguous): **`0x1800dfc4`** → `0x1800d801 0x1800d8c9 0x1800d9c9 0x1800d815 0x1800d8dd 0x1800d9dd 0x1800d849 0x1800d925`; also `0x18020234`, `0x18021c28`.
- **TODO (dispatch fix)**: confirm which table `0x18005ed6` actually indexes and the `r4`→command mapping, then either (a) patch `[0x18006000]` to the correct base after launch, or (b) add a write-hook to catch the PC that stores the bad base and fix that init code, or (c) build a minimal 8-entry table in SRAM (help=print, bootx=`0x18006011`) and point `[0x18006000]` at it.

---

## STATUS (2026-08-18) — CLEAN BASELINE: iBSS now runs as the real image

**Milestone**: Gated OFF all brute-force iBSS code patches (`S5L8900_IBSS_CLEAN=1`) so iBSS runs as the real unmodified image, then found & fixed the ONE genuine QEMU emulation gap that stopped it.

**Clean-baseline switch** (`hw/arm/s5l8900.c`, new `s5l8900_ibss_clean()`):
- Gates the iBSS buffer patches (0x3c00 fill, BL callers, 0x3724, 0x5bf0, 0x5400), the post-write patches (0x1154, 0x10540…, 0x702a broken B, 0x6f70, 0x4ea0, 0x5400), the exception-handler redirects (0x4040/0x4068/0x408c→trampoline), the CLOCK1 runtime-patch block, and the entire periodic crash-recovery state machine (early-return after diagnostics, timer kept armed).

**Genuine gap found & fixed**:
- iBSS polls PLL status `CLOCK1+0x40` = `0x3c500040` with `tst r1, r3` where `r1 = 1<<n` (single bit).
- Two polls: mask `0x2` (bit1 — OK with old `0x3`) and mask `0x8` (bit3 — **HUNG**: `0x3` has no bit 3).
- Fix: `s5l8900_clock_read` now returns `0xFFFFFFFF` (all lock/ready bits set) for offset `0x40`, satisfying any single-bit "PLL locked" poll. (Old brute-force code patched iBSS to *skip* this poll; the clean baseline needed the real register to report the bit.)

**Result** (clean run, `S5L8900_IBSS_CLEAN=1`):
- iBSS executes **779 distinct TBs** of real code (was 228 before the fix) and reaches its **natural completion halt**:
  - `0x2200135e: str r2,[r3]` → writes `0x400000` to `0x3e300000` (WDT_CTRL)
  - `0x22001360: b .` → infinite loop = iBSS done, sleeping, waiting for the host to send iBEC over USB DFU.
- This is **expected iBSS behavior** (DFU baseboard subsystem waits for the next image), not a crash. All halt addresses verified as genuine iBSS code (file offset `0x800 + (runtime − 0x22000000)`), not QEMU patches.

**Next step** (to reach the iBoot console): after iBSS reaches `0x22001360`, inject iBEC cleanly (load iBEC + set up the iBSS→iBEC handoff instead of the old brute-force jump), run iBEC so it loads iBoot, then tackle the iBoot console-dispatch bug below.

**Repro**:
```
S5L8900_IBSS_CLEAN=1 timeout 12 ./build/qemu-system-arm -M s5l8900 \
  -bios "work/ROM BOOT, S5L8900 Rev.2" \
  -kernel "work/iPod1,1_1.1_3A101a_Restore/Firmware/dfu/iBSS.n45ap.RELEASE.dfu" \
  -d in_asm -D /tmp/ibss_clean_trace.log -nographic 2>/tmp/ibss_clean_run.err
```
Look for `CLOCK1[0x40] read` (mask values) and the final `CLEAN-PERIODIC` stuck at `pc=0x22001360`.

---

## STATUS (2026-08-18, iBoot console dispatch) — `[0x18006000]` table bug

**Goal**: iBoot's console should run the preseeded `help` then `bootx 60000000` and boot the kernelcache at `0x60000000`. iBoot DOES reach the console (prints the `iBoot` prompt) but crashes in the command-dispatch function.

**Crash site** (iBoot offset, runtime = `0x18000000 + off`):
```
0x18005ed6: ldr  r3, [pc, #0x128]   ; r3 = [0x18006000]
0x18005ed8: lsls r2, r4, #2         ; r2 = r4*4
0x18005eda: ldr  r3, [r2, r3]       ; r3 = [0x18006000 + r4*4]
0x18005edc: mov  pc, r3             ; jumps to garbage -> safe loop 0x2200f828
```
The state machine just above forces `r4` to `5` or `7` (index used here).

**Root cause being chased**: `[0x18006000]` must be a pointer to a table of u32 function pointers.
- Image literal `[0x18006000] = 0x18005AE0`; **runtime value = `0x18002201`** (measured).
- `0x18002201` = iBoot `0x2200` = a Thumb *function*, NOT a table. So `ldr r3,[r2,r3]` reads code bytes → `mov pc,r3` jumps to garbage.
- `0x6000` is **NOT in BSS** (BSS = `0x18021980`–`0x18026000`), so it is written by iBoot init code at runtime, not zeroed.
- **TODO: find WHO stores `0x18002201` into `[0x18006000]`** (CPU store, PC expected in `0x1800xxxx`), then find/build the correct table base.

**Established this session**:
- Staging `0x23000000` is loaded from `work/iBoot.decrypted` (`hw/arm/s5l8900.c:5020`). Load-time patches: `buf[0x454/4]=0xEA00000A` (force-skip self-copy), `buf[0x474/4]=buf[0x478/4]=0xE1A00000` (NOP CP15 cache ops). Copied staging→runtime `0x18000000` in JUMP CB (`:765`), remainder filled `0x47704770` (`:769-771`), then a 1 MiB read+write "mirror" of `0x18000000` (`:1770-1774`; effectively a no-op because `S5L8900_USBOTG_BASE == S5L8900_IBOOT_RUNTIME == 0x18000000`).
- The earlier "reset handler mismatch" was a **decode error**: `0x440` is **ARM** mode (`mov r0,pc; ldr r1,[pc,#0xc0]; ...`), not Thumb. Image is consistent — iBoot executes from `0x18000440` (ARM reset handler): zeroes BSS, sets mode stacks, SVC mode (`0x12`), `BX [0x508]` (entry dispatcher `0x4C20`) → `main_init` `0x5CA0`.
- Candidate 8-entry table at `0x5EE0`: `0x18005b00 0x18005b00 0x18005b14 0x18005b74 0x18005bb2 0x18005c04 0x18005c1e 0x18005c46` (all EVEN — may not be the Thumb dispatch table; unconfirmed).
- No literal `0x18006000` or `0x18002201` in image; `0x00006000` literal at `0x21A7C`.
- Command name strings: `help`@`0x1AB2C`, `bootx`@`0x1B16C`, `version`@`0x1BCE6`, `reboot`@`0x1B35C`.
- Console input preseeded (device code): SRAM `0x22011100` = `"help\nbootx 60000000\n"`, index `0x22011180`; getchar `0x49C0` (wrapper `0x17D6A`); strcmp stub `0x17DE0` returns 0 (force match).
- **Build**: `ninja -C build qemu-system-arm`.
- **Headless trace**: `timeout 30 ./build/qemu-system-arm -M s5l8900 -bios "work/ROM BOOT, S5L8900 Rev.2" -kernel "work/iPod1,1_1.1_3A101a_Restore/Firmware/dfu/iBSS.n45ap.RELEASE.dfu" -nographic -d in_asm,unimp,guest_errors -D /tmp/inasm.txt`
- **One-time `DISPATCH:` diagnostic** lives in `s5l8900_step_trace_cb` (`hw/arm/s5l8900.c` ~line 720): when PC lands in SRAM safe-loop range `[0x2200f000,0x22010000)` it dumps `[0x18006000]` + 16 words at the aligned base + 16 words at `0x18002200` (guarded by `static disp_dumped`). **KEEP until fixed, then remove.** Note: the 1 ms step-timer no longer count-limits itself (auto-reschedules forever).

**GDB status** (`-gdb tcp::1234`, run QEMU with `-S`):
- `-S` pauses CPU at reset vector `0x2200ff00` (confirmed). Reading state works: `gdb -batch -ex "set pagination off" -ex "target remote :1234" -ex "set architecture arm" -ex "print/x \$pc"`.
- A `watch *(unsigned int*)0x18006000` + `commands{printf; continue}end` + `continue` (script `/tmp/watch.gdb`) produced **NO output** — not yet diagnosed. The watchpoint may fail to set on a raw address, or the batch `continue` didn't proceed. **Re-test with full (unfiltered) GDB output.**

**Resume steps (in order)**:
1. Re-run the GDB write-watchpoint with FULL output to see why it produced nothing. If `watch *(int*)0x18006000` won't set, fall back to detecting the change in `s5l8900_step_trace_cb` (track `[0x18006000]` per tick, log PC on the tick it changes — coarse but works) OR add a QEMU memory-write hook.
2. Identify the CPU store that writes `0x18002201` into `[0x18006000]` (PC in `0x1800xxxx`) and what table it *intended* to build.
3. Find the correct command dispatch table: search `work/iBoot.decrypted` for 8 consecutive odd `0x1800xxxx` Thumb pointers, or for xrefs to the command-name strings (`0x1AB2C`/`0x1B16C`).
4. Minimal fix — pick one: (a) patch `[0x18006000]` to the correct table base after launch; (b) fix the init code writing the wrong value; or (c) build a tiny command table (a `help` handler that prints help text, a `bootx` handler that jumps to the kernel-boot path) in writable SRAM and point `[0x18006000]` at it.
5. Verify: `help` prints help text, then `bootx 60000000` boots the kernelcache at `0x60000000` (kernelcache is preloaded there by the JUMP CB).

---

## Progress (history)

### Done
- Custom `s5l8900` machine type registered and functional
- ROM boots from SecureROM at `0x20000000`, hardware init completes
- USB stubs simulate enumeration, trigger IRQ through VIC0
- iBSS loaded and patched with crypto/hardware function stubs
- iBEC loaded at `0x0A000000`, self-copied to `0x18000000`
- Timer uses `QEMU_CLOCK_REALTIME` for reliable periodic callbacks
- UART stub at `0xE0002000` outputs to QEMU serial port
- iBoot decryption successful: AES-128-CBC, key `188458a6d15034dfe386f23b61d43774`, IV=0
- Decrypted iBoot binary at `work/iBoot.decrypted` (140288 bytes)
- iBoot loading moved outside `kernel_filename` conditional - iBoot now loads at `0x23000000` unconditionally
- Heap stubs added for main init (0x5CA0): malloc, free, allocator, heap init all stubbed to return fixed values from `0x22018000`
- BSS-GAP handler now directly modifies CPU state from periodic timer callback
- Periodic timer stopped before redirect (`timer_del`) to prevent interference
- `s5l8900_iboot_launched` flag set in BSS-GAP handler
- ARM trampoline at SRAM `0x2200F960` (LDR r14,[pc,#4]; BX r14) correctly jumps to main_init
- Banked SPs initialized to `0x22030000` for all exception modes
- evec handler at `0x00000100` fixed: uses ARM infinite loop (`B #-4`) instead of `BX LR`
- Used capstone to fully disassemble main_init (0x5CA0-0x5E00) and trace execution path
- Identified root cause: iBoot globals reference `0x18000000`-range addresses; running at `0x23000000` causes pointer mismatches
- Added comprehensive stubs for all unstubbed functions: 0x49C0, 0x17DE0, 0x57A8, 0x595E, 0x17E00, 0x75F4, 0x5490, 0x7E80, 0x7E40, 0x189C0, 0x18A20, 0x43E0, 0x4A88, 0x2FA8, 0x17866
- Changed 0x763C stub to return 0 (match) to take shortcut path
- **iBoot now prints "Hello from iPod!" and "iBoot" to serial output**
- **Fixed by copying iBoot from staging `0x23000000` to runtime `0x18000000` in jump callback**
- Replaced 58 occurrences of `S5L8900_IBOOT_BASE` with `S5L8900_IBOOT_RUNTIME`
- MMU, cache, and alignment enforcement disabled in SCTLR (M=0, C=0, A=0)
- Exception handler replaced with force-safe handler at `0x2200EC00` that switches to SVC mode and jumps to safe ARM loop at `0xF920`
- iBoot exception handler at `0xF000` also replaced with force-safe variant
- ARM+Thumb safe loops kept as infinite loops (`B #-4`) instead of `MOV r0,#1; BX LR`
- Fixed Thumb vector table patches: only ARM vectors patched (ARM1176 always enters exceptions in ARM mode), Thumb area filled with ARM NOPs
- Granular PC tracing added for iBoot runtime region (`0x18000000-0x18020000`)
- iBoot runtime region added to periodic callback tracking (`in_iboot_rt` check)
- Escape handler with full register dump on crash (LR, SP, CPSR, all GPRs, stack contents)
- Unified redirect target: all escapes redirect to SRAM safe ARM loop at `0x2200F920`
- Runtime data/BSS execution catch added (prevents executing from data section)
- LR validation in both BSS-GAP handler and periodic callback
- Exception handler trace marker at SRAM `0xED00`

### In Progress
- iBoot prints strings but crashes: LR gets corrupted to `0xEB000002` (ARM PLD instruction, not valid address)
- Crash chain: valid iBoot PC (0x180189E6) -> CPU executes data section (0x180355E6) -> crash at 0xFE0xxxxx
- Root cause: iBoot functions access uninitialized global data structures; zeros interpreted as function pointers
- CPU settles at safe loop after 2 escapes

### Blocked
- iBoot crashes from corrupted function returns before completing initialization
- LR corruption happens between BSS-GAP reads (timer too slow to catch in time)
- Would need to reverse-engineer iBoot global data structures or stub entire 0x18000+ code range
- 76 BX lr instructions in iBoot - too many to patch individually

## Key Decisions
- iBoot staging at `0x23000000`, runtime at `0x18000000` (matches iBoot's expected load address)
- Copy iBoot from staging to runtime in jump callback after iBEC is done with `0x18000000`
- Direct CPU state modification from periodic timer callback instead of `run_on_cpu`
- Stub heap functions instead of implementing full malloc/free
- Stop periodic timer before redirect to prevent interference
- Use `queue_tb_flush` + `CPU_INTERRUPT_EXITTB` to force exec loop restart
- Exception handlers force execution to safe loop instead of returning to faulting instructions
- Disable alignment enforcement to prevent spurious data aborts
- Only ARM vector tables patched (ARM1176 enters all exceptions in ARM mode)
- All redirects unified to SRAM safe ARM loop at `0x2200F920`

## Next Steps
- To get iBoot further: reverse-engineer global data structures at 0x18000+ range
- Identify specific functions that corrupt LR by tracing BL call chain from main_init
- Initialize iBoot globals with safe values instead of zeros
- Consider using QEMU's CPU log/hooks for per-instruction tracing to find exact LR corruption point

## Critical Context
- iBoot binary at `/Users/chris/dev/ipod-touch-1g/work/iBoot.decrypted` (140288 bytes)
- `S5L8900_IBOOT_BASE` = `0x23000000` (staging), `S5L8900_IBOOT_RUNTIME` = `0x18000000` (runtime)
- BSS region: `0x21980-0x26000` (pre-zeroed in buffer)
- Main init function: `0x5CA0` (Thumb), called from entry dispatcher at `0x4C00`
- ARM trampoline at `0x2200F960`: `LDR r14,[pc,#4]; BX r14` with literal `0x18005CA1`
- Force-safe exception handler at `0x2200EC00`: switches to SVC, jumps to safe loop at `0xF920`
- iBoot exception handler at `0x2200F000`: same force-safe pattern
- Safe ARM loop at `0x2200F920`: `B #-4`
- Safe Thumb loop at `0x2200FE00`: `B #-4`
- CPSR set to `0xD3` (SVC mode, I+F masked, V=0 for low vectors)
- SP set to `0x22030000`, all banked SPs initialized
- Global catch-all I/O region returns zeros for unmapped memory
- iBoot header at offset 0x000C: `0x18000000` (expected load address)
- Crash pattern: LR corrupted to `0xEB000002` (ARM PLD instruction bytes, not valid address)
- Crash PC: `0x180189E6` (valid iBoot code doing bit manipulation)
- CPU jumps to `0x180355E6` (beyond binary, zero-filled) before crashing at `0xFE0xxxxx`
- `tb_flush__exclusive_or_serial` crashes from timer callback; use `queue_tb_flush` instead
- QEMU build at `/Users/chris/dev/ipod-touch-1g/build/qemu-system-arm-unsigned`
- Capstone used for disassembly; all BL targets from main_init identified and mapped
- 76 BX lr instructions in iBoot binary, 3 POP {...,pc}, 1 BLX rN

## Relevant Files
- `hw/arm/s5l8900.c`: Main QEMU device emulation file — contains ROM loading, iBSS/iBEC/iBoot loading, patching, UART, memory mapping, BSS-GAP handler, CPU redirect, step trace timer, evec handler, comprehensive function stubs, iBoot staging-to-runtime copy, force-safe exception handlers, granular PC tracing, LR validation, escape handler
- `work/iBoot.decrypted`: Decrypted iBoot binary (140288 bytes)
- `work/ROM BOOT, S5L8900 Rev.2`: SecureROM firmware
- `work/iPod1,1_1.1_3A101a_Restore/Firmware/dfu/iBSS.n45ap.RELEASE.dfu`: iBSS firmware
- `work/run.sh`: Launch script with GDB integration
- `work/progress.md`: Progress tracking file
- `work/notes.txt`: Memory map and reference notes
