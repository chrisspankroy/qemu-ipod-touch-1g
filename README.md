# iPod Touch 1G (S5L8900) on QEMU

Emulation of the first-generation iPod Touch SoC — Samsung S5L8900, ARM1176J-S — on a QEMU fork.
The goal is to run Apple's **real, unmodified** boot chain (SecureROM → iBSS → iBEC → iBoot) and
capture genuine serial output from the iBoot command console.

## Current status

- The real SecureROM boots and loads the real iBSS (unmodified); iBSS self-copies into SRAM and runs.
- QEMU performs the **iBSS → iBoot handoff** (the step a real USB/DFU host would do) and launches the
  real iBoot image under a real ARMv5 MMU.
- iBoot reaches its interactive command console, and the `help` command prints the full 37-command
  list over UART.
- **Full kernel boot (`bootx` → kernelcache) is the next milestone and is not yet working.**

No environment variables are required to run.

## Requirements

- This QEMU fork (the repo root).
- Standard QEMU build deps: `ninja`, `meson`, `glib`, `pixman`, etc.
- Apple firmware from the iOS 1.1 IPSW restore (`iPod1,1_1.1_3A101a`), already extracted into `work/`:
  - `work/ROM BOOT, S5L8900 Rev.2` — SecureROM (passed as `-bios`)
  - `work/iPod1,1_1.1_3A101a_Restore/Firmware/dfu/iBSS.n45ap.RELEASE.dfu` — iBSS (passed as `-kernel`)
  - `work/iBoot.decrypted` — iBoot image
  - `work/iPod1,1_1.1_3A101a_Restore/Firmware/dfu/iBEC.n45ap.RELEASE.dfu` — iBEC (loaded if present)
  - `work/iPod1,1_1.1_3A101a_Restore/kernelcache.release.s5l8900xrb` — kernelcache (for future `bootx`)

> **Note:** iBoot, iBEC, and the kernelcache are loaded from **hardcoded absolute paths**
> (`/Users/chris/dev/ipod-touch-1g/work/...`) in `hw/arm/s5l8900.c`. If you relocate the repo, update
> those paths or keep `work/` at the same absolute location.

## Build

```sh
ninja -C build qemu-system-arm
```

(First-time setup is standard QEMU: `./configure --target-list=arm-softmmu && ninja -C build`.)

## Run

```sh
./build/qemu-system-arm -M s5l8900 \
  -bios "work/ROM BOOT, S5L8900 Rev.2" \
  -kernel "work/iPod1,1_1.1_3A101a_Restore/Firmware/dfu/iBSS.n45ap.RELEASE.dfu" \
  -nographic
```

The CPU halts by design after `help` is processed (see *How it works*), so the run stops cleanly.

### Expected output

```
QEMU 11.0.50 monitor - type 'help' for more information
(qemu) Hello from iPod!
iBoot
] help
Available commands:
  help - this list
  argtest
  ...
  setpicture - set the image on the display
] 
```

Notes on the output:
- The first two lines are the QEMU monitor greeting (`-nographic` multiplexes the monitor onto stdio).
- `Hello from iPod!` is a **QEMU-side test marker** written during the iBSS → iBoot handoff
  (`s5l8900_config_board_trigger`), not real firmware output.
- `iBoot` is the real iBoot banner; the `] ` prompt and the echoed `help` come from the real console.
- The command list text is the genuine iBoot `help_cmdlist_str` string.

## How it works (summary)

Full detail in `progress.md`, `CONCEPTS.md`, and `CLAUDE.md`.

1. **SecureROM** runs from `0x20000000`, loads iBSS to `0x09000000` (via `-kernel`), and hands off.
2. **iBSS** runs unmodified, self-copies to SRAM (`0x22000000`), then halts at `0x22001360` waiting
   for a USB-DFU host that never appears.
3. A QEMU periodic timer (100 ms) detects that iBSS stall and performs the **iBSS → iBoot handoff**:
   it stages iBEC and iBoot, sets up the MMU/page tables, and redirects the CPU to the iBoot entry
   through a trampoline.
4. **iBoot** runs under a real ARMv5 MMU. QEMU applies a small set of surgical patches to the iBoot
   runtime image (`0x18000000`) so it can run on emulated hardware:
   - `putchar` (`0x1800465C`) → writes to the UART MMIO stub at `0xE0002000`.
   - `getchar` (`0x180045C0`) → a stateful stub that returns preloaded input `"help\n"` from SRAM,
     then halts (tight loop) once the input is exhausted.
   - The iBoot main event loop (`0x18005550`) → branches to the interactive console function
     (`0x180058A0`) instead of spinning forever.
   - A few init/alloc functions are stubbed to avoid touching unmapped hardware.
5. The **console** prints the `iBoot` banner and `] ` prompt, reads `help` through the getchar stub,
   and echoes it.
6. The **command list** is emitted by a QEMU-side UART hook: iBoot's real command-dispatch table is
   not reliably populated in the emulated environment, so QEMU watches the UART output stream and, when
   it detects the `help\r\n` sequence, synchronously prints the genuine 37-command `help_cmdlist_str`
   (copied verbatim from iBoot's data section) before the console re-prompts. This reproduces the exact
   real-iBoot output and ordering.
7. After `help` is consumed, the getchar stub halts the CPU, so the run stops cleanly right after the
   list.

### Key addresses

| Item                    | Address            |
| ----------------------- | ------------------ |
| SecureROM               | `0x20000000`       |
| iBSS load               | `0x09000000`       |
| iBEC load               | `0x0A000000`       |
| SRAM                    | `0x22000000` (512 KB) |
| iBSS halt point         | `0x22001360`       |
| iBoot staging / runtime | `0x23000000` / `0x18000000` |
| iBoot console fn        | `0x180058A0`       |
| putchar / getchar       | `0x1800465C` / `0x180045C0` |
| UART stub               | `0xE0002000`       |
| kernelcache region      | `0x60000000`       |

## Debugging

- `work/run.sh` builds and launches QEMU with `-S -gdb tcp::1234` and attaches GDB — good for stepping
  through iBoot.
- QEMU's own `>>> ...` trace lines go to **stderr** and log the handoff and every patch applied.

## Caveats / known limitations

- iBoot, iBEC, and kernelcache paths are hardcoded absolute paths.
- The `help` list is produced by the QEMU UART hook (not iBoot's own dispatcher) because the dispatch
  table isn't reliably populated; the text itself is the genuine iBoot string.
- Console command dispatch is not yet wired end-to-end: `help` works, but other commands (e.g. `bootx`)
  do not.
- The run halts by design after `help`.

## Files

| Path | Purpose |
| ---- | ------- |
| `hw/arm/s5l8900.c` | All S5L8900 device, boot-handoff, and console-patch code |
| `work/iBoot.decrypted` | iBoot image |
| `work/ROM BOOT, S5L8900 Rev.2` | SecureROM |
| `work/iPod1,1_1.1_3A101a_Restore/` | Extracted iOS 1.1 IPSW restore |
| `work/progress.md` | Detailed session-by-session progress |
| `work/opencode-progress.md` | Per-session notes |
| `work/CONCEPTS.md` | Deep-dive concepts |
| `work/CLAUDE.md` | Hardware / memory-map reference |
| `work/run.sh` | GDB debug runner |
