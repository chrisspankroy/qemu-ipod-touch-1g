# Appendix

Quick-reference material. Keep this open while you work.

---

## A. Address tables

### A.1. Physical memory map (S5L8900, iPod1,1)

| Region | Base | Size | Notes |
|---|---|---|---|
| Exception vectors (evec) | `0x00000000` | `0x1000` | ROM copies its vectors here |
| iBSS load | `0x09000000` | 256 KB | where `-kernel` iBSS lands |
| iBEC load | `0x0A000000` | 256 KB | |
| **SecureROM (VROM)** | `0x20000000` | 64 KB (region 1 MB) | boots from here |
| **SRAM** | `0x22000000` | 512 KB | iBSS self-copies here; iBoot page table here |
| iBoot staging | `0x23000000` | 1 MB | |
| USB-OTG / **iBoot runtime** | `0x18000000` | 2 MB | iBoot runs here (its link address) |
| A-bit RAM | `0x60000000` | 512 MB | kernelcache preload |
| Peripheral window | `0x38000000` | 128 MB | |
| CLOCK0 | `0x38100000` | | PLL programming |
| USB OTG (DWC2) | `0x38c00000` | | DFU engine |
| VIC0 / VIC1 / EDGEIC | `0x38e00000` / `0x38e01000` / `0x38e02000` | | interrupt controllers |
| GPIOIC | `0x39a00000` | | |
| NAND | `0x3c300000` | | |
| USB PHY | `0x3c400000` | | |
| CLOCK1 | `0x3c500000` | | **`+0x40` = PLL lock status** |
| Timer | `0x3e200000` / `0x3e400000` | | |
| **WDT** | `0x3e300000` | | iBSS enables+kicks before parking |
| PMU | `0x3e500000` | | **`+0x04` = power status** |
| **UART** | `0xE0002000` | `0x1000` | `+0` data, `+5` line status |

### A.2. Special runtime addresses

| Item | Address |
|---|---|
| iBSS natural halt (`b .`) | `0x22001360` |
| iBoot page table (in SRAM) | `0x22038000` |
| Preloaded input buffer | `0x22011100` (index at `0x22011180`) |
| SRAM safe loop | `0x2200FE00` |
| Exception-handler storage block | `0x2200F800` (IFAR/DFAR/CPSR/LR at `+0x24..+0x30`) |
| iBoot BSS | `0x18021980` – `0x18026000` |
| iBoot reset handler (ARM) | `0x18000040` |
| iBoot `main_init` (Thumb; = console) | `0x180058A0` |
| iBoot main event loop | `0x18005550` |
| iBoot console function | `0x180058A0` |
| iBoot `putchar` | `0x1800465C` |
| iBoot `getchar` | `0x180045C0` |
| iBoot command table (37 cmds) | `0x1801F28C` |
| iBoot `bootx` handler | `0x18006011` |

### A.3. File ⇄ address offset formulas

The single most-used reference in the whole course. "file" = byte offset in
the on-disk file; "link/runtime" = the address the code thinks it's at.

| Image | File layout | Formula |
|---|---|---|
| **SecureROM** | raw, no header, no encryption | `file F = addr A − 0x20000000` |
| **iBSS / iBEC `.dfu`** | `89001.0` wrapper (0x800) + plaintext payload | payload `P` at file `0x800 + P`; runtime `R` (after self-copy to `0x22000000`) → `P = R − 0x22000000`; so **`file F = R − 0x22000000 + 0x800`** |
| **iBoot** (`iBoot.decrypted`) | `IMG2`/`ibot` subheader (0x400) + payload | payload linked at `0x18000000`; link `L` → `P = L − 0x18000000`; so **`file F = L − 0x18000000 + 0x400`** |

Worked examples:
- iBSS halt at runtime `0x22001360` → file `0x22001360 − 0x22000000 + 0x800 = 0x1B60`.
- iBoot console at link `0x180058A0` → file `0x180058A0 − 0x18000000 + 0x400 = 0x5CA0`.
- iBoot `putchar` at link `0x1800465C` → file `0x465C + 0x400 = 0x4A5C`.

> Note the two "header" sizes that keep appearing: **0x800** (the `.dfu` /
> `89001.0` wrapper, skipped when loading iBSS/iBEC) and **0x400** (the inner
> `IMG2`/`ibot` subheader inside `iBoot.decrypted`, skipped when loading the
> iBoot payload). Don't conflate them.

---

## B. Cheat sheets

### B.1. QEMU command line

```sh
# Build
ninja -C build qemu-system-arm

# Run (the standard invocation)
./build/qemu-system-arm -M s5l8900 \
  -bios "work/ROM BOOT, S5L8900 Rev.2" \
  -kernel "work/iPod1,1_1.1_3A101a_Restore/Firmware/dfu/iBSS.n45ap.RELEASE.dfu" \
  -nographic

# Debug: start halted, open GDB on :1234, log unimplemented + guest errors
./build/qemu-system-arm -M s5l8900 -bios "$ROM" -kernel "$IBSS" \
  -nographic -S -gdb tcp::1234 -d unimp,guest_errors -D /tmp/q.log

# Trace every translated basic block (use with timeout!)
timeout 5 ./build/qemu-system-arm -M s5l8900 -bios "$ROM" -kernel "$IBSS" \
  -nographic -d in_asm -D /tmp/trace.log
```

Key flags: `-bios`→`machine->firmware`, `-kernel`→`machine->kernel_filename`,
`-S` (start halted), `-gdb tcp::PORT`, `-d in_asm,unimp,guest_errors`,
`-D file` (redirect -d logs), `-singlestep`.

### B.2. GDB (ARM)

```gdb
set architecture arm
target remote :1234

# inspect
info registers                 # PC=$pc, CPSR=$cpsr, r0..r15
x/20i $pc                      # disassemble 20 instructions at PC
x/xw 0x2202bff8                # one guest word
x/4xw 0x2200F820               # the fault-capture storage block

# control
hbreak *0x22001360             # HARDWARE bp (required for ROM / read-only)
break *0x180058A0              # software bp (writable RAM)
continue
stepi

# poke-and-see (test a hypothesis live)
set {int}0x3c500040 = 0xFFFFFFFF
set {int}0x3e500004 = 8
```

### B.3. Reading a fault-capture block

Layout written by the planted handler (lesson 08.5):

```
0x2200F824  IFAR   (faulting instruction address, if any)
0x2200F828  DFAR   (faulting DATA address — the key value)
0x2200F82C  CPSR   (mode in [4:0], T bit = bit 5)
0x2200F830  LR     (return link -> faulting PC + 8 (ARM) or + 4 (Thumb))
```

So: **faulting instruction = LR − 8 (ARM) or LR − 4 (Thumb)**; faulting
address = DFAR; fault type = DFSR (read from the C side, `env.cp15.dfsr_s`).

---

## C. ARM/Thumb instruction encodings (verified in this codebase)

Use these for hand-encoding stubs and patches. **Always verify a non-trivial
encoding with capstone before writing it to guest memory.**

| Mnemonic | Encoding | Bytes (LE) | Notes |
|---|---|---|---|
| ARM `B .` (self-loop) | `0xEAFFFFFE` | `FE FF FF EA` | infinite loop |
| ARM `B <off>` | `0xEA000000 \| (off>>2)` | — | off in words, sign-extended |
| ARM `NOP` | `0xE1A00000` | `00 00 A0 E1` | |
| ARM `LDR r0,[pc,#imm]` | `0xE59F0000 \| imm` | e.g. `0xE59F003C` | reads `[PC+8+imm]` |
| ARM `BX r0` | `0xE12FFF10` | `10 FF 2F E1` | `BX rX` = `0xE12FFF10 \| X` |
| ARM `MOV pc,#0` | `0xE320F000` | | |
| ARM `MRS rX,cpsr` | `0xE10F0000 \| (X<<12)` | | |
| Thumb `B .` (self-loop) | `0xE7FE` | `FE E7` | **not** `0xFEFE` |
| Thumb `NOP` | `0xBF00` | `00 BF` | |
| Thumb `LDR r0,[pc,#imm]` | `0x4800 \| (imm*4)` | e.g. `0x4800` | reads `[PC+4+imm*4]` |
| Thumb `BX r0` | `0x4700` | `00 47` | |
| Thumb `MOV r0,#n` | `0x2000 \| n` | | |
| Thumb `BX LR` | `0x4770` | `70 47` | safe return |

PC-relative gotchas:
- **ARM** `LDR r0,[pc,#imm]`: the literal is at `instruction_addr + 8 + imm`.
- **Thumb** `LDR r0,[pc,#imm]`: the literal is at `instruction_addr + 4 + imm*4`.
- The ARM `B` targets `PC + 8 + (signext(imm24)<<2)`; Thumb `B` targets
  `PC + 4 + signext(imm8)<<1`.

---

## D. Firmware container & crypto reference

### D.1. The `89001.0` wrapper (`.dfu` and all-flash `.img2`)

```
offset 0x000  magic "89001.0" + version byte
offset 0x00C  payload size (e.g. 0x22400)
offset 0x014  payload end offset (0x800 + size)
offset 0x800  payload begins
   - iBSS / iBEC : PLAINTEXT ARM code (no encryption)
   - iBoot       : AES-128-CBC ciphertext
trailing       RSA signature + certificate
```

### D.2. Decrypting iBoot (reproduces `work/iBoot.decrypted`)

```sh
IB="work/iPod1,1_1.1_3A101a_Restore/Firmware/all_flash/all_flash.n45ap.production/iBoot.n45ap.RELEASE.img2"
dd if="$IB" bs=1 skip=2048 count=140288 of=/tmp/payload.enc
openssl enc -d -aes-128-cbc -nopad \
  -K 188458a6d15034dfe386f23b61d43774 \
  -iv 00000000000000000000000000000000 \
  -in /tmp/payload.enc -out /tmp/iBoot.full
cmp /tmp/iBoot.full work/iBoot.decrypted && echo MATCH
```

- `-nopad` is required (the image isn't block-padded the way OpenSSL expects
  by default).
- Result: 140288 bytes = `IMG2`/`ibot` subheader (0x400) + code payload
  (0x22000). The load address `0x18000000` is at subheader offset `0x0C`.

### D.3. Root filesystem

`022-3601-4.dmg` (system/rootfs) is AES-encrypted; the key is in
`3A101a_root_filesystem_key.txt`. Only needed for a full `bootx` (lesson 12).

---

## E. DFSR fault-type codes (ARMv5)

The Data Fault Status Register low 4 bits tell you *why* a data access faulted:

| DFSR[3:0] | Type | Typical cause in this project |
|---|---|---|
| `0b0010` | Translation fault (section) | page-table entry missing / `[1:0]=00` (the 0x30-vs-0x32 bug) |
| `0b0011` | Translation fault (page) | — |
| `0b0110` | Permission fault | domain / AP mis-set (check DACR) |
| `0b1111` | External / abort | access hit a region QEMU treats as invalid |

Read it from the C side (`cpu->env.cp15.dfsr_s`) — not from inside the guest
exception handler (nested-exception risk, lesson 08.5).

---

## F. Glossary

- **ARM1176J-S** — the CPU core in the S5L8900 SoC; ARMv5TEJ, has Jazelle,
  Thumb-2, and an MMU.
- **S5L8900** — the Samsung "SoC" (system-on-chip) of the iPod Touch 1G.
- **n45ap** — the board identifier for the iPod Touch 1G (Apple's naming).
- **SecureROM / VROM** — the read-only boot ROM in silicon; runs first.
- **iBSS / iBEC / iBoot** — Apple's boot-loader chain: iBSS (stub, receives
  iBEC over USB DFU) → iBEC (receives iBoot) → iBoot (full bootloader, console,
  boots the kernel).
- **kernelcache** — a Mach-O bundle: the XNU kernel + kernel extensions.
- **DFU** — Device Firmware Upgrade; the USB protocol the boot ROM uses to
  receive the next stage.
- **MMU / page table** — the ARMv5 short-descriptor translation mechanism
  (4096-entry L1 table, 1 MB sections here).
- **TTBR0 / SCTLR / DACR** — CP15 registers: table base, control (M/A/C/I
  bits), domain access control.
- **CPSR** — current program state register (mode in [4:0], T bit = bit 5,
  I/F interrupt masks).
- **Thumb / ARM** — the two instruction sets; the T bit selects; `BX`/odd
  addresses switch.
- **TCG** — QEMU's dynamic translator (JITs guest code to host code).
- **MemoryRegion / ops** — QEMU's memory-model primitive (address range +
  read/write callbacks).
- **catch-all** — a low-priority memory region covering all unmapped space
  (reads 0, writes no-op).
- **staging / runtime** — iBoot is loaded to a staging region, then copied to
  its runtime link address `0x18000000`.
- **Path A** — the "run iBoot at its true link address so no literal surgery
  is needed" approach (payload only, no header).
- **poolwatch / step timer / PC sampling** — the three instrumentation probes
  (lesson 08.3).
- **fault-capture handler** — a planted exception handler that dumps
  DFAR/IFAR/CPSR/LR and parks (lesson 08.5).
- **SHSH** — Apple's signed-boot ticket; the iPod 1G has **no** SHSH /
  downgrade protection.

---

## G. File map

### G.1. This course

| File | Contents |
|---|---|
| `README.md` | overview, journey map, how to use |
| `01-big-picture.md` | project, boot chain, what "done" means |
| `02-toolbox.md` | environment setup |
| `03-arm-basics.md` | minimal ARM |
| `04-firmware.md` | IPSW, extraction, IMG2, iBoot decryption, SecureROM |
| `05-read-the-rom.md` | reverse-engineering the SecureROM |
| `06-qemu-internals.md` | how QEMU emulates; the machine/device API |
| `07-machine-model.md` | build `s5l8900.c` from empty to a booting SecureROM |
| `08-debug-loop.md` | the core RE methodology in QEMU |
| `09-ibss-handoff.md` | iBSS, the handoff, the ARMv5 MMU |
| `10-crack-iboot.md` | reverse-engineer + surgically patch iBoot |
| `11-ship-the-console.md` | the console, the UART hook, verification |
| `12-graduate.md` | next steps (bootx / USB-DFU), skill map |
| `appendix.md` | this file |

### G.2. Repo files you'll touch

| Path | Purpose |
|---|---|
| `hw/arm/s5l8900.c` | the deliverable — all device/boot/console code |
| `hw/arm/Kconfig`, `configs/devices/arm-softmmu/default.mak`, `hw/arm/meson.build` | machine wiring |
| `work/ROM BOOT, S5L8900 Rev.2` | 64 KB SecureROM (`-bios`) |
| `work/iPod1,1_1.1_3A101a_Restore/Firmware/dfu/iBSS.n45ap.RELEASE.dfu` | iBSS (`-kernel`) |
| `work/iPod1,1_1.1_3A101a_Restore/Firmware/dfu/iBEC.n45ap.RELEASE.dfu` | iBEC |
| `work/iBoot.decrypted` | decrypted iBoot (0x400 header + 0x22000 payload) |
| `work/iPod1,1_1.1_3A101a_Restore/Firmware/all_flash/.../iBoot.n45ap.RELEASE.img2` | encrypted iBoot source |
| `work/iPod1,1_1.1_3A101a_Restore/kernelcache.release.s5l8900xrb` | kernelcache (for bootx) |
| `work/3A101a_root_filesystem_key.txt` | rootfs key |
| `work/run.sh` | GDB debug runner |
| `work/xref.py` | capstone xref scanner |
| `work/progress.md` | session-by-session history |
| `work/notes.txt`, `CLAUDE.md`, `CONCEPTS.md` | hardware map + deep-dive |
| `build/qemu-system-arm` | the built emulator |

---

**You made it.** Re-read lesson 08 (the loop) whenever you feel lost — it's
the whole game. Then go build something.
