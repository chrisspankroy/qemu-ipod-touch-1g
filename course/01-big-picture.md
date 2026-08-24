# Lesson 01: The Big Picture

**Goal:** Understand what this project is, why it's built this way, and what
"recreated" will mean when you finish.

**Prerequisites:** none. This is orientation.

---

## 1. The device

The iPod Touch 1st generation (iPod1,1) uses a Samsung **S5L8900** SoC:

- CPU: **ARM1176JZF-S** (ARMv5TE, 266 MHz, with a real MMU)
- Internal **SRAM**: 512 KB at `0x22000000` (no DRAM on-chip; DDR is external)
- Storage: NAND flash (two 4 GB chips)
- Peripherals live in the `0x38000000`–`0x3fffffff` region
- A 64 KB **SecureROM** at `0x20000000` that runs at reset, no matter what

Full memory map: `CLAUDE.md` at the repo root. Keep it open in a tab from now
on.

Two facts about this device shape the entire project:

1. **DFU mode lives in the SecureROM.** Even with completely erased flash, the
   chip comes up on USB as a DFU device that will accept firmware. This makes
   the device unbrickable — and it makes the SecureROM the most interesting
   piece of firmware on the device.
2. **iOS 1.x had no SHSH/downgrade protection.** The old firmware can be
   obtained legally and freely (it's still on Apple's servers via the IPSW
   restore files), and it can be run, studied, and (as we do) emulated
   without any modern cryptographic wall.

## 2. The boot chain

On real hardware, booting works like this:

```
SecureROM (64KB, in ROM)
   │  verifies + jumps to
   ▼
LLB ("low level bootloader", ~64KB, in NOR)
   │
   ▼
iBoot (~500KB, in NOR)
   │
   ▼
kernelcache (the XNU kernel + kexts, ~3MB)
   │
   ▼
iOS userland
```

Each stage **verifies the next stage** (Apple Secure Boot: AES-decrypt the
payload, SHA1 it, check the RSA signature against Apple's root certificate
embedded in ROM) and only then jumps to it.

There is a second, rescue boot chain used when the device is held in DFU
(recovery) mode. It skips the NOR-based stages:

```
SecureROM  ──DFU over USB──▶  iBSS  ──▶  iBEC  ──▶  iBoot  ──▶  kernel
```

**iBSS** is a tiny "baseband-less subsystem" loader: it brings up just enough
hardware (USB, clocks, watchdog) to keep the DFU channel alive, then waits for
the *next* image. **iBEC** is the "recovery environment controller": it takes
the next DFU image (iBoot) and sets up a more complete environment for it.

This DFU chain is what we run in this project — because it's the chain that
starts in ROM with nothing else required, and because the ROM's DFU protocol
is a well-defined contract we can play the other side of.

## 3. The problem in one sentence

> **The ROM is waiting for a USB host that doesn't exist.**

Power on a real device, and the ROM comes up, initializes hardware, and spins
in a DFU wait loop: "host, send me iBSS." On a bench, the host is a Mac
running iTunes/`irecovery`. In our project, the host is… **us, inside the
emulator.**

That's the entire project in one line. Every lesson is some part of this:

1. **Make the chip exist** in software (a QEMU machine model: CPU, SRAM, ROM,
   UART, clocks, interrupt controller, USB/DMA stubs) → lessons 03, 06, 07.
2. **Make the ROM run** unmodified → lessons 05, 07, 08.
3. **Be the USB host**: when the ROM/iBSS waits for an image, hand it the real
   iBSS, then iBEC, then iBoot — including the setup each stage needs (where
   to load it, what state to leave the CPU in) → lessons 09.
4. **Make iBoot useful in the emulated world**: it expects a real console and
   a real device tree; we patch a handful of its functions so it can print to
   our UART and read preseeded input → lessons 10, 11.

## 4. What you will end up with

A working QEMU command that boots four real Apple firmware images and prints
the iBoot console (the block in the course README). Plus, more valuably:

- A 6000-line machine model you *understand line by line* because you built it
  in stages, not one you copied.
- A fluent working knowledge of ARM bare-metal, MMIO, the ARM MMU, USB/DFU,
  and QEMU's device model.
- A repeatable **debug loop** (lesson 08) that you can apply to any
  embedded-firmware problem ever again.

## 5. What you will NOT do (and why)

- **We do not emulate USB at the wire level.** Implementing the DWC2
  controller + PHY + host stack is a multi-month job. Instead we *observe*
  where firmware blocks waiting for the host, and have QEMU perform the
  host's job directly (stage the next image into RAM, fix up CPU state, jump).
  This is a deliberate engineering trade-off: it reproduces the *observable*
  behavior (serial output, execution of real code) without the wire.
  Lesson 12 discusses what it would take to do it "for real."
- **We stop at the iBoot console.** Booting the full kernel (`bootx`) is the
  documented next milestone and is not solved in this repo. It is your
  graduation project (lesson 12).

## 6. Read before the next lesson

Skim, don't study:

- `CLAUDE.md` (root) — the memory map.
- `work/notes.txt` — the original 26-line note that started it all.
- `work/progress.md` — just the headers and the "Current Goal" section. You'll
  re-read it deeply in later lessons.

## Checkpoint

Answer these in your own notes (one paragraph each):

1. In the DFU boot chain, which image runs first, and what is it *waiting for*
   when it stops?
2. Why can't we just load iBoot directly in QEMU and skip the ROM/iBSS/iBEC?
   (Hint: think about what state each stage leaves behind, and what "real,
   unmodified" means as a project goal.)
3. What does QEMU stand in for, in the sentence "the ROM is waiting for a USB
   host"?
4. Look at `hw/arm/s5l8900.c` and count how many `#define` lines you see in
   the first 100. Why do you think the file starts with so many address
   constants?

If you can answer all four, you're ready for [Lesson 02: The Toolbox](02-toolbox.md).
