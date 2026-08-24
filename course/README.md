# Course: Booting the Real iPod Touch 1G Firmware in QEMU

A hands-on reverse engineering course. By the end, you will have **recreated,
from scratch, the work in this repository**: a QEMU machine model for the
Samsung S5L8900 SoC that boots the **real, unmodified** Apple boot chain —
SecureROM → iBSS → iBEC → iBoot — and prints the genuine iBoot command
console over emulated serial.

You will learn this by doing the same work, in the same order, with the same
files.

---

## What "the work" is, in one paragraph

The iPod Touch 1G boots through a chain of small firmware images, each
verified by the previous one. The first stage, the **SecureROM**, is burned
into the chip and speaks a USB **DFU** (Device Firmware Upgrade) protocol:
wait for a host to send the next image, verify it, jump to it. We take the
real 64 KB SecureROM, run it in an emulator we write ourselves, and make the
emulator act as the missing USB host — feeding it the real iBSS, iBEC, and
iBoot images until iBoot's interactive console comes up on serial.

The end state (what you will reproduce):

```sh
./build/qemu-system-arm -M s5l8900 \
  -bios "work/ROM BOOT, S5L8900 Rev.2" \
  -kernel "work/iPod1,1_1.1_3A101a_Restore/Firmware/dfu/iBSS.n45ap.RELEASE.dfu" \
  -nographic
```

prints:

```
Hello from iPod!
iBoot
] help
Available commands:
  help - this list
  argtest
  ...
  setpicture - set the image on the display
]
```

The `iBoot`, the `] ` prompt, and the command list are the **real firmware**
talking through emulated hardware.

## What you need before starting

- **C** at a comfortable level (pointers, structs, `#define`s). You will read
  QEMU's C and write ~500 lines of it.
- **A terminal you are not afraid of.** Unix CLI, grep, xxd, git.
- macOS (this repo was built on macOS with Homebrew) or Linux.
- A disassembler. **Hopper** was used here (the `*.hop` project files in
  `work/` are Hopper projects). Free alternatives: **Ghidra** or **radare2**.
  Lesson 02 covers all three, and most analysis can be done with a 20-line
  Python + [capstone](https://www.capstone-engine.org/) script.
- Time: roughly 4–8 focused hours per lesson. The whole course is a multi-week
  project if done carefully. Rushing the ARM fundamentals (lesson 03) makes
  everything after it 10x harder.

## The repo you are working in

This course lives inside the actual project repo. Know what is what:

| Path | What it is |
|---|---|
| `/` (repo root) | A fork of **QEMU** (the emulator). You will build and modify it. |
| `hw/arm/s5l8900.c` | **The deliverable.** The complete S5L8900 machine model (6138 lines). You will build this incrementally in lessons 07–11. Read it as a *reference*, not something to memorize. |
| `build/` | QEMU build tree (`ninja -C build`). |
| `work/` | **All the firmware and notes.** ROM dumps, the extracted iOS 1.1 IPSW, decrypted iBoot, progress logs, helper scripts. |
| `work/ROM BOOT, S5L8900 Rev.2` | The 64 KB SecureROM binary (raw, unmodified). |
| `work/iBoot.decrypted` | iBoot: 0x400-byte img2 header + decrypted payload. |
| `work/iPod1,1_1.1_3A101a_Restore/` | Extracted iOS 1.1 restore image (iBSS, iBEC, kernelcache, …). |
| `work/progress.md` | The real session-by-session history of how this was done, including dead ends. **Read it as a primary source.** |
| `CONCEPTS.md` | Deep reference on ARM, USB, DFU, and QEMU internals. The "textbook" for this course. |
| `CLAUDE.md` | Hardware memory map and project reference. |

## How to use this course

1. **Work in order.** Each lesson's "Do" section is a set of actions; the
   "Checkpoint" section is your definition of done. Don't skip checkpoints —
   they are how you know you actually understood something.
2. **Do it, don't just read it.** Every lesson has commands to run and a
   checkpoint to hit. If you only read, you will have learned 20%.
3. **Keep your own notes.** Copy `work/progress.md`'s structure (latest state
   on top, repro commands, open questions). Lesson 12 explains why this
   habit is the single most valuable thing you can take from this course.
4. **When stuck, use the repo.** `hw/arm/s5l8900.c` and `work/progress.md`
   contain the answers to almost everything. But find the answer *yourself
   first* — that's the point.
5. **You will make the same mistakes we did.** They are documented. When you
   hit one of them, the lesson tells you the symptom and the fix.

## Lesson map

| # | Lesson | You will… |
|---|--------|-----------|
| 01 | [The Big Picture](01-big-picture.md) | Understand the project, the boot chain, and what "done" means |
| 02 | [The Toolbox](02-toolbox.md) | Set up QEMU, GDB, a disassembler, and capstone; learn the address↔offset arithmetic |
| 03 | [ARM From Zero](03-arm-basics.md) | Read real ARM/Thumb disassembly: registers, CPSR, AAPCS, pipeline, MMIO, CP15, exceptions |
| 04 | [Get The Firmware](04-firmware.md) | Obtain and extract the IPSW, understand img2, decrypt iBoot, obtain the SecureROM |
| 05 | [Read The SecureROM](05-read-the-rom.md) | Your first real RE: reset vector → hardware init → USB/DFU main loop, with a real disassembler |
| 06 | [QEMU Internals](06-qemu-internals.md) | How QEMU boots a machine, memory regions, device ops, timers, logging, GDB |
| 07 | [Build The Machine](07-machine-model.md) | Write `s5l8900.c` from an empty file until the real SecureROM boots in QEMU |
| 08 | [The Debug Loop](08-debug-loop.md) | The core methodology: trace, break, capture faults, interpret them, fix, repeat |
| 09 | [iBSS And The Handoff](09-ibss-handoff.md) | Run unmodified iBSS, find the one emulation gap, and perform the iBSS→iBoot handoff under a real MMU |
| 10 | [Crack iBoot](10-crack-iboot.md) | RE the iBoot binary: layout, strings, xrefs, putchar/getchar, the dispatch-table mystery, runtime patching |
| 11 | [Ship The Console](11-ship-the-console.md) | Wire the console end-to-end: preseeded input, `help`, the UART hook, clean halt |
| 12 | [Graduation](12-graduate.md) | Where to go next (kernel boot, real DFU), the verification mindset, and further study |
| — | [Appendix](appendix.md) | Master address tables, GDB/QEMU cheat sheets, capstone snippets, glossary |

## A word about how this was actually done

Be warned before you start: the history in `work/progress.md` is not a
straight line. There were brute-force patching strategies that were later
abandoned, crashes that took days, stale binaries that produced wrong
observations, and a "clean baseline" rework that threw out hacks in favor of
emulating the hardware correctly. **That zig-zag is normal and is the actual
skill.** If your experience following this course is mostly confusion and
small victories, you are doing it right.

The one rule that separates progress from spinning: **every claim must be
verifiable by running something.** "I think iBSS hangs because of the PLL
poll" is not a claim. "Run with `-d in_asm`, and the PC is at `0x22000f94`
executing `tst r1,r3` after writing mask `0x8` to `CLOCK1+0x40`" is a claim.
Lesson 08 is entirely about building that habit.
