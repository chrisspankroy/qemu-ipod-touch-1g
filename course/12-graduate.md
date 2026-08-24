# Lesson 12: Graduate

**Goal:** Know exactly where you are, what "done" means, and what the honest
next steps are. Then map the skills you've built to the larger world. This is
a reflection + roadmap lesson, not a build lesson.

**Prerequisites:** Lessons 01–11.

---

## 1. Where you are

You have a QEMU machine that:
- Boots the **real, unmodified SecureROM** (surgically redirected).
- Runs the **real, unmodified iBSS** (self-copies to SRAM, parks at
  `0x22001360`).
- Performs the **iBSS → iBoot handoff** (the USB-host's job) under a **real
  ARMv5 MMU**.
- Runs the **real iBoot** to its interactive console and prints the genuine
  37-command `help` list over the UART.
- Halts cleanly, deterministically, by design.

That is a genuine, verifiable milestone: **real Apple firmware, executing on
emulated silicon, doing real things.** You did not fake the firmware; you
emulated the hardware and let the firmware run.

The one thing that is **not** done: **full kernel boot** (`bootx` →
kernelcache → iOS userland). The README says so plainly. That's the frontier.

## 2. What `bootx` actually requires (why it's the hard part)

`bootx <addr>` asks iBoot to boot a **kernelcache** (a Mach-O bundle: the XNU
kernel + kernel extensions) at `<addr>`. The handler (link addr `0x18006011`)
would, on real hardware:

1. **Load the kernelcache** into memory. (The project already *preloads*
   `kernelcache.release.s5l8900xrb` at `0x60000000`, and `pt[0x600]` maps it —
   so the image is already in place.)
2. **Verify it** — RSA-signature check against a trusted key, plus a
   hash check. (We'd need to either implement the verification or bypass it.)
3. **Load the DeviceTree (DT)** — a hardware description (memory size,
   peripheral addresses, board info) the kernel reads at boot. The real DT
   comes from the board config; we'd need a valid one for `n45ap`.
4. **Set up the kernel's boot arguments** (the "boot-args" string, the DT
   pointer, the mach header, etc.) in the right registers/memory.
5. **Jump to the kernel's entry** — and from here, **the XNU kernel takes
   over.**

Step 5 is the cliff. The kernel is a *real operating system kernel*. It will:
- Build **its own MMU page tables** (we can't keep using ours).
- Initialize **its own** clock, timer, and interrupt controllers.
- Probe **real hardware** it expects: the NAND controllers (for the root
  filesystem), the display, USB, the PMU, a dozen peripherals.
- Mount the **root filesystem** (the `022-3601-4.dmg` rootfs, decrypted with
  the key in `3A101a_root_filesystem_key.txt`).

So `bootx` isn't one more patch — it's "now emulate enough of the whole SoC
that a 10-year-old OS kernel initializes and mounts a filesystem." That's why
it's the next *milestone*, not the next *step*.

## 3. Three honest paths forward

Pick based on what you want to learn:

### Path A — Push `bootx` (the deep end)
Get the kernelcache to actually boot iOS userland.
- **Pros:** the ultimate prize — a real OS boots on your emulator.
- **Cons:** you'll be modeling a large fraction of the S5L8900 (NAND, timers,
  DT, PMU, …) and fighting XNU's hardware assumptions. Months of work.
- **First sub-steps:** (1) get `bootx` to pass verification (or bypass it and
  jump to the kernel entry); (2) supply a valid DeviceTree; (3) model the
  minimum peripherals the kernel probes at init; (4) watch it with the
  fault-capture handler and stub as you go.

### Path B — Emulate real USB-DFU (the most authentic)
Instead of QEMU "playing the host," implement a **USB OTG controller** so
iBSS/iBEC load the next stages **over real DFU** — no handoff hack, no
skipping, the firmware does everything.
- **Pros:** the most faithful reproduction; iBSS and iBEC run their *real*
  loader/verify/decrypt paths; removes the "QEMU did the host's job" caveat.
- **Cons:** a USB device model is substantial (the DWC2/OTG controller, the
  DFU protocol state machine, the 64-byte packet layer).
- **Why it's appealing:** it's the difference between "we faked the handshake"
  (the current caveat) and "the real DFU stack ran." It also generalizes — a
  USB-DFU model is useful for *any* ARM DFU device.

### Path C — Stop here, solidify, and move on
The console is a real, complete, verifiable deliverable. You can:
- Clean up `s5l8900.c` (it's 6000+ lines of scar tissue; a clean 800-line
  version is a great refactor exercise and would teach you the codebase cold).
- Write the missing documentation.
- Take the skills to a new device (section 5).
- **Cons:** none — it's legitimate to declare a milestone done.

## 4. What you actually proved

Strip away the iPod, and here's the transferable claim you can now make:
- You can take **opaque, encrypted, signed firmware** and get it running on
  **emulated hardware you built yourself**.
- You can **reverse-engineer** a ~140 KB bootloader to the function level
  (putchar, getchar, console, command table) using only the binary.
- You can build a **machine model** (memory map, devices, MMU) from first
  principles and a datasheet-sized memory map.
- You have a **repeatable debug methodology** (the loop, the probes, fault
  capture) that works on *any* stuck/faulting firmware.
- You understand the **ARM boot model** (reset → vectors → stubs → MMU →
  kernel) at a level that transfers to any ARM SoC.

## 5. Where these skills go

- **Any ARM SoC's boot chain.** SecureROM → bootloader → kernel is universal.
  iPhone 2G/3G, other Apple devices, countless embedded boards: same shape,
  same methods.
- **QEMU as a research platform.** Emulating real firmware (not just Linux)
  is a legitimate research technique for security, forensics, and preservation
  of dead hardware. The iPod Touch 1G is *dead hardware* — no more updates,
  no more SHSH, and (per `CLAUDE.md`) **no downgrade protection** — making it
  an ideal preservation/RE target.
- **Apple boot-chain research.** The 1G's lack of SHSH means it's downgrade-
  able and a clean target for boot-chain and jailbreak research. Understanding
  iBoot/iBSS/iBEC deeply is directly relevant.
- **General embedded RE.** capstone + GDB + stubbing + MMU debugging is the
  core toolkit for any closed firmware.

## 6. "You've got it if you can…" (final self-test)

- Explain the full boot chain from power-on to the `] ` prompt, naming every
  image and every QEMU intervention.
- Rebuild `hw/arm/s5l8900.c` from an empty file to a working console, without
  looking at the existing 6000-line version.
- Given a *new* ARM device (memory map + one firmware blob), outline the exact
  plan to get it to its first real output on QEMU.
- Diagnose a faulting or hung firmware run and produce the lesson-08 fault
  report from scratch.
- For every byte on the final screen, state who wrote it and why.

If you can do all five, you've genuinely reproduced the project — and the
skills are yours, not the repo's.

## 7. Suggested first move (if you continue)

The highest-value, most-educational next step is usually **Path B (real
USB-DFU)**, because it removes the project's biggest caveat and the USB model
is reusable. If you want the OS, **Path A** is the goal but expect a long
haul. Either way, start by writing a clean, commented ~800-line `s5l8900.c`
that reproduces the console (Path C), *then* branch into A or B. A clean base
makes the new work tractable.

---

Next: [Appendix](appendix.md) — address tables, cheat sheets, encodings,
glossary.
