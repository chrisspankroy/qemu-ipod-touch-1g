# Lesson 10: Crack iBoot

**Goal:** Get iBoot to run *far enough* to reach its interactive console.
iBoot is a ~140 KB bootloader that, on real hardware, initializes NAND, the
display, USB, the keychain, and dozens of subsystems. Most of that cannot
run on our stub hardware. The skill here: **reverse-engineer the one path
you care about (the console) and surgically stub the rest**, applying the
smallest possible set of patches — each documented with a *why*.

**Prerequisites:** Lessons 04–05 (ARM + RE), 08 (debug loop), 09 (iBoot is
running under the MMU).

---

## 1. The situation

After lesson 09, iBoot's reset handler and `main_init` start executing.
`main_init` (Thumb, link addr `0x180058A0`) calls a long chain of init
functions. The problem: those inits touch hardware we haven't modeled
(NAND controllers, display, keychain, timer banks) and many will either
fault (unmapped MMIO) or poll forever (waiting on a device that never comes).
We don't need any of that. We want the **command console**.

So the plan is *inversion*: instead of "make every init succeed," we
**bypass the init chain's tail and jump straight to the console**, and we
**stub the two I/O primitives the console depends on** (putchar, getchar).

## 2. Reverse-engineer the console path

Use your lesson-05 toolkit: capstone + `work/xref.py` + the string table.
Here is the reconnaissance, and *how you'd find each piece yourself*.

### 2.1. Find `putchar` (the char-output routine)

The console eventually calls a per-character output function. How to find it:
1. Grep the iBoot payload for the **UART base literal `0xE0002000`**.
2. The code that writes a byte to that address is (or calls) `putchar`.
3. Walk the cross-references back to find the function. In this build it
   resolves to `uart_putchar` at link addr **`0x1800465C`**.

> The general technique: *a device's base address is a beacon.* Find the
> literal, find the code that uses it, and you've found the driver.

### 2.2. Find `getchar` (the char-input routine)

Mirror of putchar: the routine that *reads* a character (from the serial/USB
input path). Resolves to **`0x180045C0`**, reached through a small wrapper.
It has internal state (a pointer to a state struct, `0x180210f0`), because
on real hardware it's polling a USB/serial input queue.

### 2.3. Find the console function

The interactive shell that prints the `iBoot` banner and the `] ` prompt,
reads a line, and dispatches commands. Found by:
1. Find the **`iBoot`** and **`] `** strings in the payload (lesson 05's
   string-scan).
2. Find the code that prints them → that's the console function:
   **`0x180058A0`**.

### 2.4. Find the event loop and `main_init`

- `main_init` (`0x180058A0`, Thumb) is the C entry the reset handler calls.
- The **main event loop** at **`0x18005550`** is the top-level dispatch
  (`bl pump; bl cacheflush; ...`). This is where iBoot would sit after init,
  polling for events.

> **Beware the `0x400` header.** iBoot's link base is `0x18000000`, but the
> on-disk `iBoot.decrypted` has a `0x400`-byte img2 subheader *before* the
> payload. So a function at link `0x180058A0` sits at **file** offset
> `0x5CA0`. Older notes cite `main_init`/`console` as `0x18005CA0` — that is
> the *file* offset mistakenly written as a link address. The real link
> address is `0x180058A0` (a clean `push {r4,r5,r6,r7,lr}` prologue);
> `0x18005CA0` is mid-function. Same `0x400` gap as `putchar`
> (file `0x4A5C` ⇄ link `0x1800465C`). Appendix A.3.

### 2.5. Find the command table and the `help` string

The console dispatches commands via a **table of 37 commands** (link addr
`0x1801F28C`), each entry pointing to a name string (e.g. `help` at
`0x1AB2C`, `bootx` at `0x1B16C`) and a handler. The **`help` command list**
("Available commands:\n  help - this list\n  ...") is a single string in the
data section — this is the `help_cmdlist_str` you'll see printed.

> Do this recon yourself. For each of the five items above, find it in
> `work/iBoot.decrypted` using capstone + `xref.py` + a string scan. The
> addresses in this lesson are the answers; your job is to reproduce the
> *method* and land on the same addresses.

## 3. The three patches (the surgical core)

Everything is applied to the iBoot runtime image (at `0x18000000`) **after**
the staging→runtime copy and **before** the CPU is parked at the entry.
Each patch is one small block of hand-encoded instructions + a comment.

### Patch 1 — `putchar` → direct UART write

Replace iBoot's `putchar` body with: write the character register to the
UART data register `0xE0002000`, return. That's it — a one or two
instruction stub. iBoot's real `putchar` goes through a full serial driver
with FIFO/line-status polling; our UART is a simple byte sink, so a direct
`strb` is all it needs.

```
; at putchar (0x1800465C):
  STRB r0, =0xE0002000   ; write char to UART data reg
  BX   LR
```
(Encoded as the correct Thumb/ARM sequence for the calling convention.)

### Patch 2 — `getchar` → stateful preloaded-input stub

Replace `getchar` with a stateful stub that:
1. Reads the next byte from a **preloaded input buffer** in SRAM
   (`0x22011100`), using an **index** (`0x22011180`).
2. Returns it to the caller (so the console "types" the command for itself).
3. **When the buffer is exhausted, halts the CPU** (a tight `b .` loop).

This is how the run *stops cleanly by design*: after the last character of
the preloaded input is consumed, getchar loops forever, the console re-prompts,
and nothing else happens. No real input device is needed.

```
; at getchar (0x180045C0):
  LDR  r1, =0x22011100      ; input buffer base
  LDR  r2, =0x22011180      ; index address
  LDR  r3, [r2]             ; current index
  LDRB r0, [r1, r3]         ; next char
  ADD  r3, r3, #1
  STR  r3, [r2]             ; advance index
  LDR  r4, =0x22011104      ; end of "help\n" (5 chars)
  CMP  r3, r4
  BNE  .return
  B    .                    ; exhausted -> halt
.return:
  BX   LR
```

And in `s5l8900_init` / the handoff, **preload** the buffer:
`cpu_physical_memory_write(0x22011100, "help\n", 5)` and set the index to 0.
(You can preload `"help\nbootx 0x60000000\n"` to also try booting — that's
lesson 12.)

### Patch 3 — event loop → jump straight to the console

Replace the start of the main event loop (`0x18005550`) with a branch to the
console function (`0x180058A0`). This **skips the rest of the init/event
pump** (which needs hardware we don't have) and drops the CPU directly into
the interactive shell.

```
; at event loop (0x18005550):
  B 0x180058A0
```

That's the whole inversion: init runs, and instead of falling into the
hardware-hungry event loop, we jump to the console.

## 4. Stub the rest of the init chain

Even with the event-loop redirect, `main_init` still calls a series of init
functions *before* reaching the loop. Some of those touch unmapped hardware.
The project stubs the offenders:

- **Allocator / init functions** that read an unmapped "heap size" global:
  seed the global to a sane value (the code patches literals at
  `0x180210E8`, `0x180210E4`, `0x18022FA0`, etc.) so the alloc path returns a
  valid pointer instead of 0.
- **Functions that poll a device forever**: redirect them to a safe return
  (`BX LR`) or a short delay.

You will find these *empirically*: run, watch the fault-capture handler /
stuck-PC detector (lesson 08), see which init faults or hangs, and stub
*that one*. Don't stub the whole chain up front — you can't know which
functions are safe. **Stub only what demonstrably breaks.**

## 5. The philosophy (internalize this)

- **Patch as little as possible.** Every patch is a place that can break
  when something else changes. The final image has a *small, documented* set.
- **Prefer emulating hardware over patching code.** If an init hangs because
  a register reads 0, adding a correct stub value (lesson 07's rule) is
  better than patching the firmware to skip the check. Code patches are
  heavier and more brittle.
- **Comment every patch with a *why*.** The real `s5l8900.c` has a comment
  block above each patch explaining what it is, why it's needed, and the
  exact bytes. That's what makes a 6000-line scarred file readable at all.
- **Each patch is minimal and local.** A few bytes at a known offset. No
  relocations, no re-linking. You're *hot-patching a running image in memory*.

## 6. The dispatch-table mystery (setup for lesson 11)

Here's the catch. The console reads `help` (fed by the getchar stub), echoes
it, and *should* call iBoot's `help` handler, which prints the command list.
But it doesn't — **iBoot's command-dispatch table isn't reliably populated**
in our environment, because we skipped some init that fills it. So iBoot's
own `help` handler either faults or finds an empty table.

The project's solution (lesson 11): **a QEMU-side UART hook** watches the
output stream, and when it sees the console echo `help`, it *itself* writes
the genuine `help_cmdlist_str` (copied verbatim from iBoot's data section) to
the serial port. The *text* is real iBoot; the *delivery* is QEMU. This
reproduces the exact real output and ordering without depending on iBoot's
broken dispatch.

Keep this in your head: when firmware's own machinery is unreliable on
emulated hardware, you can reproduce its *observable behavior* from the
outside. That's a legitimate (and common) technique — with the caveat that it's
a **reproduction**, not the firmware doing it.

## 7. Checkpoint

You can now answer:
1. How do you find a driver function from a device's base address?
2. What are the three core patches, and what does each do?
3. Why does the getchar stub halt the CPU when the input is exhausted?
4. Why stub only *some* init functions, and how do you know which ones?
5. What's the difference between "emulating hardware" and "patching code," and
   why is the former preferred?
6. Why does `help` need a QEMU-side hook rather than iBoot's own handler?

## 8. Exercises

- Re-derive all five reconnaissance addresses (putchar, getchar, console,
  event loop, command table) from `work/iBoot.decrypted` with capstone +
  `xref.py` alone. Write down your method for each.
- Preload the input buffer with `"echo hi\n"` and observe the console echo it
  (even though `echo`'s dispatch isn't wired — you'll see the echo, then the
  prompt). This confirms the getchar/putchar/echo path works independently of
  dispatch.
- Add a fourth patch: redirect a specific faulting init to `BX LR`. Document
  it with a *why* comment, exactly like the project's style.

Next: [Lesson 11: Ship the Console](11-ship-the-console.md)
