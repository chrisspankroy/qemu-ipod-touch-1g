# Lesson 03: ARM From Zero

**Goal:** Read real ARM disassembly without fear. You don't need to *be* an
ARM architect; you need a working model of registers, modes, the pipeline,
MMIO, the MMU, and exceptions — exactly as deep as this project needs.

**Prerequisites:** Lesson 02 (toolbox ready).

**Reference:** `CONCEPTS.md` sections 1–5 are the deep version of this
lesson. Learn this lesson; consult that doc when a concept needs more.

---

## 1. The registers

16 general-purpose registers, `r0`–`r15`. Three have fixed meaning:

| Register | Alias | Role |
|---|---|---|
| r13 | `sp` | Stack pointer |
| r14 | `lr` | Link register = return address |
| r15 | `pc` | Program counter |

The critical difference from x86: **the PC is a normal register.**
`mov pc, lr` is a function return. `ldr pc, [r0]` is a jump through a
pointer. `bl foo` saves the return address into `lr` and jumps. You will see
all of these constantly.

**AAPCS** (the calling convention) in 30 seconds:
- Arguments in `r0..r3` (more on the stack). Return value in `r0`.
- `r4–r11` and `sp` are **callee-saved**: a function must `push`/`pop` them
  if it uses them. `r0–r3`, `r12` are scratch.
- So a function that pushes `r4, r5, lr` at entry will pop them (with `pc`
  in place of `lr`) at exit. **Seeing `push {r4-r7, lr}` is your signal that
  you're at a function start.**

## 2. The CPSR — the mode register

The CPSR packs flags, interrupt masks, and the **current processor mode**:

```
[31:28] N Z C V   condition flags
 [7]    I         IRQ masked
 [6]    F         FIQ masked
 [5]    T         1 = Thumb mode, 0 = ARM mode
 [4:0]  mode      0x10 USR, 0x12 IRQ, 0x13 SVC, 0x17 ABT, ...
```

Things you will actually do with this:

- **Read it in GDB:** `p/x $cpsr`. Bottom 5 bits = mode. Bit 5 = ARM/Thumb.
- `0x13` = SVC (supervisor) mode — what bootloaders run in.
- `0x17` = **Abort** mode. If you ever see CPSR mode 0x17, the CPU took a
  data or prefetch abort (a bad memory access). This is the #1 crash signature
  in this project.
- `0x800001d7`? That's `E=1` (big-endian bit set, bit 25 of the *saved*
  context in one of our runs — a real bug we chased), mode `0x17`, T=1.
  Decoding a CPSR by hand is a skill; practice it on the examples in
  `work/progress.md`.

## 3. ARM vs Thumb — the T bit

Two instruction encodings:

- **ARM**: every instruction is 32 bits (4 bytes). PC always 4-byte aligned.
- **Thumb**: 16-bit instructions (32-bit "Thumb-2" variants exist). Denser.

The T bit in CPSR selects which one the CPU decodes. `bx r0` branches **and
flips T** based on bit 0 of the target: even address → ARM, odd address →
Thumb.

Why this matters here, concretely:

- The **SecureROM is 100% ARM mode.** Every instruction is 4 bytes.
- **iBoot is mostly Thumb.** When you disassemble iBoot you must use Thumb
  mode or you get garbage. And when *we patch iBoot*, the odd LSB of the
  target address is part of the meaning. A patch of `b 0x180058A0` must
  actually be `b 0x180058A1` if the target is Thumb.
- In GDB, `x/10i $pc` decodes using the current T bit — if the disassembly
  looks like nonsense, check `p/x $cpsr` bit 5.

## 4. The pipeline, and why returns are `lr - 4`

ARM1176 has a 3-stage pipeline. Consequences you must internalize:

- While the instruction at address X *executes*, the `pc` register already
  holds `X + 8`.
- `bl foo` sets `lr = (address of the bl) + 4` — i.e., the instruction *after*
  the bl. That's why function prologues can `ldr r0, [pc, #N]` safely.
- When the CPU takes an exception, it stores an **adjusted** return address in
  the mode's `lr`:
  - IRQ/FIQ: `lr = PC + 4` → return with `subs pc, lr, #4`
  - Data abort: `lr = PC + 12` (prefetch abort: `PC + 4`) → return with
    `subs pc, lr, #8` / `#4`
  - SWI/Undefined: `lr = PC + 8` → return with `movs pc, lr`

The `subs pc, lr, #4` spelling is magic: writing the result to `pc` **and**
having the `s` suffix makes the CPU restore CPSR from the SPSR in the same
instruction. That single instruction is ARM's `iret`.

**Practice:** in the ROM (your disassembler), find `irq_handler` (it's the
function the IRQ vector points at; CONCEPTS.md puts it near `0x20000428`).
Find the `subs pc, lr, #4` that ends it. Confirm your mental model.

## 5. `ldr r0, =0x38c00000` — literal pools

The single most important "reading disassembly" pattern for this project.

ARM can only encode small immediates. To load a 32-bit constant like a
peripheral base address, the compiler emits:

```
ldr r0, [pc, #0x14]     ; "load word from 0x14 bytes ahead of pc"
...
; --- literal pool (data, looks like code boundaries) ---
.dc.d 0x38c00000        ; the actual constant, inlined in the code stream
```

So when you see `ldr r0, [pc, #N]` with a **large** N, or data words that
look like addresses (values in `0x2xxxxxxx`/`0x3xxxxxxx`/`0x1xxxxxxx`
ranges) sitting where code should be — **that's a literal pool.** Those words
are data, not instructions.

This matters enormously in lesson 10: iBoot's broken command-dispatch table
base sits in a literal pool at `0x18006000`. Misreading a pool as code (or
code as a pool) is the classic failure.

## 6. Memory-mapped I/O — peripherals are just addresses

There is no "device driver syscall" on bare metal. Every peripheral register
is a 32-bit word at a fixed physical address, accessed with ordinary
`ldr`/`str`:

```
0x38e00000 + 0x10  =  VIC0 VICINTENABLE  (write 1 to enable an IRQ channel)
0x3c500000 + 0x40  =  CLOCK1 PLL status  (bit n = PLL n locked)
0x3e300000         =  WDT control        (write 0x400000 to kick it)
```

The CPU's memory controller decodes the top bits of the address and routes
the bus transaction to the peripheral instead of RAM. From the CPU's view,
`str r0, [r1]` with `r1 = 0x3e300000` is just a store.

**The pattern to recognize in disassembly:**

```
ldr r1, =0x3c500040    ; load peripheral address
1: ldr r0, [r1]        ; read register
   tst r0, #8          ; test a status bit
   beq 1b              ; ...spin until set
```

That's a **status poll / busy-wait.** Firmware full of these. It also means:
if your emulator returns the wrong value from that address, the firmware
**spins forever** — a very common symptom, with a very specific diagnosis
(lesson 08). iBSS hanging on the PLL lock poll is exactly this (lesson 09).

## 7. CP15 — the control-plane registers

CP15 is where the MMU and caches live. Accessed with `mcr`/`mrc p15, ...`.
You need to *recognize* these, and know what the big ones do:

| Register | Encoding (cRn) | What it does |
|---|---|---|
| **SCTLR** | c1,c0 | System control. Bit 0 = **MMU on/off**, bit 2 = D-cache, bit 12 = I-cache, bit 13 = high vectors |
| **TTBR0** | c2,c0 | Base address of the page table (L1 table) |
| **DACR** | c3,c0 | Domain access control (16 domains, permissions per domain) |
| DFAR/IFAR | c5/c6 | Fault address registers (VA that faulted) |
| DFSR | c5,c0,#3 | Fault *status* (why it faulted) |

In this project, QEMU enables the MMU for us with `SCTLR=0x1007`
(M+A+C on, low vectors) and a page table we build (lesson 09). When GDB shows
you `mcr p15, 0, r0, c1, c0, 0`, you should now know: *something is turning
the MMU on or off.*

## 8. Exceptions and the vector table

When an exception happens (reset, bad instruction, bad memory access, IRQ),
the CPU: saves CPSR to the SPSR of the new mode, saves an adjusted return
address to that mode's `lr`, switches mode, and jumps to a **fixed address**:

```
0x00000000  reset
0x00000004  undefined instruction
0x00000008  SWI/SVC
0x0000000c  prefetch abort
0x00000010  data abort
0x00000018  IRQ
0x0000001c  FIQ
```

(Or at `0xffff0000` if SCTLR bit 13 "high vectors" is set — it isn't here.)

So **whatever is mapped at `0x0` decides what happens when anything goes
wrong.** On real S5L8900, `0x0` is… not RAM. The ROM solves this by copying
its vector table into SRAM at `0x0` during init (the SRAM is remapped there
by the memory controller in ROM-only mode).

This is why our QEMU machine model **must** put a real 4 KB RAM region at
`0x00000000` (the "evec" region in `s5l8900.c`), and why a missing `0x0`
region produces instant data-abort spirals. Lesson 07 builds it.

Our project also *writes its own exception handler* into SRAM at
`0x2200F800` — 8 hand-encoded ARM instructions that capture CPSR/DFAR/IFAR/LR
and park the CPU — so that when iBoot does fault, we can read out exactly
what faulted and where (lesson 08, with the full byte sequence).

## 9. Interrupts, at recognition level

The CPU has one IRQ pin. Dozens of peripherals. A **VIC** (ARM PL190
Vectored Interrupt Controller) sits between them: each peripheral gets a
channel; the ROM programs `VICVECTADDR[n]` with a handler address; when the
channel fires, the generic IRQ handler reads `VICADDRESS` (offset `0xf00`)
and the VIC hands back the handler address to jump to. S5L8900 has VIC0
(channels 0–31) at `0x38e00000` and VIC1 (32–63) at `0x38e01000`; an
**EDGEIC** at `0x38e02000` converts edge-triggered lines (like USB's) into
levels the VIC understands.

You don't need to design this. You need to: recognize the register bases
when you see them, know that "USB interrupt" is channel 39 (VIC1 channel 7),
and understand that QEMU's job is to make those registers *behave* (lesson 07).
CONCEPTS.md section 5 walks the full 17-step USB IRQ chain.

## 10. Reading a real function, end to end

Do this now, in your disassembler, on the SecureROM. This is your rite of
passage; lesson 05 does the same walkthrough in full depth.

1. Go to `0x20000000`. See `b 0x200000c4`.
2. Go to `0x200000c4` (`BEGIN_HARDWARE_INIT`). Walk down, and for each block
   answer: what address is being written, and what does that address mean
   (look it up in `CLAUDE.md`)?
3. You should be able to identify, in order: a WDT kick (`0x3e300000`), stack
   pointer setup for each mode (look for `msr cpsr_c, #...` + `mov sp, ...`
   pairs), CP15 init (`mcr p15`), cache enable, and VIC vector programming
   (writes to `0x38e00xxx`).
4. Find where it calls the C startup (`0x2000034c`) and, eventually, the
   main loop (`0x20002bc0`).

You don't need to understand every instruction. You need to be able to say:
*"this block initializes the watchdog; this block sets IRQ stack to
0x2200xxxx; this block programs VIC channel 7 to jump to 0x20001eac."*

## Checkpoint

Without looking at CONCEPTS.md:

1. A function starts with `push {r4, r5, r6, lr}`. Which of r4–r6, lr is the
   return address, and why must it be saved?
2. `p/x $cpsr` returns `0x400000d3`. What mode? ARM or Thumb? IRQs masked?
   (Answer: 0xd3 = 0b11010011 → mode 0x13 SVC, T=1 Thumb, I=1 IRQ masked,
   N=1.)
3. Firmware spins at a `ldr/tst/beq` loop on address `0x3c500040`. In one
   sentence, what is it waiting for, and what would make it stop waiting?
4. An exception fires. List the three things the CPU hardware does
   automatically before the handler runs.
5. You see `ldr r3, [pc, #0x128]` in iBoot and, 0x128 bytes later, a data
   word `0xd2842201`. Is that data word an instruction? What is this
   construct called?

---

## Pitfalls

- **Decoding Thumb as ARM.** Garbage output with `udf`/weird ops → check the
  T bit, switch disassembler mode.
- **Treating a literal pool as code.** If disassembly "runs off the rails"
  into a wall of `ldr`s and odd addresses, you've hit a pool. Step past it.
- **Assuming `pc` is the current instruction.** Remember `pc = current + 8`
  in ARM mode. It bites anyone computing `ldr r0, [pc, #N]` targets by hand.
- **Forgetting banked registers.** `lr` and `sp` in IRQ mode are *different
  physical registers* than in SVC mode. A handler that doesn't save `r0–r3,
  r12` is fine; one that assumes it shares `sp` with SVC is broken.

Next: [Lesson 04: Get The Firmware](04-firmware.md)
