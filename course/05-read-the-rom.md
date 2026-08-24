# Lesson 05: Read The SecureROM

**Goal:** Your first complete reverse engineering engagement. Take the 64 KB
SecureROM binary — no symbols, no source, no docs — and reconstruct its full
execution path: reset → hardware init → C runtime → USB init → DFU main loop.
When you finish, you'll be able to explain, in your own words, what the ROM
is doing and *where it is waiting* — which is exactly the knowledge the QEMU
side (lessons 07–09) needs.

**Prerequisites:** Lessons 02–04. A disassembler you're comfortable in
(Hopper/Ghidra/r2). `CLAUDE.md`'s memory map open.

**Reference answer key:** `CONCEPTS.md` section 13 is the completed
walkthrough with the project's final annotations. Use it to check your work,
not to skip it.

---

## 1. The six moves of binary RE

Everything in this project is combinations of six moves. Learn them here on
the ROM:

1. **Find the entry.** For a bare binary, the entry is the base address. Read
   the first instruction; it's usually a branch to the real start.
2. **Find function boundaries.** Look for prologues
   (`push {r4..., lr}`) and epilogs (`pop {r4..., pc}`). Follow `bl`
   (branch-and-link) targets — a `bl` target is almost always a function
   start. Use your disassembler's xref feature religiously.
3. **Find hardware accesses.** Any `ldr rX, =0x38xxxxxx` (or a literal pool
   word in the `0x38...`/`0x3c...`/`0x3e...` range) is a peripheral
   register. Match the address against `CLAUDE.md`'s memory map and you
   immediately know *which device* the code is talking to.
4. **Follow the data.** Global state lives at fixed addresses. When a
   function repeatedly loads a pointer and offsets into it, you've found a
   struct — draw the struct.
5. **Reconstruct pseudo-C.** Don't translate instruction-by-instruction.
   Identify loops, branches, and calls; write the C you think produced it.
   The assembly is the *evidence*, the pseudo-C is the *claim*.
6. **Name everything you understand.** A named function is a solved subproblem
   you never have to redo. This is not busywork; it is the compounding
   interest of RE.

## 2. Open the ROM

- Hopper: `work/ROM BOOT, S5L8900 Rev.hop` is an existing annotated project —
  but **do the walkthrough below yourself first**, then open the .hop to see
  what a fully annotated ROM looks like (and to compare against your own
  labels).
- Ghidra/r2: import the raw file `work/ROM BOOT, S5L8900 Rev.2`, ARM LE, base
  `0x20000000`.

Set a label at `0x20000000`. That's your entry.

## 3. Move 1: the entry

```
0x20000000:  b 0x200000c4
```

One instruction, the whole story: power on at `0x20000000`, immediately
branch to `0x200000c4`. Label it (the project calls it
`BEGIN_HARDWARE_INIT` — pick your own name, note the correspondence).

*Why branch at all?* Because the code that matters isn't at the very top;
the top is a vector/padding area. (The ROM also keeps its exception vector
table near the top — section 5.)

## 4. Move 2+3: walk BEGIN_HARDWARE_INIT (0x200000c4)

This is the pre-C-runtime initialization: no stack, no heap, no C — pure
assembly doing hardware bring-up. Walk it top to bottom. You don't need every
instruction; you need the **blocks**. For each block: what address does it
touch, and what is that address (per `CLAUDE.md`)?

You should be able to identify, in this order:

1. **Watchdog disarm.** The S5L8900 resets itself if the watchdog isn't
   kicked. The very first thing any init does is write the magic kick value
   to `0x3e300000` (WDT_CTRL). Find the `ldr rX, =0x3e300000` (or a pool word
   of that value) and the store.
2. **Stack pointers for every mode.** Look for `msr cpsr_c, #0x12` / `#0x13` /
   `#0x11` ... (mode switches) each followed by setting `sp` to an SRAM
   address (`0x2200xxxx`). This is lesson 03's banked-register story made
   real: each mode gets its own stack before any C code runs.
3. **CP15 init.** `mcr p15, ...` sequences: zero FCSE PID, set DACR, flush
   TLB/caches. Recognize the encodings from lesson 03's table.
4. **Enable caches.** I-cache and D-cache enable (more `mcr p15` + barrier
   instructions).
5. **VIC programming.** Stores to `0x38e00xxx` (VIC0) and `0x38e01xxx`
   (VIC1): writing handler addresses into the `VICVECTADDR[n]` slots
   (offset `0x100+`), then writing the enable mask (offset `0x10`). The
   handler addresses written there are *ROM* addresses — xref them and you'll
   land on real functions (e.g., the USB ISR).
6. **Copy the vector table to 0x0.** A block that copies words from the ROM
   (top region) into `0x00000000`–`0x00000020`. Lesson 03: exceptions
   dispatch from `0x0`, so the ROM must install its vectors there. **This is
   why QEMU needs RAM at `0x0`** (lesson 07) — now you've seen the ROM-side
   requirement for it.
7. **Jump into C.** A `bl` to the C startup (the project labels it
   `startup_data_bss_init` at `0x2000034c`). It never returns.

**Checkpoint A:** You can list those seven blocks in order, each with the
address(es) it touches and your one-line explanation. Write it down.

## 5. The C runtime: startup_data_bss_init (0x2000034c)

Starts with `mov r6, #0x20000000` — the ROM's own base. This function:

1. Copies initialized globals (`.data`) from ROM into SRAM (there are
   linker-provided boundary addresses in the literal pools; the copy is a
   `ldr/str` loop between two pointer pairs).
2. Zeros `.bss` (a loop storing 0 between two pointers).
3. Sets a flag, then calls the real `main`-equivalent (the project's
   `main_entry`, `0x20003790`).

Reconstruct the pseudo-C (lesson 03's §11 in CONCEPTS.md shows the canonical
form). Notice: **this is the same thing every bootloader does** — it's not
Apple magic, it's the price of having no OS loader.

## 6. main_entry (0x20003790) — three calls deep

`main_entry` is short and honest. It should resolve to roughly:

```c
usb_phy_init();      // power up the analog USB PHY (0x3c400000)
usb_init();          // configure DWC2 (0x38c00000): soft reset, speed,
                     //   endpoint 0, connect D+ pullup
usb_dfu_main_loop(); // never returns
```

Verify each by following the `bl`s and checking the peripheral addresses
each callee touches. Name the callees. This is your first win: you've
reconstructed a real function's behavior from nothing.

## 7. The DFU main loop (0x20002bc0) — where it all waits

This is the most important function for the whole project, because **QEMU
has to know exactly what this loop is waiting for.** The big prologue
(`push {r4–fp, lr}`) tells you it's a substantial function. Work through it
until you can write the skeleton:

```c
while (1) {
    if (new_setup_packet_flag) {        // set by the USB ISR
        dfu_handle_request();           // parse the 8-byte SETUP packet
    }
    if (all_blocks_received) {
        img2_load_and_verify();         // signature check
        jump_to_next_stage();
    }
}
```

Key sub-functions to find (xref from the loop; the project's labels):

| Function | Address | Role |
|---|---|---|
| `dfu_handle_request` | `0x200007a8` | SETUP packet parser — branches on `bRequest` (1=DNLOAD, 3=GETSTATUS, 5=GETSTATE, 6=ABORT) |
| `usb_out_transfer_setup` | `0x200023d4` | Arms the OUT endpoint + DMA for a firmware block |
| `dfu_transfer_wait_loop` | `0x200027cc` | Spins on a "transfer complete" flag |
| `usb_dma_complete` | `0x200024b4` | Spins on the DMA busy bit (`0x38000000`), then sets the completion flag |
| `usb_isr_entry` | `0x20001eac` | USB interrupt handler (set the flag, ACK the EDGEIC) |

**The two nested spin loops** (`usb_dma_complete` spinning on
`0x38000000 & 1`, `dfu_transfer_wait_loop` spinning on a flag in the USB
state struct) are the heart of the matter: on real hardware the USB
controller + DMA engine make those bits change. In QEMU, *we* have to make
them change. File that thought — it's the seed of lesson 07's device stubs.

**Checkpoint B:** In one paragraph each:
1. What exactly is `dfu_transfer_wait_loop` spinning on, and what event
   (on real hardware) makes it stop?
2. Which function decides what to do with each incoming DFU command, and
   what are the four commands it handles?

## 8. The USB state struct

Throughout the USB code you'll see the same pattern: load a pointer from a
fixed SRAM address, then offset into it. The project puts that pointer at
**`0x2202bff8`** (a word in SRAM holding the address of a big state struct).
Two fields that matter later:

- `+0x98` — "USB event pending" flag (set by ISR, consumed by main loop)
- `+0xa0` — "DMA transfer complete" flag (set by `usb_dma_complete`)

Find at least three accesses into this struct and confirm the pattern. You're
now reading *state machines*, not just instructions — that's the level the
rest of the course operates at.

## 9. Deliverable: your annotated execution path

Write a one-page document (your own, not copied):

```
Power on
  PC = 0x20000000, SVC mode, MMU off
  b 0x200000c4 (BEGIN_HARDWARE_INIT)
    1. WDT kick: 0x400000? -> 0x3e300000     [verify the value]
    2. SP setup: IRQ=0x2200xxxx SVC=0x2200xxxx ...
    3. CP15: ...
    4. caches on
    5. VIC0/VIC1: channel N -> 0x20001eac (usb_isr_entry), enabled
    6. vectors copied to 0x00000000
    bl startup_data_bss_init (0x2000034c)
      .data ROM->SRAM, .bss zeroed
      bl main_entry (0x20003790)
        usb_phy_init()  [0x3c400000]
        usb_init()      [0x38c00000]
        usb_dfu_main_loop()  [0x20002bc0]
          SPINS FOREVER waiting for: <fill this in>
```

Every line must be something you verified in the disassembler. This document
is your entry ticket to lesson 07: the machine model exists to satisfy the
exact addresses this document lists.

## Exercises

1. Find the panic handler (project: `panic_data_abort` at `0x20001158` —
   verify: it's a `b .` loop). What vector points at it, and what does that
   tell you about where the vector table lives?
2. In `dfu_handle_request`, find where the 8-byte SETUP packet's `bRequest`
   byte is compared against 1 (DNLOAD). What happens for a `bRequest` it
   doesn't recognize?
3. Find the USB device descriptor data (it's a static table in the ROM: a
   run of bytes starting with length 0x12, type 0x01, version 0x0200,
   class 0xFE (Miscellaneous), subclass 0x02 — the DFU class). Where is it,
   and which function references it?
4. (Stretch) The ROM verifies the received image's RSA signature before
   jumping. Find the signature-verification code region and identify the
   public-key material it uses (a long run of constant words). Don't crack
   the crypto — just locate the attack surface.

## Pitfalls

- **Decoding a literal pool as code** and "finding functions" that are
  actually data. If you see a wall of `ldr rN, [pc, #...]` and address-like
  words, stop — you're in a pool.
- **Chasing every `bl`.** Some callees are used once and irrelevant. Depth-
  first on the *main path* first (reset → init → main → loop); breadth later.
- **Trusting the first disassembly.** Your disassembler may have merged two
  functions or started one mid-instruction. Xrefs from `bl` targets are your
  ground truth for function starts.
- **Forgetting this ROM is pure ARM.** No Thumb. Every instruction 4 bytes.
  (iBoot in lesson 10 is the opposite.)

Next: [Lesson 06: QEMU Internals](06-qemu-internals.md)
