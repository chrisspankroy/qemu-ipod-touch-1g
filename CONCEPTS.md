# Bare-Metal ARM: A Manual for the iPod Touch 1G ROM

**Audience**: You know Linux internals, C, networking, and security.  
**Goal**: Understand exactly what the S5L8900 SecureROM is doing and why.

---

## Table of Contents

1. [ARM Architecture Fundamentals](#1-arm-architecture-fundamentals)
2. [Memory-Mapped I/O and the Hardware Address Space](#2-memory-mapped-io-and-the-hardware-address-space)
3. [The CP15 Coprocessor — ARM's Control Plane](#3-the-cp15-coprocessor--arms-control-plane)
4. [Exception Handling and the Vector Table](#4-exception-handling-and-the-vector-table)
5. [Interrupts: VIC, EDGEIC, and the Full IRQ Chain](#5-interrupts-vic-edgeic-and-the-full-irq-chain)
6. [DMA — Direct Memory Access](#6-dma--direct-memory-access)
7. [USB From First Principles](#7-usb-from-first-principles)
8. [DFU — Device Firmware Upgrade Protocol](#8-dfu--device-firmware-upgrade-protocol)
9. [The DWC2 Synopsys USB Controller](#9-the-dwc2-synopsys-usb-controller)
10. [The Boot Chain and Apple Secure Boot](#10-the-boot-chain-and-apple-secure-boot)
11. [Bare-Metal C Runtime — How a Program Starts With No OS](#11-bare-metal-c-runtime--how-a-program-starts-with-no-os)
12. [QEMU Internals — How Emulation Works](#12-qemu-internals--how-emulation-works)
13. [Putting It All Together — The ROM's Full Execution Path](#13-putting-it-all-together--the-roms-full-execution-path)

---

## 1. ARM Architecture Fundamentals

### Registers

ARM1176 (the core in the S5L8900) has 16 general-purpose integer registers visible at any time, named r0–r15. Three of them have hardwired meaning:

| Register | Alias | Role |
|---|---|---|
| r13 | SP | Stack pointer |
| r14 | LR | Link register (return address) |
| r15 | PC | Program counter |

Unlike x86, the PC is a general-purpose register you can read and write directly. Writing to r15 is a jump. `mov pc, lr` is a function return. `ldr pc, [r0]` jumps through a pointer. The ROM uses all of these.

**r0–r3** are argument and scratch registers (caller-saved).  
**r4–r11** are callee-saved — a function that uses them must push/pop them.  
**r12** (IP) is an intra-procedure scratch register.

### The CPSR — Current Program Status Register

This is equivalent to x86's EFLAGS but with much more packed into it:

```
Bits [31:28]  N Z C V    — condition flags (Negative, Zero, Carry, oVerflow)
Bit  [7]      I          — IRQ disable (1 = IRQs masked)
Bit  [6]      F          — FIQ disable (1 = FIQs masked)
Bit  [5]      T          — Thumb mode (1 = CPU executing Thumb instructions)
Bits [4:0]    M[4:0]     — processor mode (see table below)
```

The mode bits determine which CPU privilege level you're in and which physical registers are active:

| Mode bits | Mode name | When used |
|---|---|---|
| 0x10 | User (USR) | Normal application code |
| 0x11 | FIQ | Fast interrupt handler |
| 0x12 | IRQ | Normal interrupt handler |
| 0x13 | Supervisor (SVC) | OS kernel / SWI handler |
| 0x17 | Abort (ABT) | Prefetch/data abort handler |
| 0x1b | Undefined (UND) | Undefined instruction handler |
| 0x1f | System (SYS) | Privileged, shares USR registers |

**This is the ARM equivalent of x86 privilege rings**, but the granularity is different. User mode is ring 3. Everything else (IRQ, SVC, ABT, etc.) is ring 0. The SecureROM runs entirely in SVC mode — there is no user mode because there's no OS.

### Banked Registers — The Hidden Register File

ARM has a trick that Linux kernel people sometimes find surprising: each privileged mode has its own private copies of some registers. When you switch from SVC to IRQ mode, the CPU *transparently swaps in* a different r13 and r14. This means:

- The IRQ handler gets a clean stack pointer (r13_irq) without saving the SVC stack pointer
- The IRQ return address lands in r14_irq (LR), not the SVC LR
- FIQ mode additionally banks r8–r12 for maximum handler speed

The ROM's `irq_handler` saves r0–r3 and r12 because those are *not* banked — they're shared between modes. It doesn't save r13 or r14 because those are automatically banked.

### SPSR — Saved Program Status Register

When an exception fires, the CPU saves the current CPSR into the SPSR of the new mode. When you `subs pc, lr, #4` or `movs pc, lr` to return from an exception, the hardware automatically restores CPSR from SPSR. This is the ARM equivalent of `iret` on x86.

Each privileged mode has its own SPSR. You cannot access SPSR in User or System mode.

### ARM vs Thumb Instruction Sets

ARM has two instruction encoding modes:

- **ARM mode**: All instructions are 32 bits wide. The PC is always 4-byte aligned.
- **Thumb mode**: Instructions are 16 bits (or 32 bits in Thumb-2). More compact. Less orthogonal.

The T bit in CPSR selects which set the CPU decodes. You switch with `BX` (branch and exchange) — the low bit of the target address sets T. `BX r0` with `r0 = 0x200000c5` would jump to 0x200000c4 in Thumb mode.

**The SecureROM is 100% ARM mode.** No Thumb. This is typical of early Apple firmware — Thumb was added in later designs for code density.

### The Pipeline and the +8 Mystery

ARM1176 has a 3-stage pipeline: Fetch → Decode → Execute. By the time an instruction *executes*, the PC register has already advanced by 8 bytes (two fetch stages ahead). This means:

- When you execute the instruction at address X, `r15 == X + 8`
- `BL foo` stores `LR = PC_of_BL + 4` (next instruction), NOT the BL instruction itself
- Exception return addresses need adjustment: IRQ returns to `LR - 4`, data abort to `LR - 8`

This is why the ROM's `irq_handler` returns with `subs pc, lr, #4` and the data abort handler returns with `subs pc, lr, #8`. The `subs` (not `sub`) form writes the result to PC and simultaneously restores CPSR from SPSR — that single instruction does what x86 needs `iret` for.

### Calling Convention (AAPCS)

The ARM Procedure Call Standard:

1. Arguments go in r0, r1, r2, r3 (first four). More args go on the stack.
2. Return value goes in r0 (64-bit: r0+r1).
3. Callee must preserve r4–r11 and r13 (SP).
4. r0–r3, r12 are caller-saved (may be trashed by any call).
5. `BL` saves the return address in r14 (LR). `MOV pc, lr` returns.

A typical function prologue:
```
push {r4, r5, r6, lr}   ; save callee-saved regs + LR
...
pop {r4, r5, r6, pc}    ; restore + return (pc = old LR)
```

---

## 2. Memory-Mapped I/O and the Hardware Address Space

### The Fundamental Model

In an operating system, peripherals are accessed through a driver that calls `ioremap()` or similar, translating a physical address into a kernel virtual address. On bare metal there is no such indirection — you just write to the physical address directly, because there's no MMU translation active (or the translation is identity-mapped).

Every peripheral register on the S5L8900 is a 32-bit word at a specific physical address. Reading or writing it is a load/store instruction:

```c
// Linux kernel style (virtual address after ioremap):
writel(value, base + offset);
readl(base + offset);

// Bare metal style (direct physical address):
*(volatile uint32_t *)(0x38c00000 + offset) = value;
value = *(volatile uint32_t *)(0x38c00000 + offset);
```

The `volatile` keyword tells the C compiler: *do not optimize away this memory access*. Without it, a compiler seeing `x = *ptr; x = *ptr;` would eliminate the second read as redundant. For MMIO registers, a second read might return a different value (interrupt status changed), so `volatile` forces both accesses.

### Why Peripheral Accesses Don't Hit RAM

The S5L8900's memory controller decodes the top bits of the address to route the transaction:

```
0x20000000 – 0x2000ffff  →  SecureROM (read-only, on-chip)
0x22000000 – 0x2207ffff  →  Internal SRAM (512KB, read-write)
0x38000000 – 0x3fffffff  →  Peripheral bus (each device at its sub-range)
```

When the CPU executes `ldr r0, [r1]` with r1 = 0x38c00000, the memory controller sees the address is in the peripheral range and routes it to the USB DWC2 controller's register bus, not to RAM. The DWC2 hardware interprets the address offset and returns the register value. **The CPU has no idea it's talking to hardware vs. memory — to the CPU it's just a load instruction.**

This is exactly how `/dev/mem` works in Linux — you mmap physical memory and read/write it the same way you'd read/write a byte array.

### The volatile Hazard in Practice

A common bug in embedded code: forgetting `volatile` on a spin-wait:

```c
// Wrong — compiler may hoist the read out of the loop:
while (*(uint32_t *)0x38000000 & 1) {}

// Correct:
while (*(volatile uint32_t *)0x38000000 & 1) {}
```

This is exactly what the ROM's DMA completion handler does at 0x200024bc — it polls bit 0 of 0x38000000 in a tight loop. The ROM is assembly so the volatile issue doesn't apply, but this is why it matters in C.

### The S5L8900 Peripheral Map (Relevant Subset)

```
0x38000000   USB DMA controller
0x38100000   CLOCK0
0x38c00000   USB DWC2 OTG core (the main USB controller)
0x38e00000   VIC0 (Vectored Interrupt Controller, channels 0–31)
0x38e01000   VIC1 (Vectored Interrupt Controller, channels 32–63)
0x38e02000   EDGEIC (Edge Interrupt Controller)
0x39a00000   GPIOIC (GPIO Interrupt Controller)
0x3c400000   USB PHY (analog USB front-end)
0x3c500000   CLOCK1 (PLL and clock dividers)
0x3d000000   NAND flash controller
0x3e300000   WDT (Watchdog Timer)
0x3e400000   TIMER (hardware timer)
```

---

## 3. The CP15 Coprocessor — ARM's Control Plane

### What a Coprocessor Is

ARM's architecture reserves 16 coprocessor slots (CP0–CP15). Some are used by floating point or SIMD units. CP15 is the "system control coprocessor" — it controls the MMU, caches, TLBs, and other architectural features. Think of it as the ARM equivalent of x86 MSRs (Model Specific Registers), accessed with `RDMSR`/`WRMSR`.

The instruction syntax:
```
MCR p15, <op1>, <Rd>, <CRn>, <CRm>, <op2>   ; Move to Coprocessor (write)
MRC p15, <op1>, <Rd>, <CRn>, <CRm>, <op2>   ; Move from Coprocessor (read)
```

The `c1`, `c2`, etc. are coprocessor register names. Different combinations of CRn/CRm/op1/op2 address different registers.

### Key CP15 Registers

**SCTLR — System Control Register** (`p15, 0, r0, c1, c0, 0`)

This is the master control register. The important bits:

```
Bit 0  (M)  : MMU enable. 0 = physical addresses, 1 = virtual addresses
Bit 1  (A)  : Alignment fault enable. 1 = data abort on unaligned access
Bit 2  (C)  : Data cache enable
Bit 12 (I)  : Instruction cache enable
Bit 13 (V)  : High vectors. 0 = vectors at 0x00000000, 1 = vectors at 0xFFFF0000
```

The ROM starts with everything off. `BEGIN_HARDWARE_INIT` sets M=0 (no MMU), then later enables caches once the stack is set up. It never enables the MMU — it runs entirely in physical address mode. This is typical for first-stage bootloaders where you want absolute predictability with no page-fault risk.

**TTBR0 — Translation Table Base Register 0** (`p15, 0, r0, c2, c0, 0`)

The physical base address of the Level-1 page table. Only meaningful when M=1 (MMU on). The ROM sets this (the function is `set_TTBR0`) but since it never enables the MMU, it's a no-op in practice.

**DACR — Domain Access Control Register** (`p15, 0, r0, c3, c0, 0`)

Controls access permissions for 16 "memory domains." With MMU off, irrelevant. The ROM sets it to 0x00000001 (domain 0 = full access) as boilerplate.

**FCSE PID** (`p15, 0, r0, c13, c0, 0`)

Fast Context Switch Extension — adds a 7-bit process ID to virtual addresses below 32MB, allowing process isolation without full TLB flushes. The ROM zeros this.

**Cache operations** use write-only CP15 registers. Reading them is undefined:

```
MCR p15, 0, r0, c7, c5, 0    ; Invalidate entire I-cache
MCR p15, 0, r0, c7, c6, 0    ; Invalidate entire D-cache
MCR p15, 0, r0, c7, c10, 2   ; Data Synchronisation Barrier (DSB)
MCR p15, 0, r0, c8, c7, 0    ; Invalidate entire TLB
```

These are the equivalents of `clflush` + `mfence` on x86, or `__flush_dcache_area` in Linux.

### Why You Care About This

When you attach GDB to QEMU and see the ROM writing to addresses like 0x38100000 or 0x3c500000, those are peripheral registers. When you see `MCR p15,...` instructions, those are CPU configuration operations that QEMU handles internally (it emulates the ARM1176 CP15). You cannot intercept CP15 accesses with MMIO stubs — QEMU handles them as part of the CPU model.

---

## 4. Exception Handling and the Vector Table

### The ARM Exception Model

ARM exceptions are functionally equivalent to x86 interrupts but with a fixed, simpler dispatch mechanism.

When an exception occurs, the CPU:
1. Saves CPSR → SPSR of the target mode
2. Sets CPSR to the target mode (changes mode bits, disables IRQ, sometimes FIQ)
3. Saves the return address → LR of the target mode (with offset adjustment)
4. Sets PC to the exception vector address

No stack frame is pushed automatically. No IDT lookup. No ring transition descriptor. The vector table is just an array of 8 words starting at address 0x00000000 (or 0xFFFF0000 if SCTLR.V is set).

### The Vector Table Layout

```
Offset  Exception type          Triggered when
------  -----------------       -------------------------------------------
+0x00   Reset                   Power-on or warm reset
+0x04   Undefined Instruction   CPU fetches an unknown opcode
+0x08   SWI (Software Interrupt) SWI/SVC instruction executed
+0x0c   Prefetch Abort          CPU can't fetch instruction (bad address)
+0x10   Data Abort              Load/store to invalid address
+0x14   Reserved                (not used, was IRQ in older ARMs)
+0x18   IRQ                     External interrupt via nIRQ pin
+0x1c   FIQ                     Fast interrupt via nFIQ pin
```

Each entry is typically a branch instruction: `ldr pc, [pc, #offset]` or `b handler`. The ROM uses indirect branches (`ldr pc, =handler`) through a pointer table just past the vector table.

### Why 0x00000000?

The ARM reset vector is at 0x0. But the ROM is at 0x20000000. The ROM's first instruction (at 0x20000000) is a branch to `BEGIN_HARDWARE_INIT`. This is the *reset vector in ROM*.

For other exceptions (undef, IRQ, etc.) to work, the CPU looks for their handlers at 0x4, 0x8, etc. — which is address 0x0 + offset. During `BEGIN_HARDWARE_INIT`, the ROM copies its own vector table from 0x20000000 to SRAM at 0x00000000. After that, exceptions dispatch through the SRAM copy at 0x0, which contains `ldr pc, =real_handler` entries pointing back into the ROM.

This is exactly how Linux sets up its vector table on ARM — `vectors_start` gets copied to `0xffff0000` (high vectors) or `0x00000000` during boot.

### Return From Exception — The Adjusted LR

The return address saved into LR when an exception fires is adjusted by the hardware to account for the pipeline. The offsets are:

| Exception | Return instruction | Why |
|---|---|---|
| SWI | `movs pc, lr` (LR = next instruction) | Resume at instruction after SWI |
| IRQ | `subs pc, lr, #4` (LR = PC+4) | Resume at instruction that was about to execute |
| FIQ | `subs pc, lr, #4` | Same as IRQ |
| Prefetch Abort | `subs pc, lr, #4` | Retry the faulting fetch |
| Data Abort | `subs pc, lr, #8` | Retry the faulting load/store |
| Undefined | `movs pc, lr` (LR = next instruction) | Skip the bad instruction |

The `subs pc, lr, #N` form (note the `s` suffix on a destination of `pc`) is special ARM syntax that simultaneously: computes `pc = lr - N` AND restores CPSR from SPSR. This atomically exits the exception mode and resumes the previous context. This is the equivalent of x86's `IRETD`.

### SWI — Software Interrupts (System Calls)

`SWI #N` (also written `SVC #N` in newer ARM syntax) is the ARM equivalent of `int 0x80` or `syscall` on x86. It triggers the SWI exception, switching to SVC mode.

The ROM uses SWI to implement a few privileged operations. The SWI handler at 0x200003c0 reads the SWI number from the instruction encoding (the low 24 bits) and dispatches. The ROM's SWI #0xFF is used to switch CPU mode — a privileged operation that only makes sense in a bare-metal context where you control all modes.

### Panic Handlers

When something goes wrong, the ROM's abort handlers (`panic_undef`, `panic_prefetch`, `panic_data_abort`) all do the same thing: spin forever (`b .`). On real hardware, the watchdog timer eventually fires and resets the chip. In QEMU, you'll just see the emulator hang.

**For debugging**, data aborts are your most common problem when a MMIO stub is missing. If the ROM accesses an address that QEMU has no handler for and the "unimplemented" fallback isn't set up correctly, you'll get a data abort and the ROM will spin in `panic_data_abort` at 0x20001158.

---

## 5. Interrupts: VIC, EDGEIC, and the Full IRQ Chain

### The Problem With a Single IRQ Line

The ARM CPU has one IRQ input pin (nIRQ) and one FIQ input pin (nFIQ). But the S5L8900 has dozens of interrupt sources (USB, NAND, timers, GPIOs, ...). How does one pin serve dozens of sources?

The answer is an interrupt controller — a hardware block that sits between the peripherals and the CPU. The CPU asks the controller "which peripheral fired?" instead of having to poll all of them.

### PL190 VIC — Vectored Interrupt Controller

The S5L8900 uses two ARM PL190 VIC blocks: VIC0 at 0x38e00000 (channels 0–31) and VIC1 at 0x38e01000 (channels 32–63).

The PL190 is a standard ARM peripheral. Its key registers:

| Offset | Name | Function |
|---|---|---|
| `+0x000` | VICIRQSTATUS | Bitmask of pending enabled IRQs |
| `+0x004` | VICFIQSTATUS | Bitmask of pending enabled FIQs |
| `+0x008` | VICRAWINTR | Raw (unmasked) pending interrupts |
| `+0x00c` | VICINTSELECT | Per-channel IRQ vs FIQ selection |
| `+0x010` | VICINTENABLE | Enable mask — write 1 to enable a channel |
| `+0x014` | VICINTENCLEAR | Write 1 to disable a channel |
| `+0x100` | VICVECTADDR0 | Handler address for channel 0 |
| `+0x104` | VICVECTADDR1 | Handler address for channel 1 |
| ... | ... | ... |
| `+0x17c` | VICVECTADDR31 | Handler address for channel 31 |
| `+0xf00` | VICADDRESS | Returns handler address of highest-priority pending IRQ |

**How vectored dispatch works:**

1. ROM programs `VICVECTADDR[n]` with the address of the handler for channel n.
2. A peripheral asserts its interrupt line → VIC sees the corresponding bit go high.
3. VIC raises nIRQ to the CPU.
4. CPU jumps to the IRQ exception vector → `irq_handler`.
5. `irq_handler` reads `VIC0->VICADDRESS` (offset 0xf00).
6. The VIC returns the handler address it was programmed with for the highest-priority pending enabled interrupt.
7. `irq_handler` jumps to that address.
8. Handler executes, then writes any value to `VICADDRESS` to signal end-of-interrupt (EOI). This clears the pending flag.

This is very similar to the 8259A PIC on x86, or APIC vectored interrupts — except the ARM VIC stores the handler *address* rather than a vector number, so there's no IDT lookup.

### Priority

The VIC uses channel number as priority: channel 0 is highest priority, channel 31 is lowest. If both channel 3 and channel 7 are pending, `VICADDRESS` returns the channel 3 handler. The ROM's USB interrupt is wired to a specific channel — from the EDGEIC analysis, it's channel 7 of VIC1 (global channel 39).

### EDGEIC — Edge Interrupt Controller

The VIC inputs are *level-triggered* — a peripheral holds its interrupt line high until the driver acknowledges it. But some peripherals only pulse their line briefly (edge-triggered). The EDGEIC at 0x38e02000 sits between edge-triggered peripherals and the VIC.

The EDGEIC latches edges and presents them as a level to the VIC. When the driver wants to acknowledge the interrupt, it writes to the EDGEIC's pending register (not the VIC's — the VIC's pending bit follows the EDGEIC output level). Writing 1 to an EDGEIC pending bit clears it, which lowers the signal to the VIC, which then clears the VIC pending bit.

EDGEIC registers:
```
+0x008  : channels 0–31  edge-latched pending bits (write 1 to clear)
+0x00c  : channels 32–63 edge-latched pending bits (write 1 to clear)
```

Channel 39 (bit 7 of +0xc) is the USB interrupt channel. This is why `usb_isr` calls `edgeic_assert_ack(0x27)` — 0x27 = 39 decimal — to clear the USB edge latch.

### The Full IRQ Chain for USB

```
USB DWC2 controller asserts interrupt
    ↓
EDGEIC channel 39 latches the edge
    ↓
EDGEIC raises level to VIC1 input 7 (channel 39)
    ↓
VIC1 sees channel 7 pending, raises nIRQ to CPU
    ↓
CPU finishes current instruction, saves CPSR→SPSR_irq, saves PC+4→LR_irq
    ↓
CPU switches to IRQ mode, jumps to 0x00000018 (IRQ vector in SRAM)
    ↓
SRAM[0x18] = ldr pc, =irq_handler (in ROM at 0x20000428)
    ↓
irq_handler saves r0-r3,r12, calls vic_irq_dispatch
    ↓
vic_irq_dispatch reads VIC1->VICADDRESS (0x38e01f00)
    ↓
VIC1 returns the handler address programmed for channel 7
    ↓
vic_irq_dispatch jumps to that address (usb_isr_entry at 0x20001eac)
    ↓
usb_isr_entry sets usb_struct->0x98 = 1, calls edgeic_assert_ack(0x27)
    ↓
edgeic_assert_ack clears EDGEIC channel 39 latch
    ↓
EDGEIC lowers its output to VIC1 channel 7
    ↓
VIC1 clears pending bit for channel 7
    ↓
irq_handler returns (subs pc, lr, #4 → resume interrupted code)
```

Seventeen steps from "peripheral wants attention" to "back to normal code." Understanding this chain is critical for QEMU — you need to model enough of it for the ROM to work correctly.

### IRQ vs FIQ

FIQ (Fast IRQ) is ARM's high-priority interrupt mechanism. Differences from IRQ:

1. FIQ banks r8–r12, so handlers can use them without save/restore overhead.
2. FIQ is typically given to the highest-priority time-critical peripheral.
3. While in FIQ mode, both IRQ and FIQ are masked (unlike IRQ mode which only masks IRQ).

The ROM has a `fiq_handler` at 0x20000418 but it's rarely triggered in normal DFU operation. The USB interrupt is routed as IRQ, not FIQ.

---

## 6. DMA — Direct Memory Access

### Why DMA Exists

When the USB controller receives data, someone has to move those bytes from the USB FIFO registers into SRAM. Two options:

**CPU-copy (PIO — Programmed I/O):** CPU runs a loop reading from the USB FIFO address and writing to SRAM. Simple, but burns CPU cycles proportional to transfer size.

**DMA:** A dedicated hardware engine does the copy. CPU programs the DMA controller with source address, destination address, and byte count, then goes back to whatever it was doing. DMA engine signals completion via interrupt or a status bit.

This is exactly the same tradeoff as sendfile() vs read()+write() in Linux — DMA is the kernel's equivalent of zero-copy.

### The USB DMA Controller at 0x38000000

The S5L8900 has a USB-specific DMA controller separate from the DWC2 itself. Based on ROM analysis:

| Offset | Function |
|---|---|
| `+0x00` | Status/control. Bit 0 = DMA busy |
| `+0x0c` | Trigger/kick register |

The DMA transfer sequence (as the ROM does it):

1. Program destination address and length somewhere (exact registers TBD from deeper analysis).
2. Write to `0x38000000 + 0xc` to start the transfer.
3. The DMA controller copies data from USB FIFO to SRAM.
4. Bit 0 of `0x38000000` is set (busy) during the transfer.
5. When done, bit 0 clears.
6. `usb_dma_complete` polls bit 0 until clear, then sets `usb_struct->0xa0 = 1`.

### Polling vs Interrupt-Driven Completion

The ROM's DMA completion handler (`usb_dma_complete`) is a *polling* handler, not interrupt-driven. It sits in a tight loop:

```c
while (*(volatile uint32_t *)0x38000000 & 1) { /* spin */ }
```

This is intentional for a bootloader — you want simplicity and determinism, not interrupt latency. The main DFU wait loop (`dfu_transfer_wait_loop`) is similarly a spin loop on `usb_struct->0xa0`.

**Implication for QEMU**: There are two nested spin loops. For the ROM to proceed, QEMU must:
1. Deliver a USB SETUP packet → ROM sets up DMA and starts spinning on bit 0.
2. Set bit 0 of 0x38000000 during the transfer (busy).
3. Clear bit 0 when done (complete).
4. The ROM's completion handler then sets `usb_struct->0xa0 = 1`.
5. The outer wait loop sees the flag and proceeds.

If QEMU's DMA stub always returns 0 for bit 0, the DMA completion handler exits immediately but never gets called in the first place (because the ROM hasn't set up a DMA transfer yet). If it always returns 1, the DMA completion handler spins forever.

---

## 7. USB From First Principles

### The USB Topology

USB is a host-centric bus. There is always exactly one host (the computer) and one or more devices (the iPod). The host initiates all transactions — devices cannot send data without being asked.

```
Host (computer)
  └── Root hub
        └── [Optional external hubs]
              └── Device (iPod)
                    ├── Configuration 1
                    │     ├── Interface 0 (DFU class)
                    │     │     ├── Endpoint 0 IN  (control)
                    │     │     └── Endpoint 0 OUT (control)
                    │     └── Interface 1 ...
                    └── Configuration 2 ...
```

### Endpoints

An endpoint is a communication channel. Each device has a set of endpoints, identified by number (0–15) and direction (IN = device→host, OUT = host→device).

**Endpoint 0** is special: every USB device must have endpoint 0, it's bidirectional, and it handles the *control transfer* used for enumeration and class commands. The DFU protocol uses only endpoint 0.

### Transfer Types

| Type | Usage | Flow control |
|---|---|---|
| Control | Enumeration, class commands | Yes — defined protocol |
| Bulk | Mass storage, DFU data | Yes — retransmit on error |
| Interrupt | HID (keyboard, mouse) | Polled by host |
| Isochronous | Audio, video | No — drop on error |

DFU uses **control transfers** for all operations. Control transfers have three phases:

**1. SETUP phase**: Host sends an 8-byte SETUP packet describing the request.  
**2. DATA phase** (optional): Data flows in one direction (IN or OUT depending on request).  
**3. STATUS phase**: A zero-length packet in the opposite direction confirms completion.

The 8-byte SETUP packet structure:
```
Byte 0  bmRequestType  Direction | Type | Recipient
Byte 1  bRequest       Command code (e.g., 1=DNLOAD, 3=GETSTATUS)
Byte 2  wValue LSB     Command-specific parameter (low byte)
Byte 3  wValue MSB     Command-specific parameter (high byte)
Byte 4  wIndex LSB     Interface/endpoint number (usually 0)
Byte 5  wIndex MSB
Byte 6  wLength LSB    Length of DATA phase in bytes
Byte 7  wLength MSB
```

### USB Enumeration

When a device is plugged in:

1. Host detects D+ or D- pullup (device signals connect).
2. Host issues a bus reset (drives both lines low for >10ms).
3. Host sends GET_DESCRIPTOR(Device) to address 0.
4. Device responds with its Device Descriptor (VID, PID, USB version, etc.).
5. Host assigns an address via SET_ADDRESS.
6. Host fetches Configuration Descriptor, Interface Descriptors, etc.
7. Host selects a configuration via SET_CONFIGURATION.
8. Device is now enumerated and ready for class-specific traffic.

The DFU ROM has to respond to all of this before the host will send DFU commands. The ROM's USB descriptor data (Device Descriptor, Config Descriptor, DFU Interface Descriptor, String Descriptors) is embedded as static tables in the ROM.

---

## 8. DFU — Device Firmware Upgrade Protocol

DFU is a USB device class defined by the USB Implementers Forum (USB-IF). It defines a standard way to download firmware to a device.

### DFU States

DFU has a formal state machine:

```
appIDLE
appDETACH        ← host sends DFU_DETACH, device reboots into DFU mode
dfuIDLE          ← waiting for DNLOAD or UPLOAD
dfuDNLOAD-SYNC   ← host sent DNLOAD, device processing
dfuDNBUSY        ← device busy with last block
dfuDNLOAD-IDLE   ← block received OK, ready for next block
dfuMANIFEST-SYNC ← all blocks received, manifesting (programming flash)
dfuMANIFEST      ← manifestation in progress
dfuMANIFEST-WAIT-RESET ← done, waiting for reset
dfuUPLOAD-IDLE   ← ready for UPLOAD
dfuERROR         ← error occurred
```

The ROM starts in `dfuIDLE` (it's a ROM, it's always in DFU mode).

### DFU Commands (bRequest values)

| Value | Command | Direction | Meaning |
|---|---|---|---|
| 0 | DFU_DETACH | OUT | Request app to enter DFU mode |
| 1 | DFU_DNLOAD | OUT | Send a firmware block to device |
| 2 | DFU_UPLOAD | IN | Read firmware from device |
| 3 | DFU_GETSTATUS | IN | Get current DFU status |
| 4 | DFU_CLRSTATUS | OUT | Clear error status |
| 5 | DFU_GETSTATE | IN | Get current DFU state |
| 6 | DFU_ABORT | OUT | Abort and return to dfuIDLE |

### DFU Download Sequence

A complete firmware upload (from host's perspective) looks like:

```
Host                                    Device
----                                    ------
SETUP: DFU_DNLOAD wValue=0 wLength=2048  →
  DATA: [2048 bytes of firmware block 0] →
  STATUS: host reads ZLP (zero-length)   ←  [device ACKs]

SETUP: DFU_GETSTATUS                     →
  DATA:                                  ←  [status: dfuDNLOAD-SYNC, bwPollTimeout=0]

SETUP: DFU_DNLOAD wValue=1 wLength=2048  →
  DATA: [2048 bytes of firmware block 1] →
  STATUS: ZLP                            ←

... (repeat for each block) ...

SETUP: DFU_DNLOAD wValue=N wLength=0     →   ← zero-length = end of firmware
  STATUS: ZLP                            ←

SETUP: DFU_GETSTATUS                     →
  DATA:                                  ←  [status: dfuMANIFEST, bwPollTimeout=2000]

[device programs flash, verifies signature, reboots]
```

### Apple's DFU Implementation

Apple's ROM DFU differs from the USB-IF spec in a few ways:

1. **Signature verification happens at the end.** The ROM accepts all blocks without checking, then verifies the full image (IMG2 format with RSA-SHA1 signature) before jumping to it.
2. **No UPLOAD support.** The ROM's DFU_UPLOAD handler returns an error — you can't read back firmware.
3. **Custom timeouts.** The bwPollTimeout in GETSTATUS responses is Apple-specific.

### What `dfu_handle_request` Does

The ROM's SETUP packet parser at 0x200007a8 reads the 8-byte SETUP packet and branches:

- `bRequest == 1` (DNLOAD): validate wLength, call `usb_out_transfer_setup` to arm a USB OUT transfer, wait in `dfu_transfer_wait_loop` for data.
- `bRequest == 3` (GETSTATUS): send back a 6-byte status struct directly.
- `bRequest == 5` (GETSTATE): send back a 1-byte state value.
- `bRequest == 6` (ABORT): reset state to dfuIDLE.
- Others: STALL the endpoint (signal error to host).

---

## 9. The DWC2 Synopsys USB Controller

### What DWC2 Is

DesignWare USB 2.0 OTG (On-The-Go) is an IP block licensed by Synopsys and used by Apple, Raspberry Pi, Allwinner, and many others. "OTG" means it can operate as either host or device. The S5L8900 uses it in device mode only.

The Linux kernel has a DWC2 driver at `drivers/usb/dwc2/`. If you want the full register reference, that driver's header files are authoritative (Synopsys doesn't publish the full datasheet publicly, but the kernel driver headers document everything).

### Key DWC2 Registers

The DWC2 base is 0x38c00000 on the S5L8900. Register offsets:

**Global registers (apply to the whole core):**

| Offset | Name | Function |
|---|---|---|
| `+0x000` | GOTGCTL | OTG control and status |
| `+0x008` | GINTSTS | Global interrupt status (read to find what fired, write 1 to clear) |
| `+0x00c` | GINTMSK | Global interrupt mask (1 = enable) |
| `+0x010` | GRXSTSR | Receive status debug read |
| `+0x014` | GRXSTSP | Receive status read and pop |
| `+0x018` | GRXFSIZ | Receive FIFO size |
| `+0x040` | GSNPSID | Synopsys ID register (read-only, returns core version) |
| `+0x048` | GHWCFG2 | Hardware config 2 (num endpoints, DMA capable, etc.) |
| `+0x008` | GRSTCTL | Global reset control |

Wait — GRSTCTL is at offset 8 and GINTSTS is also listed at offset 8? The Linux kernel source and Synopsys documentation use GRSTCTL at 0x010. The ROM source shows `USB+0x8 = GRSTCTL (soft reset)` during init. There's some ambiguity here between different DWC2 versions. Trust the ROM's access pattern over any external doc.

**Device mode registers (used when operating as USB device):**

| Offset | Name | Function |
|---|---|---|
| `+0x800` | DCFG | Device configuration (speed, address) |
| `+0x804` | DCTL | Device control (connect/disconnect D+ pullup) |
| `+0x808` | DSTS | Device status (enumerated speed, suspend state) |
| `+0x810` | DIEPMSK | Device IN endpoint interrupt mask |
| `+0x814` | DOEPMSK | Device OUT endpoint interrupt mask |
| `+0x818` | DAINT | Device all endpoints interrupt status |

**DCFG** — Device Configuration:
```
Bits [1:0]  DevSpd    : 0=HS, 1=FS, 3=LS
Bits [10:4] DevAddr   : USB device address (assigned by host during SET_ADDRESS)
```

**DCTL** — Device Control:
```
Bit 1   SftDiscon  : 1 = disconnect (pull D+ low), 0 = connect
```

The ROM controls USB connect/disconnect via this bit. It sets `SftDiscon=0` to connect, which enables the D+ pullup and signals to the host that a full-speed device is present.

**DSTS** — Device Status:
```
Bits [2:1]  EnumSpd   : 0=HS, 1=FS (what speed the host enumerated us as)
Bit  [0]    SuspSts   : 1 = suspended
```

### GRSTCTL — The Soft Reset Sequence

During init, the ROM performs a DWC2 soft reset:

1. Write `GRSTCTL |= (1 << 0)` — set the CoreSoftRst bit.
2. Poll `GRSTCTL` until bit 31 (AHBIdle) is set AND bit 0 (CoreSoftRst) is clear.
3. This indicates the reset completed and the AHB bus is idle.

In QEMU, if the stub always returns 0 for GRSTCTL, the ROM will spin forever waiting for AHBIdle. The stub needs to return `(1 << 31)` (AHBIdle=1, CoreSoftRst=0) to satisfy the poll.

### GSNPSID — Core ID

Returns the Synopsys IP core version, e.g., `0x4F54280A` or `0x4F54330A`. The ROM may check this. Returning 0 is probably safe (it'll treat it as an unrecognized but non-fatal value), but returning a realistic value is better.

### USB PHY — Physical Layer

The DWC2 handles protocol framing but the actual differential signaling on the USB cable is done by the USB PHY at 0x3c400000. The PHY converts digital signals to/from the analog USB D+/D- lines.

PHY registers (S5L8900-specific, not a standard block):

| Offset | Name | Function |
|---|---|---|
| `+0x000` | OPHYPWR | PHY power control |
| `+0x004` | OPHYCLK | PHY clock selection |
| `+0x008` | ORSTCON | PHY reset control |

The ROM's `usb_phy_init` function programs these to power up and configure the PHY before enabling the DWC2. In QEMU, writes to these registers can be silently ignored — the PHY doesn't have observable behavior for the ROM's purposes.

---

## 10. The Boot Chain and Apple Secure Boot

### Why Multiple Boot Stages?

The SecureROM is only 64KB. That's not enough space to initialize DRAM, talk to NAND flash, verify a kernel, and load it — plus it's ROM and can't be updated. So Apple uses a chain of progressively larger bootloaders:

```
SecureROM (64KB, in ROM)
  → LLB (Low Level Bootloader, in NOR flash, ~64KB)
      → iBoot (in NOR flash, ~500KB)
          → kernelcache
              → iOS userland
```

Each stage verifies the next before jumping to it. If any stage fails verification, the chain stops and the device enters recovery mode.

DFU mode is built into the SecureROM so that even if NOR flash is blank or corrupted, the device can be reflashed. This is the "unbreakable" entry point.

### IMG2 — Apple Firmware Container Format

LLB (and all other boot images) are wrapped in Apple's IMG2 format:

```
Offset  Size  Field
------  ----  -----
+0x000  4     Magic: 0x494D4732 ("IMG2")
+0x004  4     Image type tag (e.g., "illb" = LLB, "ibot" = iBoot)
+0x008  4     Header size
+0x00c  4     Security epoch
+0x010  ?     AES-encrypted payload (firmware binary)
+...    20    SHA1 hash of decrypted payload
+...    256   RSA-2048 signature over the hash
+...    ?     Certificate chain (Apple Secure Boot cert + device cert)
```

The exact layout varies slightly between firmware versions, but the core structure is: encrypted payload + SHA1 + RSA signature + certificate chain.

### Apple Secure Boot Verification

The ROM's verification sequence (implemented in the crypto functions annotated around 0x2000ab68+):

1. **Parse the IMG2 header.** Check magic, validate sizes.
2. **Decrypt the payload.** The firmware payload is AES-128-CBC encrypted. The key and IV are derived from device-specific fuses (UID key) combined with the image type tag. **This is why you can't use one device's decrypted firmware on another device.**
3. **Compute SHA1 over the decrypted payload.** 
4. **Verify the RSA signature.** The stored SHA1 hash is signed with Apple's Secure Boot private key. The ROM verifies it using the corresponding public key (embedded in ROM). The signature proves Apple created this firmware.
5. **Verify the certificate chain.** The certificate chain in the image traces back to the Apple Root CA embedded in ROM. This confirms the signing key is legitimate.

If any step fails, the ROM STALLs the DFU endpoint and returns to `dfuIDLE`.

### Why This Matters for Security Research

The bootrom vulnerability that enables jailbreaks on these early devices is exploitable precisely because:

1. The ROM cannot be patched (it's ROM).
2. If there's a bug in the DFU USB parsing code or the signature verification code, it can be exploited from DFU mode before Secure Boot runs.
3. Classic attacks exploit memory safety bugs in the SETUP packet parser or in the IMG2 header parser — sending malformed data that corrupts the `usb_struct` or stack.

The `dfu_handle_request` code you've already analyzed is exactly the attack surface. Every `str r0, [r1, #N]` instruction that writes host-provided data to memory is a potential write primitive if the offset calculation can be manipulated.

---

## 11. Bare-Metal C Runtime — How a Program Starts With No OS

### The Problem With Global Variables

In a normal C program running on Linux, the OS loader sets up:
- Stack (via `execve`, mapped by the kernel)
- `.data` segment (initialized globals) already contains the right values in the ELF file
- `.bss` segment (zero-initialized globals) zeroed by the loader

In a ROM, there's no loader. The ROM binary is burned into the chip and mapped read-only. The CPU starts executing at the reset vector, but:

- The `.data` section needs to be in writable SRAM, not ROM
- Nobody has copied it there yet
- The `.bss` section needs to be zeroed, but nobody has done that

If the ROM's C code uses any global variable before `startup_data_bss_init` runs, it gets garbage (whatever was in SRAM at power-on).

### What startup_data_bss_init Does

The ROM's `startup_data_bss_init` at 0x2000034c does exactly what Linux's `start_kernel` relies on the bootloader having done:

```c
// Conceptually:
extern char _data_rom_start[], _data_start[], _data_end[];
extern char _bss_start[], _bss_end[];

void startup_data_bss_init(void) {
    // 1. Copy .data from ROM to SRAM
    memcpy(_data_start, _data_rom_start, _data_end - _data_start);
    
    // 2. Zero .bss
    memset(_bss_start, 0, _bss_end - _bss_start);
    
    // 3. Set initialized flag
    initialized = 1;
    
    // 4. Call main
    main_entry();
}
```

The actual assembly uses literal pool values for `_data_rom_start`, `_data_start`, `_data_end`, `_bss_start`, `_bss_end` — these are filled in by the linker script at compile time.

### The Stack

The ROM sets up the stack pointer in `BEGIN_HARDWARE_INIT` before any C code runs:

```
// Assembly in BEGIN_HARDWARE_INIT:
ldr sp, =0x22026c84    // or similar SRAM top address
```

Each ARM mode (SVC, IRQ, FIQ, ABT, UND) has its own stack pointer. The ROM sets them all up during init. IRQ mode gets a separate smaller stack because IRQ handlers run with all IRQs masked — they need to be fast and can't take much stack space.

### The usb_struct — A Global State Object

`usb_struct` is the ROM's main USB state object. It's a global variable (in `.bss`, zeroed at startup). The ROM stores a pointer to it at a fixed SRAM address (0x2202bff8 based on analysis). Every subsystem that needs USB state loads this pointer and offsets into the struct.

Key fields decoded so far:

| Offset | Type | Meaning |
|---|---|---|
| +0x98 | uint32_t | USB interrupt fired flag (set by ISR, cleared by handler) |
| +0xa0 | uint32_t | DMA transfer complete flag (set by usb_dma_complete) |
| +0x8ec | ptr | Pointer to endpoint state sub-struct |

---

## 12. QEMU Internals — How Emulation Works

### QEMU's Architecture

QEMU is a system emulator. For ARM, it:

1. **Translates ARM instructions to host machine code** using TCG (Tiny Code Generator). This is JIT compilation — ARM basic blocks are compiled to x86_64 (or native) on first execution and cached.

2. **Emulates memory** via a flat physical address space with a dispatch table. Each range of physical addresses is registered with read/write callbacks.

3. **Emulates peripherals** as software objects that register memory regions. When translated code reads/writes a peripheral address, the callbacks fire.

### MemoryRegionOps — How Peripheral Stubs Work

In the QEMU source you've been modifying (`s5l8900.c`), peripheral stubs look like:

```c
static uint64_t my_peripheral_read(void *opaque, hwaddr offset, unsigned size)
{
    MyState *s = opaque;
    return some_value;
}

static void my_peripheral_write(void *opaque, hwaddr offset,
                                 uint64_t value, unsigned size)
{
    MyState *s = opaque;
    // handle the write
}

static const MemoryRegionOps my_ops = {
    .read  = my_peripheral_read,
    .write = my_peripheral_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};
```

`opaque` is whatever pointer you pass when creating the memory region. `offset` is the byte offset *within* the registered region (so if you registered at 0x38c00000, and the ROM reads 0x38c00014, `offset = 0x14`). `size` is 1, 2, or 4 bytes.

### The `unimp` Device

You'll see this in `s5l8900.c`:

```c
create_unimplemented_device("s5l8900.periph", 0x38000000, 0x08000000);
```

This registers a catch-all for any address in the peripheral range that you haven't given a specific handler. The `-d unimp` QEMU flag prints a message to stderr when any unimplemented region is accessed. This is how you identify which peripherals the ROM is trying to use.

**Critical**: more-specific registrations (your VIC, clock, USB stubs) shadow the `unimp` region at their addresses. The `unimp` only fires for addresses not covered by a more-specific handler.

### GDB Integration

QEMU's `-S -gdb tcp::1234` flags:
- `-S`: Start with CPU halted (wait for GDB's `continue` command)
- `-gdb tcp::1234`: Listen for GDB connection on port 1234

Once connected, GDB speaks the Remote Serial Protocol (RSP) over TCP to QEMU. All standard GDB commands work: `info registers`, `x/10i $pc`, `break *0x20002770`, etc.

**Useful GDB commands for this project:**

```gdb
# Show all ARM registers
info registers

# Disassemble from current PC
x/20i $pc

# Set a hardware breakpoint (works even in ROM which is read-only)
hbreak *0x200027cc

# Read a memory word
x/xw 0x2202bff8

# Write a memory word (to simulate DMA completion)
set {int}0x38000000 = 0

# Continue execution
continue

# Single-step
stepi

# Show CPSR
p/x $cpsr
```

### The unimp Log

When you run QEMU with `-d unimp`, the log at `/tmp/qemu_out.txt` will contain lines like:

```
unimp: s5l8900.periph: write to unimplemented address 0x3c500040 (value 0x1)
unimp: s5l8900.periph: read from unimplemented address 0x38000000
```

This is your roadmap for which stubs to implement next. Each unimp hit is a peripheral the ROM accessed that QEMU didn't handle.

---

## 13. Putting It All Together — The ROM's Full Execution Path

Walk through the ROM from power-on to DFU wait, with the concepts from all previous sections:

### Step 1: CPU Reset (Address 0x20000000)

CPU powers on. SCTLR defaults to 0 (MMU off, caches off, vectors at 0x0). PC is set to 0x20000000 (hardware-defined reset vector for S5L8900). ROM's first word is `ea 00 00 2f` = `b 0x200000c4`.

### Step 2: BEGIN_HARDWARE_INIT (0x200000c4)

Before any stack or C runtime is available, pure assembly initializes hardware in order:

1. **WDT**: Write magic value to watchdog to prevent reset during init.
2. **Stack pointers**: Set r13 for each ARM mode (SVC, IRQ, FIQ, ABT, UND). Uses `msr cpsr_c, #mode` to switch modes and `ldr sp, =SRAM_addr` for each.
3. **CP15 initialization**: Zero FCSE PID, set DACR, flush TLB and caches.
4. **Enable caches**: `enable_icache()`, `enable_dcache()`.
5. **VIC setup**: Write handler addresses to VICVECTADDR registers, call `VICINTENABLE` for USB channel.
6. **EDGEIC setup**: Configure edge-triggered interrupt routing.
7. **Copy exception vectors to SRAM at 0x0**: So exceptions after this point work correctly.
8. **Call `startup_data_bss_init`**: Never returns.

### Step 3: startup_data_bss_init (0x2000034c)

1. Copy `.data` from ROM (above code section) to SRAM at ~0x22020800.
2. Zero `.bss` in SRAM.
3. Set `initialized = 1`.
4. Call `main_entry`.

### Step 4: main_entry (0x20003790)

1. Call `usb_phy_init` — powers up USB PHY at 0x3c400000.
2. Call `usb_init` — configures DWC2 at 0x38c00000 (GRSTCTL reset, GUSBCFG speed, DCFG address=0, DCTL connect).
3. Call `usb_dfu_main_loop` — enters the main event loop.

### Step 5: usb_dfu_main_loop (0x20002bc0)

A simple event loop. Pseudo-code:

```c
while (1) {
    if (usb_struct->state == TRANSFER_PENDING) {
        // A DFU_DNLOAD SETUP packet arrived (ISR already set this flag)
        usb_out_transfer_setup();        // arm USB OUT endpoint
        dfu_transfer_wait_loop();        // spin until data arrives
        process_received_block();
    }
    if (usb_struct->all_blocks_received) {
        img2_load_and_verify();          // verify signature
        jump_to_llb();                   // if valid
    }
    // else loop and wait for next interrupt
}
```

### Step 6: DFU DNLOAD Transfer — Detailed

When `dfu-util` (on the host) sends `DFU_DNLOAD`:

1. **USB DWC2 receives the SETUP packet** → asserts interrupt.
2. **EDGEIC latches channel 39** → raises VIC1 channel 7.
3. **CPU takes IRQ exception** → jumps to `irq_handler`.
4. **irq_handler** → `vic_irq_dispatch` → reads VIC1 VICADDRESS → jumps to `usb_isr_entry`.
5. **usb_isr** sets `usb_struct->0x98 = 1`, ACKs EDGEIC channel 39.
6. **IRQ returns** → resumes wherever `usb_dfu_main_loop` was spinning.
7. **Main loop sees 0x98 flag** → calls `dfu_handle_request`.
8. **dfu_handle_request** reads the 8-byte SETUP packet from a DWC2 FIFO register, sees `bRequest=1` (DNLOAD), validates `wLength`.
9. Calls `usb_out_transfer_setup(wLength)` → programs DWC2 OUT endpoint to receive wLength bytes, programs USB DMA controller.
10. Enters **`dfu_transfer_wait_loop`** — clears `usb_struct->0xa0`, then spins on it.
11. **USB DWC2** receives the OUT data packet, DMA engine moves bytes to SRAM.
12. **`usb_dma_complete`** polls 0x38000000 bit 0, sees it clear, sets `usb_struct->0xa0 = 1`.
13. **`dfu_transfer_wait_loop`** sees the flag, returns.
14. Block data is now in SRAM. Loop back, wait for next block.

### Step 7: Verification and Jump

After DFU_DNLOAD with wLength=0 (end-of-firmware marker), the ROM:

1. Calls `img2_load_and_verify`:
   - Checks IMG2 magic.
   - Decrypts payload with AES (using UID-derived key).
   - Computes SHA1 of decrypted payload.
   - Verifies RSA signature on SHA1.
   - Verifies certificate chain.
2. If valid: `ldr pc, =llb_entry_point` — jumps to the decrypted LLB in SRAM.
3. If invalid: sets DFU state to `dfuERROR`, returns STALL on next request.

### The Attack Surface for Exploitation

Every step above where host-supplied data is read and used to compute a memory address or copy destination is potential attack surface:

- **SETUP packet parsing**: `wLength` is used as a size for the DMA transfer. If it's not properly validated, oversized transfers could overflow the SRAM buffer.
- **IMG2 parsing**: The header contains sizes and offsets used to locate the payload, signature, and certificate. Malformed headers can cause the parser to read/write out-of-bounds.
- **The DFU block counter**: `wValue` in DNLOAD is the block number. If used to index an array without bounds checking, it's a classic integer overflow / OOB write.

Known historical exploits on this hardware (e.g., the original iPhone/iPod Touch jailbreaks) exploited exactly these kinds of parser bugs in the SecureROM to gain code execution before Secure Boot completes.

---

## Quick Reference Card

### ARM Instruction Decoder

| Pattern | Meaning |
|---|---|
| `push {r4-r7, lr}` | Save regs + return addr; function entry |
| `pop {r4-r7, pc}` | Restore + return (pc = old lr) |
| `bl 0x20001234` | Call function at 0x20001234 (LR = next insn) |
| `bx lr` | Return to caller |
| `ldr r0, [r1, #8]` | r0 = *(r1 + 8) |
| `str r0, [r1, #8]` | *(r1 + 8) = r0 |
| `ldr r0, =0x38c00000` | r0 = 0x38c00000 (PC-relative literal load) |
| `orr r0, r0, #1` | r0 \|= 1 |
| `bic r0, r0, #1` | r0 &= ~1 (Bit Clear) |
| `tst r0, #1` | sets Z if (r0 & 1) == 0 |
| `cmp r0, #0` | sets flags from r0-0 |
| `beq label` | branch if Z set (last cmp/tst was equal/zero) |
| `bne label` | branch if Z clear |
| `subs pc, lr, #4` | exception return (IRQ/FIQ) |
| `movs pc, lr` | exception return (SWI/UND) |
| `msr cpsr_c, r0` | write control byte of CPSR (mode switch) |
| `mrs r0, cpsr` | read CPSR into r0 |
| `mcr p15,...` | write CP15 register |
| `mrc p15,...` | read CP15 register |

### Key Addresses

| Address | What |
|---|---|
| 0x20000000 | SecureROM base |
| 0x20000000 | reset_vector (first instruction) |
| 0x200000c4 | BEGIN_HARDWARE_INIT |
| 0x2000034c | startup_data_bss_init |
| 0x20003790 | main_entry |
| 0x2000068c | usb_init |
| 0x200007a8 | dfu_handle_request |
| 0x200023d4 | usb_out_transfer_setup |
| 0x200027cc | dfu_transfer_wait_loop |
| 0x200024b4 | usb_dma_complete |
| 0x20001eac | usb_isr_entry |
| 0x20001158 | panic_data_abort |
| 0x2202bff8 | usb_struct pointer (in SRAM) |
| 0x38000000 | USB DMA controller |
| 0x38c00000 | USB DWC2 OTG |
| 0x38e00000 | VIC0 |
| 0x38e01000 | VIC1 |
| 0x38e02000 | EDGEIC |
| 0x3c400000 | USB PHY |
| 0x3c500000 | CLOCK1 |

### GDB Cheat Sheet for This Project

```gdb
target remote :1234          # Connect to QEMU
set architecture arm         # Ensure ARM mode
info registers               # Show all regs
x/20i $pc                   # Disassemble from PC
x/xw 0x2202bff8             # Read usb_struct pointer
hbreak *0x200027cc          # Break at DFU wait loop
hbreak *0x20001158          # Break on panic (data abort)
hbreak *0x200024b4          # Break at DMA complete
continue                     # Run
stepi                        # Step one instruction
set {int}0x38000000 = 0     # Clear DMA busy flag manually
p/x $cpsr                   # Show CPU mode and flags
```
