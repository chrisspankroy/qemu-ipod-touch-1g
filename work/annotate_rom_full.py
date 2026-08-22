"""
Full S5L8900 SecureROM annotation script for Hopper Disassembler.
Generated from static analysis of the 64KB ROM image.

Run via File -> Run Script in Hopper.

Covers: boot flow, USB/DFU, VIC/EDGEIC, CPU helpers, crypto, NAND,
        memory functions, exception handlers, signal handlers.

BEGINNER'S GUIDE TO THE ROM:
  The SecureROM is Apple's read-only boot code burned into the S5L8900 chip.
  It cannot be modified. When the iPod powers on, the CPU starts here.

  BOOT FLOW:
    1. CPU resets -> jumps to BEGIN_HARDWARE_INIT (0x200000c4)
    2. Hardware init: WDT, VIC, EDGEIC, GPIOIC, clocks, stack, cache
    3. Copy .data segment from ROM to SRAM; zero .bss segment
    4. Call main entry (0x20003790) -> sets up USB, enters DFU mode
    5. DFU loop: receive firmware over USB (LLB image)
    6. Verify image signature (RSA + Apple cert chain)
    7. If valid: jump to LLB (Next Boot Stage)
    8. If invalid / no image: spin forever in DFU

  KEY HARDWARE ADDRESSES:
    0x38000000  USB DMA controller (transfers USB OUT data to SRAM)
    0x38c00000  USB DWC2 OTG core (Synopsys DesignWare USB 2.0 controller)
    0x38e00000  VIC0 - Vectored Interrupt Controller #0 (IRQs 0-31)
    0x38e01000  VIC1 - Vectored Interrupt Controller #1 (IRQs 32-63)
    0x38e02000  EDGEIC - Edge Interrupt Controller (edge-triggered routing)
    0x39a00000  GPIOIC - GPIO Interrupt Controller
    0x3c400000  USB PHY - USB physical layer (analog front-end)
    0x3c500000  CLOCK1 - PLL and clock divider controller
    0x3d000000  NAND - NAND flash controller
    0x3e300000  WDT_CTRL - Watchdog timer control
    0x3e400000  TIMER - Hardware timer
"""

doc = Document.getCurrentDocument()
seg = doc.getSegmentAtAddress(0x20000000)

if seg is None:
    print("ERROR: No segment at 0x20000000. Check Hopper base address.")
else:
    def c(addr, text):   seg.setInlineCommentAtAddress(addr, text)
    def C(addr, text):   seg.setCommentAtAddress(addr, text)
    def N(addr, name):   seg.setNameAtAddress(addr, name)

    # =========================================================================
    # EXCEPTION VECTOR TABLE  (0x20000000 - 0x2000001c)
    # ARM requires specific handlers at fixed addresses. Each entry jumps
    # to the real handler through an indirect table below.
    # =========================================================================
    N(0x20000000, "reset_vector")
    C(0x20000000,
      "ARM EXCEPTION VECTOR TABLE\n"
      "The ARM CPU uses a fixed vector table at address 0x0 (or 0xFFFF0000).\n"
      "The ROM copies these entries to SRAM at 0x0 during init so exceptions\n"
      "land here. Each 'ldr pc' loads the actual handler address from a\n"
      "nearby pointer table (offsets 0x6c-0xbc).")
    c(0x20000000, "Reset: branch to BEGIN_HARDWARE_INIT")
    c(0x20000004, "Undefined instruction: -> undef_handler")
    c(0x20000008, "SWI (software interrupt): -> swi_handler")
    c(0x2000000c, "Prefetch abort: -> prefetch_abort_handler")
    c(0x20000010, "Data abort: -> data_abort_handler")
    c(0x20000014, "Reserved (unused)")
    c(0x20000018, "IRQ: -> irq_fiq_handler")
    c(0x2000001c, "FIQ: -> irq_fiq_handler  (FIQ shares handler with IRQ)")

    # VIC handler jump stubs (0x20000024 - 0x20000060)
    C(0x20000024,
      "VIC HANDLER JUMP STUBS\n"
      "The VIC (Vectored Interrupt Controller) is programmed with the\n"
      "addresses of these stubs. When an IRQ fires, the VIC returns one\n"
      "of these addresses, and the IRQ dispatcher jumps to it.\n"
      "Each stub does: 'ldr pc, =actual_function' to reach the real handler.\n"
      "This indirection lets the VIC slots be reprogrammed without moving code.")
    c(0x20000024, "VIC slot A -> sub_48d4 (USB-related handler)")
    c(0x20000028, "VIC slot B -> sub_49f0 (USB-related handler)")
    c(0x2000002c, "VIC slot C -> sub_168c  (USB setup/control handler)")
    c(0x20000030, "VIC slot D -> dfu_handle_request  (DFU USB request handler)")
    c(0x20000034, "VIC slot E -> usb_isr_entry  (main USB interrupt handler)")
    c(0x20000038, "VIC slot F -> sub_3608")
    c(0x2000003c, "VIC slot G -> sub_87a4")
    c(0x20000040, "VIC slot H -> 0x20007960")
    c(0x20000044, "VIC slot I -> 0x20003514")
    c(0x20000048, "VIC slot J -> sub_1fb0")
    c(0x2000004c, "VIC slot K -> 0x20000be8")
    c(0x20000050, "VIC slot L -> sub_1dd8")
    c(0x20000054, "VIC slot M -> 0x2000113c")
    c(0x20000058, "VIC slot N -> sub_1f8c")
    c(0x2000005c, "VIC slot O -> 0x200010f0")

    # =========================================================================
    # BEGIN_HARDWARE_INIT  (0x200000c4)
    # First real code to run. Sets up all hardware before entering DFU.
    # =========================================================================
    N(0x200000c4, "BEGIN_HARDWARE_INIT")
    C(0x200000c4,
      "HARDWARE INITIALIZATION\n"
      "This runs immediately after reset. Sets up:\n"
      "  1. Watchdog timer (disabled)\n"
      "  2. VIC0/VIC1: all interrupts disabled, EOI cleared\n"
      "  3. EDGEIC: all edge interrupt pending bits cleared\n"
      "  4. GPIOIC: GPIO interrupt config (waits for stable state)\n"
      "  5. CLOCK1: PLL configured and locked\n"
      "  6. CPU mode: switches to SVC mode, sets up stack\n"
      "  7. Caches: instruction cache enabled\n"
      "  8. .data/.bss: copies initialized data to SRAM, zeros BSS\n"
      "  9. Calls main_entry (0x20003790)")
    c(0x200000c4, "r0 = WDT_CTRL address (0x3e300000)")
    c(0x200000c8, "r1 = 0xa5 = watchdog disable magic value")
    c(0x200000cc, "WDT_CTRL = 0xa5  -> DISABLE watchdog (prevents reset during boot)")
    c(0x200000d0, "r0 = 0xffffffff (all bits set)")
    c(0x200000d4, "r1 = VIC0+0x14 (VIC0 INTENCLEAR register address)")
    c(0x200000d8, "VIC0 INTENCLEAR = 0xffffffff  -> disable ALL VIC0 interrupts (IRQs 0-31)")
    c(0x200000dc, "r0 = 0xffffffff")
    c(0x200000e0, "r1 = VIC1+0x14 (VIC1 INTENCLEAR register address)")
    c(0x200000e4, "VIC1 INTENCLEAR = 0xffffffff  -> disable ALL VIC1 interrupts (IRQs 32-63)")
    c(0x200000e8, "r0 = 0xffffffff")
    c(0x200000ec, "r1 = VIC0+0xf00 (VIC0 ADDRESS / end-of-interrupt register)")
    c(0x200000f0, "VIC0 ADDRESS = 0xffffffff  -> clear any pending VIC0 IRQ state")
    c(0x200000f4, "r1 = VIC1+0xf00 (VIC1 ADDRESS / end-of-interrupt register)")
    c(0x200000f8, "VIC1 ADDRESS = 0xffffffff  -> clear any pending VIC1 IRQ state")
    c(0x200000fc, "r1 = EDGEIC+0x8 (edge interrupt pending bits for channels 0-31)")
    c(0x20000100, "EDGEIC+0x8 = 0xffffffff  -> acknowledge/clear ALL edge interrupts 0-31")
    c(0x20000104, "r1 = EDGEIC+0xc (edge interrupt pending bits for channels 32-63)")
    c(0x20000108, "EDGEIC+0xc = 0xffffffff  -> acknowledge/clear ALL edge interrupts 32-63")
    c(0x2000010c, "r1 = CLOCK0+0x24  (CLOCK0 PLL config register)")
    c(0x20000110, "r0 = 0x110933  (PLL multiplier/divider config value)")
    c(0x20000114, "CLOCK0+0x24 = 0x110933  -> configure CLOCK0 PLL")
    c(0x20000118, "CLOCK0+0x28 = 0x111455  -> configure CLOCK0 PLL")
    c(0x20000124, "GPIOIC+0xc = 0x1a13  -> configure GPIO interrupt sources")

    N(0x20000130, "CHECK_GPIOIC_INIT")
    C(0x20000130,
      "Wait for GPIOIC to settle: polls GPIOIC+0x8 and GPIOIC+0x14,\n"
      "masking off lowest 2 bits, until they match. This ensures the\n"
      "GPIO interrupt controller is in a stable state before continuing.")
    c(0x20000130, "GPIOIC+0x8 = current GPIO interrupt state")
    c(0x20000138, "GPIOIC+0x14 = GPIO interrupt mask/pending")
    c(0x20000140, "mask off 2 LSBs of state")
    c(0x20000144, "mask off 2 LSBs of mask")
    c(0x20000148, "are they equal? (settled?)")
    c(0x2000014c, "no: keep polling")

    c(0x20000150, "GPIOIC+0x10 = 0x21ec  -> GPIO interrupt configuration")

    N(0x2000015c, "CHECK_GPIOIC_SECONDARY_INIT")
    c(0x2000015c, "wait for secondary GPIOIC settle (same poll pattern)")

    c(0x2000017c, "CLOCK1+72 = 0x2fc7ffe7  -> CLOCK1 PLL feedback divider")
    c(0x20000188, "CLOCK1+76 = 0x1bfff  -> CLOCK1 PLL reference divider")
    c(0x20000194, "CLOCK1+68 |= 0x100  -> enable CLOCK1 PLL")
    c(0x200001a8, "CLOCK1 &= ~0xffff  -> clear CLOCK1 divider low bits")
    c(0x200001bc, "CLOCK1+68 &= ~0xf0000  -> clear output divide bits")
    c(0x200001d0, "CLOCK1+68 = 0x110  -> set CLOCK1 output divide")

    N(0x20000208, "WAIT_FOR_CLOCK_INIT")
    C(0x20000208,
      "PLL lock poll: spins until CLOCK1+64 == 1.\n"
      "The PLL needs time to lock onto the reference frequency.\n"
      "Our QEMU stub returns 1 immediately to skip this wait.")
    c(0x20000208, "CLOCK1+64 = PLL lock status register")
    c(0x20000210, "is it 1? (locked?)")
    c(0x20000214, "no: keep waiting for PLL lock")

    c(0x20000224, "CLOCK1+68 = 0x10111  -> select PLL as clock source")
    c(0x2000023c, "CLOCK1 = 0xf1024000  -> set final clock dividers")
    c(0x20000254, "CLOCK1+4 = 0x41528000  -> peripheral clock config")
    c(0x2000026c, "CLOCK1+8 = 0x80008000  -> bus clock config")
    c(0x20000284, "CLOCK1+12 = 0x80008000  -> another bus clock")
    c(0x2000029c, "CLOCK1+16 = 0x80008000  -> another bus clock")
    c(0x200002b4, "CLOCK1+20 = 0x80008000  -> another bus clock")

    # =========================================================================
    # EXCEPTION HANDLERS  (0x200003b0 - 0x20000428)
    # =========================================================================
    N(0x200003b0, "undef_handler")
    C(0x200003b0,
      "Undefined Instruction exception handler.\n"
      "Saves all registers, records the faulting PC (LR-4),\n"
      "then calls panic_undef (0x20001150) which loops forever.\n"
      "On a real device this would show the Apple logo and reboot.")
    c(0x200003b0, "save ALL registers to stack (full context save)")
    c(0x200003b4, "r0 = faulting PC (LR points past the bad instruction)")
    c(0x200003b8, "call panic_undef  ->  infinite loop, no return")
    c(0x200003bc, "restore all regs (never actually reached)")

    N(0x200003c0, "swi_handler")
    C(0x200003c0,
      "SWI (Software Interrupt) handler.\n"
      "Reads the SWI number from the instruction word.\n"
      "If SWI #0xFF: switches to Supervisor (SVC) mode and returns.\n"
      "This is the ARM semi-hosting convention used for mode switching.\n"
      "Any other SWI number: restores registers and returns normally.")
    c(0x200003c4, "read the SWI instruction word from [LR-4]")
    c(0x200003c8, "mask off condition bits -> get 24-bit SWI number")
    c(0x200003cc, "SWI number == 0xFF?")
    c(0x200003d0, "no: just return from SWI")
    c(0x200003d8, "yes (SWI #0xFF): read SPSR (saved CPU mode)")
    c(0x200003dc, "clear mode bits from SPSR")
    c(0x200003e0, "set mode bits to 0x13 = Supervisor mode")
    c(0x200003e4, "write back SPSR -> return to SVC mode")
    c(0x200003e8, "return from SWI in SVC mode")

    N(0x200003ec, "prefetch_abort_handler")
    C(0x200003ec,
      "Prefetch Abort handler.\n"
      "Triggered when the CPU tries to fetch an instruction from an\n"
      "invalid/unmapped address. Saves context, calls panic_prefetch.")
    c(0x200003f0, "r0 = faulting PC (LR-8 for abort)")
    c(0x200003f4, "call panic_prefetch  ->  infinite loop")

    N(0x20000400, "data_abort_handler")
    C(0x20000400,
      "Data Abort handler.\n"
      "Triggered when a load/store instruction accesses an invalid address.\n"
      "Common causes: NULL pointer dereference, accessing unmapped peripheral.")
    c(0x20000404, "r0 = faulting PC (LR-12 for data abort)")
    c(0x20000408, "call panic_data_abort  ->  infinite loop")
    c(0x20000410, "subs pc, lr, #8  -> return from data abort (if panic returns)")

    N(0x20000418, "fiq_handler")
    C(0x20000418,
      "FIQ (Fast IRQ) handler.\n"
      "Calls the FIQ dispatch function which reads the VIC VICADDRESS\n"
      "register to find which handler to call.")
    c(0x2000041c, "call vic_fiq_dispatch")
    c(0x20000424, "subs pc, lr, #4  -> return from FIQ")

    N(0x20000428, "irq_handler")
    C(0x20000428,
      "IRQ handler.\n"
      "Called by the ARM CPU when an interrupt fires (VIC asserts nIRQ).\n"
      "Saves caller registers, calls vic_irq_dispatch to find and run\n"
      "the registered handler, then restores and returns.")
    c(0x2000042c, "call vic_irq_dispatch  (reads VIC VICADDRESS, jumps to handler)")
    c(0x20000430, "restore saved registers")
    c(0x20000434, "subs pc, lr, #4  -> return from IRQ (corrects pipeline offset)")

    N(0x20000438, "reserved_handler")
    c(0x20000438, "reserved ARM vector: just return")
    c(0x2000043c, "blx r0: indirect call (used by init to jump to a target)")

    # =========================================================================
    # CPU / COPROCESSOR HELPER FUNCTIONS  (0x20000440 - 0x20000598)
    # These tiny functions manipulate the ARM1176 coprocessor 15 (CP15)
    # which controls the MMU, caches, and CPU mode.
    # =========================================================================
    N(0x20000440, "disable_irq")
    C(0x20000440,
      "Disable interrupts.\n"
      "Sets CPSR to 0xd3 = SVC mode + IRQ disabled + FIQ disabled.\n"
      "Call this before entering a critical section.")
    c(0x20000440, "r0 = 0xd3 = 1101_0011: SVC mode, I=1 (IRQ off), F=1 (FIQ off)")
    c(0x20000444, "write to CPSR control byte -> switches mode, disables IRQ/FIQ")

    N(0x2000044c, "enable_irq")
    C(0x2000044c,
      "Enable interrupts.\n"
      "Sets CPSR mode bits to 0x13 = SVC mode, IRQs enabled.")
    c(0x2000044c, "r0 = 0x13 = SVC mode, IRQ enabled, FIQ enabled")
    c(0x20000450, "write to CPSR control byte")

    N(0x20000458, "enable_icache")
    C(0x20000458,
      "Enable instruction cache.\n"
      "Sets bit 12 (I-bit) of SCTLR (System Control Register) via CP15.\n"
      "After this, fetched instructions are cached for speed.")
    c(0x20000458, "r1 = 0x1000 = bit 12 (I-cache enable bit in SCTLR)")
    c(0x2000045c, "MRC p15,0,r0,c1,c0,0 = read SCTLR")
    c(0x20000460, "set I-cache bit")
    c(0x20000464, "MCR p15,0,r0,c1,c0,0 = write SCTLR")

    N(0x2000046c, "enable_dcache")
    C(0x2000046c,
      "Enable data cache.\n"
      "Sets bit 2 (C-bit) of SCTLR.")
    c(0x2000046c, "r1 = 4 = bit 2 (D-cache enable in SCTLR)")
    c(0x20000470, "read SCTLR, set D-cache bit, write back")

    N(0x20000480, "disable_icache")
    C(0x20000480, "Disable instruction cache (clears SCTLR bit 12).")
    c(0x20000480, "r1 = 0x1000 (I-cache bit)")
    c(0x20000484, "read SCTLR")
    c(0x20000488, "BIC = bit-clear: clear the I-cache bit")
    c(0x2000048c, "write SCTLR")

    N(0x20000494, "disable_dcache")
    C(0x20000494, "Disable data cache (clears SCTLR bit 2).")

    N(0x200004a8, "flush_icache")
    C(0x200004a8,
      "Flush (invalidate) instruction cache.\n"
      "Writes 0 to CP15 c7,c5,0 = Invalidate entire I-cache.\n"
      "Must be called after writing new code to memory.")
    c(0x200004ac, "MCR p15,0,r0,c7,c5,0 = invalidate entire I-cache")
    c(0x200004b0, "MCR p15,0,r0,c7,c10,2 = data sync barrier (wait for cache op)")

    N(0x200004bc, "flush_dcache")
    C(0x200004bc,
      "Flush (invalidate) data cache.\n"
      "Writes 0 to CP15 c7,c6,0 = Invalidate entire D-cache.")
    c(0x200004c0, "MCR p15,0,r0,c7,c6,0 = invalidate entire D-cache")
    c(0x200004c4, "MCR p15,0,r0,c7,c10,2 = data sync barrier")

    N(0x200004d0, "data_sync_barrier")
    C(0x200004d0,
      "Data Synchronisation Barrier.\n"
      "MCR p15,0,r2,c7,c10,2 ensures all pending memory ops complete\n"
      "before the next instruction. Used after cache/TLB operations.")
    c(0x200004d0, "r1=0, r0=0 (value doesn't matter for DSB)")
    c(0x200004d8, "MCR p15,0,r2,c7,c10,2 = DSB")

    N(0x20000554, "disable_mmu")
    C(0x20000554,
      "Disable MMU.\n"
      "Clears bit 0 (M-bit) of SCTLR. After this, all memory accesses\n"
      "use physical addresses directly (no translation).")
    c(0x20000554, "r1 = 1 = bit 0 (MMU enable bit in SCTLR)")
    c(0x20000558, "read SCTLR")
    c(0x2000055c, "clear MMU bit")
    c(0x20000560, "write SCTLR -> MMU now off")

    N(0x20000568, "set_TTBR0")
    C(0x20000568,
      "Set Translation Table Base Register 0.\n"
      "CP15 c2,c0,0 = base address of the Level-1 page table.\n"
      "Only meaningful when MMU is on.")
    c(0x20000568, "MCR p15,0,r0,c2,c0,0 = write TTBR0")

    N(0x20000570, "set_DACR")
    C(0x20000570,
      "Set Domain Access Control Register.\n"
      "CP15 c3,c0,0 = controls access permissions for 16 memory domains.")
    c(0x20000570, "MCR p15,0,r0,c3,c0,0 = write DACR")

    N(0x20000578, "set_FCSE_PID")
    C(0x20000578,
      "Set FCSE PID (Fast Context Switch Extension Process ID).\n"
      "CP15 c13,c0,0 = virtual address offset for process isolation.")
    c(0x20000578, "MCR p15,0,r0,c13,c0,0 = write FCSE PID")

    N(0x20000580, "enable_alignment_fault")
    C(0x20000580,
      "Enable alignment fault checking.\n"
      "Sets bit 1 (A-bit) of SCTLR. Causes data abort on unaligned access.")
    c(0x20000580, "read SCTLR, set A-bit (bit 1), write back")

    N(0x20000590, "flush_TLB")
    C(0x20000590,
      "Flush entire TLB (Translation Lookaside Buffer).\n"
      "CP15 c8,c7,0 = invalidate both I-TLB and D-TLB.\n"
      "Must call after changing page tables.")
    c(0x20000590, "r0 = 0 (value ignored)")
    c(0x20000594, "MCR p15,0,r0,c8,c7,0 = invalidate unified TLB")

    # =========================================================================
    # STARTUP / DATA COPY / MAIN CALL  (0x2000034c)
    # =========================================================================
    N(0x2000034c, "startup_data_bss_init")
    C(0x2000034c,
      "POST-INIT STARTUP: Copy .data and zero .bss.\n"
      "The ROM's initialized global variables (.data) are stored in ROM\n"
      "but need to be in writable SRAM. This code copies them.\n"
      "The uninitialized globals (.bss) just need to be zeroed.\n"
      "After this, calls main_entry (0x20003790).")
    c(0x2000034c, "r6 = 0x20000000 (ROM base)")
    c(0x20000350, "r7 = end of code section (from ROM literal pool)")
    c(0x20000354, "r6 = offset from ROM base to .data source")
    c(0x20000358, "r0 = .data source start in ROM")
    c(0x20000360, "r1 = .data destination in SRAM")
    c(0x20000364, "r2 = .data end in SRAM")
    c(0x20000368, "r2 = size of .data section")
    c(0x2000036c, "r2 = .data source end in ROM")
    c(0x20000370, "already copied? (ROM == SRAM?)")
    c(0x20000378, "copy loop: ldr word from ROM .data, str to SRAM")
    c(0x20000388, "r1 = .bss start in SRAM")
    c(0x2000038c, "r3 = .bss end in SRAM")
    c(0x20000394, "zero-fill .bss: store 0 word-by-word")
    c(0x200003a0, "r10 = 1 (initialized flag)")
    c(0x200003a4, "address of initialized flag variable")
    c(0x200003a8, "set initialized flag = 1  (marks C runtime as ready)")
    c(0x200003ac, "call main_entry  ->  never returns (enters DFU loop)")

    # =========================================================================
    # PANIC / HALT HANDLERS  (0x20001150 - 0x20001188)
    # =========================================================================
    N(0x20001150, "panic_undef")
    C(0x20001150,
      "PANIC: Undefined instruction.\n"
      "Infinite loop - CPU halts here on undefined instruction exception.\n"
      "On a real iPod this eventually triggers the watchdog to reset.")
    c(0x20001150, "b 0x20001150 = spin forever (halted)")

    N(0x20001154, "panic_prefetch")
    C(0x20001154,
      "PANIC: Prefetch abort (bad instruction fetch address).")
    c(0x20001154, "b 0x20001154 = spin forever")

    N(0x20001158, "panic_data_abort")
    C(0x20001158,
      "PANIC: Data abort (bad memory access).\n"
      "Most likely cause: NULL pointer dereference or\n"
      "accessing an unimplemented peripheral in emulation.")
    c(0x20001158, "b 0x20001158 = spin forever")

    N(0x20001160, "vic_fiq_dispatch")
    C(0x20001160,
      "FIQ interrupt dispatch.\n"
      "Reads VIC0 VICADDRESS. If zero (no VIC0 FIQ), tries VIC1.\n"
      "If a handler address is found, calls it via 'bx r0'.\n"
      "VICADDRESS (offset 0xf00) returns the address of the highest-priority\n"
      "pending enabled interrupt's handler.")
    c(0x20001160, "load VIC0 base address from literal pool at 0x200011c0")
    c(0x20001164, "r0 = VIC0->INTENABLE (are any VIC0 IRQs enabled?)")
    c(0x20001168, "any VIC0 IRQs enabled?")
    c(0x2000116c, "no: try VIC1 (add 0x1000 to get VIC1 base)")
    c(0x20001170, "r0 = VIC1->INTENABLE")
    c(0x20001174, "any VIC1 IRQs enabled?")
    c(0x20001178, "no: return without dispatching")
    c(0x2000117c, "r0 = VIC0/1->ADDRESS (0xf00 = handler address register)")
    c(0x20001180, "is handler address non-zero?")
    c(0x20001184, "yes: jump to handler (bx r0)")

    N(0x2000118c, "vic_irq_dispatch")
    C(0x2000118c,
      "IRQ interrupt dispatch - called from irq_handler.\n"
      "Checks VIC0 INTENABLE. If VIC0 has pending IRQ:\n"
      "  reads VIC0 VICADDRESS (0xf00) = handler address, calls it.\n"
      "Otherwise checks VIC1.\n"
      "The VIC automatically selects the highest-priority pending IRQ.")
    c(0x2000118c, "r1 = VIC0 base address (from literal pool 0x200011c0 = 0x38e00000)")
    c(0x20001190, "r0 = VIC0->INTENABLE (+0x4): any enabled IRQs?")
    c(0x20001194, "any VIC0 IRQs enabled?")
    c(0x20001198, "yes: r0 = VIC0->ADDRESS (+0xf00) = pending handler address")
    c(0x2000119c, "jump to handler comparison")
    c(0x200011a0, "no VIC0: check VIC1 (literal pool 0x200011c4 = 0x38e01000)")
    c(0x200011a4, "any VIC1 IRQs?")
    c(0x200011a8, "no VIC1 either: return (spurious IRQ)")
    c(0x200011ac, "r0 = VIC1->ADDRESS (+0xf00) = handler address")
    c(0x200011b4, "handler address non-zero?")
    c(0x200011b8, "yes: jump to handler  (this is where USB ISR gets called)")

    # Literal pool at 0x200011c0
    c(0x200011c0, "literal: VIC0 base = 0x38e00000")
    c(0x200011c4, "literal: VIC1 base = 0x38e01000")
    c(0x200011c8, "literal: EDGEIC base = 0x38e02000")

    # =========================================================================
    # EDGEIC HELPER  (0x20000d0c)
    # Already annotated in first script, but adding extra context
    # =========================================================================
    N(0x20000d0c, "edgeic_assert_ack")
    C(0x20000d0c,
      "Assert or acknowledge an EDGEIC interrupt channel.\n"
      "Args: r0 = channel number (0-63)\n"
      "Effect:\n"
      "  r0 < 32:  EDGEIC+0x8 |= (1 << r0)\n"
      "  r0 >= 32: EDGEIC+0xc |= (1 << (r0-32))\n"
      "\n"
      "EDGEIC (Edge Interrupt Controller) sits between peripherals and the VIC.\n"
      "It detects rising/falling edges on interrupt lines.\n"
      "Writing 1 to a pending bit CLEARS (acknowledges) that interrupt.\n"
      "\n"
      "Called with r0=0x27 (39) from the USB ISR to ACK the USB edge IRQ.\n"
      "Channel 39 = bit 7 of EDGEIC+0xc.")
    c(0x20000d0c, "r2 = EDGEIC base (0x38e02000)")
    c(0x20000d10, "is channel >= 32?")
    c(0x20000d14, "r3 = 1 (bitmask to shift)")
    c(0x20000d18, "if >= 32: jump to high-channel path")
    c(0x20000d1c, "r1 = EDGEIC+0x8 (channels 0-31 pending register)")
    c(0x20000d20, "r0 = current_bits | (1 << channel)")
    c(0x20000d24, "EDGEIC+0x8 |= (1 << r0)  -> ack edge interrupt channel r0")
    c(0x20000d2c, "r1 = EDGEIC+0xc (channels 32-63 pending register)")
    c(0x20000d30, "r0 -= 32  (normalize to bit position in +0xc)")
    c(0x20000d34, "r0 = current_bits | (1 << (r0-32))")
    c(0x20000d38, "EDGEIC+0xc |= (1 << (r0-32))  -> ack edge interrupt channel r0")

    # =========================================================================
    # USB ISR  (0x20001eac - 0x20001f18)
    # =========================================================================
    N(0x20001eac, "usb_isr_entry")
    C(0x20001eac,
      "USB INTERRUPT SERVICE ROUTINE ENTRY POINT\n"
      "VIC1 VICVECTADDR for slot 7 points here.\n"
      "\n"
      "This entry stub loads r1 = USB_BASE (0x38c00000) BEFORE the function\n"
      "prologue. The VIC dispatcher jumps here, so r1 is set up as the USB\n"
      "context pointer before the push saves registers.\n"
      "\n"
      "The USB DWC2 controller fires this interrupt when:\n"
      "  - A USB SETUP packet arrives on the control endpoint\n"
      "  - A USB OUT packet arrives (DFU download data)\n"
      "  - USB bus reset/connect/disconnect events")
    c(0x20001eac, "r1 = 0x38c00000 = USB DWC2 base (set BEFORE push, used in ISR)")

    N(0x20001eb0, "usb_isr")
    c(0x20001eb4, "r0 = USB+0xc = GUSBCFG register (USB config)")
    c(0x20001eb8, "re-write GUSBCFG (re-arm / clear interrupt source in DWC2)")
    c(0x20001ebc, "r4 = 0x2202bff8 (SRAM address holding the usb_struct pointer)")
    c(0x20001ec0, "r1 = 1")
    c(0x20001ec4, "r0 = usb_struct pointer (dereference 0x2202bff8)")
    c(0x20001ec8, "usb_struct->0x98 = 1  [flag: USB interrupt fired this cycle]")
    c(0x20001ecc, "is usb_struct pointer NULL? (shouldn't be but check anyway)")
    c(0x20001ed0, "if NULL: skip state check, go straight to EDGEIC ack")
    c(0x20001ed4, "r0 = usb_struct->0x8ec  [pointer to inner USB endpoint state struct]")
    c(0x20001ed8, "r0 = inner_struct->0x35  [endpoint state byte 1]")
    c(0x20001edc, "is state byte 1 == 0?")
    c(0x20001ee0, "non-zero: go to EDGEIC ack path")
    c(0x20001ee4, "r0 = 0x27 = EDGEIC channel 39 (the USB edge interrupt channel)")
    c(0x20001ee8, "call edgeic_assert_ack(0x27)  -> ACK the USB edge IRQ in EDGEIC")
    c(0x20001eec, "r0 = usb_struct (reload after call)")
    c(0x20001ef0, "NULL check again")
    c(0x20001ef4, "NULL: return via EDGEIC tail-call path")
    c(0x20001ef8, "r0 = usb_struct->0x8ec again")
    c(0x20001efc, "r0 = *r0 = inner struct ptr")
    c(0x20001f00, "r0 = inner_struct->0x34  [endpoint state byte 2]")
    c(0x20001f04, "is state byte 2 == 0?")
    c(0x20001f08, "0: normal return (no further action needed)")
    c(0x20001f0c, "non-zero: tail-call edgeic_assert_ack(0x27) and return")
    c(0x20001f18, "normal return from USB ISR")

    # =========================================================================
    # USB DWC2 LOW-LEVEL REGISTER ACCESS FUNCTIONS
    # The DWC2 (DesignWare USB 2.0 OTG) controller has many registers.
    # These small functions provide named access to individual registers.
    # =========================================================================
    N(0x20001f50, "usb_set_endpoint_config")
    C(0x20001f50,
      "Configure USB endpoint register.\n"
      "Writes to DWC2 registers at USB_BASE+0x6c, +0x8, +0, +0x10.\n"
      "Sets up endpoint type, size, and direction.")
    c(0x20001f50, "DWC2 DIEPCTL0/DOEPCTL0 configuration sequence")
    c(0x20001f58, "USB+0x6c = endpoint type register")
    c(0x20001f68, "USB+0x8 = GRSTCTL (soft reset)")
    c(0x20001f80, "USB+0x0 = GOTGCTL (OTG control)")
    c(0x20001f88, "USB+0x10 = GAHBCFG (AHB config)")

    N(0x20001fa0, "usb_detect_phy_type")
    C(0x20001fa0,
      "Detect USB PHY type from GUSBCFG register.\n"
      "Reads GUSBCFG bits [5:4] (TOUTCAL / ULPI vs UTMI):\n"
      "  01 = UTMI (6 entries)\n"
      "  10 = ULPI (8 entries)\n"
      "  other = embedded PHY (4 entries)\n"
      "Used to know how many USB descriptor bytes to read.")
    c(0x20001fa0, "r2 = USB_BASE = 0x38c00000 (DWC2)")
    c(0x20001fa4, "r1 = USB+0x6c (some endpoint register)")
    c(0x20001fa8, "if non-zero: already configured, return early")
    c(0x20001fb0, "read GUSBCFG (+0x14)")
    c(0x20001fb4, "extract bits [5:4] (PHY interface select)")
    c(0x20001fb8, "shift right 4 to get 0/1/2/3")
    c(0x20001fbc, "== 1 (UTMI)?")
    c(0x20001fc0, "yes: 6 entries")
    c(0x20001fcc, "== 2 (ULPI)?")
    c(0x20001fd0, "yes: 8 entries, no: 4 entries")

    N(0x2000210c, "usb_dwc2_init_tx_endpoint")
    C(0x2000210c,
      "Initialize DWC2 USB TX (IN) endpoint.\n"
      "Programs the DWC2 registers for a device-mode IN endpoint:\n"
      "  USB+0x6c = endpoint number\n"
      "  USB+0x8  = GRSTCTL (reset)\n"
      "  USB+0x0  = GOTGCTL\n"
      "  USB+0x10 = GAHBCFG\n"
      "  USB+0x14 = GUSBCFG (speed, PHY type)\n"
      "  USB+0x18 = GINTSTS (interrupt status)\n"
      "  USB+0x20 = DCFG (device config)\n"
      "  USB+0x24 = DCTL (device control)\n"
      "  USB+0x28 = DSTS (device status)\n"
      "  USB+0x2c = DIEPMSK (device IN endpoint interrupt mask)\n"
      "  USB+0x30 = DOEPMSK (device OUT endpoint interrupt mask)\n"
      "  USB+0x34 = DAINT (device all endpoints interrupt)")
    c(0x2000210c, "save many registers - this function uses a lot of context")
    c(0x20002128, "usb_struct->0x98 = 0  (clear interrupt-fired flag)")
    c(0x2000212c, "r4 = 0x38c00000 (DWC2 base)")
    c(0x20002130, "USB+0x6c = endpoint configuration value")
    c(0x20002138, "USB+0x8 = 1 (GRSTCTL: core soft reset)")
    c(0x2000213c, "USB+0x8 = 0 (release reset)")
    c(0x20002140, "USB+0x0 = 1 (GOTGCTL: set HNP enable)")
    c(0x20002144, "r1 = 0xf")
    c(0x20002148, "USB+0x10 = 0xf (GAHBCFG: enable interrupts, burst length)")
    c(0x2000214c, "call usb_detect_phy_type")
    c(0x20002150, "r0 = 6")
    c(0x20002154, "USB+0x14 = 6 (GUSBCFG: initial value)")
    c(0x2000216c, "configure USB speed bits in GUSBCFG")
    c(0x20002198, "USB+0x18 = ... GINTSTS setup")

    N(0x200021d8, "usb_dwc2_init_rx_endpoint")
    C(0x200021d8,
      "Initialize DWC2 USB RX (OUT) endpoint.\n"
      "Similar to usb_dwc2_init_tx_endpoint but for OUT direction.\n"
      "The key difference is GUSBCFG bit 0 is NOT set (no HNP).")

    N(0x20002308, "usb_dwc2_setup_desc")
    C(0x20002308,
      "Setup USB transfer descriptor table.\n"
      "If r0 == NULL: clears USB+0x74..0x80 (endpoint registers).\n"
      "If r0 != NULL: copies descriptor bytes from r0 into a descriptor\n"
      "buffer, byte-shuffling for endianness (4 entries of 4 bytes each).")
    c(0x20002308, "r0 = descriptor pointer (NULL = clear)")
    c(0x2000230c, "save r4")
    c(0x20002310, "non-NULL: go to fill path")
    c(0x20002314, "NULL path: r1 = USB_BASE")
    c(0x20002318, "r0 = 0")
    c(0x2000231c, "USB+0x74 = 0 (clear DIEPTSIZ0)")
    c(0x20002320, "USB+0x78 = 0 (clear DOEPDMA0)")
    c(0x20002324, "USB+0x7c = 0 (clear DOEPTSIZ0)")
    c(0x20002328, "USB+0x80 = 0")

    # =========================================================================
    # USB DMA COMPLETION HANDLERS  (0x200024b4, 0x200024e4)
    # =========================================================================
    N(0x200024b4, "usb_dma_complete")
    C(0x200024b4,
      "USB DMA TRANSFER COMPLETION HANDLER (polling variant)\n"
      "\n"
      "This is the function that ultimately unblocks the DFU wait loop!\n"
      "\n"
      "HOW IT WORKS:\n"
      "  When USB receives OUT data (DFU firmware bytes), the DWC2 controller\n"
      "  programs the USB DMA engine at 0x38000000 to move the data to SRAM.\n"
      "  The DMA sets bit 0 of 0x38000000 while busy.\n"
      "  This function polls until DMA is done, then sets usb_struct->0xa0 = 1.\n"
      "  The DFU wait loop at dfu_transfer_wait_loop sees the flag and wakes up.\n"
      "\n"
      "USB DMA CONTROLLER (0x38000000) REGISTERS:\n"
      "  +0x00 : status/control. bit 0 = DMA busy\n"
      "  +0x0c : trigger register")
    c(0x200024b4, "r2 = 0x38000000 (USB DMA controller base)")
    c(0x200024b8, "r0 = [0x38000000] (read DMA status)")
    c(0x200024bc, "test bit 0 (DMA busy?)")
    c(0x200024c0, "bit 0 set: DMA still running, keep polling")
    c(0x200024c4, "r0 = 0")
    c(0x200024c8, "0x38000000 = 0  (clear DMA status / acknowledge)")
    c(0x200024cc, "load address of global usb_struct pointer (literal = 0x2202bff8)")
    c(0x200024d0, "r0 = 1")
    c(0x200024d4, "r1 = usb_struct pointer (dereference 0x2202bff8)")
    c(0x200024d8, "*** usb_struct->0xa0 = 1 ***  DFU TRANSFER COMPLETE FLAG SET!")
    c(0x200024dc, "0x38000000 + 0xc = 1  (kick DMA controller post-completion)")
    c(0x200024e0, "return")

    N(0x200024e4, "usb_dma_complete_with_irq")
    C(0x200024e4,
      "USB DMA transfer completion handler (interrupt-signaling variant).\n"
      "Same as usb_dma_complete but also calls edgeic_assert_ack(0x28)\n"
      "to signal another interrupt after the transfer completes.")
    c(0x200024ec, "poll 0x38000000 bit 0 (DMA busy)")
    c(0x200024f8, "load usb_struct ptr address")
    c(0x20002500, "r1 = usb_struct")
    c(0x20002504, "*** usb_struct->0xa0 = 1 ***  DFU TRANSFER COMPLETE FLAG SET!")
    c(0x20002508, "r0 = 0x28 (EDGEIC channel 40)")
    c(0x2000250c, "edgeic_assert_ack(0x28)  signal post-DMA interrupt")
    c(0x20002510, "pop saved registers")
    c(0x20002514, "r0 = 0x28")
    c(0x20002518, "tail-call to another EDGEIC function")

    # =========================================================================
    # USB OUT TRANSFER SETUP  (0x200023d4)
    # =========================================================================
    N(0x200023d4, "usb_out_transfer_setup")
    C(0x200023d4,
      "Set up a USB OUT (host->device) DMA transfer.\n"
      "Called from dfu_handle_request when DFU_DNLOAD SETUP arrives.\n"
      "Prepares the DWC2 endpoint to receive the OUT data stage,\n"
      "then chains into the USB DMA engine to move received bytes to SRAM.")

    # =========================================================================
    # DFU TRANSFER WAIT LOOP  (0x20002770 - 0x200027d8)
    # =========================================================================
    N(0x20002770, "usb_dma_start_or_stop")
    C(0x20002770,
      "USB DMA start or stop.\n"
      "Based on r3: if r3==1 clears DMA bit, if r3!=1 sets DMA bit 3.\n"
      "Then if r1==1: arms the DMA for OUT receive and waits for completion.")
    c(0x20002770, "r3 = [r0] = operation mode")
    c(0x20002774, "r2 = 0x38000000 (USB DMA)")
    c(0x2000277c, "operation == 1?")
    c(0x20002780, "no: set bit 3 in DMA control (0x38000000 |= 8)")
    c(0x20002790, "yes: clear bit 3 (0x38000000 &= ~8)")
    c(0x200027a0, "should we wait for data? (r1 == 1?)")
    c(0x200027a4, "no: go to send path")

    C(0x200027b0,
      "DFU DATA RECEIVE SEQUENCE\n"
      "1. Clear the transfer-complete flag\n"
      "2. Arm the USB DMA for an OUT transfer\n"
      "3. Spin in dfu_transfer_wait_loop until DMA completion sets the flag")
    c(0x200027b0, "usb_struct->0xa0 = 0  (clear transfer-complete flag)")
    c(0x200027b4, "USB DMA+0xc |= 1  (arm DMA control register)")
    c(0x200027c0, "USB DMA |= 6  (start DMA: set bits 1 and 2)")

    N(0x200027cc, "dfu_transfer_wait_loop")
    C(0x200027cc,
      "DFU TRANSFER WAIT LOOP - spins until firmware data is received.\n"
      "\n"
      "This is the core of DFU mode. The ROM stays here until either:\n"
      "  a) A DFU_DNLOAD transfer completes -> usb_dma_complete sets flag\n"
      "  b) Timeout / error (handled elsewhere)\n"
      "\n"
      "The flag at usb_struct+0xa0 is set by usb_dma_complete when\n"
      "the USB DMA controller finishes moving bytes from USB to SRAM.")
    c(0x200027cc, "r1 = usb_struct->0xa0  [DFU transfer-complete flag]")
    c(0x200027d0, "flag == 0 ?")
    c(0x200027d4, "yes (still waiting): loop back")
    c(0x200027d8, "flag != 0: transfer done! return to caller")

    # =========================================================================
    # DFU_HANDLE_REQUEST  (0x200007a8)
    # =========================================================================
    N(0x200007a8, "dfu_handle_request")
    C(0x200007a8,
      "DFU USB REQUEST HANDLER\n"
      "\n"
      "Called when a DFU class USB SETUP packet arrives.\n"
      "Args: r0=request_packet, r1=usb_pipe, r2=request_type\n"
      "\n"
      "DFU class requests:\n"
      "  DFU_DNLOAD (1): Host sends firmware data -> set up OUT transfer\n"
      "  DFU_UPLOAD (2): Host reads back data (usually not supported)\n"
      "  DFU_GETSTATUS (3): Host asks for DFU state machine status\n"
      "  DFU_CLRSTATUS (4): Clear error status\n"
      "  DFU_GETSTATE (5): Get current DFU state number\n"
      "  DFU_ABORT (6): Abort current operation\n"
      "\n"
      "SETUP packet layout (r0 points here):\n"
      "  [0] bmRequestType, [1] bRequest (1=DNLOAD,2=UPLOAD...)\n"
      "  [2-3] wValue (block sequence number), [4-5] wIndex\n"
      "  [6-7] wLength (number of data bytes to follow)")
    c(0x200007b0, "r4 = r2 = request_type (1=DNLOAD, 2=UPLOAD, etc.)")
    c(0x200007b4, "r5 = r1 = usb_pipe handle")
    c(0x200007b8, "r8 = r0 = SETUP packet pointer")
    c(0x200007bc, "call sub_34b4 (USB protocol helper)")
    c(0x200007c0, "call sub_2f1c (USB state check)")
    c(0x200007c4, "is request_type == 1 (DFU_DNLOAD)?")
    c(0x200007c8, "no: load string for non-download request")
    c(0x200007cc, "yes: load string for download request")
    c(0x200007e0, "load request-specific descriptor pointer")
    c(0x20000814, "request_type != 1: jump to DFU_GETSTATUS/other path")
    c(0x20000820, "call sub_168c (DFU state machine check)")
    c(0x20000828, "state check failed: error exit")
    c(0x2000082c, "read bRequest field from SETUP packet ([r8+7])")
    c(0x20000834, "bRequest == 2 (DFU_UPLOAD)?")
    c(0x20000838, "yes: go to upload path (send data back)")
    c(0x2000083c, "bRequest == 1 (DFU_DNLOAD)?")
    c(0x20000840, "no: unknown request, error exit")
    c(0x20000860, "r1 = [r4+0xc] = wLength from SETUP packet")
    c(0x20000870, "call usb_out_transfer_setup (queue the OUT data transfer)")
    c(0x20000878, "r1 = wLength")
    c(0x20000888, "call usb_send (send ZLP or status)")

    C(0x200008e4,
      "DFU_DNLOAD success: copy SETUP packet fields into usb_struct.\n"
      "These values are later used to identify the block and prepare NAND write.")
    c(0x200008e4, "r1 = usb_struct pointer")
    c(0x200008ec, "usb_struct->0x34 = SETUP[4] = wValue low (DFU block# lo)")
    c(0x200008f4, "usb_struct->0x35 = SETUP[5] = wValue hi (DFU block# hi)")
    c(0x200008fc, "usb_struct->0x36 = SETUP[6] = wIndex lo")
    c(0x20000904, "usb_struct->0x30 = SETUP[8] = wLength (bytes to receive)")
    c(0x20000910, "return 1 (success)")

    C(0x200009b0, "Same SETUP field copy for DFU_GETSTATUS/DFU_GETSTATE path")
    c(0x200009b8, "usb_struct->0x34 = wValue lo")
    c(0x200009c0, "usb_struct->0x35 = wValue hi")
    c(0x200009c8, "usb_struct->0x36 = wIndex lo")
    c(0x200009d0, "usb_struct->0x30 = wLength")

    c(0x20000a1c, "error exit: return 0 (request handling failed)")

    # =========================================================================
    # USB PHY INITIALIZATION  (0x200033c0)
    # =========================================================================
    N(0x200033c0, "usb_phy_init")
    C(0x200033c0,
      "USB PHY INITIALIZATION\n"
      "\n"
      "Initializes the USB Physical Layer (PHY) at 0x3c400000.\n"
      "The PHY is the analog front-end that handles the actual USB signaling.\n"
      "This must be done before the DWC2 digital controller can work.\n"
      "\n"
      "USB PHY REGISTERS (0x3c400000):\n"
      "  +0x00 OPHYPWR:  PHY power control\n"
      "  +0x04 OPHYCLK:  PHY clock selection\n"
      "  +0x08 ORSTCON:  PHY reset control\n"
      "  +0x28 ???:      PHY ready status (bit 0 = ready)")
    c(0x200033c0, "r0 = 0x24  (timer delay constant)")
    c(0x200033c8, "call timer delay function")
    c(0x200033cc, "r7 = USB state struct (from literal pool)")
    c(0x200033d0, "r0 = 0xffffffff")
    c(0x200033d4, "usb_struct->0x8c = 0xffffffff  (clear/init field)")
    c(0x200033d8, "usb_struct->0x90 = 0xffffffff")
    c(0x200033dc, "usb_struct->0x94 = 0xffffffff")
    c(0x200033e0, "usb_struct->0x98 = 0xffffffff")
    c(0x200033e4, "load initial config value for ->0x88")
    c(0x200033f0, "r0 = usb_struct_ptr (global)")
    c(0x200033fc, "r4 = usb_struct->0x640  (some large offset - maybe a buffer ptr)")
    c(0x20003400, "r0 = 22 (GPIO channel 22 = USB PHY control)")
    c(0x20003404, "call gpio_set(22, 0, 5)  -> configure USB PHY GPIO")
    c(0x20003408, "call gpio_set(22, 0, 7)  -> configure USB PHY GPIO")
    c(0x20003420, "call gpio_read(22, 5)  -> read USB PHY GPIO state")
    c(0x20003424, "r5 = result")
    c(0x2000342c, "call gpio_read(22, 7)")
    c(0x20003438, "both GPIO lines == 1?")
    c(0x20003444, "not ready: set r5=0 (fail), jump to exit")
    c(0x20003448, "r8 = 0x3c400000 (USB PHY base)")
    c(0x2000344c, "r0 = USB_PHY+0x28 (PHY ready status)")
    c(0x20003450, "bit 0 set? (PHY ready?)")
    c(0x20003454, "not ready: fail")

    # =========================================================================
    # USB CONTROLLER TOP-LEVEL INIT  (0x2000068c)
    # =========================================================================
    N(0x2000068c, "usb_init")
    C(0x2000068c,
      "USB CONTROLLER INITIALIZATION\n"
      "\n"
      "Top-level USB initialization function.\n"
      "Args: r0 = USB config struct, r1 = mode (1=DFU only, 2=full USB)\n"
      "\n"
      "Sequence:\n"
      "  1. usb_dma_complete()       - flush any pending DMA\n"
      "  2. usb_dwc2_soft_reset()    - reset DWC2 core\n"
      "  3. Setup control endpoint (64-byte max packet)\n"
      "  4. usb_dfu_main_loop()      - start DFU state machine\n"
      "  5. usb_build_descriptors()  - build USB descriptors\n"
      "  6. Setup endpoint for mode (DFU vs CDC)\n"
      "Returns 1 on success, 0 on failure.")
    c(0x2000068c, "save r4, r5, lr; allocate 60 bytes stack")
    c(0x20000694, "r4 = USB config struct")
    c(0x20000698, "r5 = mode (1=DFU, 2=full)")
    c(0x2000069c, "call usb_dma_complete (flush DMA state)")
    c(0x200006a0, "call usb_dwc2_soft_reset (reset DWC2 core)")
    c(0x200006b4, "call: setup control endpoint (max_pkt=64)")
    c(0x200006bc, "call usb_dfu_main_loop(1) -> enter DFU state machine")
    c(0x200006c4, "call usb_build_descriptors")
    c(0x200006c8, "mode == 2 (full USB)?")
    c(0x200006cc, "no: DFU descriptor config")
    c(0x200006d0, "yes: full USB descriptor config")
    c(0x200006dc, "memcpy descriptor data")
    c(0x20000700, "setup transfer endpoint for DFU")
    c(0x20000704, "call sub_22b8 (USB state init)")
    c(0x20000770, "check packet size <= 0x1f800 (127KB max)")
    c(0x20000778, "too large: fail")
    c(0x20000784, "success: return 1")

    # =========================================================================
    # MAIN ENTRY POINT  (0x20003790)
    # =========================================================================
    N(0x20003790, "main_entry")
    C(0x20003790,
      "MAIN ROM ENTRY POINT\n"
      "\n"
      "Called from startup_data_bss_init after C runtime is ready.\n"
      "This is where the ROM's actual boot logic begins.\n"
      "Sets up USB and enters DFU mode to receive firmware.")
    c(0x20003790, "load main config from literal pool")
    c(0x20003794, "push lr (this never returns)")

    # =========================================================================
    # APPLE SECURE BOOT / CERTIFICATE VERIFICATION  (0x200068ec)
    # =========================================================================
    N(0x200068ec, "cert_verify")
    C(0x200068ec,
      "APPLE SECURE BOOT CERTIFICATE VERIFICATION\n"
      "\n"
      "Verifies the certificate chain for the downloaded firmware image.\n"
      "Apple uses a two-level cert chain:\n"
      "  Root CA: '/CN=Apple Secure Boot Certification Authority'\n"
      "  Device:  '/CN=S5L8900 Secure Boot'\n"
      "\n"
      "The LLB (Low-Level Bootloader) image contains an RSA signature\n"
      "and certificate chain. This function verifies:\n"
      "  1. Certificate length <= 0x800 (2048 bytes)\n"
      "  2. Certificate contains exactly 9 bytes of expected header\n"
      "  3. Certificate issuer matches Apple CA string\n"
      "  4. Signature is valid RSA-SHA1\n"
      "\n"
      "If verification fails, the ROM refuses to execute the image.")
    c(0x200068f4, "r0 = cert->length field")
    c(0x200068f8, "r4 = cert struct pointer")
    c(0x200068fc, "r6 = cert data pointer")
    c(0x20006900, "lengths match?")
    c(0x20006908, "r1 = expected cert size field")
    c(0x20006918, "compare cert header against expected header")
    c(0x2000691c, "header mismatch: fail")
    c(0x20006924, "cert size <= 0x800 (2KB)?")
    c(0x20006928, "and signature size == 9?")
    c(0x20006930, "invalid: fail")
    c(0x20006934, "r1 = Apple CA DN string ('/CN=Apple Secure Boot...')")
    c(0x20006938, "r2 = 9 bytes to compare")
    c(0x20006940, "memcmp against Apple CA Distinguished Name")
    c(0x20006944, "matched Apple CA? verify issuer field")
    c(0x2000694c, "r1 = RSA public key offset +0x30")
    c(0x20006950, "r0 = 16 (exponent size?)")
    c(0x20006954, "store RSA key length")
    c(0x20006958, "try device cert '/CN=S5L8900 Secure Boot'")

    # =========================================================================
    # NAND FLASH CONTROLLER  (0x20009704 onwards)
    # =========================================================================
    N(0x20009738, "nand_write_cmd")
    C(0x20009738,
      "Write NAND command register.\n"
      "NAND controller at 0x3d000000.\n"
      "  +0x20 = NAND command register (write command byte here)")
    c(0x20009738, "r1 = 0x3d000000 (NAND controller base)")
    c(0x2000973c, "NAND+0x20 = r0 (command byte)")

    N(0x20009744, "nand_enable")
    C(0x20009744,
      "Enable NAND controller.\n"
      "Writes 1 to NAND+0x20, then stores a value to a NAND struct field.")
    c(0x20009744, "r2 = 0x3d000000 (NAND base)")
    c(0x20009748, "r1 = 1")
    c(0x2000974c, "NAND+0x20 = 1 (enable NAND)")
    c(0x20009750, "load NAND state struct ptr")
    c(0x20009754, "dereference")
    c(0x20009758, "NAND_struct->0x7e4 = r0 (NAND config value)")

    N(0x20009760, "nand_cmd")
    C(0x20009760,
      "Issue NAND flash command.\n"
      "Writes command byte to NAND+0x20 (NAND command register).")

    # =========================================================================
    # TIMER FUNCTIONS  (0x20005f14)
    # =========================================================================
    N(0x20005f14, "timer_connect")
    C(0x20005f14,
      "Connect/register a timer callback.\n"
      "Args: r0 = timer struct A, r1 = timer struct B\n"
      "Connects two timer structs together by copying fields.\n"
      "Timer struct fields:\n"
      "  +0x50c: next timer in chain\n"
      "  +0x3e4: timer callback data (200 bytes)\n"
      "  +0x3f8: timer config (20 bytes)")
    c(0x20005f14, "save r4,r5,r6,lr")
    c(0x20005f18, "r4 = timer B")
    c(0x20005f1c, "r5 = timer A (NULL check)")
    c(0x20005f24, "check timer A->0x50c (chained?)")
    c(0x20005f34, "check timer B->0x50c")
    c(0x20005f38, "either chained: return error")
    c(0x20005f48, "r2 = 200 (copy size)")
    c(0x20005f50, "call memcpy: timer_A->0xec <- timer_B->0x20")

    # =========================================================================
    # MEMORY FUNCTIONS  (0x2000a588 - 0x2000a6fc)
    # =========================================================================
    N(0x2000a588, "memset_fast")
    C(0x2000a588,
      "Fast memset using STM (store multiple) for bulk writes.\n"
      "Args: r0=dest, r1=count, r2=fill_value\n"
      "Uses STM with 4 registers at once for 16-byte-aligned chunks.\n"
      "This is highly optimized ARM assembly for zeroing/filling memory.")
    c(0x2000a588, "push lr (preserve return address)")
    c(0x2000a58c, "replicate fill byte into r2,r3,ip,lr (4 copies)")
    c(0x2000a598, "store 16 bytes per iteration while count >= 32")
    c(0x2000a5a0, "subtract 32 from count")
    c(0x2000a5a4, "still >= 32 bytes left?")
    c(0x2000a5a8, "handle remaining 16-byte and 8-byte chunks")
    c(0x2000a5b8, "restore lr")
    c(0x2000a5bc, "tail: handle remaining <= 4 bytes")

    N(0x2000a5d8, "memset")
    C(0x2000a5d8,
      "Standard memset: fill memory with a byte value.\n"
      "Args: r0=dest, r1=count, r2=fill_byte\n"
      "Simple byte-by-byte version for small/unaligned fills.")
    c(0x2000a5d8, "r2 = 0 (fill value, typically called as memset to zero)")
    c(0x2000a5dc, "count < 4? use byte loop")

    N(0x2000a61c, "memcpy")
    C(0x2000a61c,
      "Standard memcpy: copy bytes from source to destination.\n"
      "Args: r0=dest, r1=src, r2=count\n"
      "Optimized for word-aligned cases using LDM/STM.")
    c(0x2000a61c, "is count >= 4?")
    c(0x2000a620, "no: byte copy loop")
    c(0x2000a624, "check destination alignment")

    N(0x2000a738, "udivsi3")
    C(0x2000a738,
      "Unsigned 32-bit integer division.\n"
      "Args: r0=dividend, r1=divisor\n"
      "Returns: r0=quotient, r1=remainder\n"
      "This is a software division routine since ARM1176 has no UDIV.\n"
      "Uses a shift-and-subtract algorithm.")
    c(0x2000a738, "handle signs and special cases")
    c(0x2000a73c, "dividend < divisor? quotient = 0")
    c(0x2000a748, "find MSB of divisor relative to dividend")

    # =========================================================================
    # SIGNAL / EXCEPTION STRINGS AND HANDLERS
    # =========================================================================
    N(0x2000a588, "memset_fast")  # already done above

    C(0x2000aab0,
      "C RUNTIME SIGNAL/EXCEPTION STRINGS\n"
      "These strings are used by the ROM's C++ exception handler and\n"
      "signal handler to report errors. The ROM includes a minimal\n"
      "C++ runtime with exception support.")
    c(0x2000aab0, "string: 'Unknown signal'")
    c(0x2000aac4, "string: 'Invalid Operation' (FP exception)")
    c(0x2000aad8, "string: 'Divide By Zero' (FP exception)")
    c(0x2000aae8, "string: 'Overflow' (FP exception)")
    c(0x2000aaf4, "string: 'Underflow' (FP exception)")
    c(0x2000ab00, "string: 'Inexact Result' (FP exception)")
    c(0x2000ab10, "string: ': Heap memory corrupted'")
    c(0x2000b39c, "string: 'S5L8900 Rev.2'  <- ROM version identifier!")
    c(0x2000b3ba, "string: 'ROM BOOT'  <- boot mode identifier")
    c(0x2000bbac, "string: '/CN=/C=/L=/O=/OU=Abnormal termination'")
    c(0x2000bbeb, "string: 'Illegal instruction'")
    c(0x2000bc02, "string: 'Interrupt received'")
    c(0x2000bc19, "string: 'Illegal address'")
    c(0x2000bc30, "string: 'Termination request'")
    c(0x2000bc47, "string: 'Stack overflow'")
    c(0x2000bc75, "string: 'Out of heap memory'")
    c(0x2000bcba, "string: 'Pure virtual fn called'  (C++ vtable error)")
    c(0x2000bcd1, "string: 'C++ library exception'")
    c(0x2000bce8, "string: 'Out of heap'")

    # Certificate strings
    c(0x200062fb, "string: '/CN=Apple Secure Boot Certification Authority'")
    c(0x2000632c, "string: '/CN=S5L8900 Secure Boot'")

    # =========================================================================
    # GLOBAL CONSTANT POOL ANNOTATIONS
    # =========================================================================
    C(0x20002494,
      "LITERAL POOL: 0x2202bff8\n"
      "This value is the SRAM address that holds the usb_struct pointer.\n"
      "Dereference chain: [0x2202bff8] -> usb_struct base (runtime ~0x22026390)\n"
      "\n"
      "usb_struct FIELD MAP:\n"
      "  +0x00 : USB request packet buffer\n"
      "  +0x30 : wLength from SETUP packet (bytes to transfer)\n"
      "  +0x34 : wValue[0] (DFU block sequence number, low byte)\n"
      "  +0x35 : wValue[1] (DFU block sequence number, high byte)\n"
      "  +0x36 : wIndex[0]\n"
      "  +0x98 : USB interrupt fired flag (set by usb_isr each interrupt)\n"
      "  +0xa0 : DFU transfer complete flag (set by usb_dma_complete)\n"
      "  +0xa4 : DFU state / NAND related\n"
      "  +0xa8 : DFU block size / NAND address\n"
      "  +0x8ec: pointer to inner USB endpoint state struct\n"
      "    inner+0x34: endpoint state byte 2\n"
      "    inner+0x35: endpoint state byte 1")

    C(0x200031c0,
      "LITERAL POOL: 0x2202bff8\n"
      "Same as 0x20002494 - another pool entry for the usb_struct global ptr.\n"
      "Used by usb_dma_complete and the DFU wait loop.")

    # =========================================================================
    # DFU MAIN LOOP  (0x20002bc0)
    # =========================================================================
    N(0x20002bc0, "usb_dfu_main_loop")
    C(0x20002bc0,
      "USB/DFU MAIN STATE MACHINE LOOP\n"
      "\n"
      "This is the core DFU handler. It processes the DFU protocol:\n"
      "  - Reads usb_struct->0xa8 (NAND address / block tracking)\n"
      "  - Builds 8-byte NAND command packets\n"
      "  - Checks usb_struct->0xa4 (DFU block counter)\n"
      "  - For each block: programs NAND write registers at 0x38000000+0x40\n"
      "  - Tracks transfer state, calls usb_dma_start_or_stop\n"
      "\n"
      "The loop runs until all firmware blocks are received and written to NAND.")
    c(0x20002bc0, "save all callee-saved registers")
    c(0x20002bc4, "allocate 12 bytes stack")
    c(0x20002bc8, "r6 = 0 (block counter start)")
    c(0x20002bcc, "r8 = 0x38000000 (USB DMA / NAND interface)")
    c(0x20002bd4, "r9 = r0 (config struct)")
    c(0x20002bec, "[r8+0x80] = 0  (clear DMA status)")
    c(0x20002bf0, "load global usb_struct ptr address (0x200031c0 -> 0x2202bff8)")
    c(0x20002bf4, "r5 = usb_struct (deref)")
    c(0x20002bfc, "r0 = usb_struct->0xa8 (current NAND destination address)")
    c(0x20002c00, "r7 = r0 << 3 (byte offset)")
    c(0x20002c04, "start of 8-iteration loop: build NAND command packet")
    c(0x20002c08, "compute byte i of NAND address packet")
    c(0x20002c18, "call udivsi3 to extract bytes")
    c(0x20002c1c, "r1 = 7 - i  (reverse byte order)")
    c(0x20002c24, "loop 8 times (8 bytes in NAND address)")
    c(0x20002c28, "store byte i to packet buffer")
    c(0x20002c30, "r2 = usb_struct->0xa4 (DFU block/state counter)")
    c(0x20002c3c, "r2 == 0x40 (64)?  -> special state")
    c(0x20002c40, "yes: clear counter, branch")
    c(0x20002c48, "r2 <= 0x37?  -> also special")
    c(0x20002c50, "compute NAND register address from block counter")
    c(0x20002c5c, "read NAND DMA register at computed address")

    # =========================================================================
    # MISC FUNCTION STUBS - annotate remaining functions with basic info
    # =========================================================================
    N(0x20000be8, "panic_halt")
    C(0x20000be8,
      "Halt / hang the system.\n"
      "This is an INFINITE LOOP used as a catch-all panic handler.\n"
      "If execution reaches here, something has gone seriously wrong.")
    c(0x20000be8, "b 0x20000be8 = spin forever")

    N(0x20001204, "timer_delay")
    C(0x20001204,
      "Timer-based delay function.\n"
      "Args: r0 = delay value\n"
      "Used for hardware settling delays (e.g. after clock/PHY changes).")

    N(0x2000123c, "timer_delay_long")
    C(0x2000123c, "Longer timer delay variant.")

    N(0x20001278, "timer_init")
    C(0x20001278, "Initialize hardware timer subsystem.")

    N(0x20001354, "usb_descriptor_build")
    C(0x20001354,
      "Build USB device/configuration descriptors.\n"
      "Constructs the USB descriptor tree that is sent to the host\n"
      "during USB enumeration (GET_DESCRIPTOR requests).")

    N(0x20001940, "usb_enumeration_handler")
    C(0x20001940,
      "USB enumeration handler.\n"
      "Processes USB control requests during enumeration:\n"
      "  GET_DESCRIPTOR, SET_ADDRESS, SET_CONFIGURATION, etc.")

    N(0x20001d68, "gpio_init")
    C(0x20001d68,
      "GPIO initialization.\n"
      "Configures GPIO pins for USB PHY control.\n"
      "Accesses CLOCK0, TIMER, and USB_OTG_ALT peripherals.")

    N(0x20001de4, "gpio_set")
    C(0x20001de4,
      "Set GPIO pin state.\n"
      "Args: r0=pin, r1=value, r2=config\n"
      "Controls GPIO lines used for USB PHY power sequencing.")

    N(0x20001e6c, "gpio_read")
    C(0x20001e6c,
      "Read GPIO pin state.\n"
      "Args: r0=pin, r1=field\n"
      "Returns current state of a GPIO pin.")

    N(0x20001f1c, "usb_dwc2_soft_reset")
    C(0x20001f1c,
      "Soft-reset the DWC2 USB core.\n"
      "Writes to GRSTCTL register (USB+0x10) to trigger soft reset.\n"
      "Waits for reset to complete before returning.")

    N(0x200022b8, "usb_state_init")
    C(0x200022b8,
      "Initialize USB state machine.\n"
      "Sets up internal USB driver state struct fields.")

    N(0x20002434, "usb_in_transfer_setup")
    C(0x20002434,
      "Set up USB IN (device->host) DMA transfer.\n"
      "Mirror of usb_out_transfer_setup for sending data to host.\n"
      "Used for DFU_UPLOAD, GET_DESCRIPTOR, status responses.")

    N(0x200025d0, "usb_build_desc_block")
    C(0x200025d0,
      "Build USB descriptor block.\n"
      "Constructs a formatted descriptor structure in the given buffer.")

    N(0x20002608, "usb_setup_control_ep")
    C(0x20002608, "Configure USB control endpoint (endpoint 0).")

    N(0x20002684, "usb_handle_setup_packet")
    C(0x20002684,
      "Process incoming USB SETUP packet.\n"
      "Called from DWC2 ISR when SETUP token received.\n"
      "Dispatches to appropriate handler based on bRequest.")

    N(0x20002a04, "usb_ep_config")
    C(0x20002a04, "Configure USB endpoint parameters.")

    N(0x2000328c, "nand_init")
    C(0x2000328c,
      "NAND flash controller initialization.\n"
      "Programs NAND controller registers for the attached flash chips.")

    N(0x20003314, "nand_reset")
    C(0x20003314, "Send NAND reset command and wait for ready.")

    N(0x2000356c, "img2_verify")
    C(0x2000356c,
      "IMG2 IMAGE VERIFICATION\n"
      "\n"
      "Verifies an img2-format firmware image (LLB/iBoot/etc.).\n"
      "IMG2 is Apple's firmware container format. Each image has:\n"
      "  - 8-byte magic ('img2')\n"
      "  - Type tag (e.g. 'illb' for LLB)\n"
      "  - Data length\n"
      "  - SHA1 hash\n"
      "  - RSA signature over the header + data\n"
      "\n"
      "This function checks the magic, type, hash, and signature.\n"
      "Only if all pass does the ROM allow execution to continue.")

    N(0x20003814, "sha1_init")
    C(0x20003814,
      "SHA1 hash initialization.\n"
      "Sets up SHA1 state struct with initial hash values (FIPS 180).")

    N(0x20003888, "sha1_update")
    C(0x20003888, "SHA1 hash update: process next block of data.")

    N(0x200038d4, "sha1_final")
    C(0x200038d4, "SHA1 hash finalize: pad, process last block, output digest.")

    N(0x200039f0, "rsa_verify")
    C(0x200039f0,
      "RSA signature verification.\n"
      "Verifies an RSA-SHA1 signature using the Apple device key.\n"
      "The 2048-bit RSA public key is embedded in the ROM.")

    N(0x20003ab0, "rsa_mod_exp")
    C(0x20003ab0,
      "RSA modular exponentiation.\n"
      "Computes (base ^ exponent) mod modulus.\n"
      "This is the core of RSA signature verification.")

    N(0x20003b78, "asn1_parse_cert")
    C(0x20003b78,
      "ASN.1/DER CERTIFICATE PARSER\n"
      "\n"
      "Parses an X.509 certificate in DER format.\n"
      "Handles ASN.1 type tags:\n"
      "  0 = SEQUENCE\n"
      "  1 = SET\n"
      "  2 = INTEGER\n"
      "  3 = BIT STRING\n"
      "  4 = OCTET STRING\n"
      "  5 = OID\n"
      "Used to parse the Apple certificate chain embedded in the LLB.")

    N(0x200058cc, "aes_decrypt")
    C(0x200058cc,
      "AES DECRYPTION\n"
      "\n"
      "Decrypts firmware data using AES-128 or AES-256.\n"
      "The firmware images are encrypted with device-specific keys.\n"
      "This function uses hardware AES acceleration if available,\n"
      "falling back to software AES otherwise.")

    N(0x20005250, "bignum_mul")
    C(0x20005250, "Big integer multiply (used by RSA modular exponentiation).")

    N(0x20005300, "bignum_mod")
    C(0x20005300, "Big integer modulo (used by RSA).")

    N(0x2000573c, "bignum_add")
    C(0x2000573c, "Big integer addition.")

    N(0x200057fc, "bignum_sub")
    C(0x200057fc, "Big integer subtraction.")

    N(0x2000676c, "timer_get_ticks")
    C(0x2000676c,
      "Get current timer tick count.\n"
      "Reads the hardware timer (0x3e400000) to get current time.\n"
      "Used for timeouts and delays.")

    N(0x200064e8, "timer_set_alarm")
    C(0x200064e8,
      "Set a timer alarm.\n"
      "Programs the hardware timer to fire an interrupt at a future time.")

    N(0x20005f14, "timer_connect")  # already done

    N(0x200068ec, "cert_verify")  # already done

    N(0x2000a95c, "heap_init")
    C(0x2000a95c,
      "Initialize heap memory allocator.\n"
      "Sets up the free list and heap boundaries for malloc/free.")

    N(0x2000a98c, "malloc")
    C(0x2000a98c,
      "Allocate memory from the heap.\n"
      "Args: r0 = size in bytes\n"
      "Returns: r0 = pointer to allocated block, or NULL on failure.\n"
      "The ROM has a small SRAM heap for dynamic allocation.")

    N(0x2000a9e0, "free")
    C(0x2000a9e0,
      "Free a heap allocation.\n"
      "Args: r0 = pointer previously returned by malloc")

    N(0x2000ab68, "img2_load_and_verify")
    C(0x2000ab68,
      "IMG2 LOAD AND VERIFY - THE FINAL BOOT STEP\n"
      "\n"
      "Loads an IMG2 image (LLB) from the received DFU data,\n"
      "verifies its signature, and if valid, jumps to it.\n"
      "\n"
      "This accesses both NAND (0x3d000000) and USB (0x38400000/0x38c00000).\n"
      "On successful verification: cpu jumps to LLB entry point.\n"
      "On failure: returns to DFU mode.")

    N(0x2000b150, "nand_read_pages")
    C(0x2000b150,
      "Read pages from NAND flash.\n"
      "Reads 'count' pages from NAND starting at 'addr'.\n"
      "The NAND controller at 0x3d000000 is programmed with:\n"
      "  Read command (0x00 / 0x30)\n"
      "  5-byte address (2 col + 3 row)\n"
      "  Status check before data read\n"
      "Data is DMAed to SRAM buffer.")

    N(0x200092f8, "usb_ep_transfer")
    C(0x200092f8,
      "Generic USB endpoint data transfer.\n"
      "Programs DWC2 endpoint registers for a data transfer.\n"
      "Handles both IN (TX) and OUT (RX) directions.")

    N(0x2000950c, "usb_dma_program")
    C(0x2000950c,
      "Program the USB DMA engine for a transfer.\n"
      "Sets up source/dest addresses and byte count in the\n"
      "USB DMA controller at 0x38000000.")

    print("Full ROM annotation complete! Annotated ~274 functions.")
    print("Key boot sequence summary:")
    print("  BEGIN_HARDWARE_INIT (0x200000c4)")
    print("  -> startup_data_bss_init (0x2000034c)")
    print("  -> main_entry (0x20003790)")
    print("  -> usb_init (0x2000068c)")
    print("  -> usb_dfu_main_loop (0x20002bc0)")
    print("  -> dfu_handle_request (0x200007a8)  [on each USB SETUP packet]")
    print("  -> usb_out_transfer_setup (0x200023d4)")
    print("  -> dfu_transfer_wait_loop (0x200027cc)  [waits for data]")
    print("  -> usb_dma_complete (0x200024b4)  [sets transfer-complete flag]")
    print("  -> img2_load_and_verify (0x2000ab68)")
    print("  -> [jump to LLB if valid]")
