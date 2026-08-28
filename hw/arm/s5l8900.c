#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/core/boards.h"
#include "hw/arm/machines-qom.h"
#include "hw/core/loader.h"
#include "hw/misc/unimp.h"
#include "target/arm/cpu.h"
#include "system/address-spaces.h"
#include "system/cpus.h"
#include "system/reset.h"
#include "system/memory.h"
#include "system/system.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "exec/tb-flush.h"
#include "exec/translation-block.h"
#include "exec/cpu-common.h"
#include "chardev/char.h"

#define S5L8900_EVEC_BASE   0x00000000
#define S5L8900_EVEC_SIZE   0x1000
#define S5L8900_VROM_BASE   0x20000000
#define S5L8900_VROM_SIZE   0x100000
#define S5L8900_RAM_BASE    0x22000000
#define S5L8900_RAM_SIZE    (512 * KiB)
#define S5L8900_IBSS_BASE   0x09000000
#define S5L8900_IBSS_SIZE   (256 * KiB)
#define S5L8900_IBEC_BASE   0x0A000000
#define S5L8900_IBEC_SIZE   (256 * KiB)
#define S5L8900_IBOOT_BASE  0x23000000
#define S5L8900_IBOOT_SIZE  (1 * MiB)
#define S5L8900_IBOOT_RUNTIME 0x18000000
#define S5L8900_USBOTG_BASE 0x18000000
#define S5L8900_USBOTG_SIZE (2 * MiB)
#define S5L8900_PERIPH_BASE 0x38000000
#define S5L8900_PERIPH_SIZE 0x08000000
#define S5L8900_CLOCK0_BASE 0x38100000
#define S5L8900_VIC0_BASE   0x38e00000
#define S5L8900_VIC1_BASE   0x38e01000
#define S5L8900_USB_BASE    0x38c00000
#define S5L8900_CLOCK1_BASE 0x3c500000
#define S5L8900_WDT_BASE    0x3e300000
#define S5L8900_PMU_BASE    0x3e500000
#define S5L8900_UART_BASE   0xE0002000
#define S5L8900_UART_SIZE   0x1000
#define IMG2_HDR_SIZE       0x800

static void s5l8900_cpu_reset(void *opaque)
{
    ARMCPU *cpu = opaque;
    // ARM1176 resets to 0x0; S5L8900 boots from SecureROM at 0x20000000
    // r15 is pc
    cpu->env.regs[15] = S5L8900_VROM_BASE;
}

static uint64_t s5l8900_catchall_read(void *o, hwaddr off, unsigned size) {
    qemu_log_mask(LOG_UNIMP, "s5l8900.catchall: r%u 0x%08llx\n", size, off);
    return 0;
}

static void s5l8900_catchall_write(void *o, hwaddr off, uint64_t v, unsigned size) {
    qemu_log_mask(LOG_UNIMP, "s5l8900.catchall: w%u 0x%08llx <= 0x%08x\n", size, off, (unsigned)v);
}

static const MemoryRegionOps s5l8900_catchall_ops = {
    .read  = s5l8900_catchall_read,
    .write = s5l8900_catchall_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

typedef struct {
    uint32_t property;
} S5L8900WDTState;

static uint64_t s5l8900_wdt_read(void *o, hwaddr off, unsigned size) {
    S5L8900WDTState *s = o;
    if (off == 0) {
        return s->property;
    }
    qemu_log_mask(LOG_UNIMP, "s5l8900.wdt: r%u 0x%08llx\n", size, off);
    return 0;
}

static void s5l8900_wdt_write(void *o, hwaddr off, uint64_t v, unsigned size) {
    S5L8900WDTState *s = o;
    if (off == 0) {
        s->property = v;
        return;
    }
    qemu_log_mask(LOG_UNIMP, "s5l8900.wdt: w%u 0x%08llx <= 0x%08x\n", size, off, (unsigned)v);
}

static const MemoryRegionOps s5l8900_wdt_ops = {
    .read  = s5l8900_wdt_read,
    .write = s5l8900_wdt_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

// S5L8900 uses the PL192 VIC, docs are here https://support.arm.com/documentation/ddi0273/a

// Shows the status of the interrupts after masking by the VICINTENABLE and VICINTSELECT Registers
// ro
#define VIC_IRQSTATUS            0x0
// Shows the status of the FIQ interrupts after masking by the VICINTENABLE and VICINTSELECT Registers
// ro
#define VIC_FIQSTATUS            0x4
// Shows the status of the interrupts before masking by the Enable Registers
// ro
#define VIC_RAWINTR              0x8
// Shows type of interrupt for interrupt request (0 = IRQ, 1 = FIQ)
// rw
#define VIC_INTSELECT            0xc
// Enables the interrupt request lines, which allow the interrupts to reach the processor
// You can only enable interrupts with this register. Use VICINTENCLEAR to disable
// 0 = disabled, 1 = enabled
// rw
#define VIC_INTENABLE            0x10
// Clears corresponding bits in the VICINTENABLE Register (write-only; 0 = no effect, 1 = disable in VICINTENABLE)
// wo
#define VIC_INTENCLEAR           0x14
// Enable software interrupt
// read: 0 = no software interrupt, 1 = software interrupt active
// write: 0 = no effect, 1 = software interrupt enabled
// rw
#define VIC_SOFTINT              0x18
// Clears corresponding bits in the VICSOFTINT Register (write-only; 0 = no effect, 1 = disable in VICSOFTINT)
// wo
#define VIC_SOFTINTCLEAR         0x1c
// Enables or disables protected register access, stopping register accesses when the processor is in User mode
// only bit 0 is used. 0 = no protection mode, 1 = protection mode enabled
// rw
#define VIC_PROTECTION           0x20
// Contains the software mask value for the interrupt priority levels
// Only bits [0:15] are used. 0 = masked, 1 = not masked
// There are 15 priority levels, this controls if that priority should be masked or not
// rw
#define VIC_SWPRIORITYMASK       0x24
// Sets the priority of the daisy-chained VIC's interrupts
// rw
#define VIC_PRIORITYDAISY        0x28
// Contains the ISR vector addresses
// First at 0x100, Last at 0x17c
// rw
#define VIC_VECTADDR_BEGIN       0x100
#define VIC_VECTADDR_END         0x17c
// Select the interrupt priority level for the 32 vectored interrupt sources
// The value can be from 0-15
// Default is priority 15 (lowest)
// If priority match happens to 2 active interrupts, the lowest-numbered interrupt gets priority
// rw
#define VIC_VECTPRIORITY_BEGIN   0x200
#define VIC_VECTPRIORITY_END     0x27c
// Contains the Interrupt Service Routine (ISR) address of the currently active interrupt
// If no interrupt is currently active, the register holds the ISR address of the last active interrupt
// A write of any value to this register clears the current interrupt
// rw
#define VIC_ADDRESS              0xf00
// The PERIPHID registers are weird
// They are 4 separate 8-bit registers that should be interpreted as a single 32-bit register
// Part number is bits [11:0]. This identifies the peripheral. The three digit product code 0x192 is used for the PrimeCell VIC.
// Designer is bits [19:12]. This is the identification of the designer. ARM Limited is 0x41 (ASCII A).
// Revision number is bits [23:20]. This is the revision number of the peripheral. The revision number starts from 0 and the value is revision-dependent.
// Configuration is bits [31:24]. This is the configuration option of the peripheral. The configuration value is 0.
// Using a 0 revision and 0 configuration value, that results in a bit string of 00000000000001000001000110010010
// Splitting that into 4 8-bit registers: 00000000 00000100 00010001 10010010
// All 4 are ro
// bits [0-7] == 10010010 == 146 == 0x92
#define VIC_PERIPHID0             0xfe0
// bits [8-15] == 00010001 == 17 == 0x11
#define VIC_PERIPHID1             0xfe4
// bits [16-23] == 00000100 == 4 == 0x4
#define VIC_PERIPHID2             0xfe8
// bits [24-31] == 00000000 == 0 == 0x0
#define VIC_PERIPHID3             0xfec
// The PCELLID registers are similar to the PERIPHID registers but stay with 8-bit values so they are clearer
// All 4 are ro
// bits [0-7] == 0xd
#define VIC_PCELLID0              0xff0
// bits [8-15] == 0xf0
#define VIC_PCELLID1              0xff4
// bits [16-23] == 0x5
#define VIC_PCELLID2              0xff8
// bits [24-31] == 0xb1
#define VIC_PCELLID3              0xffc

typedef struct {
    uint32_t rawintr;
    uint32_t intselect;
    uint32_t intenable;
    uint32_t softint;
    uint32_t protection;
    uint32_t swprioritymask;
    uint32_t prioritydaisy;
    uint32_t vectaddr[32];
    uint32_t vectpriority[32];
    uint32_t lastactive;
} S5L8900VICState;

static void s5l8900_vic_reset(void *opaque)
{
    S5L8900VICState *s = opaque;
    
    s->rawintr = 0;
    s->intselect = 0;
    s->intenable = 0;
    s->softint = 0;
    s->protection = 0;
    s->swprioritymask = 0xffff;
    s->prioritydaisy = 0xf;
    for (int i = 0; i < 32; i++) {
        s->vectaddr[i] = 0;
    }
    for (int i = 0; i < 32; i++) {
        s->vectpriority[i] = 0xf;
    }
    s->lastactive = 0;
}

static uint64_t s5l8900_vic_read(void *o, hwaddr off, unsigned size) {
    S5L8900VICState *s = o;

    switch(off) {
        case VIC_IRQSTATUS:
            return ((s->rawintr | s->softint) & s->intenable) & ~s->intselect;
        case VIC_FIQSTATUS: 
            return ((s->rawintr | s->softint) & s->intenable) & s->intselect;
        case VIC_RAWINTR:
            return s->rawintr | s->softint;
        case VIC_INTSELECT:
            return s->intselect;
        case VIC_INTENABLE:
            return s->intenable;
        case VIC_SOFTINT:
            return s->softint;
        case VIC_PROTECTION:
            return s->protection;
        case VIC_SWPRIORITYMASK:
            return s->swprioritymask;
        case VIC_PRIORITYDAISY:
            return s->prioritydaisy;
        case VIC_VECTADDR_BEGIN ... VIC_VECTADDR_END:
            return s->vectaddr[(off - VIC_VECTADDR_BEGIN) / 4];
        case VIC_VECTPRIORITY_BEGIN ... VIC_VECTPRIORITY_END:
            return s->vectpriority[(off - VIC_VECTPRIORITY_BEGIN) / 4];
        case VIC_ADDRESS:
            if (s->rawintr == 0) {
                // No active interrupts, this read is undefined behavior. Do anything you want
                return s->lastactive;
            }
            // You need to find the highest-priority interrupt (closer to 0 is higher priority), with the lowest-indexed interrupt as the tiebreaker
            int highest_priority_idx = 31;
            int highest_priority = 0xf;
            for (int i = 31; i >= 0; i--) {
                if (((s->rawintr | s->softint) & s->intenable & ~s->intselect) & (1u << i)) {
                    if (s->vectpriority[i] <= highest_priority) {
                        highest_priority_idx = i;
                        highest_priority = s->vectpriority[i];
                    }
                }
            }
            s->lastactive = s->vectaddr[highest_priority_idx];
            return s->vectaddr[highest_priority_idx];
        case VIC_PERIPHID0:
            return 0x92;
        case VIC_PERIPHID1:
            return 0x11;
        case VIC_PERIPHID2:
            return 0x4;
        case VIC_PERIPHID3:
            return 0;
        case VIC_PCELLID0:
            return 0xd;
        case VIC_PCELLID1:
            return 0xf0;
        case VIC_PCELLID2:
            return 0x5;
        case VIC_PCELLID3:
            return 0xb1;
        default:
            qemu_log_mask(LOG_UNIMP, "s5l8900.vic: r%u 0x%08llx\n", size, off);
            return 0;
    }
}

static void s5l8900_vic_write(void *o, hwaddr off, uint64_t v, unsigned size) {
    S5L8900VICState *s = o;

    switch(off) {
        case VIC_INTSELECT:
            s->intselect = v;
            return;
        case VIC_INTENABLE:
            // Writes of 0 value should have no effect
            s->intenable |= v;
            return;
        case VIC_INTENCLEAR:
            // Writes of 0 value should have no effect
            s->intenable &= ~v;
            return; 
        case VIC_SOFTINT:
            // Writes of 0 value should have no effect
            s->softint |= v;
            return;
        case VIC_SOFTINTCLEAR:
            // Writes of 0 value should have no effect
            s->softint &= ~v;
            return;
        case VIC_PROTECTION:
            s->protection = v;
            return;
        case VIC_SWPRIORITYMASK:
            s->swprioritymask = v;
            return;
        case VIC_PRIORITYDAISY:
            s->prioritydaisy = v;
            return;
        case VIC_VECTADDR_BEGIN ... VIC_VECTADDR_END:
            s->vectaddr[(off - VIC_VECTADDR_BEGIN) / 4] = v;
            return;
        case VIC_VECTPRIORITY_BEGIN ... VIC_VECTPRIORITY_END:
            s->vectpriority[(off - VIC_VECTPRIORITY_BEGIN) / 4] = v;
            return; 
        case VIC_ADDRESS:
            // A full implementation would mask lower-priority interrupts while an interrupt is being serviced (after a VIC_ADDRESS read)
            // In that context, a write here would remove that mask
            // But for now we don't do that since we dont need to
            return;
        default:
            qemu_log_mask(LOG_UNIMP, "s5l8900.vic: w%u 0x%08llx <= 0x%08x\n", size, off, (unsigned)v);
            return;
    }
}

static const MemoryRegionOps s5l8900_vic_ops = {
    .read  = s5l8900_vic_read,
    .write = s5l8900_vic_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void s5l8900_init(MachineState *machine)
{
    fprintf(stderr, "chris: beginning s5l8900 init\n");
    fflush(stderr);
    MemoryRegion *sysmem = get_system_memory();

    // catchall
    MemoryRegion *catchall = g_new0(MemoryRegion, 1);
    memory_region_init_io(catchall, NULL, &s5l8900_catchall_ops, NULL, "s5l8900.catchall", UINT64_MAX);
    memory_region_add_subregion_overlap(sysmem, 0, catchall, 0);

    // srom
    MemoryRegion *vrom = g_new0(MemoryRegion, 1);
    memory_region_init_ram(vrom, NULL, "s5l8900.vrom", S5L8900_VROM_SIZE, &error_fatal);
    memory_region_set_readonly(vrom, true);
    memory_region_add_subregion_overlap(sysmem, S5L8900_VROM_BASE, vrom, 1);

    // evec
    MemoryRegion *evec = g_new0(MemoryRegion, 1);
    memory_region_init_ram(evec, NULL, "s5l8900.evec", S5L8900_EVEC_SIZE, &error_fatal);
    memory_region_add_subregion_overlap(sysmem, 0, evec, 1);

    // watchdog wdt
    MemoryRegion *wdt = g_new0(MemoryRegion, 1);
    S5L8900WDTState *wdt_state = g_new0(S5L8900WDTState, 1);
    memory_region_init_io(wdt, NULL, &s5l8900_wdt_ops, wdt_state, "s5l8900.wdt", 0x1000);
    memory_region_add_subregion_overlap(sysmem, S5L8900_WDT_BASE, wdt, 1);

    // vic0
    MemoryRegion *vic0 = g_new0(MemoryRegion, 1);
    S5L8900VICState *vic0_state = g_new0(S5L8900VICState, 1);
    memory_region_init_io(vic0, NULL, &s5l8900_vic_ops, vic0_state, "s5l8900.vic0", 0x1000);
    memory_region_add_subregion_overlap(sysmem, S5L8900_VIC0_BASE, vic0, 1);
    qemu_register_reset(s5l8900_vic_reset, vic0_state);

    // vic1
    MemoryRegion *vic1 = g_new0(MemoryRegion, 1);
    S5L8900VICState *vic1_state = g_new0(S5L8900VICState, 1);
    memory_region_init_io(vic1, NULL, &s5l8900_vic_ops, vic1_state, "s5l8900.vic1", 0x1000);
    memory_region_add_subregion_overlap(sysmem, S5L8900_VIC1_BASE, vic1, 1);
    qemu_register_reset(s5l8900_vic_reset, vic1_state);

    if (machine->firmware) {
        if (rom_add_file_fixed(machine->firmware, S5L8900_VROM_BASE, -1) < 0) {
            error_report("s5l8900: could not load ROM %s", machine->firmware);
            exit(1);
        }
    }
    else {
        error_report("s5l8900: you must pass -bios with a path to the ROM");
        exit(1);
    }

    ARMCPU *cpu = ARM_CPU(cpu_create(machine->cpu_type));
    qemu_register_reset(s5l8900_cpu_reset, cpu);

}

static void s5l8900_machine_init(MachineClass *mc)
{
    mc->desc             = "Apple iPod Touch 1G (S5L8900)";
    mc->init             = s5l8900_init;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("arm1176");
    mc->default_ram_size = S5L8900_RAM_SIZE;
}
DEFINE_MACHINE_ARM("s5l8900", s5l8900_machine_init)
