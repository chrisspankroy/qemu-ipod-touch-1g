#include "qemu/osdep.h"
#include "qemu/units.h"
#include "hw/core/boards.h"
#include "hw/arm/machines-qom.h"
#include "hw/core/loader.h"
#include "hw/misc/unimp.h"
#include "target/arm/cpu.h"
#include "system/address-spaces.h"
#include "system/reset.h"
#include "qapi/error.h"

/* From notes.txt memory map */
#define S5L8900_VROM_BASE   0x20000000
#define S5L8900_VROM_SIZE   0x10000      /* 64KB SecureROM */
#define S5L8900_PERIPH_BASE 0x38000000
#define S5L8900_PERIPH_SIZE 0x08000000   /* 128MB peripheral region */
#define S5L8900_VIC0_BASE   0x38e00000
#define S5L8900_VIC1_BASE   0x38e01000
/* Actual SRAM base discovered by tracing ROM startup:
 * stacks at 0x22020844-0x22026c84, BSS/data at 0x22020800-0x22026c84 */
#define S5L8900_RAM_BASE    0x22000000
#define S5L8900_RAM_SIZE    (512 * KiB)

/* Exception vectors live at 0x0 on reset (SCTLR.V=0).
 * Map a small RAM so aborts don't cascade into unmapped space. */
#define S5L8900_EVEC_BASE   0x00000000
#define S5L8900_EVEC_SIZE   0x1000

/* ---- CLOCK1 stub (0x3c500000) ------------------------------------------ *
 * The ROM programs the PLL then polls CLOCK1+0x40 waiting for value 1
 * (lock status). Return 1 for all reads to satisfy the lock check.
 * Writes are silently ignored until we understand the clock tree better.
 * ------------------------------------------------------------------------ */
static uint64_t s5l8900_clock_read(void *opaque, hwaddr offset, unsigned size)
{
    return 1;
}

static void s5l8900_clock_write(void *opaque, hwaddr offset,
                                uint64_t value, unsigned size)
{
}

static const MemoryRegionOps s5l8900_clock_ops = {
    .read  = s5l8900_clock_read,
    .write = s5l8900_clock_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void s5l8900_cpu_reset(void *opaque)
{
    ARMCPU *cpu = opaque;
    /* ARM1176 normally resets to 0x0; S5L8900 boots from SecureROM at 0x20000000 */
    cpu->env.regs[15] = S5L8900_VROM_BASE;
}

static void s5l8900_init(MachineState *machine)
{
    MemoryRegion *sysmem = get_system_memory();
    MemoryRegion *ram  = g_new0(MemoryRegion, 1);
    MemoryRegion *vrom = g_new0(MemoryRegion, 1);
    MemoryRegion *evec = g_new0(MemoryRegion, 1);

    /* ARM1176JZF-S */
    ARMCPU *cpu = ARM_CPU(cpu_create(machine->cpu_type));
    qemu_register_reset(s5l8900_cpu_reset, cpu);

    /* Exception vector RAM at 0x0 (SCTLR.V=0 on reset).
     * Without this, any exception cascades into unmapped space. */
    memory_region_init_ram(evec, NULL, "s5l8900.evec",
                           S5L8900_EVEC_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, S5L8900_EVEC_BASE, evec);

    /* Internal SRAM */
    memory_region_init_ram(ram, NULL, "s5l8900.ram",
                           S5L8900_RAM_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, S5L8900_RAM_BASE, ram);

    /* SecureROM */
    memory_region_init_rom(vrom, NULL, "s5l8900.vrom",
                           S5L8900_VROM_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, S5L8900_VROM_BASE, vrom);

    if (machine->firmware) {
        load_image_mr(machine->firmware, vrom);
    }

    /* CLOCK1 stub — must be added before the catch-all unimp region */
    MemoryRegion *clock1 = g_new0(MemoryRegion, 1);
    memory_region_init_io(clock1, NULL, &s5l8900_clock_ops, NULL,
                          "s5l8900.clock1", 0x1000);
    memory_region_add_subregion(sysmem, 0x3c500000, clock1);

    /* Catch and log all peripheral accesses we haven't implemented yet */
    create_unimplemented_device("s5l8900.periph",
                                S5L8900_PERIPH_BASE, S5L8900_PERIPH_SIZE);
}

static void s5l8900_machine_init(MachineClass *mc)
{
    mc->desc             = "Apple iPod Touch 1G (S5L8900)";
    mc->init             = s5l8900_init;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("arm1176");
    mc->default_ram_size = S5L8900_RAM_SIZE;
}

DEFINE_MACHINE_ARM("s5l8900", s5l8900_machine_init)
