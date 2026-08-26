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

static void s5l8900_init(MachineState *machine)
{
    fprintf(stderr, "chris: beginning s5l8900 init\n");
    fflush(stderr);
    MemoryRegion *sysmem = get_system_memory();

    // catchall
    MemoryRegion *catchall = g_new0(MemoryRegion, 1);
    memory_region_init_io(catchall, NULL, &s5l8900_catchall_ops, NULL,
                          "s5l8900.catchall", UINT64_MAX);
    memory_region_add_subregion_overlap(sysmem, 0, catchall, 0);

    // srom
    MemoryRegion *vrom = g_new0(MemoryRegion, 1);
    memory_region_init_ram(vrom, NULL, "s5l8900.vrom",
                       S5L8900_VROM_SIZE, &error_fatal);
    memory_region_set_readonly(vrom, true);
    memory_region_add_subregion(sysmem, S5L8900_VROM_BASE, vrom);

    // evec
    MemoryRegion *evec = g_new0(MemoryRegion, 1);
    memory_region_init_ram(evec, NULL, "s5l8900.evec", S5L8900_EVEC_SIZE, &error_fatal);
    memory_region_add_subregion_overlap(sysmem, 0, evec, 1);

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
