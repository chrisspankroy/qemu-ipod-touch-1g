#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/log.h"
#include "hw/core/boards.h"
#include "hw/arm/machines-qom.h"
#include "hw/core/loader.h"
#include "hw/misc/unimp.h"
#include "target/arm/cpu.h"
#include "system/address-spaces.h"
#include "system/reset.h"
#include "system/memory.h"
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

  /* ---- PL190 VIC stub (0x38e00000, 0x38e01000) ---------------------------
   * Stores vectored handler addresses written by the ROM and returns them
   * via VICADDRESS (offset 0xf00) when an IRQ is pending.
   * ----------------------------------------------------------------------- */
  #define VIC_VECTADDR_BASE   0x100   /* VICVECTADDR0..31 */
  #define VIC_VECTADDR_END    0x17c
  #define VIC_INTENABLE       0x010
  #define VIC_INTENCLEAR      0x014
  #define VIC_INTSELECT       0x00c
  #define VIC_ADDRESS         0xf00

  typedef struct {
      uint32_t vectaddr[32];  /* programmed ISR addresses */
      uint32_t intenable;     /* enabled IRQ bitmask */
      uint32_t pending;       /* asserted IRQ bitmask */
  } S5L8900VICState;

  static uint64_t s5l8900_vic_read(void *opaque, hwaddr offset, unsigned size)
  {
      S5L8900VICState *s = opaque;

      if (offset >= VIC_VECTADDR_BASE && offset <= VIC_VECTADDR_END) {
          return s->vectaddr[(offset - VIC_VECTADDR_BASE) / 4];
      }
      switch (offset) {
      case 0x000: /* VICIRQSTATUS */
          return s->pending & s->intenable;
      case VIC_INTENABLE:
          return s->intenable;
      case VIC_ADDRESS:
          /* Return handler address for lowest pending enabled IRQ */
          for (int i = 0; i < 32; i++) {
              if ((s->pending & s->intenable) & (1u << i)) {
                  return s->vectaddr[i];
              }
          }
          return 0;
      default:
        qemu_log_mask(LOG_UNIMP, "s5l8900.vic: unimplemented write offset 0x%"HWADDR_PRIx"\n", offset);
          return 0;
      }
  }

  static void s5l8900_vic_write(void *opaque, hwaddr offset,
                                 uint64_t value, unsigned size)
  {
      S5L8900VICState *s = opaque;

      if (offset >= VIC_VECTADDR_BASE && offset <= VIC_VECTADDR_END) {
          s->vectaddr[(offset - VIC_VECTADDR_BASE) / 4] = value;
          return;
      }
      switch (offset) {
      case VIC_INTENABLE:
          s->intenable |= value;
          break;
      case VIC_INTENCLEAR:
          s->intenable &= ~value;
          break;
      case VIC_INTSELECT:
          break; /* ignore FIQ/IRQ routing for now */
      case VIC_ADDRESS:
          s->pending = 0; /* end-of-interrupt: clear all pending */
          break;
      default:
        qemu_log_mask(LOG_UNIMP, "s5l8900.vic: unimplemented write offset 0x%"HWADDR_PRIx"\n", offset);
          break;
      }
  }

  static const MemoryRegionOps s5l8900_vic_ops = {
      .read  = s5l8900_vic_read,
      .write = s5l8900_vic_write,
      .endianness = DEVICE_LITTLE_ENDIAN,
  };

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

/* ---- USB stub (0x38c00000) ------------------------------------------ */
typedef struct {
    uint32_t gusbcfg; // Global USB Configuration register
    uint32_t dctl;    // Device control register
    uint32_t dcfg;    // Device config register
} S5L8900USBState;

#define DWC2_GOTGINT 0x4
#define DWC2_GUSBCFG 0x14
#define DWC2_DCFG    0x800
#define DWC2_DCTL    0x804
#define DWC2_GRSTCTL 0x10
#define DWC2_DSTS 0x808

static uint64_t s5l8900_usb_read(void *opaque, hwaddr offset, unsigned size)
{
    S5L8900USBState *s = opaque;
    switch (offset) {
        case DWC2_GUSBCFG:
            return s->gusbcfg;
        case DWC2_DCFG:
            return s->dcfg;
        case DWC2_DCTL:
            return s->dctl;
        case DWC2_GRSTCTL:
            return 1u << 31; // set bit 31, rest are unset
        case DWC2_DSTS:
            return 1u << 1; // set bit 1, rest are unset
        default:
            qemu_log_mask(LOG_UNIMP, "s5l8900.usb: unimplemented read offset 0x%"HWADDR_PRIx"\n", offset);
            return 0;
    }
}

static void s5l8900_usb_write(void *opaque, hwaddr offset,
                                uint64_t value, unsigned size)
{
    S5L8900USBState *s = opaque;

    switch (offset) {
        case DWC2_GUSBCFG:
            s->gusbcfg = value;
            break;
        case DWC2_DCFG:
            s->dcfg = value;
            break;
        case DWC2_DCTL:
            s->dctl = value;
            break;
        case DWC2_GOTGINT:
            uint32_t usb_struct_ptr;
            cpu_physical_memory_read(0x200031c0, &usb_struct_ptr, 4);
            uint32_t usb_struct;
            cpu_physical_memory_read(usb_struct_ptr, &usb_struct, 4);
            uint32_t write_value = 1;
            cpu_physical_memory_write(usb_struct + 0x98, &write_value, 4);
            cpu_physical_memory_write(usb_struct + 0xa0, &write_value, 4);
            break; 
        default:
            qemu_log_mask(LOG_UNIMP, "s5l8900.usb: unimplemented write offset 0x%"HWADDR_PRIx"\n", offset);
            break;
    }
}

static const MemoryRegionOps s5l8900_usb_ops = {
    .read  = s5l8900_usb_read,
    .write = s5l8900_usb_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

typedef struct {
  uint32_t ctrl;
  uint32_t destaddr;
  uint32_t copysize;
  uint8_t fifo[8];
  uint32_t unknown[5];
} S5L8900DMAState;

#define DMA_CTRL    0x0
#define DMA_UNKNOWN 0x20
#define DMA_FIFO 0x40
#define DMA_DESTADDR   0x84
#define DMA_COPYSIZE    0x8c

static uint64_t s5l8900_dma_read(void *opaque, hwaddr offset, unsigned size)
{
    S5L8900DMAState *s = opaque;
    switch (offset) {
        case DMA_CTRL:
            return s->ctrl;
        case DMA_DESTADDR:
            return s->destaddr;
        case DMA_COPYSIZE:
            return s->copysize;
        case DMA_FIFO:
            return s->fifo[0] | (s->fifo[1] << 8) | (s->fifo[2] << 16) | (s->fifo[3] << 24);
        case DMA_FIFO + 0x4:
            return s->fifo[4] | (s->fifo[5] << 8) | (s->fifo[6] << 16) | (s->fifo[7] << 24);
        case DMA_UNKNOWN:
            return s->unknown[0];
        case DMA_UNKNOWN + 0x4:
            return s->unknown[1];
        case DMA_UNKNOWN + 0x8:
            return s->unknown[2];
        case DMA_UNKNOWN + 0xc:
            return s->unknown[3];
        case DMA_UNKNOWN + 0x10:
            return s->unknown[4];
        default:
            qemu_log_mask(LOG_UNIMP, "s5l8900.dma: unimplemented read offset 0x%"HWADDR_PRIx"\n", offset);
            return 0;
    }
}

#define DMA_CTRL_START  (1u << 1 | 1u << 2)
#define DMA_XFR_DIR (1u << 3) 

static void s5l8900_dma_write(void *opaque, hwaddr offset,
                                uint64_t value, unsigned size)
{
    S5L8900DMAState *s = opaque;
    switch (offset) {
        case DMA_CTRL:
            s->ctrl = value;
            // If bits 1 and 2 are set (start transfer)
            if ((value & DMA_CTRL_START) == DMA_CTRL_START) {

                uint32_t usb_struct_ptr;
                cpu_physical_memory_read(0x200031c0, &usb_struct_ptr, 4);
                uint32_t usb_struct;
                cpu_physical_memory_read(usb_struct_ptr, &usb_struct, 4);

                if ((value & DMA_XFR_DIR) == DMA_XFR_DIR) {
                    // TX (device to host)
                    qemu_log_mask(LOG_UNIMP, "s5l8900.dma: TX transfer kicked dest=0x%x size=0x%x\n", s->destaddr, s->copysize);
                }
                else {
                    // RX (host to device)
                    qemu_log_mask(LOG_UNIMP, "s5l8900.dma: RX transfer kicked dest=0x%x size=0x%x\n", s->destaddr, s->copysize);
                    uint32_t word1 = 0x121;
                    uint32_t word2 = 0x400000;
                    // Write our SETUP packet to RAM
                    cpu_physical_memory_write(s->destaddr, &word1, 4);
                    cpu_physical_memory_write(s->destaddr + 0x4, &word2, 4);
                    // Write our SETUP packet to DMA FIFO
                    s->fifo[0] = 0x21;
                    s->fifo[1] = 0x01;
                    s->fifo[2] = 0x00;
                    s->fifo[3] = 0x00;
                    s->fifo[4] = 0x00;
                    s->fifo[5] = 0x00;
                    s->fifo[6] = 0x40;
                    s->fifo[7] = 0x00;
                }
                // Signal completion
                uint32_t write_value = 1;
                cpu_physical_memory_write(usb_struct + 0x98, &write_value, 4);
                cpu_physical_memory_write(usb_struct + 0xa0, &write_value, 4);
                s->ctrl = 0;
            }
            break;
        case DMA_DESTADDR:
            s->destaddr = value;
            break;
        case DMA_COPYSIZE:
            s->copysize = value;
            break;
        case DMA_UNKNOWN:
            s->unknown[0] = value;
            break;
        case DMA_UNKNOWN + 0x4:
            s->unknown[1] = value;
            break;
        case DMA_UNKNOWN + 0x8:
            s->unknown[2] = value;
            break;
        case DMA_UNKNOWN + 0xc:
            s->unknown[3] = value;
            break;
        case DMA_UNKNOWN + 0x10:
            s->unknown[4] = value;
            break;
        default:
            qemu_log_mask(LOG_UNIMP, "s5l8900.dma: unimplemented write offset 0x%"HWADDR_PRIx"\n", offset);
            break;
    }
}

static const MemoryRegionOps s5l8900_dma_ops = {
    .read  = s5l8900_dma_read,
    .write = s5l8900_dma_write,
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
    MemoryRegion *unknown = g_new0(MemoryRegion, 1);

    S5L8900VICState *vic0 = g_new0(S5L8900VICState, 1);
    S5L8900VICState *vic1 = g_new0(S5L8900VICState, 1);

    MemoryRegion *vic0_mr = g_new0(MemoryRegion, 1);
    MemoryRegion *vic1_mr = g_new0(MemoryRegion, 1);

    memory_region_init_io(vic0_mr, NULL, &s5l8900_vic_ops, vic0,
                            "s5l8900.vic0", 0x1000);
    memory_region_add_subregion(sysmem, 0x38e00000, vic0_mr);

    memory_region_init_io(vic1_mr, NULL, &s5l8900_vic_ops, vic1,
                            "s5l8900.vic1", 0x1000);
    memory_region_add_subregion(sysmem, 0x38e01000, vic1_mr);

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

    /* Unknown region */
    memory_region_init_ram(unknown, NULL, "s5l8900.unknownregion",
                           0x10000, &error_fatal);
    memory_region_add_subregion(sysmem, 0x24000000, unknown);

    /* SecureROM */
    memory_region_init_rom(vrom, NULL, "s5l8900.vrom",
                           S5L8900_VROM_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, S5L8900_VROM_BASE, vrom);

    if (machine->firmware) {
        load_image_mr(machine->firmware, vrom);
    }

    // Alias ROM to 0x0
    MemoryRegion *vrom_alias = g_new0(MemoryRegion, 1);
    memory_region_init_alias(vrom_alias, NULL, "s5l8900.vromalias", vrom, 0x0, S5L8900_VROM_SIZE);
    memory_region_add_subregion(sysmem, 0x0, vrom_alias);

    /* CLOCK1 stub */
    MemoryRegion *clock1 = g_new0(MemoryRegion, 1);
    memory_region_init_io(clock1, NULL, &s5l8900_clock_ops, NULL,
                          "s5l8900.clock1", 0x1000);
    memory_region_add_subregion(sysmem, 0x3c500000, clock1);

    /* USB stub */
    S5L8900USBState *usb_state = g_new0(S5L8900USBState, 1);
    MemoryRegion *usb = g_new0(MemoryRegion, 1);
    memory_region_init_io(usb, NULL, &s5l8900_usb_ops, usb_state,
                          "s5l8900.usb", 0x1000);
    memory_region_add_subregion(sysmem, 0x38c00000, usb);

    /* DMA stub */
    S5L8900DMAState *dma_state = g_new0(S5L8900DMAState, 1);
    MemoryRegion *dma = g_new0(MemoryRegion, 1);
    memory_region_init_io(dma, NULL, &s5l8900_dma_ops, dma_state,
                           "s5l8900.dma", 0x100);
    memory_region_add_subregion(sysmem, 0x38000000, dma);

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
