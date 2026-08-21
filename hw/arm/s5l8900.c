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
#include "system/reset.h"
#include "system/memory.h"
#include "system/system.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "exec/tb-flush.h"
#include "exec/translation-block.h"
#include "exec/cpu-common.h"
#include "chardev/char.h"

/* From notes.txt memory map */
#define S5L8900_VROM_BASE   0x20000000
#define S5L8900_VROM_SIZE   0x100000     /* 1MB: 64KB SecureROM + 960KB data/stack region */
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

/* iBSS load address (separate region from SRAM) */
#define S5L8900_IBSS_BASE   0x09000000
#define S5L8900_IBSS_SIZE   (256 * KiB)

/* iBEC load address (DFU recovery bootloader) */
#define S5L8900_IBEC_BASE   0x0A000000
#define S5L8900_IBEC_SIZE   (256 * KiB)

/* USB OTG RAM region (iBEC execution destination) */
#define S5L8900_USBOTG_BASE 0x18000000
#define S5L8900_USBOTG_SIZE (2 * MiB)

/* iBoot staging address (loaded by QEMU init, patched here) */
#define S5L8900_IBOOT_BASE  0x23000000
#define S5L8900_IBOOT_SIZE  (1 * MiB)

/* iBoot runtime address (where iBoot actually executes).
 * iBoot's literal pools reference 0x180xxxxx addresses.
 * We copy from staging (0x23000000) to runtime (0x18000000)
 * in the jump callback after iBEC is done. */
#define S5L8900_IBOOT_RUNTIME 0x18000000

/* Safe loop address in SRAM for exception vector redirects */
#define S5L8900_SAFELOOP_ADDR (S5L8900_RAM_BASE + 0xFE00)

/* UART (Apple KeyLemon/SLIM UART at 0xE0002000) */
#define S5L8900_UART_BASE   0xE0002000
#define S5L8900_UART_SIZE   0x1000

/* IMG2 header size (skipped when loading payloads) */
#define IMG2_HDR_SIZE       0x800

/* Large RAM region for A-bit translated addresses.
 * When the A-bit (CPSR bit 24) is set, all data accesses add 0x10000000
 * to the address. iBEC uses this for separate heap/stack regions.
 * Without this, writes to 0x7ffffxxx go to the catch-all black hole,
 * and reads return 0, corrupting function pointers. */
#define S5L8900_ABIT_BASE   0x60000000
 #define S5L8900_ABIT_SIZE   0x20000000   /* 512MB */

/* ROM function stub regions.
 * iBEC loads function pointers from its constant pool that point to
 * ROM functions at 0x30000000 (SecureROM base on real hardware) and
 * 0x03D00000 (ROM data/callback region). These addresses are not
 * mapped in QEMU (ROM is at 0x20000000). Map small RAM regions
 * filled with ARM BX LR so any ROM function call returns safely. */
#define S5L8900_ROMSTUB1_BASE  0x30000000
#define S5L8900_ROMSTUB1_SIZE  0x10000    /* 64KB */
#define S5L8900_ROMSTUB2_BASE  0x03D00000
#define S5L8900_ROMSTUB2_SIZE  0x10000    /* 64KB */

/* iBoot beyond-code fill pattern (0x47704770) gets loaded as function
 * pointers, causing calls into the 0x477xxxxx upper RAM region. Map a
 * stub there filled with ARM BX LR so those calls return safely. */
#define S5L8900_ROMSTUB3_BASE  0x47700000
#define S5L8900_ROMSTUB3_SIZE  0x00800000 /* 8MB */

/* Shared Thumb-mode PC tracker for crash recovery.
 * Set by peripheral stub on Thumb-mode PC access, and by config_board
 * handler when redirecting to iBEC Thumb entry. */
  static uint32_t s5l8900_last_thumb_pc = 0;
  static int s5l8900_ibec_init_patched = 0;  /* iBEC entry BLs patched flag */
  static int s5l8900_iboot_launched = 0;     /* set when jump callback launches iBoot */
  static uint32_t s5l8900_last_valid_rt_pc = 0;  /* last valid PC in iBoot runtime region */

 /* CLEAN BASELINE: when S5L8900_IBSS_CLEAN is set, DO NOT apply any brute-force
  * iBSS code patches (fills, skip-loops, stubs, pointer fixes, exception-handler
  * redirects). iBSS is loaded and run as the real image so we can observe its
  * genuine first fault / missing-hardware need. */
 static int s5l8900_ibss_clean(void)
 {
     static int clean = -1;
     if (clean < 0) {
         clean = (getenv("S5L8900_IBSS_CLEAN") != NULL) ? 1 : 0;
     }
     return clean;
 }

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
       uint32_t regs[0x1000 / 4]; /* Generic register storage for unimplemented offsets */
   } S5L8900VICState;

  static uint64_t s5l8900_vic_read(void *opaque, hwaddr offset, unsigned size)
  {
      S5L8900VICState *s = opaque;

      if (offset >= VIC_VECTADDR_BASE && offset <= VIC_VECTADDR_END) {
          return s->vectaddr[(offset - VIC_VECTADDR_BASE) / 4];
      }
      switch (offset) {
      case 0x000: /* VICIRQSTATUS */
          return s->pending;
      case VIC_INTENABLE:
          return s->intenable;
      case VIC_ADDRESS:
          /* Return handler address for lowest pending enabled IRQ */
          for (int i = 0; i < 32; i++) {
              if (s->pending & (1u << i)) {
                  return s->vectaddr[i];
              }
          }
          return 0;
     default:
        if (s && offset < sizeof(s->regs)) {
            return s->regs[offset / 4];
        }
        qemu_log_mask(LOG_UNIMP, "s5l8900.vic: unimplemented read offset 0x%"HWADDR_PRIx"\n", offset);
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
          cpu_reset_interrupt(qemu_get_cpu(0), CPU_INTERRUPT_HARD);
          break;
      default:
        if (s && offset < sizeof(s->regs)) {
            s->regs[offset / 4] = (uint32_t)value;
        } else {
            qemu_log_mask(LOG_UNIMP, "s5l8900.vic: unimplemented write offset 0x%"HWADDR_PRIx"\n", offset);
        }
          break;
      }
  }

  static const MemoryRegionOps s5l8900_vic_ops = {
      .read  = s5l8900_vic_read,
      .write = s5l8900_vic_write,
      .endianness = DEVICE_LITTLE_ENDIAN,
  };

/* ---- CLOCK1 stub (0x3c500000) ------------------------------------------ *
 * The ROM programs the PLL and polls CLOCK1+0x40 waiting for exact value 1
 * (PLL lock status). After the ROM passes this poll, it writes 0x10111 to
 * CLOCK1+0x44 to select the PLL as the clock source. iBSS later polls the
 * same register but uses 'tst rX, #2' expecting bit 1 set.
 *
 * Solution: store written values; for offset 0x40 return 1 when the ROM's
 * initial PLL config is active (0x44 < 0x10000), and return 0x3 once the
 * ROM switches to PLL source (0x44 >= 0x10000), satisfying both phases.
 * ------------------------------------------------------------------------ */
typedef struct {
    uint32_t regs[0x1000 / 4];  /* Generic register storage */
} S5L8900ClockState;

static uint64_t s5l8900_clock_read(void *opaque, hwaddr offset, unsigned size)
{
    S5L8900ClockState *s = opaque;

    if (offset == 0x40 && s) {
        /* PLL lock status register (CLOCK1 only).
         * ROM's WAIT_FOR_CLOCK_INIT poll is bypassed by ROM patches (0x37c0 skip).
         * iBSS polls with 'tst r1, r3' (r1=0x2) -> needs bit 1 set.
         * Always return 0x3: PLL locked + selected as clock source. */
        ARMCPU *cpu = ARM_CPU(qemu_get_cpu(0));
        {
            static int clock_log_cnt = 0;
            clock_log_cnt++;
            if (clock_log_cnt <= 30 || (clock_log_cnt % 500 == 0)) {
                fprintf(stderr, "CLOCK1[0x40] read #%d: pc=0x%08x guest_r1(mask)=0x%08x -> returning all-ones\n",
                        clock_log_cnt, (unsigned int)cpu->env.regs[15],
                        (unsigned int)cpu->env.regs[1]);
                fflush(stderr);
            }
        }

        /* Apply iBSS runtime patches to RAM_BASE after self-copy.
          * The self-copy from 0x09000000 may not propagate our IBSS patches
          * (IMG2 section-based copy, not raw memcpy). Apply directly to RAM.
          * Use ARM-mode instructions for entries (0x4ea0, 0x5400) since callers
          * may execute in ARM mode. Use Thumb NOPs for the BL callers. */
         {
              static int clock_runtime_patched = 0;
              static int clock_clean_logged = 0;
              if (s5l8900_ibss_clean()) {
                  if (!clock_clean_logged) { clock_clean_logged = 1; fprintf(stderr, "CLOCK1: CLEAN BASELINE - skipping iBSS runtime patches\n"); }
              } else if (!clock_runtime_patched) {
                  clock_runtime_patched = 1;
                  /* Patch 0x3c00-0x4fff with ARM-mode instructions.
                     * CRITICAL: Thumb bytes in this region decode as ARM LDR/STR that
                     * read data and use it as jump targets. Fill entire region
                     * with ARM MOV r0,#1; BX LR.
                     * EXPERIMENT: gate behind S5L8900_IBSS_FILL_3C00 (default SKIP)
                     * so the real iBSS Thumb code in this region is preserved. */
                    if (getenv("S5L8900_IBSS_FILL_3C00")) {
                        uint32_t arm_ret[] = { 0xE3A00001, 0xE12FFF1E };
                        size_t region_size = 0x4400 - 0x3c00;
                        uint8_t *safe_region = g_malloc(region_size);
                        for (size_t i = 0; i < region_size; i += 8) {
                            memcpy(safe_region + i, arm_ret, sizeof(arm_ret));
                        }
                        cpu_physical_memory_write(S5L8900_RAM_BASE + 0x3c00,
                                                  safe_region, region_size);
                        g_free(safe_region);
                        fprintf(stderr, "CLOCK1: patched 0x3c00-0x4fff -> ARM MOV r0,#1/BX LR (%zu bytes)\n", region_size);
                    } else {
                        fprintf(stderr, "CLOCK1: SKIPPED 0x3c00-0x4fff fill (S5L8900_IBSS_FILL_3C00 unset; real iBSS code preserved)\n");
                    }

                  /* Patch 0x5400 entry: Thumb MOV r0,#0; BX LR */
                  {
                      uint8_t ret_0_thumb[] = { 0x00, 0x20, 0x70, 0x47 };
                      cpu_physical_memory_write(S5L8900_RAM_BASE + 0x5400,
                                                ret_0_thumb, sizeof(ret_0_thumb));
                  }

                  /* Write safe function pointer to 0x22012ccc.
                   * This is the data that r0 points to when the bad function runs.
                   * Point it to our safe loop at 0x4ea0 so any indirect jumps
                   * land on safe code instead of garbage addresses. */
                  {
                      uint32_t safe_ptr = S5L8900_RAM_BASE + 0x4ea0;
                      cpu_physical_memory_write(0x22012ccc, &safe_ptr, 4);
                      /* Also fix nearby pointers */
                      for (int p = -0x40; p < 0x40; p += 4) {
                          uint32_t val;
                          cpu_physical_memory_read(0x22012ccc + p, &val, 4);
                          if (val >= 0x40000000 && val < 0x100000000 &&
                              val >= S5L8900_RAM_BASE &&
                              val < S5L8900_RAM_BASE + 0x10000) {
                              // Valid RAM pointer, leave it
                          } else if (val >= 0x00200000 && val < 0x00300000) {
                              // Pre-self-copy pointer, translate to post-copy
                              uint32_t translated = val + (S5L8900_RAM_BASE - 0x00200000);
                              cpu_physical_memory_write(0x22012ccc + p, &translated, 4);
                          } else if (val >= 0xe0000000) {
                              // Obviously bad, point to safe loop
                              cpu_physical_memory_write(0x22012ccc + p, &safe_ptr, 4);
                          }
                      }
                   fprintf(stderr, "CLOCK1: fixed data pointers at 0x22012ccc\n");
                   }

                /* Skip iBSS polling loop at 0x3724: BLE #-4 -> NOP */
                  {
                      uint8_t thumb_nop[] = { 0x00, 0xBF };
                      cpu_physical_memory_write(S5L8900_RAM_BASE + 0x3724, thumb_nop, sizeof(thumb_nop));
                  }

                  /* Patch stuck function at 0x5bf0: Thumb MOV r0,#1; BX LR */
                  {
                      uint8_t thumb_ret[] = { 0x01, 0x20, 0x70, 0x47 };
                      cpu_physical_memory_write(S5L8900_RAM_BASE + 0x5bf0, thumb_ret, sizeof(thumb_ret));
                  }

                 queue_tb_flush(CPU(cpu));
                  fprintf(stderr, "CLOCK1: applied iBSS runtime patches to RAM_BASE (ARM+Thumb)\n");
     }
     }

         /* PLL status: report ALL lock/ready bits set so any single-bit
          * 'tst mask, reg' poll (mask = 1<<n) the iBSS performs succeeds.
          * (Previously returned only 0x3, which hung iBSS polls waiting on
          * bit 2 or higher of 0x3c500040.) */
         return 0xFFFFFFFF;
    }
    if (s && offset < sizeof(s->regs)) {
        return s->regs[offset / 4];
    }
    return 0;
}

static void s5l8900_clock_write(void *opaque, hwaddr offset,
                                  uint64_t value, unsigned size)
{
    S5L8900ClockState *s = opaque;
    if (s && offset < sizeof(s->regs)) {
        s->regs[offset / 4] = (uint32_t)value;
    }
}

static const MemoryRegionOps s5l8900_clock_ops = {
    .read  = s5l8900_clock_read,
    .write = s5l8900_clock_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/* ---- PMU/Sleep controller stub (0x3e500000) --------------------------------
 * iBSS polls 0x3e500004 for power status. Return 0x8 (bit 3 set) so the
 * check function (lsl #27; lsrs #31) returns 1. */
typedef struct {
    uint32_t regs[0x10000 / 4];
} S5L8900PMUState;

static uint64_t s5l8900_pmu_read(void *opaque, hwaddr offset, unsigned size)
{
    S5L8900PMUState *s = opaque;
    if (offset == 0x04) {
        return 0x8; /* Power ready bit */
    }
    if (s && offset < sizeof(s->regs)) {
        return s->regs[offset / 4];
    }
    return 0;
}

static void s5l8900_pmu_write(void *opaque, hwaddr offset,
                                uint64_t value, unsigned size)
{
    S5L8900PMUState *s = opaque;
    if (s && offset < sizeof(s->regs)) {
        s->regs[offset / 4] = (uint32_t)value;
    }
}

static const MemoryRegionOps s5l8900_pmu_ops = {
    .read  = s5l8900_pmu_read,
    .write = s5l8900_pmu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/* ---- KASLR/Keystore peripheral stub (0x38620000) --------------------------------
 * iBEC accesses this peripheral during boot. It initializes control registers
 * (0x04-0x3C) and then polls 0x80 (status) in a tight loop. The catch-all
 * returns 0, causing an infinite loop. This stub stores written values and
 * returns 0x1 for the status register (0x80) to indicate "ready/complete". */
typedef struct {
    uint32_t regs[0x200 / 4];  /* Register storage */
} S5L8900KeystoreState;

static uint64_t s5l8900_keystore_read(void *opaque, hwaddr offset, unsigned size)
{
    S5L8900KeystoreState *s = opaque;
    ARMCPU *cpu = ARM_CPU(qemu_get_cpu(0));
    if (offset == 0x80) {
        qemu_log_mask(LOG_UNIMP, "s5l8900.keystore: status read 0x80 -> 0x1 pc=0x%x\n",
                      (unsigned int)cpu->env.regs[15]);
        return 0x1;
    }
    if (s && offset < sizeof(s->regs)) {
        qemu_log_mask(LOG_UNIMP, "s5l8900.keystore: read 0x%x (size=%u) pc=0x%x\n",
                      (unsigned int)offset, size, (unsigned int)cpu->env.regs[15]);
        return s->regs[offset / 4];
    }
    return 0;
}

static void s5l8900_keystore_write(void *opaque, hwaddr offset,
                                     uint64_t value, unsigned size)
{
    S5L8900KeystoreState *s = opaque;
    if (s && offset < sizeof(s->regs)) {
        s->regs[offset / 4] = (uint32_t)value;
    }
}

static const MemoryRegionOps s5l8900_keystore_ops = {
    .read  = s5l8900_keystore_read,
    .write = s5l8900_keystore_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/* Forward declarations for USB OTG patching and config_board trigger */
static void s5l8900_usbotg_apply_patches(void);
static void s5l8900_config_board_trigger(ARMCPU *cpu);

   /* Exception vector RAM: IO-backed region that redirects all exceptions
     * to a safe infinite loop. Prevents exception cascades from unmapped
     * memory, bad instruction fetches, and mode-switch failures. */
typedef struct {
    uint8_t ram[S5L8900_EVEC_SIZE];
} S5L8900EvecState;

static S5L8900EvecState *s5l8900_evec_state = NULL;

 static void s5l8900_evec_redirect_all(void)
 {
     if (!s5l8900_evec_state) return;
     /* Redirect all exception vectors to a safe ARM-mode infinite loop.
       * ARM vectors are at 4-byte offsets: 0x00(Reset), 0x04(Undefined),
       * 0x08(SWI), 0x0C(Prefetch), 0x10(Data), 0x18(IRQ), 0x1C(FIQ).
       * Each vector gets a B instruction to common handler at 0x100.
       * The common handler is an infinite ARM loop (B itself).
       * Fill rest of 0x100 page with NOPs to prevent garbage execution. */
     uint32_t safe_loop = S5L8900_RAM_BASE + 0xF920; /* ARM safe loop address */

     /* Write B #offset at each 4-byte vector slot.
       * ARM B encoding: 0xEA000000 | (signed_imm24 << 2)
       * Offset from vector at 0x00 to handler at 0x100: 0x100/4 = 64 = 0x40
       * Offset from vector at 0x04 to handler at 0x100: (0x100-0x04)/4 = 63 = 0x3F
       * General: (0x100 - addr) / 4 */
     for (int addr = 0; addr < 0x100; addr += 4) {
         int32_t imm = (0x100 - addr) / 4;
         uint32_t b_instr = 0xEA000000 | ((imm & 0xFFFFFF) << 2);
         cpu_physical_memory_write(S5L8900_EVEC_BASE + addr, &b_instr, 4);
     }

      /* Write common exception handler at 0x100: jump to safe iBoot loop.
        * DO NOT use BX LR (returns to faulting instruction -> infinite exception loop).
        * Instead, use BLX to switch to Thumb mode and loop at 0x4EC. */
      {
          uint32_t loop_addr = 0x100;
          uint32_t safe_thumb = S5L8900_IBOOT_BASE + 0x4EC | 1; /* Thumb bit */
          /* MOVW r0, #0x2300; MOVT r0, #0x04EC; BLX r0 */
          uint32_t movw = 0xF2400000 | ((safe_thumb & 0xFFFF) << 0); /* MOVW r0, #imm16 */
          /* Actually, simpler: just use ARM infinite loop */
          uint32_t b_self = 0xEAFFFFFE; /* B #-4 (ARM infinite loop) */
          cpu_physical_memory_write(S5L8900_EVEC_BASE + loop_addr, &b_self, 4);
          cpu_physical_memory_write(S5L8900_EVEC_BASE + loop_addr + 4, &b_self, 4);

          /* Also write ARM safe loop at 0xF920 in SRAM */
          uint32_t sram_loop = S5L8900_RAM_BASE + 0xF920;
          cpu_physical_memory_write(sram_loop, &b_self, 4);
          cpu_physical_memory_write(sram_loop + 4, &b_self, 4);
          cpu_physical_memory_write(sram_loop + 8, &b_self, 4);
      }
 }

 static uint64_t s5l8900_evec_read(void *opaque, hwaddr offset, unsigned size)
   {
       S5L8900EvecState *s = opaque;
     if (!s) return 0; /* evec disabled, return NOP */
     if (s && offset + size <= S5L8900_EVEC_SIZE) {
        uint64_t val = ldn_le_p(s->ram + offset, size);
        /* Log exception type */
        {
            static const char *vec_names[] = {
                [0] = "Reset", [1] = "Undefined", [2] = "SWI",
                [3] = "PrefetchAbort", [4] = "DataAbort", [5] = "IRQ", [6] = "FIQ"
            };
            static int exc_log_cnt = 0;
            if (++exc_log_cnt <= 20) {
                int idx = offset / 4;
                const char *name = (idx < 7) ? vec_names[idx] : "Unknown";
                ARMCPU *cpu = ARM_CPU(qemu_get_cpu(0));
                fprintf(stderr, ">>> EXCEPTION: %s (0x%02x) pc=0x%08x lr=0x%08x thumb=%d cpsr=0x%08x [cnt=%d]\n",
                        name, (unsigned int)offset,
                        (unsigned int)cpu->env.regs[15], (unsigned int)cpu->env.regs[14],
                        cpu->env.thumb, (unsigned int)cpu->env.uncached_cpsr,
                        exc_log_cnt);
            }
        }

            /* Exception loop detection: if exception vectors are being read
            * repeatedly, we're in an exception loop. After 10 exceptions,
            * redirect CPU directly to iBEC by setting PC and Thumb mode. */
           static int exc_loop_cnt = 0;
            if (++exc_loop_cnt >= 10 && exc_loop_cnt <= 12) {
                 ARMCPU *cpu = ARM_CPU(qemu_get_cpu(0));
                 qemu_log_mask(LOG_UNIMP,
                     "s5l8900.evec: EXCEPTION LOOP DETECTED [%d]! pc=0x%08x thumb=%d lr=0x%08x\n",
                     exc_loop_cnt, (unsigned int)cpu->env.regs[15],
                     cpu->env.thumb, (unsigned int)cpu->env.regs[14]);
/* Write ARM infinite loop at 0xF900.
                    * The periodic callback detects PC in this region and prints to serial.
                    * This avoids STRB to MMIO which triggers data aborts. */
                   {
                       uint32_t loop = 0xEAFFFFFE; /* B #-4 */
                       for (int i = 0; i < 64; i++)
                           cpu_physical_memory_write(S5L8900_RAM_BASE + 0xF900 + i * 4, &loop, 4);
                   }
                   /* Set up stack and registers */
                   cpu->env.regs[13] = S5L8900_RAM_BASE + 0x20000; /* SP */
                   cpu->env.regs[0] = 0;
                   cpu->env.regs[1] = 0;
                   cpu->env.regs[2] = 0;
                   /* Enter ARM mode directly at UART loop */
                   cpu->env.regs[15] = S5L8900_RAM_BASE + 0xF900;
                  cpu->env.thumb = false;
                  cpu->env.uncached_cpsr = (cpu->env.uncached_cpsr & ~0x1F) | 0xD3; /* SVC + I+F */
                  CPU(cpu)->exit_request = 0;
                  queue_tb_flush(CPU(cpu));
                  cpu_reset_interrupt(qemu_get_cpu(0), CPU_INTERRUPT_EXITTB);
                  qemu_log_mask(LOG_UNIMP,
                      "s5l8900.evec: redirected CPU to ARM UART loop at 0x%08x\n",
                      S5L8900_RAM_BASE + 0xF900);
             }
         return val;
     }
     return 0;
 }

static void s5l8900_evec_write(void *opaque, hwaddr offset,
                                uint64_t value, unsigned size)
{
    S5L8900EvecState *s = opaque;
    if (s && offset + size <= S5L8900_EVEC_SIZE) {
        stn_le_p(s->ram + offset, size, value);
    }
    /* Any write to the vector table: re-patch all vectors to trampoline.
     * This handles ROM/iBSS overwrites. */
    s5l8900_evec_redirect_all();
}

static const MemoryRegionOps s5l8900_evec_ops = {
    .read  = s5l8900_evec_read,
    .write = s5l8900_evec_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/* Periodic CPU state dump for debugging infinite loops.
 * Fires every 2 seconds via QEMU virtual timer. */
static QEMUTimer *s5l8900_periodic_timer = NULL;
static QEMUTimer *s5l8900_step_timer = NULL; /* One-shot timer for immediate post-redirect tracing */
static int s5l8900_step_count = 0; /* Number of steps to trace */
static Chardev *s5l8900_serial_chr = NULL; /* Serial Chardev for direct writes */

/* Fine-grained (3us) watcher on the iBoot dispatch literal pool
 * 0x18005FD0..0x18006008. iBoot's init relocates this pool (rewrites the
 * table base + a data structure). The 1ms POOLCHG only saw the change
 * post-crash; this fine timer catches the writer PC within ~a few TBs of
 * the store. Also snapshots the dispatch (0x18005ED6) register state. */
static QEMUTimer *s5l8900_poolwatch_timer = NULL;
static int s5l8900_poolwatch_changes = 0;
static int s5l8900_poolwatch_armed = 0;
static uint32_t s5l8900_poolwatch_prev[16];
static int s5l8900_poolwatch_prev_valid = 0;

/* High-priority IO read hook on the dispatch literal-pool window
 * 0x18006000..0x18006008. Forwards every read to the backing USBOTG RAM
 * but logs the offset, value and PC. This DEFINITIVELY shows which word the
 * dispatch LDR (0x18005ed6: ldr r3,[pc,#0x128]) reads as the table base,
 * resolving the [0x6000] vs [0x6002] PC-convention ambiguity. */
static uint8_t *s5l8900_poolbase_ram = NULL;
static int s5l8900_poolbase_reads = 0;
static int s5l8900_group_dumped = 0;

/* Write-hook on the iBoot ARM code region (0x18017000-0x1801A000). Forwards
 * every read/write to the backing USBOTG RAM but logs the writer PC on any
 * write to the code region. iBoot inits corrupt this region (e.g. the multiply
 * at 0x180189c0 loses its prologue); the log pinpoints the exact store. Armed
 * (s5l8900_codehook_active) only AFTER the bulk iBoot image load so the load
 * itself is not logged. */
#define S5L8900_CODEHOOK_BASE 0x18017000
#define S5L8900_CODEHOOK_SIZE 0x9000
static int s5l8900_codehook_active = 0;
static int s5l8900_codehook_writes = 0;

/* One-time dump of the iBoot command group list, fired the first time the
 * console dispatch (0x5ED6) reads its table base. At that point the entry
 * dispatcher has run, so any registered command groups are already linked in.
 * Group node layout (from matcher 0x60DC): {next@0, entries@4}; entry 12B
 * {name@0 (ptr to C string), handler@4, flag@8}; entry list ends at name==0. */
static void s5l8900_dump_command_groups(void)
{
    uint32_t cur = 0x18021198; /* [0x61C0] head node */
    fprintf(stderr, "GROUP_DUMP: head node @0x%08x\n", cur);
    for (int g = 0; g < 16 && cur; g++) {
        uint32_t node[2] = {0};
        cpu_physical_memory_read(cur, node, 8);
        fprintf(stderr, "GROUP_DUMP: group[%d] @0x%08x next=0x%08x entries=0x%08x\n",
                g, cur, node[0], node[1]);
        if (node[1]) {
            for (int e = 0; e < 24; e++) {
                uint32_t ent[3] = {0};
                cpu_physical_memory_read(node[1] + (hwaddr)e * 12, ent, 12);
                if (!ent[0] && !ent[1]) break;
                char name[24] = {0};
                if (ent[0]) {
                    cpu_physical_memory_read(ent[0], name, sizeof(name) - 1);
                }
                fprintf(stderr, "GROUP_DUMP:   cmd[%d] name=\"%s\" handler=0x%08x flag=0x%08x\n",
                        e, name, ent[1], ent[2]);
            }
        }
        cur = node[0];
    }
}

static uint64_t s5l8900_poolbase_read(void *opaque, hwaddr offset, unsigned size)
{
    uint64_t val = 0;
    if (s5l8900_poolbase_ram && offset + size <= 8) {
        val = ldn_le_p(s5l8900_poolbase_ram + offset, size);
    }
    /* Only log reads issued by the dispatch (0x18005ed6: ldr r3,[pc,#0x128]).
     * The iBoot self-relocation loop also reads this pool heavily; filter it
     * out so the dispatch's table-base read stands out. Guard first_cpu in
     * case the callback runs outside CPU execution. */
    if (first_cpu && s5l8900_poolbase_reads < 256) {
        ARMCPU *cpu = ARM_CPU(first_cpu);
        uint32_t pc = (uint32_t)cpu->env.regs[15];
        uint32_t lr = (uint32_t)cpu->env.regs[14];
        if (pc >= 0x18005ed0 && pc <= 0x18005ee0) {
            fprintf(stderr, "DISPATCH_READ: [0x1800%04x] size=%u val=0x%08x pc=0x%08x lr=0x%08x\n",
                    0x6000 + (unsigned)offset, size, (uint32_t)val, pc, lr);
            s5l8900_poolbase_reads++;
            if (!s5l8900_group_dumped) {
                s5l8900_group_dumped = 1;
                s5l8900_dump_command_groups();
            }
        }
    }
    return val;
}

static int s5l8900_poolbase_writes = 0;

static void s5l8900_poolbase_write(void *opaque, hwaddr offset,
                                   uint64_t value, unsigned size)
{
    if (s5l8900_poolbase_ram && offset + size <= 8) {
        stn_le_p(s5l8900_poolbase_ram + offset, size, value);
    }
    /* Log every write of the [0x6000] table-base word (offset 0, size 4) with
     * the writer PC. This pinpoints the instruction that sets [0x6000] to the
     * garbage 0xd2842201 the dispatch reads. Filter to 4-byte aligned writes
     * at offset 0 to skip the bulk iBoot image load (which goes through
     * cpu_physical_memory_write and is not CPU-store traffic). */
    if (first_cpu && offset == 0 && size == 4 && s5l8900_poolbase_writes < 128) {
        ARMCPU *cpu = ARM_CPU(first_cpu);
        uint32_t pc = (uint32_t)cpu->env.regs[15];
        uint32_t lr = (uint32_t)cpu->env.regs[14];
        uint32_t r0 = (uint32_t)cpu->env.regs[0];
        uint32_t r1 = (uint32_t)cpu->env.regs[1];
        uint32_t r3 = (uint32_t)cpu->env.regs[3];
        fprintf(stderr, "POOLWRITE: [0x18006000] = 0x%08x  pc=0x%08x lr=0x%08x r0=0x%08x r1=0x%08x r3=0x%08x thumb=%d\n",
                (uint32_t)value, pc, lr, r0, r1, r3, cpu->env.thumb);
        /* Dump the actual instruction bytes at the writer PC and the caller LR,
         * plus cpsr, to resolve the Thumb/ARM mode mismatch. */
        {
            uint8_t wbuf[16] = {0};
            uint8_t lbuf[16] = {0};
            cpu_physical_memory_read(pc, wbuf, sizeof(wbuf));
            cpu_physical_memory_read(lr & ~1u, lbuf, sizeof(lbuf));
            fprintf(stderr, "POOLWRITE_CODE: cpsr=0x%08x\n", (unsigned)cpu->env.uncached_cpsr);
            fprintf(stderr, "  @pc 0x%08x:", pc);
            for (int k = 0; k < 16; k++) fprintf(stderr, " %02x", wbuf[k]);
            fprintf(stderr, "\n  @lr 0x%08x:", lr & ~1u);
            for (int k = 0; k < 16; k++) fprintf(stderr, " %02x", lbuf[k]);
            fprintf(stderr, "\n");
        }
        s5l8900_poolbase_writes++;
    }
}

static const MemoryRegionOps s5l8900_poolbase_ops = {
    .read = s5l8900_poolbase_read,
    .write = s5l8900_poolbase_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static void s5l8900_dump_ptr(const char *label, uint32_t ptr)
{
    if (ptr >= 0x1000 &&
        (ptr < 0x18000000 || ptr >= 0x1a000000) &&
        (ptr < 0x22000000 || ptr >= 0x24000000) &&
        (ptr < 0x60000000 || ptr >= 0x80000000)) {
        fprintf(stderr, "DUMPPTR %s ptr=0x%08x skipped\n", label, ptr);
        return;
    }
    uint8_t buf[64] = {0};
    cpu_physical_memory_read(ptr, buf, sizeof(buf));
    fprintf(stderr, "DUMPPTR %s ptr=0x%08x hex:", label, ptr);
    for (size_t i = 0; i < sizeof(buf); i++) {
        fprintf(stderr, "%s%02x", (i % 8 == 0) ? " " : "", buf[i]);
    }
    fprintf(stderr, " ascii:");
    for (size_t i = 0; i < sizeof(buf); i++) {
        fprintf(stderr, "%c", (buf[i] >= 32 && buf[i] < 127) ? buf[i] : '.');
    }
    fprintf(stderr, "\n");
}

static uint64_t s5l8900_codehook_read(void *opaque, hwaddr offset, unsigned size)
{
    hwaddr base = (hwaddr)(uintptr_t)opaque;
    hwaddr addr = base + offset;
    /* One-shot: log when the alloc stub 0x18007a34 is fetched, to confirm it
     * is actually executed (and dump its live bytes at that instant). */
    if (addr >= 0x18007a34 && addr < 0x18007a3c) {
        static int stub_fetch_logged = 0;
        if (stub_fetch_logged < 3 && first_cpu) {
            ARMCPU *cpu = ARM_CPU(first_cpu);
            uint8_t sb[8] = {0};
            cpu_physical_memory_read(0x18007a34, sb, sizeof(sb));
            fprintf(stderr, "STUBFETCH 0x18007a34 @%d: %02x %02x %02x %02x %02x %02x %02x %02x  pc=0x%08x lr=0x%08x thumb=%d\n",
                    stub_fetch_logged, sb[0], sb[1], sb[2], sb[3], sb[4], sb[5], sb[6], sb[7],
                    (uint32_t)cpu->env.regs[15], (uint32_t)cpu->env.regs[14], cpu->env.thumb);
            stub_fetch_logged++;
        }
    }
    /* One-shot: when the char-dispatch branch 0x180172ba is fetched, dump the
     * live table base ptr and the 'l' entry vs pristine, plus r1/r3. */
    if (addr >= 0x180172b8 && addr < 0x180172bc) {
        static int chdisp_logged = 0;
        if (chdisp_logged < 3 && first_cpu) {
            ARMCPU *cpu = ARM_CPU(first_cpu);
            uint32_t base_ptr = 0, leb = 0, r1v = 0, r3v = 0;
            cpu_physical_memory_read(0x18017628, &base_ptr, 4);
            cpu_physical_memory_read(0x18016fe0, &leb, 4);
            r1v = (uint32_t)cpu->env.regs[1];
            r3v = (uint32_t)cpu->env.regs[3];
            uint32_t lr = (uint32_t)cpu->env.regs[14];
            uint32_t r0 = (uint32_t)cpu->env.regs[0];
            uint32_t r4 = (uint32_t)cpu->env.regs[4];
            uint32_t r8 = (uint32_t)cpu->env.regs[8];
            uint32_t sl = (uint32_t)cpu->env.regs[11];
            fprintf(stderr, "CHARDISP2 fetch@%d: r1(byte)=0x%02x r3=0x%08x tblbase=0x%08x 'l'entry=0x%08x lr=0x%08x r0=0x%08x r4=0x%08x r8=0x%08x sl=0x%08x cpsr=0x%08x thumb=%d (pristine 0x2801fe7d)\n",
                    chdisp_logged, r1v, r3v, base_ptr, leb, lr, r0, r4, r8, sl,
                    (unsigned)cpu->env.uncached_cpsr, cpu->env.thumb);
            s5l8900_dump_ptr("sl-1", sl ? sl - 1 : 0);
            s5l8900_dump_ptr("r8", r8);
            s5l8900_dump_ptr("cmdbuf", 0x22011100);
            chdisp_logged++;
        }
    }
    if (s5l8900_poolbase_ram) {
        return ldn_le_p(s5l8900_poolbase_ram + (addr - 0x18000000), size);
    }
    return 0;
}

static void s5l8900_codehook_write(void *opaque, hwaddr offset,
                                   uint64_t value, unsigned size)
{
    hwaddr base = (hwaddr)(uintptr_t)opaque;
    hwaddr addr = base + offset;
    if (s5l8900_poolbase_ram) {
        stn_le_p(s5l8900_poolbase_ram + (addr - 0x18000000), size, value);
    }
    if (s5l8900_codehook_active && first_cpu) {
        ARMCPU *cpu = ARM_CPU(first_cpu);
        uint32_t pc = (uint32_t)cpu->env.regs[15];
        uint32_t lr = (uint32_t)cpu->env.regs[14];
        /* The clobbered window 0x18017a80-0x18017aa0 is logged unconditionally
         * (any PC, bypasses the write counter) to catch the exact QEMU-side
         * writer even after the mirror's traffic exhausts the counter. Everything
         * else: only log CPU stores from iBoot code, within the write budget. */
        int in_window = ((addr >= 0x18017a80) && (addr < 0x18017aa0)) ||
                        ((addr >= 0x1801f030) && (addr < 0x1801f100));
        if (!in_window && (s5l8900_codehook_writes >= 512)) {
            return;
        }
        if (!in_window && (pc < 0x18000000 || pc >= 0x18030000)) {
            return;
        }
        fprintf(stderr, "CODEWRITE: [0x%08x] = 0x%016" PRIx64 " (sz %u) pc=0x%08x lr=0x%08x thumb=%d cpsr=0x%08x\n",
                (uint32_t)addr, value, size, pc, lr, cpu->env.thumb,
                (unsigned)cpu->env.uncached_cpsr);
        {
            uint8_t wbuf[8] = {0};
            cpu_physical_memory_read(pc & ~1u, wbuf, sizeof(wbuf));
            fprintf(stderr, "CODEWRITE_CODE: @pc 0x%08x:", pc);
            for (int k = 0; k < 8; k++) fprintf(stderr, " %02x", wbuf[k]);
            fprintf(stderr, "\n");
        }
        s5l8900_codehook_writes++;
    }
}

static const MemoryRegionOps s5l8900_codehook_ops = {
    .read = s5l8900_codehook_read,
    .write = s5l8900_codehook_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/* One-shot exception diagnostic: reads exception type marker from SRAM trace. */
static void s5l8900_exc_diag_cb(void *opaque)
{
    uint32_t markers[8];
    cpu_physical_memory_read(S5L8900_RAM_BASE + 0xED00, markers, sizeof(markers));
    const char *exc_names[] = {"Reset", "Undefined", "SWI", "PAbort", "DAbort", "IRQ", "FIQ", "Reserved"};
    int found = 0;
    for (int i = 0; i < 8; i++) {
        if (markers[i] != 0) {
            fprintf(stderr, ">>> EXC DIAG: %s exception fired (marker[%d]=0x%08x)\n",
                    exc_names[i], i, markers[i]);
            found = 1;
        }
    }
    if (!found) {
        fprintf(stderr, ">>> EXC DIAG: no exceptions caught\n");
    }
}

/* CPU state dump via run_on_cpu: runs safely in CPU thread context. */
static void s5l8900_cpu_dump_cb(CPUState *cs, run_on_cpu_data data)
{
    ARMCPU *cpu = ARM_CPU(cs);
    uint32_t pc = (uint32_t)cpu->env.regs[15];
    fprintf(stderr, ">>> CPU DUMP: pc=0x%08x thumb=%d cpsr=0x%08x sp=0x%08x lr=0x%08x r0=0x%08x\n",
            pc, cpu->env.thumb,
            (unsigned int)cpu->env.uncached_cpsr,
            (unsigned int)cpu->env.regs[13],
            (unsigned int)cpu->env.regs[14],
            (unsigned int)cpu->env.regs[0]);
    /* Dump bytes at PC */
    if (pc >= 0x18000000 || pc >= 0x22000000) {
        uint32_t addr = pc & ~1u;
        uint8_t inst[16];
        cpu_physical_memory_read(addr, inst, sizeof(inst));
        fprintf(stderr, ">>> CPU DUMP: bytes @ 0x%08x: ", addr);
        for (int i = 0; i < 12; i++) fprintf(stderr, "%02x ", inst[i]);
        fprintf(stderr, "\n");
    }
}

/* Timer callback that schedules the CPU dump via run_on_cpu */
static void s5l8900_diag_timer_cb(void *opaque)
{
    s5l8900_exc_diag_cb(opaque);
    /* Dump CPU state safely */
    run_on_cpu(qemu_get_cpu(0), s5l8900_cpu_dump_cb, RUN_ON_CPU_HOST_PTR(NULL));
}

/* CPU redirect via run_on_cpu: runs in CPU thread context between TBs. */
static void s5l8900_cpu_redirect_cb(CPUState *cs, run_on_cpu_data data)
{
    ARMCPU *cpu = ARM_CPU(cs);
    uint32_t target_pc = data.host_ulong;

    fprintf(stderr, ">>> REDIRECT CB: BEFORE - pc=0x%08x thumb=%d exc=%d target=0x%08x\n",
            cpu->env.regs[15], cpu->env.thumb, (int)cs->exception_index, target_pc);

    /* Disable MMU */
    cpu->env.cp15.sctlr_s = 0;
    cpu->env.cp15.ttbr0_s = 0;
    cpu->env.cp15.ttbr1_s = 0;

    /* Redirect to target (ARM trampoline in SRAM that jumps to main_init) */
    cpu->env.regs[15] = target_pc;
    cpu->env.thumb = 0; /* ARM mode */
    cpu->env.uncached_cpsr = 0xD3; /* SVC, I+F, V=0 (low vectors) */
    arm_rebuild_hflags(&cpu->env);

    /* Set safe registers.
     * r0 = command-registration init 0x18000AE0 (Thumb). The BX r0 trampoline
     * jumps into it; it registers all console command groups (calls 0x18001dbc,
     * 0x180094a8, 0x1800f210, 0x1800085c, ...) then returns.
     * lr = main_init 0x18005CA0 (Thumb) so that after registration the CPU
     * falls back into the console input loop. Previously r0 went straight to
     * main_init, skipping 0x18000AE0, so the command group list stayed empty
     * and the dispatcher table had nothing to match. */
    cpu->env.regs[14] = S5L8900_IBOOT_RUNTIME + 0x5CA0 | 1;
    cpu->env.regs[13] = S5L8900_RAM_BASE + 0x20000;
    cpu->env.regs[0] = S5L8900_IBOOT_RUNTIME + 0x0AE0 | 1;
    fprintf(stderr, ">>> REDIRECT CB: set r0=0x%08x (reg-init) lr=0x%08x (main_init)\n",
            (unsigned int)cpu->env.regs[0], (unsigned int)cpu->env.regs[14]);

    /* CRITICAL: Initialize ALL banked stack pointers.
     * When an exception occurs, ARM switches to the banked SP for that
     * exception mode. If the banked SP is garbage, the exception handler
     * will crash. Bank indices: 0=USR/SYS, 1=SVC, 2=ABT, 3=UND, 4=IRQ, 5=FIQ */
    {
        uint32_t safe_sp = S5L8900_RAM_BASE + 0x30000;
        int i;
        for (i = 0; i < 8; i++) {
            cpu->env.banked_r13[i] = safe_sp;
        }
        fprintf(stderr, ">>> REDIRECT CB: initialized banked SPs to 0x%08x\n", safe_sp);
    }

    /* Verify ALL registers after setting */
    fprintf(stderr, ">>> REDIRECT CB: AFTER SET - pc=0x%08x sp=0x%08x lr=0x%08x thumb=%d cpsr=0x%08x exit_req=%d\n",
            cpu->env.regs[15], cpu->env.regs[13], cpu->env.regs[14],
            cpu->env.thumb, (unsigned int)cpu->env.uncached_cpsr, (int)cs->exit_request);

    /* Force exec loop to restart with new PC */
    queue_tb_flush(cs);
    cs->exception_index = -1;
    cpu_interrupt(cs, CPU_INTERRUPT_EXITTB);
    fprintf(stderr, ">>> REDIRECT CB: forced TB exit for restart\n");

    /* Verify bytes at main_init AND entry function */
    uint8_t verify[16];
    cpu_physical_memory_read(cpu->env.regs[15], verify, sizeof(verify));
    fprintf(stderr, ">>> REDIRECT CB: main_init @ 0x%08x: ", cpu->env.regs[15]);
    for (int i = 0; i < 8; i++) fprintf(stderr, "%02x ", verify[i]);
    fprintf(stderr, "\n");
    /* Also check entry function at 0x4C00 */
    cpu_physical_memory_read(S5L8900_IBOOT_BASE + 0x4C00, verify, 8);
    fprintf(stderr, ">>> REDIRECT CB: entry    @ 0x%08x: ", S5L8900_IBOOT_BASE + 0x4C00);
    for (int i = 0; i < 8; i++) fprintf(stderr, "%02x ", verify[i]);
    fprintf(stderr, "\n");

    /* Signal: iBoot launched, periodic callback should not interfere */
    s5l8900_iboot_launched = 1;

    /* DO NOT restart timers - they corrupt TCG state by reading env->regs during execution */
    if (s5l8900_periodic_timer) {
        timer_del(s5l8900_periodic_timer);
    }
    if (s5l8900_step_timer) {
        timer_del(s5l8900_step_timer);
    }
    fprintf(stderr, ">>> REDIRECT CB: iBoot launched, all timers stopped\n");

    /* One-shot: read exception trace and CPU state after 1s */
    {
        static QEMUTimer *exc_diag = NULL;
        if (!exc_diag) {
            exc_diag = timer_new_ns(QEMU_CLOCK_REALTIME, s5l8900_diag_timer_cb, NULL);
        }
        timer_mod(exc_diag, qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + 1ULL * 1000 * 1000 * 1000);
    }
}

/* Scan [start, end) for arrays of 8 consecutive Thumb function pointers
 * (pointing into iBoot image or SRAM). Used to locate the runtime command
 * dispatch table at crash time. */
static void s5l8900_scan_for_tables(hwaddr start, hwaddr end, const char *tag)
{
    hwaddr a;
    for (a = start; a + 32 <= end; a += 2) {
        uint32_t t[8];
        cpu_physical_memory_read(a, t, 32);
        int ok = 1;
        for (int k = 0; k < 8; k++) {
            uint32_t v = t[k];
            uint32_t p = v & ~1u;
            int in_iboot = (v & 1) && (p >= 0x18000000) && (p < 0x18030000);
            int in_sram  = (v & 1) && (p >= 0x22000000) && (p < 0x23000000);
            if (!in_iboot && !in_sram) { ok = 0; break; }
        }
        if (ok) {
            fprintf(stderr, "DISPATCH: %s TABLE_CAND @0x%08x: %08x %08x %08x %08x %08x %08x %08x %08x\n",
                tag, (unsigned int)a, t[0],t[1],t[2],t[3],t[4],t[5],t[6],t[7]);
        }
    }
}

/* Fine-grained (3us) pool watcher. Runs while armed; fires at TB boundaries
 * ~every 3us of real time. Tracks the dispatch literal pool
 * 0x18005FD0..0x18006008 (16 words) and logs every word change with the
 * current PC/LR/instruction, so the writer of the relocated table base is
 * caught within a few instructions of the store. Also snapshots the dispatch
 * (0x18005ED6) register state (table base, index, handler) when reached. */
static void s5l8900_poolwatch_cb(void *opaque)
{
    ARMCPU *cpu = ARM_CPU(qemu_get_cpu(0));
    uint32_t pc = (uint32_t)cpu->env.regs[15];
    uint32_t lr = (uint32_t)cpu->env.regs[14];
    uint32_t r2 = (uint32_t)cpu->env.regs[2];
    uint32_t r3 = (uint32_t)cpu->env.regs[3];
    uint32_t r4 = (uint32_t)cpu->env.regs[4];
    uint8_t thumb = cpu->env.thumb;

    /* Track the FULL SRAM dispatch table (8 entries at 0x2200d284) over time
     * and log every per-word change with the writer PC. This reveals the
     * population sequence (which entry is filled when) relative to the
     * console dispatch, so the 0x10 (unfilled) entry at dispatch is identified. */
    {
        static uint32_t last_tbl[8];
        static int last_tbl_valid = 0;
        uint32_t cur_tbl[8];
        cpu_physical_memory_read(0x2200d284, cur_tbl, 32);
        if (last_tbl_valid) {
            for (int k = 0; k < 8; k++) {
                if (cur_tbl[k] != last_tbl[k]) {
                    fprintf(stderr, "SRAMTBL: [0x2200d284+%x] e[%d] %08x -> %08x  pc=0x%08x lr=0x%08x thumb=%d\n",
                            4*k, k, last_tbl[k], cur_tbl[k], pc, lr, thumb);
                }
            }
        }
        memcpy(last_tbl, cur_tbl, sizeof(cur_tbl));
        last_tbl_valid = 1;
    }

    /* Snapshot the dispatch state the moment the CPU is at the dispatch
     * (0x18005ED6..0x18005EDD). Log table base [0x6002], index r4, offset r2,
     * and the handler word [base + r2] a few times (the value can change
     * between the base-load and the handler-load). */
    if (pc >= 0x18005ED6 && pc <= 0x18005EDD) {
        uint32_t base = 0;
        cpu_physical_memory_read(0x18006002, &base, 4);
        uint32_t handler = 0;
        if (base && (base + r2 < 0x30000000) && (base + r2 >= 0x1000)) {
            cpu_physical_memory_read(base + r2, &handler, 4);
        }
        fprintf(stderr, "DISP_STATE: pc=0x%08x lr=0x%08x thumb=%d base[0x6002]=0x%08x r4(idx)=%u r2(off)=%u handler@[0x%08x]=0x%08x\n",
                pc, lr, thumb, base, r4, r2, base + r2, handler);
    }

    /* Log every change of the dispatch table base [0x6002] with the current
     * PC, so the writer of the relocated base is caught. (The full 16-word
     * diff below is too chatty; this single-word tracker pinpoints the
     * relocation.) */
    {
        static uint32_t last_base = 0;
        uint32_t b0 = 0;
        cpu_physical_memory_read(0x18006002, &b0, 4);
        if (b0 != last_base) {
            fprintf(stderr, "POOLWATCH_BASE: base[0x6002] %08x -> %08x  pc=0x%08x lr=0x%08x thumb=%d\n",
                    last_base, b0, pc, lr, thumb);
            last_base = b0;
        }
    }

    uint32_t cur[16];
    cpu_physical_memory_read(0x18005FD0, cur, 64);
    if (s5l8900_poolwatch_prev_valid) {
        for (int k = 0; k < 16 && s5l8900_poolwatch_changes < 64; k++) {
            if (cur[k] != s5l8900_poolwatch_prev[k]) {
                fprintf(stderr, "POOLW: off=0x%08x word[%2d] %08x -> %08x  pc=0x%08x lr=0x%08x thumb=%d\n",
                        0x18005FD0 + 4*k, k, s5l8900_poolwatch_prev[k], cur[k], pc, lr, thumb);
                s5l8900_poolwatch_changes++;
            }
        }
    }
    memcpy(s5l8900_poolwatch_prev, cur, sizeof(cur));
    s5l8900_poolwatch_prev_valid = 1;

    if (s5l8900_poolwatch_armed && s5l8900_poolwatch_timer) {
        timer_mod(s5l8900_poolwatch_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + 2ULL * 1000);
    }
}

/* Immediate post-redirect tracing timer callback.
 * Fires every 1ms to trace the first instructions after iBoot redirect. */
static void s5l8900_step_trace_cb(void *opaque)
{
    ARMCPU *cpu = ARM_CPU(qemu_get_cpu(0));
    uint32_t pc = (uint32_t)cpu->env.regs[15];
    uint32_t lr = (uint32_t)cpu->env.regs[14];
    uint32_t sp = (uint32_t)cpu->env.regs[13];
    uint32_t cpsr = (uint32_t)cpu->env.uncached_cpsr;
    uint8_t thumb = cpu->env.thumb;
    int exc = (int)CPU(cpu)->exception_index;

    /* One-shot DFAR/DFSR capture: when the CPU is in Abort mode (0x17),
     * record the actual faulting address + fault status so we know exactly
     * what iBoot is trying to access and why. */
    {
        static int dfar_dumped = 0;
        uint32_t mode = cpsr & 0x1F;
        if (!dfar_dumped && mode == 0x17) {
            uint32_t dfar_s = (uint32_t)cpu->env.cp15.dfar_s;
            uint32_t dfar_ns = (uint32_t)cpu->env.cp15.dfar_ns;
            uint32_t ifar_s = (uint32_t)cpu->env.cp15.ifar_s;
            uint32_t dfsr_s = (uint32_t)cpu->env.cp15.dfsr_s;
            fprintf(stderr, "DFAR-CAP: mode=0x%02x pc=0x%08x lr=0x%08x cpsr=0x%08x exc=%d dfar_s=0x%08x dfar_ns=0x%08x ifar_s=0x%08x dfsr_s=0x%08x\n",
                    mode, pc, lr, cpsr, exc, dfar_s, dfar_ns, ifar_s, dfsr_s);
            uint32_t r[8];
            for (int k = 0; k < 8; k++) r[k] = (uint32_t)cpu->env.regs[k];
            fprintf(stderr, "DFAR-CAP: r0=0x%08x r1=0x%08x r2=0x%08x r3=0x%08x\n", r[0], r[1], r[2], r[3]);
            fprintf(stderr, "DFAR-CAP: r4=0x%08x r5=0x%08x r6=0x%08x r7=0x%08x sp=0x%08x\n", r[4], r[5], r[6], r[7], sp);
            /* Dump the pool counter + pool entries + literal pool at fault time. */
            uint32_t counter = 0, pool[8] = {0}, lit[4] = {0};
            cpu_physical_memory_read(0x180236c0, &counter, 4);
            cpu_physical_memory_read(0x180236c4, pool, 32);
            cpu_physical_memory_read(0x18007d8c, lit, 16);
            fprintf(stderr, "DFAR-CAP: poolcounter[0x180236c0]=0x%08x\n", counter);
            fprintf(stderr, "DFAR-CAP: pool[0x180236c4..]: ");
            for (int k = 0; k < 8; k++) fprintf(stderr, "0x%08x ", pool[k]);
            fprintf(stderr, "\nDFAR-CAP: lit[0x18007d8c..]: ");
            for (int k = 0; k < 4; k++) fprintf(stderr, "0x%08x ", lit[k]);
            fprintf(stderr, "\n");
            /* Wider dump: find the extent of the 0xe51ff000 fill around the pool. */
            uint32_t wide[96];
            cpu_physical_memory_read(0x18023600, wide, sizeof(wide));
            fprintf(stderr, "DFAR-CAP: wide[0x18023600..+0x180]: ");
            for (int k = 0; k < 96; k++) {
                fprintf(stderr, "%08x ", wide[k]);
                if ((k & 7) == 7) fprintf(stderr, "\nDFAR-CAP:   ");
            }
            fprintf(stderr, "\n");
            /* Also check BSS start + a sample across BSS for the same fill. */
            uint32_t bss_s = 0, bss_mid = 0, bss_e = 0;
            cpu_physical_memory_read(0x18021980, &bss_s, 4);
            cpu_physical_memory_read(0x18024000, &bss_mid, 4);
            cpu_physical_memory_read(0x18025ff0, &bss_e, 4);
            fprintf(stderr, "DFAR-CAP: bss_start[0x18021980]=0x%08x bss_mid[0x18024000]=0x%08x bss_end[0x18025ff0]=0x%08x\n",
                    bss_s, bss_mid, bss_e);
            /* Dump MMU state + the effective page-table entry for the faulting
             * section, to determine whether the live page table maps the target
             * as RW (or whether iBoot replaced TTBR0 with its own table). */
            {
                uint32_t ttbr0 = (uint32_t)cpu->env.cp15.ttbr0_s;
                uint32_t sctlr = (uint32_t)cpu->env.cp15.sctlr_s;
                uint32_t dacr  = (uint32_t)cpu->env.cp15.dacr_s;
                uint32_t ptbase = ttbr0 & 0xFFFFFC00u;
                uint32_t sect = (dfar_s >> 20) & 0xFFF;
                uint32_t ptentry = 0;
                cpu_physical_memory_read(ptbase + sect * 4, &ptentry, 4);
                fprintf(stderr,
                    "DFAR-CAP MMU: TTBR0=0x%08x SCTLR=0x%08x DACR=0x%08x ptbase=0x%08x sect=0x%03x ptentry=0x%08x\n",
                    ttbr0, sctlr, dacr, ptbase, sect, ptentry);
            }
            /* Dump the stack (SP and a window around it) to find a corrupted
             * return address or branch target. */
            {
                uint32_t stk[32];
                cpu_physical_memory_read(sp, stk, sizeof(stk));
                fprintf(stderr, "DFAR-CAP stack[sp-0x40..sp+0x3c] (sp=0x%08x):\n", (unsigned)sp);
                for (int k = 0; k < 32; k++) {
                    fprintf(stderr, "  [0x%08x]=0x%08x%s\n", sp + k * 4, stk[k],
                            (k & 7) == 7 ? "" : "");
                }
            }
            dfar_dumped = 1;
        }
    }

    uint8_t inst[8];
    uint32_t read_addr = (thumb) ? (pc & ~1u) : pc;
    cpu_physical_memory_read(read_addr, inst, sizeof(inst));

    /* After step count expires, trace once every 100 fires to monitor */
    static int step_monitor_cnt = 0;
    step_monitor_cnt++;

    /* Poll-watcher: find who writes the pool-counter global 0x180236c0.
     * Log the PC on every tick where the value changes (first 32 changes). */
    {
        static uint32_t cnt_prev = 0;
        static int cnt_prev_valid = 0;
        static int cnt_changes = 0;
        uint32_t cnt_cur = 0;
        cpu_physical_memory_read(0x180236c0, &cnt_cur, 4);
        if (cnt_prev_valid && cnt_cur != cnt_prev && cnt_changes < 32) {
            fprintf(stderr, "CNTWATCH[%d]: [0x180236c0] 0x%08x -> 0x%08x  pc=0x%08x lr=0x%08x thumb=%d cpsr=0x%08x\n",
                    cnt_changes, cnt_prev, cnt_cur, pc, lr, thumb, cpsr);
            cnt_changes++;
        }
        cnt_prev = cnt_cur;
        cnt_prev_valid = 1;
    }

    /* One-shot: when stuck at the exception-handler data slot 0x2200f828 in
     * IRQ mode, dump the low vector table + the handler bytes to see why the
     * IRQ vector leads into data. */
    if (pc == 0x2200f828) {
        static int crash_dumped = 0;
        if (!crash_dumped) {
            uint32_t vec[8];
            uint8_t h[48];
            cpu_physical_memory_read(0x00000000, vec, sizeof(vec));
            cpu_physical_memory_read(0x2200f800, h, sizeof(h));
            fprintf(stderr, "CRASH-DUMP vec[0]reset=0x%08x vec[1]undef=0x%08x vec[2]swi=0x%08x vec[3]pabt=0x%08x\n",
                    vec[0], vec[1], vec[2], vec[3]);
            fprintf(stderr, "CRASH-DUMP vec[4]dabt=0x%08x vec[5]resv=0x%08x vec[6]IRQ=0x%08x vec[7]FIQ=0x%08x\n",
                    vec[4], vec[5], vec[6], vec[7]);
            fprintf(stderr, "CRASH-DUMP handler@0x2200f800:");
            for (int k = 0; k < 48; k++) fprintf(stderr, " %02x", h[k]);
            fprintf(stderr, "\nCRASH-DUMP lr=0x%08x sp=0x%08x cpsr=0x%08x exc=%d\n",
                    lr, sp, cpsr, exc);
            crash_dumped = 1;
        }
    }

    if (s5l8900_step_count > 0) {
        s5l8900_step_count--;
        fprintf(stderr, "STEP[%02d]: pc=0x%08x thumb=%d exc=%d lr=0x%08x sp=0x%08x cpsr=0x%08x inst=%02x %02x %02x %02x %02x %02x %02x %02x\n",
                20 - s5l8900_step_count, pc, thumb, exc, lr, sp, cpsr,
                inst[0], inst[1], inst[2], inst[3], inst[4], inst[5], inst[6], inst[7]);
    } else if (step_monitor_cnt % 100 == 0) {
        fprintf(stderr, "MONITOR: pc=0x%08x thumb=%d cpsr=0x%08x\n", pc, thumb, cpsr);
        /* One-shot: at the char-dispatch branch 0x180172ba, dump the live
         * table base ptr + the 'l' entry (0x18016fe0) vs pristine 0x2801fe7d. */
        if (pc == 0x180172ba) {
            static int tbl_dumped = 0;
            if (!tbl_dumped) {
                uint32_t base_ptr, leb, r1v, r3v;
                cpu_physical_memory_read(0x18017628, &base_ptr, 4);
                cpu_physical_memory_read(0x18016fe0, &leb, 4);
                r1v = (uint32_t)cpu->env.regs[1];
                r3v = (uint32_t)cpu->env.regs[3];
                fprintf(stderr, "CHARDISP: r1(byte)=0x%02x r3=0x%08x tblbase[0x18017628]=0x%08x 'l'entry[0x18016fe0]=0x%08x (pristine 0x2801fe7d)\n",
                        r1v, r3v, base_ptr, leb);
                tbl_dumped = 1;
            }
        }
        /* If stuck in the memmove 64-byte copy loop, dump dest/src/size once. */
        if (pc >= 0x18017a58 && pc < 0x18017a80) {
            static int mm_dumped = 0;
            if (!mm_dumped) {
                uint32_t r0 = (uint32_t)cpu->env.regs[0];
                uint32_t r1 = (uint32_t)cpu->env.regs[1];
                uint32_t r2 = (uint32_t)cpu->env.regs[2];
                uint32_t r4 = (uint32_t)cpu->env.regs[4];
                fprintf(stderr, "MMOVE: pc=0x%08x r0(dest)=0x%08x r1(src)=0x%08x r2(size)=0x%08x r4=0x%08x\n",
                        pc, r0, r1, r2, r4);
                mm_dumped = 1;
            }
        }
    }

    /* UART capture: poll SRAM for characters written by patched uart_putchar.
     * This runs from the step trace timer (1ms) after the periodic timer stops. */
    {
        hwaddr uart_sram = S5L8900_RAM_BASE + 0x10600;
        uint8_t ch;
        cpu_physical_memory_read(uart_sram, &ch, 1);
        static uint8_t last_uart_ch = 0xFF;
        if (ch != last_uart_ch && ch != 0x00) {
            last_uart_ch = ch;
            Chardev *chr = s5l8900_serial_chr;
            if (chr) {
                if (ch == '\n') {
                    uint8_t cr = '\r';
                    qemu_chr_write(chr, &cr, 1, false);
                }
                if (ch >= 0x08) {
                    qemu_chr_write(chr, &ch, 1, false);
                }
            }
            fprintf(stderr, "STEP_UART: 0x%02x (%c) pc=0x%08x\n",
                    ch, (ch >= 0x20 && ch < 0x7F) ? ch : '.', pc);
        }
    }

    /* One-time dump of the console command dispatch table after a crash into
     * the SRAM safe loop (0x2200f8xx): show the runtime table base
     * [0x18006000], the table contents, and the two candidate regions, to
     * diagnose the 0x18005EDC "mov pc,r3" prefetch abort. */
    if (pc >= 0x2200f000 && pc < 0x22010000) {
        static int disp_dumped = 0;
        if (!disp_dumped) {
            disp_dumped = 1;
            /* Fault context saved by the exception handlers (LR=faulting PC+4,
             * CPSR) tells us the ACTUAL fault location. */
            uint32_t flr = 0, fcpsr = 0;
            cpu_physical_memory_read(0x2200ED00, &flr, 4);
            cpu_physical_memory_read(0x2200ED04, &fcpsr, 4);
            fprintf(stderr, "FAULT: lr(faulting PC+4)=0x%08x cpsr=0x%08x  -> fault PC~0x%08x\n",
                    flr, fcpsr, (flr & 1u) ? (flr - 2u) : (flr - 4u));
            /* Real fault context from the upgraded handler (IFAR/DFAR/CPSR/LR). */
            {
                uint32_t t[5] = {0,0,0,0,0};
                cpu_physical_memory_read(0x2200F820, t, sizeof(t));
                fprintf(stderr, "FAULT-CTX: IFAR=0x%08x DFAR=0x%08x CPSR=0x%08x LR=0x%08x pad=0x%08x\n",
                        t[1], t[2], t[3], t[4], t[0]);
            }
            uint32_t tb = 0, w[16];
            cpu_physical_memory_read(0x18006000, &tb, 4);
            fprintf(stderr, "DISPATCH: table_base[0x18006000]=0x%08x\n", tb);
            if (tb != 0 && (tb & 3) != 3) {
                uint32_t base = tb & ~3u;
                cpu_physical_memory_read(base, w, 64);
                fprintf(stderr, "DISPATCH: @%08x (16w): ", base);
                for (int k = 0; k < 16; k++) fprintf(stderr, "0x%08x ", w[k]);
                fprintf(stderr, "\n");
            }
            fprintf(stderr, "DISPATCH: @0x18002200 (16w): ");
            cpu_physical_memory_read(0x18002200, w, 64);
            for (int k = 0; k < 16; k++) fprintf(stderr, "0x%08x ", w[k]);
            fprintf(stderr, "\n");
            /* Dump the whole literal-pool region 0x5FD0..0x6008 at crash time so
             * we can diff against the pristine image and see exactly which words
             * the init code clobbered (resolves whether the crash ldr reads
             * [0x5FFC] or [0x6000]). */
            uint32_t pool[16];
            cpu_physical_memory_read(0x18005FD0, pool, 64);
            for (int k = 0; k < 16; k++) {
                fprintf(stderr, "DISPATCH: pool[0x%08x]=0x%08x\n",
                        0x18005FD0 + 4*k, pool[k]);
            }
            /* The dispatch LDR at 0x5ED6 actually reads [0x5ED6+4+0x128]=0x18006002
             * (unaligned). Dump it explicitly. */
            uint32_t tb2 = 0;
            cpu_physical_memory_read(0x18006002, &tb2, 4);
            fprintf(stderr, "DISPATCH: table_base[0x18006002]=0x%08x\n", tb2);
            /* Scan BSS (0x18021980-0x18026000) and SRAM (0x22000000-0x22040000)
             * at crash time for any command table = 8 consecutive Thumb function
             * pointers. Tells us whether iBoot's init built a table anywhere. */
            s5l8900_scan_for_tables(0x18021980, 0x18026000, "BSS");
            s5l8900_scan_for_tables(0x22000000, 0x22040000, "SRAM");
        }
    }

    /* 1ms change-detector on the dispatch literal-pool region 0x18005FD0..0x6008.
     * Compares the live runtime pool against the pristine staging copy
     * (S5L8900_IBOOT_BASE + 0x5FD0). When a word diverges, log the current PC
     * (within ~1ms of the store) plus the pristine/live value. This pinpoints the
     * writer of the clobbered pool word. */
    {
        static int change_logged = 0;
        if (change_logged < 64) {
            uint32_t cur_pool[16], pristine[16];
            cpu_physical_memory_read(0x18005FD0, cur_pool, 64);
            cpu_physical_memory_read(S5L8900_IBOOT_BASE + 0x5FD0, pristine, 64);
            for (int k = 0; k < 16 && change_logged < 64; k++) {
                if (cur_pool[k] != pristine[k]) {
                    fprintf(stderr, "POOLCHG: off=0x%08x word[%d] pristine=0x%08x live=0x%08x  pc=0x%08x thumb=%d\n",
                            0x18005FD0 + 4*k, k, pristine[k], cur_pool[k], pc, thumb);
                    change_logged++;
                }
            }
        }
    }

    /* memcpy code-clobber detector: the memcpy at 0x180178ac (called from
     * init 3's 0x18004f60 via wrapper 0x18018260) has its return branch at
     * 0x18017a94 (pristine: 49 00 00 ea = b 0x18017bc0). If that word diverges
     * from the pristine image, the memcpy is writing into the iBoot code region
     * (its destination landed on its own code). Capture r0/r1/r2/PC/LR the
     * moment it happens: LR identifies the caller, r0~ the (partial) destination. */
    {
        static int clobber_logged = 0;
        if (clobber_logged < 8) {
            uint32_t live = 0, pristine = 0;
            cpu_physical_memory_read(0x18017a94, &live, 4);
            cpu_physical_memory_read(S5L8900_IBOOT_BASE + 0x17a94, &pristine, 4);
            if (live != pristine) {
                uint32_t r0 = (uint32_t)cpu->env.regs[0];
                uint32_t r1 = (uint32_t)cpu->env.regs[1];
                uint32_t r2 = (uint32_t)cpu->env.regs[2];
                /* Dump the allocator stub 0x18007a34 (the memcpy's dest source)
                 * at clobber time. If the literal [0x18007a38] is not the SRAM
                 * pool (0x2203xxxx), iBoot overwrote our stub between the JUMP CB
                 * write and the memcpy call, so the allocator returns a code-region
                 * pointer. */
                uint8_t stub8[8] = {0};
                cpu_physical_memory_read(S5L8900_IBOOT_RUNTIME + 0x7A34, stub8, sizeof(stub8));
                uint32_t lit = 0;
                cpu_physical_memory_read(S5L8900_IBOOT_RUNTIME + 0x7A38, &lit, 4);
                fprintf(stderr, "MEMCPY_CLOBBER: 0x18017a94 live=0x%08x pristine=0x%08x  pc=0x%08x lr=0x%08x r0=0x%08x r1=0x%08x r2=0x%08x thumb=%d\n",
                        live, pristine, pc, lr, r0, r1, r2, thumb);
                fprintf(stderr, "ALLOC_STUB 0x7A34: %02x %02x %02x %02x %02x %02x %02x %02x  lit[0x7A38]=0x%08x\n",
                        stub8[0], stub8[1], stub8[2], stub8[3], stub8[4], stub8[5], stub8[6], stub8[7], lit);
                /* Dump the caller's runtime BL/BLX sites to see the real targets.
                 * 0x1801826e: BL alloc (pristine -> 0x18007a34)
                 * 0x1801827c: BLX memset (pristine -> 0x180178ac) */
                {
                    uint8_t site[8] = {0};
                    cpu_physical_memory_read(0x1801826e, site, 4);
                    fprintf(stderr, "CALLER 0x1801826e (alloc BL): %02x %02x %02x %02x\n",
                            site[0], site[1], site[2], site[3]);
                    cpu_physical_memory_read(0x1801827c, site, 4);
                    fprintf(stderr, "CALLER 0x1801827c (memset BLX): %02x %02x %02x %02x\n",
                            site[0], site[1], site[2], site[3]);
                }
                clobber_logged++;
            }
        }
    }

    /* Auto-reschedule persistently: 1ms before launch (fine step tracing),
     * then keep running after launch so MONITOR (every 100 fires) shows the
     * PC progression through console -> bootx -> kernel. Reading env->regs
     * here is safe: the timer fires from the main loop, not mid-execution. */
    if (s5l8900_step_timer) {
        timer_mod(s5l8900_step_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + 1ULL * 1000 * 1000);
    }
}

/* Jump directly to Thumb trampoline at 0xFF00, bypassing the stuck ARM loop. */
static void s5l8900_jump_to_tramp(CPUState *cs, run_on_cpu_data data)
{
    /* ONE-SHOT: dump SRAM (iBEC code, decrypted in place) for offline analysis.
     * The SRAM loader runs at pc=0x2200ff00, so iBEC's text lives in SRAM.
     * Capture 0x22000000-0x22020000 (128K) at the iBEC->iBoot handoff. */
    {
        static int s5l8900_sram_dumped = 0;
        if (!s5l8900_sram_dumped) {
            s5l8900_sram_dumped = 1;
            size_t sram_sz = 0x20000;
            uint8_t *sram = g_malloc(sram_sz);
            cpu_physical_memory_read(0x22000000, sram, sram_sz);
            FILE *f = fopen("/tmp/sram_ibec.bin", "wb");
            if (f) { fwrite(sram, 1, sram_sz, f); fclose(f); }
            g_free(sram);
            fprintf(stderr, ">>> JUMP CB: dumped SRAM 0x22000000-0x22020000 -> /tmp/sram_ibec.bin\n");
        }
    }

    /* CRITICAL: Copy iBoot from staging (0x23000000) to runtime (0x18000000).
     * iBoot's literal pools reference 0x180xxxxx addresses. Running at 0x23000000
     * causes pointer mismatches. Copy now, after iBEC is done with 0x18000000. */
    {
        size_t iboot_sz = 140288; /* iBoot payload size */
        uint8_t *iboot_buf = g_malloc(iboot_sz);
        cpu_physical_memory_read(S5L8900_IBOOT_BASE, iboot_buf, iboot_sz);
        cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME, iboot_buf, iboot_sz);
        g_free(iboot_buf);
        /* Also copy the fill pattern beyond code */
        {
            size_t fill_sz = S5L8900_IBOOT_SIZE - iboot_sz;
            uint8_t *fill = g_malloc0(fill_sz);
            uint32_t pat = 0x47704770;
            for (gsize i = 0; i + 4 <= fill_sz; i += 4)
                memcpy(fill + i, &pat, 4);
            cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + iboot_sz, fill, fill_sz);
            g_free(fill);
        }
        fprintf(stderr, ">>> JUMP CB: copied iBoot 0x%zx bytes from 0x%08x to 0x%08x\n",
                iboot_sz, S5L8900_IBOOT_BASE, S5L8900_IBOOT_RUNTIME);

        /* FIX (img2 header shift): the iBoot payload is linked at 0x18000000 but
         * the file carries a 0x400-byte img2 header, so the image runs 0x400 high
         * and every absolute pointer is off by 0x400. sub_2e84 (timer start/stop)
         * loads its register table from a literal baked as link 0x1801f030; the
         * real table (peripheral regs 0x3e200004 + n*0x20) is at runtime
         * 0x1801f430. Unpatched, [base + n*24 + 4] read a string (0x524d575b) ->
         * data abort. Point both copies of the literal at the shifted table. */
        {
            uint32_t tbl_fix = 0x1801f430;
            cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + 0x2ef0, (uint8_t *)&tbl_fix, 4);
            cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + 0x3190, (uint8_t *)&tbl_fix, 4);
            fprintf(stderr, ">>> JUMP CB: patched timer-table literal @+0x2ef0/+0x3190 = 0x%08x (was 0x1801f030)\n", tbl_fix);
        }

        /* Arm the code-region write-hook now that the bulk image load is done,
          * so only iBoot's own inits (which corrupt the ARM code region) are logged. */
        s5l8900_codehook_active = 1;
        s5l8900_codehook_writes = 0;
    }

    ARMCPU *cpu = ARM_CPU(cs);
    uint32_t iboot_reset = S5L8900_IBOOT_RUNTIME + 0x400; /* ARM reset handler */

    fprintf(stderr, ">>> JUMP CB: BEFORE - pc=0x%08x thumb=%d\n",
            cpu->env.regs[15], cpu->env.thumb);

    /* Verify iBoot reset handler bytes */
    {
        uint8_t v[16];
        cpu_physical_memory_read(iboot_reset, v, sizeof(v));
        fprintf(stderr, ">>> JUMP CB: iBoot reset @ 0x%08x: ", iboot_reset);
        for (int i = 0; i < 16; i++) fprintf(stderr, "%02x ", v[i]);
        fprintf(stderr, "\n");
    }

    /* DIAGNOSTIC (MMU experiment): do NOT force-disable the MMU here. Let iBoot's
     * own MMU init build the page table and enable the MMU. (Previously cleared
     * SCTLR M/C/A and zeroed TTBR0/1 to run iBoot flat in physical mode.) */
    fprintf(stderr, ">>> JUMP CB: [MMU experiment] leaving SCTLR/TTBR as-is (SCTLR=0x%08x)\n",
            (unsigned int)cpu->env.cp15.sctlr_s);

    fprintf(stderr, ">>> JUMP CB: about to patch exception vectors\n");
    fflush(stderr);

    /* Patch iBoot exception vectors to force-safe handler in SRAM.
      * Handler switches to SVC mode and jumps to safe ARM loop at 0xF920.
      * This prevents infinite exception loops from corrupted return addresses. */
     {
          uint32_t handler_addr = S5L8900_RAM_BASE + 0xF000; /* SRAM, safe area */
          uint32_t safe_loop = S5L8900_RAM_BASE + 0xF920;
          /* Save faulting context (LR = faulting PC+4, CPSR) to fixed SRAM slots
           * 0x2200ED00 / 0x2200ED04 BEFORE jumping to the safe loop, so we can
           * read the actual fault location. Uses a literal pool for the target
           * addresses (0x2200ED00 is BELOW this handler, so pc-relative store
           * cannot reach it). */
          uint32_t handler_code[] = {
              0xE59F1010,  /* LDR r1, [pc, #16] -> 0x2200F018 (literal 0x2200ED00) */
              0xE581A000,  /* STR lr, [r1]   (save fault LR) */
              0xE10F0000,  /* MRS r0, cpsr */
              0xE59F1008,  /* LDR r1, [pc, #8]  -> 0x2200F01C (literal 0x2200ED04) */
              0xE5810000,  /* STR r0, [r1]   (save fault CPSR) */
              0xEA000241,  /* B 0x2200F920 (ARM safe loop) */
              0x2200ED00,  /* literal: fault LR slot */
              0x2200ED04,  /* literal: fault CPSR slot */
          };
          cpu_physical_memory_write(handler_addr, handler_code, sizeof(handler_code));

        /* Patch iBoot vectors to branch to SRAM handler */
        uint32_t vec_base = S5L8900_IBOOT_RUNTIME + 0x404;
        for (uint32_t i = 0; i < 7; i++) {
            uint32_t vec_addr = vec_base + i * 4;
            /* ARM B instruction: B <handler_addr> */
            int32_t offset = (handler_addr - (vec_addr + 8)) / 4;
            uint32_t b_instr = 0xEA000000 | (offset & 0x00FFFFFF);
            cpu_physical_memory_write(vec_addr, &b_instr, 4);
        }
        fprintf(stderr, ">>> JUMP CB: patched iBoot exception vectors -> SRAM handler at 0x%08x (SUBS pc,lr,#4)\n",
                handler_addr);
        fflush(stderr);
    }

    /* CRITICAL: Patch the ARM safe loop at 0x2200f920 and Thumb safe loop at 0x2200fe00.
      * The ROM exception vectors BL to these addresses. Keep them as infinite
      * loops so the exception handler can safely redirect here. */
     {
         uint32_t arm_loop = 0xEAFFFFFE;  /* B #-4 (ARM infinite loop) */
         cpu_physical_memory_write(S5L8900_RAM_BASE + 0xF920, &arm_loop, 4);
         /* Thumb safe loop: B #-4 (0xE7FE) */
         uint16_t thumb_loop = 0xE7FE;
         cpu_physical_memory_write(S5L8900_RAM_BASE + 0xFE00, &thumb_loop, 2);
         fprintf(stderr, ">>> JUMP CB: kept ARM+Thumb safe loops as infinite loops\n");
         fflush(stderr);
     }

     /* CRITICAL: Disable evec redirection and patch both vector tables.
          * Exception handler forces execution to a safe state. Instead of
          * returning to potentially corrupted addresses, the handler:
          * 1. Checks if LR is in a valid range (iBoot code or SRAM)
          * 2. If valid: skips 4 bytes past faulting instruction, returns
          * 3. If invalid: forces execution to safe ARM loop at 0x2200F920 */
     {
          /* Write handler in SRAM at 0x2200EC00 (ARM code).
           * Handler logic:
           *   MOV r0, lr
           *   CMP r0, #iboot_start  (if lr < iboot_start -> invalid)
           *   BLT force_safe
           *   CMP r0, #iboot_end    (if lr >= iboot_end -> invalid)
           *   BGE force_safe
           *   ADD lr, lr, #4        (skip faulting instruction)
           *   SUBS pc, lr, #4       (exception return)
           * force_safe:
           *   LDR pc, [pc, #-4]     (jump to safe ARM loop) */
          {
              uint32_t handler_addr = S5L8900_RAM_BASE + 0xEC00;
              uint32_t safe_loop = S5L8900_RAM_BASE + 0xF920; /* ARM safe loop */

              /* Build handler with immediate checks.
               * Use ORR/MOVW/MOVT for large constants. */
              uint32_t iboot_lo = S5L8900_IBOOT_RUNTIME & 0xFFFF;
              uint32_t iboot_hi = S5L8900_IBOOT_RUNTIME >> 16;
              uint32_t iboot_end = S5L8900_IBOOT_RUNTIME + S5L8900_IBOOT_SIZE;
              uint32_t iboot_end_lo = iboot_end & 0xFFFF;
              uint32_t iboot_end_hi = iboot_end >> 16;

              uint8_t handler_bytes[] = {
                  // MOVW r0, #iboot_lo
                  0x40, 0xF2, (uint8_t)(iboot_lo), (uint8_t)(iboot_lo >> 8),
                  // MOVT r0, #iboot_hi
                  0xC0, 0xF2, (uint8_t)(iboot_hi), (uint8_t)(iboot_hi >> 8),
                  // CMP lr, r0
                  0x01, 0x42,
                  // BLT force_safe (lr < iboot_start)
                  0x9C, 0xBD,  // conditional: BLT -> B 0x1A (force_safe)
                  // Actually let me use a simpler approach: just always force safe
                  // This avoids complex branching and potential encoding issues
              };

              /* SIMPLIFIED: Always force execution to safe state.
               * The handler just switches to SVC mode and jumps to safe loop.
               * This avoids infinite exception loops entirely. */
               /* Save faulting context (LR = faulting PC+4, CPSR) to fixed SRAM
                * slots 0x2200ED00 / 0x2200ED04 BEFORE jumping to the safe loop.
                * 0x2200ED00 is ABOVE this handler (0x2200EC00), so pc-relative
                * stores reach it directly. */
               uint32_t handler[] = {
                   0xE58F00F8,  /* STR lr, [pc, #0xF8] -> 0x2200EC08+0xF8=0x2200ED00 (fault LR) */
                   0xE10F0000,  /* MRS r0, cpsr */
                   0xE58F00F4,  /* STR r0, [pc, #0xF4] -> 0x2200EC10+0xF4=0x2200ED04 (fault CPSR) */
                   0xEA000343,  /* B 0x2200F920 (ARM safe loop) */
                   0xE1A00000,  /* NOP (padding) */
               };
               cpu_physical_memory_write(handler_addr, handler, sizeof(handler));

               /* Patch ARM vector tables to branch to handler.
                * On ARM1176, ALL exceptions enter ARM mode and fetch from
                * the ARM vector table (0x00000000 or 0xFFFF0000).
                * The area at +0x200 is NOT used for exception entry. */
               /* The handler at 0x2200EC00 is far beyond the +/-128MB B-instruction
                * range from the 0x00000000 / 0xFFFF0000 vector tables, so a plain B
                * wraps to a negative offset and lands in unmapped space (cascaded
                * aborts). Use LDR PC,[PC,#0x18] which loads the handler address from
                * a literal pool at vec_base+0x20. For entry i at vec_base+i*4 the
                * LDR reads (vec_base+i*4)+8+0x18 = vec_base+0x20+i*4, i.e. the i-th
                * literal. The 0x18 offset is identical for every entry. */
               for (hwaddr vec_base = 0; vec_base <= 0xFFFF0000; vec_base += 0xFFFF0000) {
                   uint32_t ldr_pc = 0xE51FF006;  /* LDR pc, [pc, #0x18] */
                   for (int i = 0; i < 8; i++) {
                       hwaddr vec_addr = vec_base + i * 4;
                       cpu_physical_memory_write(vec_addr, &ldr_pc, 4);
                   }
                   /* Literal pool: 8 copies of the handler address at vec_base+0x20 */
                   for (int i = 0; i < 8; i++) {
                       hwaddr pool_addr = vec_base + 0x20 + i * 4;
                       cpu_physical_memory_write(pool_addr, &handler_addr, 4);
                   }
                   /* Also fill +0x200 area with ARM NOPs (not used, but be safe) */
                   for (int i = 0; i < 8; i++) {
                       hwaddr vec_addr = vec_base + 0x200 + i * 4;
                       uint32_t nop = 0xE1A00000;  /* ARM NOP */
                       cpu_physical_memory_write(vec_addr, &nop, 4);
                   }
               }

              /* Also patch evec RAM */
              if (s5l8900_evec_state) {
                  for (int i = 0; i < 0x100; i += 4) {
                      int32_t offset = (handler_addr - (i + 8)) / 4;
                      uint32_t b_instr = 0xEA000000 | (offset & 0x00FFFFFF);
                      stl_le_p(s5l8900_evec_state->ram + i, b_instr);
                  }
              }
              s5l8900_evec_state = NULL;
              fprintf(stderr, ">>> JUMP CB: wrote force-safe handler at 0x%08x -> safe loop 0x%08x\n",
                      handler_addr, safe_loop);
              fprintf(stderr, ">>> JUMP CB: patched all vector tables -> force-safe handler\n");
           }
          fflush(stderr);

          /* Verify the low vector table actually holds our branches before iBoot runs. */
          {
              uint32_t v[8];
              cpu_physical_memory_read(0x00000000, v, sizeof(v));
              fprintf(stderr, "VECTBL-INIT: ");
              for (int k = 0; k < 8; k++) fprintf(stderr, "0x%08x ", v[k]);
              fprintf(stderr, "\n");
          }
      }

    /* Run iBoot reset handler from 0x4C2 (skip vector table at 0x400-0x43C).
     * Exception handlers advance PC past faults, so problematic instructions are skipped.
     * The reset handler will initialize globals, zero BSS, and eventually call the boot function. */
    {
        /* Zero BSS range (already zero in QEMU, but be safe).
         * NOTE: the buffer must be a full 4 bytes - writing 4 bytes from a
         * uint8_t reads 3 bytes of uninitialized stack garbage into every word. */
        {
            uint32_t zero = 0;
            hwaddr bss_start = S5L8900_IBOOT_RUNTIME + 0x21980;
            hwaddr bss_end = S5L8900_IBOOT_RUNTIME + 0x26000;
            for (hwaddr a = bss_start; a < bss_end; a += 4)
                cpu_physical_memory_write(a, &zero, 4);
            fprintf(stderr, ">>> JUMP CB: zeroed BSS 0x%08lx-0x%08lx\n",
                    (unsigned long)bss_start, (unsigned long)bss_end);
        }

        /* Set up SVC stack pointer */
        cpu->env.regs[13] = S5L8900_RAM_BASE + 0x20000;

        /* Write ARM-mode trampoline at 0x580 that jumps to reset handler at 0x4C2 */
        uint8_t code[] = {
            // 0x580: LDR r0, [pc, #0x18] -> pool at 0x5A0
            0x18, 0x00, 0x9D, 0xE5,   /* LDR r0, [pc, #0x18] */
            // 0x584: BX r0
            0x00, 0x00, 0x10, 0xE1,   /* BX r0 */
            // 0x588-0x59C: infinite loops (fallback)
            0xFE, 0xFF, 0xFF, 0xEA,   /* B 0x588 */
            0xFE, 0xFF, 0xFF, 0xEA,   /* B 0x58C */
            0xFE, 0xFF, 0xFF, 0xEA,   /* B 0x590 */
            0xFE, 0xFF, 0xFF, 0xEA,   /* B 0x594 */
            0xFE, 0xFF, 0xFF, 0xEA,   /* B 0x598 */
            0xFE, 0xFF, 0xFF, 0xEA,   /* B 0x59C */
            // 0x5A0: literal pool: 0x230004C2 (reset handler code)
            0xC2, 0x04, 0x00, 0x23,   /* 0x230004C2 */
        };
        cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + 0x580, code, sizeof(code));
        fprintf(stderr, ">>> JUMP CB: wrote ARM trampoline at 0x580 (BX -> reset handler 0x4C2)\n");
        fflush(stderr);
    }

       /* Make the relocation check pass by patching the expected load address
        * literal at 0x4F0 to match the computed address (0x23000400).
        * The reset handler computes: r0 = PC - 0x48 = 0x23000448 - 0x48 = 0x23000400.
        * With this patch, r1 = 0x23000400, so CMP passes and BEQ skips the
        * self-overlapping copy loop. Flow: check → BSS zeroing → exception
        * stacks → jump to main at 0x4C20 (Thumb). */
       {
           uint32_t expected_addr = S5L8900_IBOOT_RUNTIME + 0x400; /* 0x23000400 */
           cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + 0x4F0, (uint8_t *)&expected_addr, 4);
            fprintf(stderr, ">>> JUMP CB: patched literal 0x4F0 to 0x%08x (match computed load addr)\n", expected_addr);
        }

        /* Patch BSS literal pool addresses: 0x1802xxxx -> 0x2302xxxx.
         * 0x4F8: BSS start (0x18021980 -> 0x23021980)
         * 0x4FC: BSS end (0x18026000 -> 0x23026000)
         * These are read by LDR at 0x484 and 0x488 for the BSS zeroing loop.
         * DO NOT patch 0x488 — that's the LDR r1 instruction, not a literal! */
        {
            uint32_t bss_start = 0x23021980;
            uint32_t bss_end = 0x23026000;
            cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + 0x4F8, (uint8_t *)&bss_start, 4);
            cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + 0x4FC, (uint8_t *)&bss_end, 4);
            fprintf(stderr, ">>> JUMP CB: patched BSS literals 0x4F8=0x%08x, 0x4FC=0x%08x\n",
                    bss_start, bss_end);
         }

       /* NOP out LDR/ADD/B sequences (0x4AC-0x4DC): These load SP literals from
        * the data table (0x500+), ADD them to PC, and B to the result. Since the
        * literals are full addresses (0x18022180, etc.), ADD computes garbage
        * branch targets into the data/BSS region, causing the CPU to crash.
        * NOP-ing lets execution fall through to MOV R0,[PC,#0x20]; MOV lr,PC; BX r0
        * at 0x4E0-0x4E8, which correctly jumps to the entry function. */
       /* NOTE: DO NOT NOP mode stack setup (0x4AC-0x4DC). It sets up SPs for
        * IRQ, FIQ, ABT, UND, and SVC modes. Without it, exception handlers
        * use garbage SPs and crash. SP literals are patched below to safe SRAM. */

       /* Set up stack for main function at 0x4C20. The function does:
        * POP {r2,r3}; POP {r4,r5,r6,r7,pc}. The exception stack setup sets
        * SPs from literals at 0x500 (abort/undef/fiq/irq) and 0x504 (svc).
        * We patch both to SRAM (0x22020000) to avoid BSS zeroing overwriting
        * our stack frame. */
      {
          uint32_t stack_base = S5L8900_RAM_BASE + 0x20000; /* 0x22020000, outside BSS */
          uint32_t ret_addr = S5L8900_IBOOT_RUNTIME + 0x4EC; /* Infinite loop after main */
          uint32_t stack_frame[7] = {0, 0, 0, 0, 0, 0, ret_addr};
          cpu_physical_memory_write(stack_base, (uint8_t *)stack_frame, sizeof(stack_frame));

          /* Patch SP literals at 0x500 and 0x504 to our stack */
          cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + 0x500, (uint8_t *)&stack_base, 4);
          cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + 0x504, (uint8_t *)&stack_base, 4);
          fprintf(stderr, ">>> JUMP CB: set up stack at 0x%08x with ret=0x%08x, patched SP literals\n",
                  stack_base, ret_addr);

           /* Patch entry point literal at 0x508: 0x18004C21 -> 0x23004C21.
            * The reset handler's LDR r0,[PC,#0x20] at 0x4E0 reads this literal,
            * then BX r0 at 0x4E8 jumps to the entry function in Thumb mode. */
           {
               uint32_t main_entry = S5L8900_IBOOT_RUNTIME + 0x4C21; /* 0x23004C21 */
               cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + 0x508, (uint8_t *)&main_entry, 4);
               fprintf(stderr, ">>> JUMP CB: patched literal 0x508 = 0x%08x (entry function, thumb bit=1)\n",
                       main_entry);
           }

           /* Patch infinite loop at 0x4EC. The main function returns here via
            * POP {..., PC} which sets PC to an even address → ARM mode.
            * Must use ARM instruction: B . (0xEAFFFFFE) */
            {
                /* Patch to Thumb infinite loop (B #-2 = 0xE7FE), repeated.
                 * Original ARM B . (0xEAFFffff) crashes if entered in Thumb mode. */
                uint8_t thumb_loop[] = { 0xFE, 0xE7, 0xFE, 0xE7 }; /* B #-2; B #-2 */
                cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + 0x4EC, thumb_loop, sizeof(thumb_loop));
            }

           /* Minimal Thumb trampoline: write 'i' to UART once, then infinite loop.
            * Layout (base = tramp = 0x22010400):
            * +0x00: LDR R0, [PC, #4]   -> R0 = [0x10404] = UART_BASE
            * +0x02: MOV R1, #'i'       -> R1 = 0x69
            * +0x04: STRB R1, [R0]      -> write 'i' to UART
            * +0x06: B .                -> infinite loop
            * +0x08: <UART_BASE>        -> 0xE0002000 (LE)
            */
           {
               uint32_t tramp = S5L8900_RAM_BASE + 0x10400;
               uint32_t uart_base = S5L8900_UART_BASE; /* 0xE0002000 */

               uint8_t tramp_code[] = {
                   /* +0x00: LDR R0, [PC, #4]  -> 0x4801 */
                   0x01, 0x48,
                   /* +0x02: MOV R1, #0x69    -> 0x2169 */
                   0x69, 0x21,
                    /* +0x04: STRB R1, [R0]    -> 0x4001 */
                    0x01, 0x40,
                    /* +0x06: B #-4 (infinite loop) -> 0xE7FE */
                    0xFE, 0xE7,
                   /* +0x08: UART_BASE (LE)   -> 0x00 0x20 0x00 0xE0 */
                   (uint8_t)(uart_base), (uint8_t)(uart_base >> 8),
                   (uint8_t)(uart_base >> 16), (uint8_t)(uart_base >> 24),
               };
                cpu_physical_memory_write(tramp, tramp_code, sizeof(tramp_code));
                queue_tb_flush(cs);
                fprintf(stderr, ">>> JUMP CB: wrote printf trampoline at 0x%08x\n", tramp);
           }

          /* Patch ALL CP15 instructions. QEMU doesn't emulate s5l8900 CP15,
           * so MRC/MCR instructions cause abort exceptions. Patch each to NOPs
           * or MOV with safe default values. */
          {
              /* 0x474: MRC p15,0,R1,c0,c7,4 -> MOV r1,#0 (CPU ID stub) */
              {
                  uint32_t mov_r1_0 = 0xE3A01000; /* MOV r1, #0 */
                  cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + 0x474, &mov_r1_0, 4);
                  /* 0x478: MRC p15,0,R1,c0,c7,5 -> MOV r1,#0 (cache type stub) */
                  uint32_t mov_r1_0b = 0xE3A01000;
                  cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + 0x478, &mov_r1_0b, 4);
              }

              /* 0x528-0x690: All CP15 instructions (MMU, cache, TLB, ID registers) -> NOP.
               * CP15 functions: 0x5A0(cpu_id), 0x5AC(ttb_read), 0x5D4(sctlr_rw),
               * 0x5E4(aux_ctl_rw), 0x5F4(clip_rw), 0x5FC(domain_rw),
               * 0x604(fault_status), 0x60C(fault_address), 0x614-0x688(TLB/cache ops).
               * Non-CP15 instructions at 0x5A4(stm), 0x5A8(bx), 0x5AC(mov),
               * 0x5C0(stm), 0x5CC(stm), 0x5D0(bx), 0x5D8(bx) are NOP'd too
               * but that's harmless - these are internal helper function epilogues. */
                /* DIAGNOSTIC: re-NOP the 0x528-0x804 CP15 helper region to reproduce
                 * the baseline data abort, so the DFAR capture sees the exact faulting
                 * address. (iBoot MMU helpers live here: 0x5AC ttb_read, 0x5D4/0x5DC
                 * sctlr, 0x5F4 dacr, 0x5FC ttbr, 0x604/0x60C fault regs, 0x614-0x688
                 * TLB/cache. arm1176 supports standard MMU/cache/TLB CP15.) */
                {
                    uint32_t nop = 0xE1A00000;
                    for (uint32_t off = 0x528; off <= 0x804; off += 4) {
                        cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + off, &nop, 4);
                    }
                }


              /* Two scattered CP15 instructions in code/data section (conditional, VS flag) */
              {
                  uint32_t nop = 0xE1A00000;
                  cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + 0x1A858, &nop, 4);
                  cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + 0x1D804, &nop, 4);
              }

              fprintf(stderr, ">>> JUMP CB: patched all CP15 instructions (0x474, 0x478, 0x528-0x804, 0x1A858, 0x1D804)\n");
          }
      }
      /* Skip reset handler entirely. Directly invoke main function at 0x4C20 (Thumb).
       * The reset handler does: relocation check, cache clean, BSS zeroing, exception
       * stack setup, then BX to main. We've already done BSS zeroing and patched
       * CP15 instructions. Set up the CPU state as if the reset handler completed.
       *
       * CRITICAL: The function at 0x4C20 expects boot-args in R1 and device info in R4.
       * These are normally set up by iBEC. We create fake structures to let it proceed. */
      {
            uint32_t main_entry = S5L8900_RAM_BASE + 0x10401; /* printf trampoline (Thumb) */
           uint32_t sp = S5L8900_RAM_BASE + 0x20000; /* SRAM stack */
           uint32_t ret_addr = S5L8900_IBOOT_RUNTIME + 0x5CA0 | 1; /* Main init function (Thumb) */

          /* Stack frame: main does POP {r4-r7,pc} and POP {r2,r3} -> 7 words */
          uint32_t frame[7] = {0, 0, 0, 0, 0, 0, ret_addr};
          cpu_physical_memory_write(sp, (uint8_t *)frame, sizeof(frame));

          /* Create fake boot-args structure in SRAM.
           * The function reads: R1[0x18], R1[0x24], R1[0x2C], R1[0x38], R1[0x40]
           * These are pointers to various boot argument fields. */
          uint32_t boot_args_addr = S5L8900_RAM_BASE + 0x10000;
          uint32_t boot_args[0x40/4] = {0};
          /* R1[0x18]: pointer to a context structure (used for R5/R8) */
          boot_args[0x18/4] = boot_args_addr + 0x100; /* Points to a safe zeroed region */
          /* R1[0x24]: string pointer (compared with strcmp, used as SL) */
          boot_args[0x24/4] = boot_args_addr + 0x200; /* Will hold "tobi" */
          /* R1[0x2C]: pointer (stored in R8) */
          boot_args[0x2C/4] = boot_args_addr + 0x100;
          /* R1[0x38]: pointer */
          boot_args[0x38/4] = boot_args_addr + 0x100;
          cpu_physical_memory_write(boot_args_addr, (uint8_t *)boot_args, sizeof(boot_args));

           /* Create fake device info structure in SRAM.
            * The entry function reads R4[0x10] as a pointer to the board ID string
            * and compares it with "n45", "n81", "n88" using strcmp. */
           uint32_t device_addr = S5L8900_RAM_BASE + 0x10100;
           uint32_t board_id_str_addr = boot_args_addr + 0x200;
           uint32_t device_name_str_addr = boot_args_addr + 0x210;
           uint32_t device_info[0x20/4] = {0};
           device_info[0x10/4] = board_id_str_addr;  /* Board ID string (compared with n45/n81/n88) */
           device_info[0x18/4] = boot_args_addr + 0x300; /* Another pointer */
           cpu_physical_memory_write(device_addr, (uint8_t *)device_info, sizeof(device_info));

           /* Write board ID and device name strings */
           {
               const char *boardid = "n45";
               uint8_t bi[16] = {0};
               memcpy(bi, boardid, strlen(boardid) + 1);
               cpu_physical_memory_write(board_id_str_addr, bi, sizeof(bi));

               const char *devname = "iPod1,1";
               uint8_t dn[16] = {0};
               memcpy(dn, devname, strlen(devname) + 1);
               cpu_physical_memory_write(device_name_str_addr, dn, sizeof(dn));
           }

          /* Zero the safe regions pointed to by boot-args */
          {
              uint8_t zeros[0x200] = {0};
              cpu_physical_memory_write(boot_args_addr + 0x100, zeros, sizeof(zeros));
          }

           /* Initialize critical BSS globals. The function at 0x5490 (memory
            * validation) reads from 0x23022FA0, 0x23022FA4, 0x23022FA8.
            * These define valid memory ranges. On real hardware, these are set
            * by the data section or iBEC. In QEMU they're zeroed (BSS).
            * Set them to cover SRAM (0x22000000-0x22040000) so validation passes. */
           {
               /* global at 0x23022FA0: upper bound for end address */
               uint32_t mem_end = S5L8900_RAM_BASE + 0x40000; /* 0x22040000 */
               cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + 0x22FA0, &mem_end, 4);
               /* global at 0x23022FA4: lower bound for start address */
               uint32_t mem_start = S5L8900_RAM_BASE; /* 0x22000000 */
               cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + 0x22FA4, &mem_start, 4);
               /* global at 0x23022FA8: alternative upper bound */
               uint32_t mem_end2 = S5L8900_RAM_BASE + 0x40000; /* 0x22040000 */
               cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + 0x22FA8, &mem_end2, 4);
               fprintf(stderr, ">>> JUMP CB: initialized memory range globals [0x%08x, 0x%08x)\n",
                       mem_start, mem_end);
           }

           /* Initialize entry function globals at 0x230210E4 and 0x230210E8.
            * 0x230210E8: pointer to a buffer (loaded into R5 at 0x4C26)
            * 0x230210E4: pointer to a string (loaded into R0 at 0x4C22, then R5=[R0])
            * The entry function uses these for device detection and data storage. */
           {
               uint32_t buf_ptr = boot_args_addr + 0x100; /* Valid buffer in SRAM */
               cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + 0x210E8, &buf_ptr, 4);
               /* 0x230210E4: also point to the buffer */
               cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + 0x210E4, &buf_ptr, 4);
               fprintf(stderr, ">>> JUMP CB: initialized entry globals -> 0x%08x\n", buf_ptr);
           }

               /* Patch uart_putchar (0x4A5C) to write char directly to UART MMIO.
                * ARMv5TEJ Thumb: LDR r2,[PC,#8]; STRB r0,[r2]; BX lr; padding; literal
                * Layout (16 bytes):
                *   0x4A5C: LDR r2, [PC, #8]  -> PC=0x4A60, loads [0x4A68]
                *   0x4A5E: STRB r0, [r2]     -> write char to UART
                *   0x4A60: BX lr             -> return
                *   0x4A62-0x4A66: padding
                *   0x4A68: 0xE0002000        -> UART MMIO literal */
               {
                   uint8_t patch_uart[] = {
                       0x02, 0x4A,  /* LDR r2, [PC, #8] -> loads [PC+8] = [0x4A68] */
                       0x00, 0x70,  /* STRB r0, [r2] */
                       0xF0, 0x47,  /* BX lr */
                       0x00, 0x00,  /* padding */
                       0x00, 0x00,  /* padding */
                       0x00, 0x00,  /* padding */
                       0x00, 0x20,  /* literal: 0xE0002000 (LE) */
                       0x00, 0xE0,
                   };
                   cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + 0x4A5C, patch_uart, sizeof(patch_uart));
                  queue_tb_flush(cs);

                  /* Verify patch */
                  {
                      uint8_t raw[16];
                      cpu_physical_memory_read(S5L8900_IBOOT_RUNTIME + 0x4A5C, raw, sizeof(raw));
                      fprintf(stderr, ">>> JUMP CB: patched uart_putchar -> UART MMIO 0xE0002000, bytes: ");
                      for (int i = 0; i < 16; i++) fprintf(stderr, "%02x ", raw[i]);
                      fprintf(stderr, "\n");
                  }

                   /* Write iBoot banner directly to serial port from QEMU */
                   {
                       Chardev *chr = s5l8900_serial_chr;
                       if (chr) {
                           const char *banner = "iBoot\n";
                           qemu_chr_write(chr, (const uint8_t *)banner, strlen(banner), false);
                       }
                       fprintf(stderr, ">>> JUMP CB: iBoot UART patched to write UART MMIO 0xE0002000\n");
                   }

                /* The console dispatch (0x5ED6) reads its table base from
                   [0x18006000]. In the pristine image that word is the
                   placeholder 0x18005AE0 (points mid-code, not a real table),
                   and NO iBoot code ever writes it - on real hardware the iBSS
                   loader fixes it up before iBoot runs. QEMU bypasses iBSS's
                   loader (direct stage + mirror), so we must install the table
                   ourselves.
                   A previous revision tried to "repair" the literal pool at
                   0x5FD2+ with UNALIGNED 4-byte writes; the real pool is
                   word-aligned (0x5FD0, 0x5FD4, ...) and those straddling
                   writes clobbered adjacent words - including [0x6000].
                   Do NOT rewrite the pool; only install the dispatch table. */
                    {
                        /* Dispatch table base in SRAM. 8 entries (r4 = 0..7):
                        *  [0] command line -> re-enter token parser at 0x5F00
                        *      (Thumb re-entry 0x5F01); it walks the command
                        *      groups, calls the handler, then b 0x5D10.
                        *  [5] empty line  -> re-enter input loop at 0x5D10.
                        *  others -> input loop (safe no-op). */
                       {
                           uint32_t table_base = S5L8900_RAM_BASE + 0x11200;
                           uint32_t cmd_handler = S5L8900_IBOOT_RUNTIME + 0x5F01; /* 0x18005F01 */
                           uint32_t loop_top    = S5L8900_IBOOT_RUNTIME + 0x5D11; /* 0x18005D11 */
                           uint32_t table[8];
                           for (int i = 0; i < 8; i++) table[i] = loop_top;
                           table[0] = cmd_handler;
                           table[5] = loop_top;
                           cpu_physical_memory_write(table_base, table, sizeof(table));
                           /* Point the dispatch at our table. */
                           cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + 0x6000, &table_base, 4);
                           fprintf(stderr, ">>> JUMP CB: installed dispatch table @0x%08x (cmd->0x%08x), [0x6000]=0x%08x\n",
                                   table_base, cmd_handler, table_base);
                        }
                   }
              }

                /* Jump DIRECTLY to main_init at 0x18005CA0 (Thumb mode).
                 * Skip the reset handler entirely - CP15 NOP range (0x528-0x804)
                 * overwrites part of it. All reset handler duties (BSS zeroing,
                 * stack setup, literal patching) have been done by the JUMP CB.
                 *
                 * Write exception vector table at 0x00000000 to catch any
                 * exceptions. Use LDR pc, [PC, #imm] to reach any address. */
                {
                    uint32_t main_init = S5L8900_IBOOT_RUNTIME + 0x5CA0; /* 0x18005CA0 */
                    uint32_t safe_loop_arm = S5L8900_RAM_BASE + 0xF900; /* ARM infinite loop */
                    uint32_t exc_handler = S5L8900_RAM_BASE + 0xF800; /* Exception return handler */

                    /* Fill 0xF600-0xF800 with safe ARM loops to prevent crash at 0xF62c.
                     * The CPU keeps landing at 0xF62c and executing Thumb code in ARM mode. */
                    {
                        uint32_t arm_loop = 0xEAFFFFFE; /* B #-4 */
                        for (int i = 0; i < 0x200 / 4; i++) {
                            cpu_physical_memory_write(S5L8900_RAM_BASE + 0xF600 + i * 4, &arm_loop, 4);
                        }
                        fprintf(stderr, ">>> JUMP CB: filled 0xF600-0xF800 with ARM loops\n");
                    }

                    /* Write safe ARM loop at 0xF900 (target for exception handler). */
                   {
                       uint32_t arm_loop = 0xEAFFFFFE; /* B #-4 */
                       for (int i = 0; i < 16; i++) {
                           cpu_physical_memory_write(S5L8900_RAM_BASE + 0xF900 + i * 4, &arm_loop, 4);
                       }
                       fprintf(stderr, ">>> JUMP CB: wrote safe ARM loop at 0xF900-0xF93F\n");
                   }

                     /* Write exception handler at 0xF800.
                      * On exception, save LR (faulting PC) to SRAM trace, then
                      * jump to safe ARM loop at 0xF900 to prevent crash loops.
                      * Layout (28 bytes):
                      *   0xF800: STR lr, [pc, #0x14]  ; PC=0xF808, stores to [0xF81C]
                      *   0xF804: LDR pc, [pc, #0xC]   ; PC=0xF80C, loads from [0xF818]
                      *   0xF808-0xF80C: padding
                      *   0xF810-0xF814: padding
                      *   0xF818: literal = safe_loop (0x2200F900)
                      *   0xF81C: trace storage (faulting LR) */
                    {
                        /* Capture the real faulting context, then park (B #-4).
                         * The observed exception is a DATA/PREFETCH ABORT (CPSR
                         * mode bits = 0x17), not an IRQ, so masking IRQ and
                         * returning would just re-fault the same access. We store
                         * CPSR/DFAR/IFAR/LR to 0x2200F82C/28/24/30 for the
                         * FAULT-CTX dump, then park so the values stay stable.
                         * (Reading DFSR here caused a nested-exception loop on the
                         * arm1176 model, so it is intentionally omitted.)
                         * Layout (handler at 0x2200F800): */
                        uint32_t handler_code[] = {
                            0xE10F0000,        /* MRS r0, CPSR                          */
                            0xE58F0020,        /* STR r0, [pc, #0x20] -> [0x2200F82C] CPSR */
                            0xEE150F10,        /* MRC p15,#0,r0,c5,c0,#0 (DFAR)          */
                            0xE58F0014,        /* STR r0, [pc, #0x14] -> [0x2200F828] DFAR */
                            0xEE160F10,        /* MRC p15,#0,r0,c6,c0,#0 (IFAR)          */
                            0xE58F0008,        /* STR r0, [pc, #8]    -> [0x2200F824] IFAR */
                            0xE58FE010,        /* STR lr, [pc, #0x10] -> [0x2200F830] LR   */
                            0xEAFFFFFE,        /* B #-4 (park)                            */
                            0x00000000,        /* pad 0x2200F820                          */
                            0x00000000,        /* 0x2200F824: IFAR                        */
                            0x00000000,        /* 0x2200F828: DFAR                        */
                            0x00000000,        /* 0x2200F82C: CPSR                        */
                            0x00000000,        /* 0x2200F830: LR                          */
                        };
                        cpu_physical_memory_write(exc_handler, handler_code, sizeof(handler_code));
                    }

                   /* Vector table at 0x00000000 (low vectors).
                    * Each vector: LDR pc, [PC, #imm] -> loads handler addr from literal pool
                    * Literal pool starts at 0x20 (right after 8 vectors). */
                   {
                       uint32_t vec_data[0x24 / 4]; /* 8 vectors + 2 literals = 9 words */
                       int w = 0;

                       /* For vector at offset V, PC = V+8 (ARM pipeline). Literal at 0x20.
                        * imm = 0x20 - (V+8) in bytes */
                       for (int i = 0; i < 8; i++) {
                           int v = i * 4;
                           int imm = 0x20 - (v + 8); /* byte offset from PC to literal */
                           /* LDR pc, [PC, #imm] = 0xE59FF000 | imm */
                           vec_data[w++] = 0xE59FF000 | imm;
                       }
                       /* Literal pool at 0x20 */
                       vec_data[w++] = exc_handler;
                       vec_data[w++] = exc_handler; /* Same for all vectors */

                       cpu_physical_memory_write(0, vec_data, sizeof(vec_data));
                       fprintf(stderr, ">>> JUMP CB: wrote vector table at 0x00000000 -> safe loop 0x%08x\n",
                               safe_loop_arm);
                   }

                    /* Write a simple ARM-mode UART-print loop to SRAM.
                     * ARM9 (ARMv5TEJ) does NOT support Thumb-2 (MOVW/MOVT),
                     * so we must use ARM mode for 32-bit immediate loads.
                     * This bypasses main_init entirely and directly writes
                     * characters to the UART MMIO in a tight loop.
                     *
                     * Layout at 0x2200FF00 (ARM):
                     *   FF00: LDR r0, [PC, #16]   ; r0 = *[0xFF18] = 0xE0002000 (UART)
                     *   FF04: MOV r1, #0x59       ; r1 = 'Y'
                     *   FF08: STRB r1, [r0]       ; write 'Y' to UART
                     *   FF0C: LDR pc, [PC, #12]   ; pc = *[0xFF20] = 0x2200FF00 (loop)
                     *   FF10-FF14: padding
                     *   FF18: 0xE0002000          ; literal: UART MMIO address
                     *   FF1C: padding
                     *   FF20: 0x2200FF00          ; literal: loop start address */
                    {
                        uint8_t arm_loop[] = {
                            /* LDR r0, [PC, #16] */
                            0x10, 0x00, 0x9F, 0xE5,
                            /* MOV r1, #0x59 */
                            0x59, 0x10, 0xA0, 0xE3,
                            /* STRB r1, [r0] */
                            0x00, 0x10, 0xC0, 0xE5,
                            /* LDR pc, [PC, #12] */
                            0x0C, 0xF0, 0x9F, 0xE5,
                            /* padding (8 bytes) */
                            0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00,
                            /* literal: UART MMIO 0xE0002000 */
                            0x00, 0x20, 0x00, 0xE0,
                            /* padding (4 bytes) */
                            0x00, 0x00, 0x00, 0x00,
                            /* literal: loop start 0x2200FF00 */
                            0x00, 0xFF, 0x00, 0x22,
                        };
                        uint32_t loop_addr = S5L8900_RAM_BASE + 0xFF00;
                        cpu_physical_memory_write(loop_addr, arm_loop, sizeof(arm_loop));
                        fprintf(stderr, ">>> JUMP CB: wrote ARM UART loop at 0x%08x (%zu bytes)\n",
                                loop_addr, sizeof(arm_loop));
                    }

                    /* Set up registers - jump to ARM UART loop (NOT Thumb) */
                    cpu->env.regs[13] = S5L8900_RAM_BASE + 0x20000; /* SP */
                    cpu->env.regs[14] = S5L8900_RAM_BASE + 0xF900; /* LR -> safe ARM loop */
                    cpu->env.thumb = 0; /* ARM mode */
                    cpu->env.regs[15] = S5L8900_RAM_BASE + 0xFF00; /* PC = ARM loop (even addr) */
                    /* Ensure SVC mode, ARM state, IRQ/FIQ masked */
                    cpu->env.uncached_cpsr = (cpu->env.uncached_cpsr & ~0x3F) | 0x13 | (1<<7) | (1<<6);
                    fprintf(stderr, ">>> JUMP CB: jumping to ARM UART loop at 0x%08x (bypassing main_init)\n",
                            (unsigned int)cpu->env.regs[15]);
               }

              /* Entry trampoline.
               *
               * iBoot's reset handler (0x440) zeroes BSS, sets up the exception
               * stacks, then BXes [0x508]. The pristine [0x508]=0x18004C21 lands
               * in the epilogue of the 0x4C04 command processor, which pops the
               * (JUMP-CB-planted) return address = main_init (0x5CA0). That path
               * NEVER runs the entry dispatcher 0x5020, so the console command
               * groups are never registered and the dispatch table stays empty.
               *
               * Instead point [0x508] at a small ARM trampoline in SRAM that:
               *   1. blx the command-registration init 0x18000AE0 (registers all
               *      console command groups: help, bootx, ... via 0x18001dbc,
               *      0x180094a8, 0x1800f210, 0x1800085c, ...), then
               *   2. ldr pc to main_init 0x5CA0 (Thumb) to run the console loop.
               * The trampoline is self-contained (blx sets lr to its own
               * continuation), so the reset handler's LR clobber is irrelevant. */
              {
                  uint32_t tramp = S5L8900_RAM_BASE + 0xF400;
                  /* Route through the FULL entry dispatcher 0x5020 (Thumb). It runs
                   * the init sequence 0x18017866 -> 0x18003fac -> 0x18004f60 ->
                   * 0x18000ae0 (command registration) -> 0x18004fc4 -> 0x180057a8 ->
                   * 0x1800595e -> 0x18005a46. The earlier inits set up state that
                   * 0x18000ae0 depends on; calling 0x18000ae0 in isolation faulted.
                   * After the dispatcher's inits we ldr pc back to main_init. */
                  uint32_t reg_init = S5L8900_IBOOT_RUNTIME + 0x5020 | 1; /* 0x18005021 (Thumb) */
                  uint32_t main_init = S5L8900_IBOOT_RUNTIME + 0x5CA0 | 1; /* 0x18005CA1 (Thumb) */
                  uint32_t code[] = {
                      0xE59F0010,      /* ldr r0, [pc, #16]  -> [tramp+0x18] = reg_init (Thumb) */
                      0xE12FFF30,      /* blx r0             -> call reg_init (Thumb, lr=tramp+8) */
                      0xE59FF004,      /* ldr pc, [pc, #4]   -> [tramp+0x14] = main_init (Thumb) */
                      0xEAFFFFFE,      /* (padding) */
                      0x00000000,      /* (padding) */
                      main_init,       /* literal @tramp+0x14 */
                      reg_init,        /* literal @tramp+0x18 */
                  };
                  cpu_physical_memory_write(tramp, code, sizeof(code));
                  cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + 0x508, (uint8_t *)&tramp, 4);
                  fprintf(stderr, ">>> JUMP CB: entry trampoline @0x%08x (blx 0x%08x reg-init, then main_init 0x%08x); [0x508]=0x%08x\n",
                          (unsigned int)tramp, (unsigned int)reg_init, (unsigned int)(main_init & ~1u), (unsigned int)tramp);
                   {
                       uint8_t db[32];
                       cpu_physical_memory_read(tramp, db, sizeof(db));
                       fprintf(stderr, ">>> JUMP CB: tramp bytes: ");
                       for (int i = 0; i < 32; i++) fprintf(stderr, "%02x ", db[i]);
                       fprintf(stderr, "\n");
                   }
               }

              /* (init-3 NOP experiment removed: code-region write hooks proved iBoot
                * inits do NOT write into the code region; the earlier clobber was
                * QEMU's own jump-callback stubs, now removed. init 3 is required to
                * initialize data structures (e.g. 0x1801f030) used by registration.) */

              /* Patch literal at 0x50C: the reset handler computes
              * base = PC - [0x50C] and compares to [0x4F0]=0x18000000. With the
              * reset handler at 0x440, the 'mov r0, pc' gives PC=0x18000448, so
              * [0x50C] must be 0x448 for base to equal 0x18000000 and take the
              * beq (skip the self-copy/relocation loop, which would self-overlap).
              * The image value was 0x48 (assumes the vector table at offset 0). */
             {
                 uint32_t off_const = 0x448;
                 cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + 0x50C, (uint8_t *)&off_const, 4);
                 fprintf(stderr, ">>> JUMP CB: patched literal 0x50C = 0x%08x (reset handler base check)\n",
                         off_const);
             }

            /* Stub heap/allocation functions called from 0x5CA0.
             * 0x5CA0 calls: 0x17F50 -> 0x7A34 -> 0x4FA0, 0x79DC, 0x4FC4
             * These are heap management functions that need initialized
             * heap state. Patch them to return a safe SRAM buffer.
             *
             * Also stub 0x763C (called from 0x5CA0) which may access
             * hardware or uninitialized globals. */
             {
                 /* Real, code-disjoint SRAM data pool for iBoot. The iBoot
                  * code image occupies 0x18000000-0x18022400; giving iBoot's
                  * heap/alloc a pool in SRAM (0x22030000) means every buffer it
                  * derives lands in SRAM, never aliasing the code. Previously the
                  * heap-init stubs returned 0 and the alloc stubs returned 5 tiny
                  * fixed offsets, so an init deriving a buffer base from a
                  * no-op'd heap init computed a pointer into the code region
                  * ([r3+0x10]=0x18017a92) and its memcpy clobbered iBoot's own
                  * code. 2MB pool, zeroed. */
                 uint32_t heap_buf = S5L8900_RAM_BASE + 0x30000; /* 0x22030000 */
                 const size_t pool_sz = 0x200000; /* 2MB */
                 uint8_t *pool_zero = g_malloc0(0x10000);
                 for (size_t z = 0; z < pool_sz; z += 0x10000)
                     cpu_physical_memory_write(heap_buf + z, pool_zero, 0x10000);
                 g_free(pool_zero);

                /* 0x17F50: malloc-like (r0=count, r1=size) -> return heap_buf
                  * ARMv5TEJ Thumb stub: LDR r0, [PC, #0]; BX lr; literal
                  * 8 bytes: LDR(2) + BX(2) + literal(4) */
                 {
                     uint8_t stub[] = {
                         0x00, 0x48,  /* LDR r0, [PC, #0] -> loads [addr+4] */
                         0xF0, 0x47,  /* BX lr */
                         (uint8_t)(heap_buf), (uint8_t)(heap_buf>>8),
                         (uint8_t)(heap_buf>>16), (uint8_t)(heap_buf>>24),
                     };
                     cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + 0x17F50, stub, sizeof(stub));
                 }

                 /* 0x7A34: allocator (r0=size) -> return heap_buf+8 */
                 {
                     uint32_t buf2 = heap_buf + 8;
                     uint8_t stub[] = {
                         0x00, 0x48,  /* LDR r0, [PC, #0] */
                         0xF0, 0x47,  /* BX lr */
                         (uint8_t)(buf2), (uint8_t)(buf2>>8),
                         (uint8_t)(buf2>>16), (uint8_t)(buf2>>24),
                     };
                     cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + 0x7A34, stub, sizeof(stub));
                 }

                 /* 0x4FA0: heap init -> return heap_buf (pool base).
                  * Returning a valid SRAM base means any buffer pointer iBoot
                  * derives from the heap-init result lands in SRAM, not 0.
                  * Thumb: LDR r0, [PC, #0]; BX lr; literal */
                 {
                     uint8_t stub[] = {
                         0x00, 0x48,     /* LDR r0, [PC, #0] */
                         0xF0, 0x47,     /* BX lr */
                         (uint8_t)(heap_buf), (uint8_t)(heap_buf>>8),
                         (uint8_t)(heap_buf>>16), (uint8_t)(heap_buf>>24),
                     };
                     cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + 0x4FA0, stub, sizeof(stub));
                 }

                  /* 0x4FC4: heap management -> return heap_buf (pool base) */
                  {
                      uint8_t stub[] = {
                          0x00, 0x48,     /* LDR r0, [PC, #0] */
                          0xF0, 0x47,     /* BX lr */
                          (uint8_t)(heap_buf), (uint8_t)(heap_buf>>8),
                          (uint8_t)(heap_buf>>16), (uint8_t)(heap_buf>>24),
                      };
                      cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + 0x4FC4, stub, sizeof(stub));
                  }

                  /* 0x17E00: bitmap "find next set bit byte" scanner (ARM).
                   * Returns an offset (r0 - base) from the scanned base. In this
                   * run it returns a huge offset (~0xC00BD4F); the caller at
                   * 0x1801826e feeds that into the alloc BL (already zeroed to a
                   * no-op), so r4 = offset+1 and the memmove reverse-copy at
                   * 0x18017a88 writes dest+size into the iBoot code region,
                   * zeroing the alloc BL and more code. Stub to return 0 so the
                   * derived size is 1 and the memmove cannot reach the code.
                   * ARM: mov r0, #0; bx lr */
                  {
                      uint8_t stub[] = {
                          0x00, 0x00, 0xA0, 0xE3,  /* mov r0, #0 */
                          0x1E, 0xFF, 0x2F, 0xE1,  /* bx lr */
                      };
                      cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + 0x17E00, stub, sizeof(stub));
                  }

                  /* 0x178AC: memmove (ARM). In this run it is called with
                   * garbage values (dest=0x39ff5dc0, src=0x2ffe0764,
                   * size=0x17fc5d78 ~ 412MB), causing an infinite 64-byte copy
                   * loop that never finishes. Stub to a no-op (bx lr) so boot
                   * can progress past this. ARM: bx lr */
                  {
                      uint8_t stub[] = {
                          0x1E, 0xFF, 0x2F, 0xE1,  /* bx lr */
                      };
                      cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + 0x178AC, stub, sizeof(stub));
                  }

                  /* 0x18017252: iBoot format/macro expander. When it hits a '%'
                   * it dispatches via a char table (base 0x18016ebc, entries
                   * 0x18016ebc + (c-0x23)*4) whose handlers live at 0x28xxxxxx --
                   * virtual addresses only valid with the iBoot MMU. We run with
                   * the MMU off, so the jump lands in unmapped RAM and the CPU
                   * executes data as code (PC-walk crash). Fix: rewrite the
                   * dispatch's `mov pc, r3` @0x180172ba into `b 0x18017282` so the
                   * specifier is consumed and the expander just keeps copying the
                   * remaining input (no handler jump). As a backup, also point
                   * every char-table entry at a Thumb BX-LR stub in SRAM. */
                  {
                      const uint8_t skip_disp[] = {0xE2, 0xEF}; /* B 0x18017282 */
                      cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + 0x172BA,
                                               skip_disp, sizeof(skip_disp));
                      /* Safe handler stub: BX lr (Thumb) at SRAM 0x2200fa00. */
                      const uint8_t bxlr[] = {0xF0, 0x47};
                      cpu_physical_memory_write(0x2200fa00, bxlr, sizeof(bxlr));
                      uint32_t stub_addr = 0x2200fa01; /* Thumb (T-bit set) */
                      for (int i = 0; i < 0x58; i++) {
                          cpu_physical_memory_write(0x18016ebc + (hwaddr)i * 4,
                                                   &stub_addr, sizeof(stub_addr));
                      }
                      fprintf(stderr,
                              ">>> JUMP CB: format-expander dispatch skip @0x180172ba; "
                              "char-table -> 0x2200fa01\n");
                  }

                  /* 0x79DC: internal alloc -> return heap_buf+0x10 */
                 {
                     uint32_t buf3 = heap_buf + 0x10;
                    uint8_t stub[] = {
                        0x00, 0x48,  /* LDR r0, [PC, #0] */
                        0xF0, 0x47,  /* BX lr */
                        (uint8_t)(buf3), (uint8_t)(buf3>>8),
                        (uint8_t)(buf3>>16), (uint8_t)(buf3>>24),
                    };
                    cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + 0x79DC, stub, sizeof(stub));
                }

                 /* 0x763C: strcmp-like (board ID check) -> return 0 (match).
                  * Returning 0 makes BEQ at 0x5CE6 taken, skipping hardware-specific
                  * strcmp/init path. All those functions are also stubbed as backup. */
                {
                    uint8_t stub[] = {
                        0x00, 0x20,     /* MOV r0, #0 */
                        0xF0, 0x47,     /* BX lr */
                    };
                    cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + 0x763C, stub, sizeof(stub));
                }

                /* 0x5C00: comparison function -> return 0,0 (safe no-op) */
                {
                    uint8_t stub[] = {
                        0x00, 0x20,     /* MOV r0, #0 */
                        0x00, 0x21,     /* MOV r1, #0 */
                        0xF0, 0x47,     /* BX lr */
                    };
                    cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + 0x5C00, stub, sizeof(stub));
                }

                 /* 0x17BE0: free-like function -> return 0 (no-op) */
                 {
                     uint8_t stub[] = {
                         0x00, 0x20,     /* MOV r0, #0 */
                         0xF0, 0x47,     /* BX lr */
                     };
                     cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + 0x17BE0, stub, sizeof(stub));
                 }

                  /* 0x49C0: stateful getchar (called via the 0x17D6A wrapper).
                   * 36-byte function fills the original 38-byte function (ends
                   * before the dead POP at 0x49E4). It reads an index from SRAM
                   * (CMD_IDX=0x22011180), returns CMD_BUF[index] (CMD_BUF=
                   * 0x22011100), advances the index, and returns 0x0A (newline)
                   * once index >= 20. The command "help\nbootx 60000000\n"
                   * prints help, then boots the kernelcache preloaded at
                   * 0x60000000. The 0x17D6A wrapper (below) is the console's
                   * actual getchar entry: it BLs 0x49C0 then does CR->LF. A
                   * prior session had stubbed 0x17D6A to return 0x0A directly,
                   * which is why 0x49C0 was never reached. */
                  {
                      const uint8_t getchar_code[] = {
                          0x06, 0x49, 0x08, 0x68, 0x14, 0x23, 0x98, 0x42,
                          0x06, 0xDA, 0x05, 0x4A, 0x02, 0x44, 0x13, 0x78,
                          0x40, 0x1C, 0x08, 0x60, 0x18, 0x46, 0x70, 0x47,
                          0x0A, 0x20, 0x70, 0x47,
                          0x80, 0x11, 0x01, 0x22, /* literal CMD_IDX 0x22011180 */
                          0x00, 0x11, 0x01, 0x22  /* literal CMD_BUF 0x22011100 */
                      };
                      /* 0x17D6A wrapper (18 bytes, original code): push {r7,lr};
                       * add r7,sp,#0; bl 0x49C0; uxtb r0,r0; cmp r0,#0xd; bne +4;
                       * movs r0,#0xa; pop {r7,pc}. */
                      const uint8_t getchar_wrapper[] = {
                          0x80, 0xB5, 0x00, 0xAF, 0xEC, 0xF7, 0x27, 0xFE,
                          0xC0, 0xB2, 0x0D, 0x28, 0x00, 0xD1, 0x0A, 0x20,
                          0x80, 0xBD
                      };
                      static const uint8_t cmd[] = "help\nbootx 60000000\n";
                      const uint32_t cmd_buf = 0x22011100; /* SRAM command buffer */
                      const uint32_t cmd_idx = 0x22011180; /* SRAM command index */
                      uint32_t idx0 = 0;
                      cpu_physical_memory_write(cmd_buf, cmd, sizeof(cmd) - 1);
                      cpu_physical_memory_write(cmd_idx, &idx0, sizeof(idx0));
                      cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + 0x49C0,
                                               getchar_code, sizeof(getchar_code));
                      cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + 0x17D6A,
                                               getchar_wrapper, sizeof(getchar_wrapper));
                      fprintf(stderr, ">>> JUMP CB: 0x49C0 getchar + 0x17D6A wrapper; buf=0x%08x idx=0x%08x\n",
                              cmd_buf, cmd_idx);
                  }

                 /* 0x17DE0: strcmp -> return 0 (strings match).
                  * Prevents iBoot from trying to read/compare board ID strings
                  * from unmapped hardware registers. */
                 {
                     uint8_t stub[] = {
                         0x00, 0x20,     /* MOV r0, #0 */
                         0xF0, 0x47,     /* BX lr */
                     };
                     cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + 0x17DE0, stub, sizeof(stub));
                 }

                 /* 0x57A8: init function (called from main_init @ 0x5CFA) -> return 0 */
                 {
                     uint8_t stub[] = {
                         0x00, 0x20,     /* MOV r0, #0 */
                         0xF0, 0x47,     /* BX lr */
                     };
                     cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + 0x57A8, stub, sizeof(stub));
                 }

                 /* 0x595E: init function (called from main_init @ 0x5CFE) -> return 0 */
                 {
                     uint8_t stub[] = {
                         0x00, 0x20,     /* MOV r0, #0 */
                         0xF0, 0x47,     /* BX lr */
                     };
                     cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + 0x595E, stub, sizeof(stub));
                 }

                  /* 0x75F4: allocation/init (called from entry dispatcher) -> return heap_buf+0x20 */
                  {
                      uint32_t buf4 = heap_buf + 0x20;
                      uint8_t stub[] = {
                          0x00, 0x48,  /* LDR r0, [PC, #0] */
                          0xF0, 0x47,  /* BX lr */
                          (uint8_t)(buf4), (uint8_t)(buf4>>8),
                          (uint8_t)(buf4>>16), (uint8_t)(buf4>>24),
                      };
                      cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + 0x75F4, stub, sizeof(stub));
                  }

                  /* 0x5490: buffer alloc (called from entry dispatcher) -> return heap_buf+0x40 */
                  {
                      uint32_t buf5 = heap_buf + 0x40;
                      uint8_t stub[] = {
                          0x00, 0x48,  /* LDR r0, [PC, #0] */
                          0xF0, 0x47,  /* BX lr */
                          (uint8_t)(buf5), (uint8_t)(buf5>>8),
                          (uint8_t)(buf5>>16), (uint8_t)(buf5>>24),
                      };
                      cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + 0x5490, stub, sizeof(stub));
                  }

                 /* 0x7E80: init function (called from entry dispatcher) -> return 0 */
                 {
                     uint8_t stub[] = {
                         0x00, 0x20,     /* MOV r0, #0 */
                         0xF0, 0x47,     /* BX lr */
                     };
                     cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + 0x7E80, stub, sizeof(stub));
                 }

                 /* 0x7E40: init function (called from entry dispatcher) -> return 0 */
                 {
                     uint8_t stub[] = {
                         0x00, 0x20,     /* MOV r0, #0 */
                         0xF0, 0x47,     /* BX lr */
                     };
                     cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + 0x7E40, stub, sizeof(stub));
                 }

                  /* 0x189C0 and 0x18A20 are ARM helper functions (64-bit multiply
                   * and a register-shuffling helper) in the ARM code region. The
                   * registration init (0x18000AE0) calls 0x189C0 via BLX to an even
                   * address, i.e. in ARM mode. A Thumb stub (movs r0,#0; bx lr)
                   * here is undefined in ARM mode -> undefined-instruction fault.
                   * Previously mislabelled "hardware read"; the real functions must
                   * run, so do NOT stub them. */

                 /* 0x43E0: init function (called from entry dispatcher) -> return 0 */
                 {
                     uint8_t stub[] = {
                         0x00, 0x20,     /* MOV r0, #0 */
                         0xF0, 0x47,     /* BX lr */
                     };
                     cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + 0x43E0, stub, sizeof(stub));
                 }

                 /* 0x4A88: free/cleanup (called from entry dispatcher) -> return 0 */
                 {
                     uint8_t stub[] = {
                         0x00, 0x20,     /* MOV r0, #0 */
                         0xF0, 0x47,     /* BX lr */
                     };
                     cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + 0x4A88, stub, sizeof(stub));
                 }

                 /* 0x2FA8: init function (called from 0x5C00 dispatcher) -> return 0 */
                 {
                     uint8_t stub[] = {
                         0x00, 0x20,     /* MOV r0, #0 */
                         0xF0, 0x47,     /* BX lr */
                     };
                     cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + 0x2FA8, stub, sizeof(stub));
                 }

                 /* 0x17866: printf/error logging (called from entry dispatcher) -> return 0 */
                 {
                     uint8_t stub[] = {
                         0x00, 0x20,     /* MOV r0, #0 */
                         0xF0, 0x47,     /* BX lr */
                     };
                     cpu_physical_memory_write(S5L8900_IBOOT_RUNTIME + 0x17866, stub, sizeof(stub));
                 }

                  queue_tb_flush(cs);
                  fprintf(stderr, ">>> JUMP CB: stubbed heap/init/hardware functions -> SRAM 0x%08x\n", heap_buf);
             }

             /* Mirror ALL patched iBoot data to USBOTG region (0x18000000).
              * iBoot literal pools reference 0x180xxxxx addresses. By mirroring
              * the fully-patched iBoot region, all data accesses to 0x180xxxxx
              * return correct values (strings, patched stubs, zeroed BSS). */
            {
                size_t mirror_size = S5L8900_IBOOT_SIZE;
                uint8_t *mirror_buf = g_malloc(mirror_size);
                cpu_physical_memory_read(S5L8900_IBOOT_RUNTIME, mirror_buf, mirror_size);
                cpu_physical_memory_write(S5L8900_USBOTG_BASE, mirror_buf, mirror_size);
                g_free(mirror_buf);
                fprintf(stderr, ">>> JUMP CB: mirrored patched iBoot (0x%zx bytes) 0x%08x -> 0x%08x\n",
                        mirror_size, S5L8900_IBOOT_RUNTIME, S5L8900_USBOTG_BASE);
            }

                /* Preload the kernelcache into A-bit RAM at 0x60000000 so that
                 * "bootx 60000000" can boot it. The 512MB region
                 * (s5l8900.abit_ram) is already mapped as RAM. */
                {
                    const char *kc_path =
                        "/Users/chris/dev/ipod-touch-1g/work/iPod1,1_1.1_3A101a_Restore/kernelcache.release.s5l8900xrb";
                    FILE *kc_f = fopen(kc_path, "rb");
                    if (kc_f) {
                        fseek(kc_f, 0, SEEK_END);
                        long kc_sz = ftell(kc_f);
                        fseek(kc_f, 0, SEEK_SET);
                        if (kc_sz > 0 && kc_sz < (long)S5L8900_ABIT_SIZE) {
                            uint8_t *kc_buf = g_malloc(kc_sz);
                            size_t kc_rd = fread(kc_buf, 1, kc_sz, kc_f);
                            cpu_physical_memory_write(S5L8900_ABIT_BASE, kc_buf, kc_rd);
                            g_free(kc_buf);
                            fprintf(stderr, ">>> JUMP CB: preloaded kernelcache %zu bytes @ 0x%08x\n",
                                    kc_rd, S5L8900_ABIT_BASE);
                        } else {
                            fprintf(stderr, ">>> JUMP CB: kernelcache bad size %ld\n", kc_sz);
                        }
                        fclose(kc_f);
                    } else {
                        fprintf(stderr, ">>> JUMP CB: WARN: cannot open kernelcache %s\n", kc_path);
                    }
                }

                /* Set up CPU state for iBoot's reset handler (ARM) at 0x18000440.
                 * The reset handler zeroes BSS (0x18021980-0x18026000), sets up
                 * the SVC/IRQ/FIQ/ABT/UND stacks, switches to SVC mode, and
                 * BXes [0x508] (entry dispatcher 0x4C20) which calls main_init.
                 * It overrides SP/LR/CPSR itself, so the values below are only
                 * the safe pre-boot state. */
              {
                  uint32_t sp = S5L8900_RAM_BASE + 0x20000; /* SRAM stack (reset handler overrides SP) */
                 cpu->env.regs[13] = sp;           /* SP (SVC mode) */
                 cpu->env.regs[14] = S5L8900_RAM_BASE + 0xF900; /* LR (reset handler overrides LR) */
                 cpu->env.thumb = 0;               /* ARM mode (reset handler is ARM) */
                 cpu->env.regs[15] = S5L8900_IBOOT_RUNTIME + 0x440; /* PC = reset handler (ARM) */
                /* SVC mode (0x13), ARM state (T=bit5=0), mask IRQ/FIQ (I=F=1) */
                cpu->env.uncached_cpsr = (cpu->env.uncached_cpsr & ~0x3F) | 0x13 | (1<<7) | (1<<6);
              }
        }

    /* FULL MMU REWORK: iBoot is loaded at PA 0x18000000. Its code runs at
     * VA 0x18000000 (identity) while its data is referenced at VA 0x28000000
     * (offset), both backed by the same physical image. Build a page table,
     * set TTBR/DACR, and enable MMU+cache so iBoot's 0x28xxxxxx data refs
     * resolve. (Previously this force-disabled the MMU, which broke iBoot's
     * virtual data references and caused a cascaded data abort.) */
      {
          #define S5L8900_PT_BASE (S5L8900_RAM_BASE + 0x38000) /* 0x22038000, in SRAM */
          static uint32_t pt[4096];
          memset(pt, 0, sizeof(pt));
           /* Section entries for QEMU's get_phys_addr_v5 walker, which requires
            *   [1:0]  = 0b10  : present, 1MB section  (desc & 3 == 2)
            *   [11:10] = 0b00 : AP = RW
            *   [8:5]  = 0b0000: domain 0
            * so the low attribute byte is 0x32 for Normal (RW cacheable) and 0x82
            * for Device (RW non-cacheable). The old 0x30/0x80 left [1:0]=0b00,
            * which the walker treats as a translation fault on every access. */
           #define SECT(pa)  ((uint32_t)(pa) | 0x32)
           #define DEV(pa)   ((uint32_t)(pa) | 0x82)
          pt[0x180] = SECT(0x18000000);   /* VA 0x18000000 -> PA 0x18000000 (code) */
          pt[0x280] = SECT(0x18000000);   /* VA 0x28000000 -> PA 0x18000000 (data) */
          pt[0x220] = SECT(S5L8900_RAM_BASE); /* VA 0x22000000 -> PA 0x22000000 (SRAM) */
          pt[0x230] = SECT(0x23000000);   /* VA 0x23000000 -> PA 0x23000000 (staging) */
          pt[0x000] = SECT(0x00000000);   /* VA 0x00000000 -> PA 0x00000000 (RAM) */
          pt[0x090] = SECT(0x09000000);   /* iBSS */
          pt[0x0A0] = SECT(0x0A000000);   /* iBEC */
           pt[0x600] = SECT(0x60000000);   /* kernelcache */
           pt[0xE00] = DEV(0xE0000000);    /* UART / device regs */
           pt[0x380] = DEV(0x38000000);    /* device regs (char-table) */
            pt[0x3E0] = DEV(0x3E000000);    /* device regs */
           /* Remaining RAM sections QEMU implements (Normal, cacheable). */
           pt[0x080] = SECT(0x08000000);   /* heap_ram */
           pt[0x240] = SECT(0x24000000);   /* unknown 64K RAM */
           pt[0x400] = SECT(0x40000000);   /* upper_ram */
           /* All S5L8900 peripheral sections QEMU implements (Device, ncache). */
           pt[0x381] = DEV(0x38100000);    /* clock0 */
           pt[0x382] = DEV(0x38200000);    /* unk2 */
           pt[0x384] = DEV(0x38400000);    /* usbctrl */
           pt[0x386] = DEV(0x38600000);    /* keystore */
           pt[0x38C] = DEV(0x38C00000);    /* usb */
           pt[0x38E] = DEV(0x38E00000);    /* vic0/vic1/edgeic */
           pt[0x39A] = DEV(0x39A00000);    /* gpioic */
           pt[0x3C3] = DEV(0x3C300000);    /* nand */
           pt[0x3C4] = DEV(0x3C400000);    /* usphy */
           pt[0x3C5] = DEV(0x3C500000);    /* clock1 */
            pt[0x3E1] = DEV(0x3E100000);    /* device regs */
            pt[0x3E2] = DEV(0x3E200000);    /* timer regs (0x3e200004 + n*0x20, 0x3e200088) */
            pt[0x3E3] = DEV(0x3E300000);    /* wdt */
            pt[0x3E4] = DEV(0x3E400000);    /* timer */
            pt[0x3E5] = DEV(0x3E500000);    /* pmu/sleep */
          cpu_physical_memory_write(S5L8900_PT_BASE, pt, sizeof(pt));

          cpu->env.cp15.ttbr0_s = S5L8900_PT_BASE;
          cpu->env.cp15.ttbr1_s = 0;
          cpu->env.cp15.dacr_s  = 0xFFFFFFFF; /* all 16 domains = Client (RW) */
          /* SCTLR: M(0)=1 A(1)=1 C(2)=1 I(12)=1 -> MMU + align + D-cache + I-cache */
          cpu->env.cp15.sctlr_s = 0x1007;
          arm_rebuild_hflags(&cpu->env);
          fprintf(stderr, ">>> MMU REWORK: page table @0x%08x TTBR0=0x%08x DACR=0x%08x SCTLR=0x%08x (MMU+cache ON)\n",
                  S5L8900_PT_BASE, (unsigned int)cpu->env.cp15.ttbr0_s,
                  (unsigned int)cpu->env.cp15.dacr_s, (unsigned int)cpu->env.cp15.sctlr_s);
      }

      /* Dump the dispatch literal-pool region after ALL QEMU patches are applied,
       * BEFORE iBoot's CPU runs. Compare with the crash-time DISPATCH dump to see
       * whether the clobber is all QEMU-side or partly CPU-side (iBoot init). */
      {
          uint32_t p0[16];
          cpu_physical_memory_read(0x18005FD0, p0, 64);
          fprintf(stderr, "POOL_AFTER_PATCH: ");
          for (int k = 0; k < 16; k++) fprintf(stderr, "0x%08x ", p0[k]);
          fprintf(stderr, "\n");
      }

      /* Signal periodic callback: iBoot is running, don't interfere */
      s5l8900_iboot_launched = 1;

      /* Start step trace timer to monitor iBoot execution */
      s5l8900_step_count = 50;
      if (s5l8900_step_timer) {
          timer_mod(s5l8900_step_timer,
                    qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + 1ULL * 1000 * 1000);
          fprintf(stderr, ">>> JUMP CB: started step trace timer (50 steps)\n");
      }

      /* Invalidate ALL TBs */
      queue_tb_flush(cs);
      cs->exception_index = -1;
      cs->exit_request = 1;
      cpu_reset_interrupt(cs, CPU_INTERRUPT_EXITTB);

     fprintf(stderr, ">>> JUMP CB: AFTER - pc=0x%08x thumb=%d cpsr=0x%08x lr=0x%08x sp=0x%08x\n",
            cpu->env.regs[15], cpu->env.thumb,
            (unsigned int)cpu->env.uncached_cpsr,
            (unsigned int)cpu->env.regs[14],
            (unsigned int)cpu->env.regs[13]);
}

static void s5l8900_periodic_dump_cb(void *opaque)
{
    ARMCPU *cpu = ARM_CPU(qemu_get_cpu(0));
    uint32_t pc = (uint32_t)cpu->env.regs[15];
    static int dump_cnt = 0;
    dump_cnt++;
    if (dump_cnt <= 5) {
        fprintf(stderr, "PERIODIC[%d]: pc=0x%08x thumb=%d\n", dump_cnt, pc, cpu->env.thumb);
        /* Check if trampoline executed */
        {
            uint8_t ram_val;
            cpu_physical_memory_read(S5L8900_RAM_BASE + 0x10500, &ram_val, 1);
            fprintf(stderr, "PERIODIC: RAM[0x10500]=0x%02x (%c) tramp_bytes=", ram_val, (ram_val >= 0x20 && ram_val <= 0x7E) ? ram_val : '.');
            uint8_t tb[4];
            cpu_physical_memory_read(S5L8900_RAM_BASE + 0x10400, tb, sizeof(tb));
            fprintf(stderr, "%02x %02x %02x %02x\n", tb[0], tb[1], tb[2], tb[3]);
        }
    }
    if (dump_cnt <= 20) {
        qemu_log_mask(LOG_UNIMP,
            "s5l8900.periodic: [%d] pc=0x%08x thumb=%d lr=0x%08x sp=0x%08x r0=0x%08x r1=0x%08x cpsr=0x%08x\n",
            dump_cnt, pc, cpu->env.thumb,
            (unsigned int)cpu->env.regs[14], (unsigned int)cpu->env.regs[13],
            (unsigned int)cpu->env.regs[0], (unsigned int)cpu->env.regs[1],
            (unsigned int)cpu->env.uncached_cpsr);

        /* Verify: dump bytes at 0x22004ea0-0x22004ebf to check if patches survived self-copy */
        if (dump_cnt == 1) {
            uint8_t verify_buf[32];
            cpu_physical_memory_read(S5L8900_RAM_BASE + 0x4ea0, verify_buf, sizeof(verify_buf));
            fprintf(stderr, "PERIODIC: bytes @ 0x%08x+0x4ea0: ", S5L8900_RAM_BASE);
            for (int i = 0; i < 32; i++) {
                fprintf(stderr, "%02x ", verify_buf[i]);
            }
            fprintf(stderr, "\n");
            fprintf(stderr, "PERIODIC: expected: ea 00 ea 00 ... (Thumb B #-2 loop)\n");

            /* Also dump bytes at 0x09004ea0 (iBSS source) */
            cpu_physical_memory_read(S5L8900_IBSS_BASE + 0x4ea0, verify_buf, sizeof(verify_buf));
            fprintf(stderr, "PERIODIC: bytes @ 0x%08x+0x4ea0: ", S5L8900_IBSS_BASE);
            for (int i = 0; i < 32; i++) {
                fprintf(stderr, "%02x ", verify_buf[i]);
            }
            fprintf(stderr, "\n");
        }
        /* Dump bytes around PC */
        if (pc >= 0x18000000) {
            uint8_t inst[16];
            cpu_physical_memory_read(pc & ~0xF, inst, 16);
            qemu_log_mask(LOG_UNIMP,
                "s5l8900.periodic:   bytes @ 0x%08x: %02x %02x %02x %02x %02x %02x %02x %02x\n",
                (unsigned int)(pc & ~0xF),
                inst[0], inst[1], inst[2], inst[3], inst[4], inst[5], inst[6], inst[7]);
        }
    }

    /* CLEAN BASELINE: after the diagnostic logging above, log a rate-limited
     * PC progression and RETURN, skipping the entire safety-redirect /
     * crash-recovery state machine below. iBSS runs unmodified so we can
     * observe its genuine first fault (via -d in_asm trace). */
    if (s5l8900_ibss_clean()) {
        static int clean_cnt = 0;
        static int clean_transitioned = 0;
        clean_cnt++;
        if (clean_cnt <= 40 || clean_cnt % 100 == 0) {
            /* mode: 0x13=USR 0x17=SVC 0x12=ABT 0x16=UND 0x11=IRQ 0x1F=FIQ */
            uint32_t mode = cpu->env.uncached_cpsr & 0x1F;
            const char *mn = (mode==0x13)?"USR":(mode==0x17)?"SVC":(mode==0x12)?"ABT":(mode==0x16)?"UND":(mode==0x11)?"IRQ":(mode==0x1F)?"FIQ":"?";
            fprintf(stderr, "CLEAN-PERIODIC[%d]: pc=0x%08x lr=0x%08x thumb=%d %s sp=0x%08x cpsr=0x%08x\n",
                    clean_cnt, pc,
                    (unsigned int)cpu->env.regs[14],
                    cpu->env.thumb, mn,
                    (unsigned int)cpu->env.regs[13],
                    (unsigned int)cpu->env.uncached_cpsr);
            fflush(stderr);
        }
        /* iBSS has reached its natural completion halt (b . at 0x22001360:
         * it parked itself after enabling the WDT, waiting for the host to
         * drive the USB DFU handshake and push iBEC). With no USB host
         * present that handshake never arrives, so we perform the same
         * iBSS->iBEC/iBoot handoff directly (the code path the real
         * "config_board\0" signal would trigger). */
        if (!clean_transitioned && pc == 0x22001360) {
            clean_transitioned = 1;
            fprintf(stderr, "CLEAN: iBSS at halt 0x22001360 - triggering iBEC/iBoot handoff\n");
            fflush(stderr);
            /* Arm the fine pool watcher BEFORE the handoff so it catches the
             * iBoot init relocation (which rewrites the dispatch table base
             * in the 0x18005FD0 literal pool) with the writer PC. */
            s5l8900_poolwatch_armed = 1;
            s5l8900_poolwatch_changes = 0;
            s5l8900_poolwatch_prev_valid = 0;
            if (s5l8900_poolwatch_timer) {
                timer_mod(s5l8900_poolwatch_timer,
                          qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + 3ULL * 1000);
            }
            s5l8900_config_board_trigger(cpu);
        }
        /* Re-arm (realtime) so we keep sampling the CPU (now iBoot after the
         * handoff, useful for tracking the console/dispatch path). */
        timer_mod(s5l8900_periodic_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + 100ULL * 1000 * 1000);
        return;
    }

    /* Safety redirect: if CPU is in upper RAM (zeros/garbage), it will
            * execute YIELD/undefined instructions and loop forever. Detect
            * and redirect using exit_request + TB flush (safe from timer cb).
            * Escalates aggressively: patches the actual caller, function entries,
            * data pointers, and crash memory, then forces CPU to safe address. */
        {
         static int upper_ram_state = 0; /* 0=idle, 1=first, 2=escalated, 3=done */
            static uint32_t last_crash_pc = 0;
            static int periodic_call_count = 0;
            if (++periodic_call_count <= 10) {
                fprintf(stderr, "PERIODIC: call #%d, pc=0x%08x, state=%d\n", periodic_call_count, pc, upper_ram_state);
            }

            /* If jump callback already launched iBoot, skip state machine and go straight to monitoring */
            if (s5l8900_iboot_launched && upper_ram_state < 100) {
                upper_ram_state = 100;
                fprintf(stderr, "PERIODIC: iBoot launched by jump callback, entering monitoring mode\n");
            }

            int in_bad_ram = (pc >= 0x40000000) || (pc >= 0x22100000 && pc < 0x30000000);
           if (upper_ram_state == 0 && in_bad_ram) {
              upper_ram_state = 1;
                  fprintf(stderr, "PERIODIC: CPU in upper RAM! pc=0x%08x lr=0x%08x thumb=%d\n",
                          pc, (unsigned int)cpu->env.regs[14], cpu->env.thumb);
                  if (s5l8900_ibss_clean()) {
                      fprintf(stderr, "PERIODIC: CLEAN BASELINE - NOT applying crash-recovery patches\n");
                  } else {
 /* Write safe Thumb infinite loop at SRAM 0xFE00.
                    * Use B #-2 (0xE000) to trap safely. Do NOT use BX LR
                    * which would return to the crashing caller. */
                    uint8_t safe_loop[] = { 0xFE, 0xE7, 0xFE, 0xE7 }; /* B #-4; B #-4 */
                  cpu_physical_memory_write(S5L8900_RAM_BASE + 0xFE00, safe_loop, 4);

                  /* Write safe ARM stub at SRAM 0xFE10 (MOV r0,#1; BX LR) */
                  uint32_t arm_stub[] = { 0xE3A00001, 0xE12FFF1E };
                  cpu_physical_memory_write(S5L8900_RAM_BASE + 0xFE10, arm_stub, 8);

                /* Patch 0x3c00-0x4fff with ARM-mode MOV r0,#1; BX LR.
                    * CRITICAL: Must cover entire region since CPU can
                    * enter at any offset. Thumb bytes decode as ARM LDR/STR that
                    * read data and use it as jump targets. */
                    if (getenv("S5L8900_IBSS_FILL_3C00")) {
                        uint32_t arm_ret[] = { 0xE3A00001, 0xE12FFF1E };
                        size_t region_size = 0x4400 - 0x3c00;
                        uint8_t *safe_region = g_malloc(region_size);
                        for (size_t i = 0; i < region_size; i += 8) {
                            memcpy(safe_region + i, arm_ret, sizeof(arm_ret));
                        }
                        cpu_physical_memory_write(S5L8900_RAM_BASE + 0x3c00, safe_region, region_size);
                        g_free(safe_region);
                        fprintf(stderr, "PERIODIC: patched 0x3c00-0x4fff -> ARM MOV r0,#1/BX LR (%zu bytes)\n", region_size);
                    } else {
                        static int per_skip_logged = 0;
                        if (!per_skip_logged) {
                            per_skip_logged = 1;
                            fprintf(stderr, "PERIODIC: SKIPPED 0x3c00-0x4fff fill (S5L8900_IBSS_FILL_3C00 unset; real iBSS code preserved)\n");
                        }
                    }

               /* Patch BL callers OUTSIDE the 0x3c00-0x4fff region. */
                   {
                       uint32_t arm_ret[] = { 0xE3A00001, 0xE12FFF1E };
                       uint32_t callers[] = { 0x5006, 0x5020, 0x5c66, 0x950e, 0x966c };
                       for (int i = 0; i < 7; i++) {
                           cpu_physical_memory_write(S5L8900_RAM_BASE + callers[i],
                                                     arm_ret, sizeof(arm_ret));
                       }
                       fprintf(stderr, "PERIODIC: patched 7 BL callers -> ARM MOV r0,#1/BX LR\n");
                   }

                  /* Patch 0x5400 entry: Thumb MOV r0,#0; BX LR */
                  {
                      uint8_t ret_0_thumb[] = { 0x00, 0x20, 0x70, 0x47 };
                      cpu_physical_memory_write(S5L8900_RAM_BASE + 0x5400, ret_0_thumb, 4);
                  }

             /* Patch data pointers in the iBSS heap region */
                  {
                      uint32_t safe_ptr = S5L8900_RAM_BASE + 0x4ea0; /* ARM code, no thumb bit */
                      cpu_physical_memory_write(0x22012ccc, &safe_ptr, 4);
                     for (int p = 0; p < 0x200; p += 4) {
                         uint32_t val;
                         cpu_physical_memory_read(0x22012ccc - 0x40 + p, &val, 4);
                         if (val >= 0x40000000 && val < 0x100000000) {
                             cpu_physical_memory_write(0x22012ccc - 0x40 + p, &safe_ptr, 4);
                         }
                     }
                 }

                 /* CRITICAL: Patch the actual crash address in upper RAM.
                   * Write an infinite loop at the crash PC to trap execution. */
                  {
                      uint32_t arm_loop = 0xEAFFFFFE; /* ARM B #-2 */
                      last_crash_pc = pc;
                      for (int off = 0; off < 0x1000; off += 4) {
                          cpu_physical_memory_write(pc + off, &arm_loop, 4);
                      }
                      fprintf(stderr, "PERIODIC: patched crash region at 0x%08x with ARM loops\n", pc);
                  }

                  /* Fix bad data pointers at 0x22012ccc.
                   * These are used as function pointers by the crashing function. */
                  {
                      uint32_t safe_ptr = S5L8900_RAM_BASE + 0x4ea0;
                      cpu_physical_memory_write(0x22012ccc, &safe_ptr, 4);
                      for (int p = -0x40; p < 0x40; p += 4) {
                          uint32_t val;
                          cpu_physical_memory_read(0x22012ccc + p, &val, 4);
                          if (val >= 0xe0000000) {
                              cpu_physical_memory_write(0x22012ccc + p, &safe_ptr, 4);
                          } else if (val >= 0x00200000 && val < 0x00300000) {
                              uint32_t translated = val + (S5L8900_RAM_BASE - 0x00200000);
                              cpu_physical_memory_write(0x22012ccc + p, &translated, 4);
                          }
                      }
                  fprintf(stderr, "PERIODIC: fixed data pointers at 0x22012ccc\n");
                   }

               /* Skip iBSS polling loop at 0x3724: BLE #-4 -> NOP */
                  {
                      uint8_t thumb_nop[] = { 0x00, 0xBF };
                      cpu_physical_memory_write(S5L8900_RAM_BASE + 0x3724, thumb_nop, sizeof(thumb_nop));
                  }

                  /* Patch stuck function at 0x5bf0: Thumb MOV r0,#1; BX LR */
                  {
                      uint8_t thumb_ret[] = { 0x01, 0x20, 0x70, 0x47 };
                      cpu_physical_memory_write(S5L8900_RAM_BASE + 0x5bf0, thumb_ret, sizeof(thumb_ret));
                  }

      queue_tb_flush(CPU(cpu));
                  }
                 }
             else if (upper_ram_state == 1) {
             /* Escalation: force CPU to safe address directly.
               * Skip the safe loop intermediate step; go straight to iBEC. */
              upper_ram_state = 2;
              fprintf(stderr, "PERIODIC: ESCALATION - forcing direct iBEC redirect\n");

               /* Write UART trampoline at SRAM 0xFF00 that directly outputs
                * "iBoot start" via UART MMIO, then hangs.
                * Layout: MOVW/MOVT r0=UART base, LDR r1=str addr, loop LDRB/STRB, hang, literal */
                   {
                       uint32_t str_addr = S5L8900_IBOOT_BASE + 0x1AE4D; /* "iBoot start" */
                       uint8_t uart_tramp[] = {
                           0x42, 0xF2, 0x00, 0x00,  /* 0xFF00: MOVW r0, #0x2000 */
                           0xCE, 0xF2, 0x00, 0x00,  /* 0xFF04: MOVT r0, #0xE000 */
                           0x04, 0x49,              /* 0xFF08: LDR r1, [PC, #0x10] -> 0xFF0C+0x10=0xFF1C */
                           0x0A, 0x78,              /* 0xFF0A: LDRB r2, [r1] */
                           0x01, 0x31,              /* 0xFF0C: ADDS r1, #1 */
                           0x00, 0x2A,              /* 0xFF0E: CMP r2, #0 */
                           0x01, 0xD0,              /* 0xFF10: BEQ #4 -> 0xFF16 */
                           0x02, 0x70,              /* 0xFF12: STRB r2, [r0] */
                           0xF9, 0xE7,              /* 0xFF14: B #-14 -> 0xFF0A */
                           0xFE, 0xE7,              /* 0xFF16: B . (hang) */
                           0x00, 0xBF, 0x00, 0xBF,  /* 0xFF18: NOP NOP */
                       };
                       cpu_physical_memory_write(S5L8900_RAM_BASE + 0xFF00, uart_tramp, sizeof(uart_tramp));
                       cpu_physical_memory_write(S5L8900_RAM_BASE + 0xFF1C, (uint8_t *)&str_addr, 4);
                       fprintf(stderr, "PERIODIC: wrote UART trampoline at 0x%08x, str=0x%08x\n",
                               S5L8900_RAM_BASE + 0xFF00, str_addr);
                   }

              /* Re-patch exception vectors (safe ARM loops) */
              s5l8900_evec_redirect_all();

              /* Replace the ARM infinite loop at 0xF900 with a Thumb redirect.
               * Thumb LDR r0,[pc,#imm] reads from (instr_addr+8) + imm*4.
               * At 0xF900 with imm=0: reads from 0xF908. */
                  {
                      uint8_t redirect_bytes[] = {
                          0x00, 0x48,   /* 0xF900: LDR r0, [pc, #0]  -> reads 0xF908 */
                          0x00, 0x47,   /* 0xF902: BX r0              -> Thumb mode switch */
                          0x00, 0x00,   /* 0xF904: padding */
                          0x00, 0x00,   /* 0xF906: padding */
                          0x01, 0xFF,   /* 0xF908: low 16 of 0x2200FF01 */
                          0x00, 0x22,   /* 0xF90A: high 16 of 0x2200FF01 */
                      };
                      cpu_physical_memory_write(S5L8900_RAM_BASE + 0xF900, redirect_bytes, sizeof(redirect_bytes));

                      /* Safe Thumb infinite loop at 0xF910 (for iBoot return address) */
                       uint16_t safe_loop[] = { 0xE7FE, 0xE7FE }; /* B #-2; B #-2 */
                      cpu_physical_memory_write(S5L8900_RAM_BASE + 0xF910, safe_loop, sizeof(safe_loop));

                      /* ARM safe loop at 0xF920 (for exception vector fallback) */
                      uint32_t arm_loop = 0xEAFFFFFE; /* B #-4 */
                      cpu_physical_memory_write(S5L8900_RAM_BASE + 0xF920, &arm_loop, 4);

                      fprintf(stderr, "PERIODIC: replaced ARM loop at 0xF900 with Thumb redirect\n");
                  }

              /* Clear pending interrupts and reset to SVC mode with IRQ+FIQ masked.
                 * Prevents illegal AArch32 mode switch (e.g., abt->fiq) during redirect. */
                cpu_reset_interrupt(qemu_get_cpu(0), CPU_INTERRUPT_HARD | CPU_INTERRUPT_EXITTB);
                cpu->env.uncached_cpsr = (cpu->env.uncached_cpsr & ~0xFF) | 0xD3; /* SVC + I+F+N, clear old mode */

                /* Disable MMU so guest code doesn't data-abort on MMIO */
                cpu->env.cp15.sctlr_s &= ~(1 | (1 << 2));
                cpu->env.cp15.ttbr0_s = 0;
                cpu->env.cp15.ttbr1_s = 0;

                /* Flush TB cache so the new instructions are used */
                queue_tb_flush(CPU(cpu));
                cpu_reset_interrupt(CPU(cpu), CPU_INTERRUPT_EXITTB);
         } else if (upper_ram_state == 2) {
              /* CPU escaped upper RAM. Apply full iBEC setup. */
              upper_ram_state = 3;
              fprintf(stderr, "PERIODIC: CPU escaped upper RAM, applying iBEC setup\n");
              s5l8900_config_board_trigger(cpu);
            } else if (upper_ram_state == 3) {
                /* Set up CPU to run iBoot entry function at 0x4C00 (Thumb).
                 * The jump callback already patched uart_putchar, initialized
                 * boot_args/device_info, and set up memory range globals.
                 * Now we just need to set registers and jump. */
                upper_ram_state = 4;
                {
                    uint32_t sp = S5L8900_RAM_BASE + 0x20000;
                    uint32_t boot_args_addr = S5L8900_RAM_BASE + 0x10000;
                    uint32_t device_addr = S5L8900_RAM_BASE + 0x10100;

                    /* Ensure boot_args and device_info are properly set up */
                    {
                        uint32_t boot_args[0x48/4] = {0};
                        boot_args[0x18/4] = boot_args_addr + 0x100;
                        boot_args[0x24/4] = boot_args_addr + 0x200;
                        boot_args[0x2C/4] = boot_args_addr + 0x100;
                        boot_args[0x38/4] = boot_args_addr + 0x100;
                        boot_args[0x40/4] = boot_args_addr + 0x100;
                        cpu_physical_memory_write(boot_args_addr, (uint8_t *)boot_args, sizeof(boot_args));

                        uint32_t device_info[0x20/4] = {0};
                        device_info[0x10/4] = boot_args_addr + 0x200;
                        device_info[0x18/4] = boot_args_addr + 0x300;
                        cpu_physical_memory_write(device_addr, (uint8_t *)device_info, sizeof(device_info));

                        /* Write "n45" board ID string */
                        {
                            uint8_t bi[] = "n45";
                            cpu_physical_memory_write(boot_args_addr + 0x200, bi, sizeof(bi));
                        }

                        /* Zero safe regions */
                        {
                            uint8_t zeros[0x200] = {0};
                            cpu_physical_memory_write(boot_args_addr + 0x100, zeros, sizeof(zeros));
                        }
                    }

                    /* Ensure uart_putchar is patched (from jump callback) */
                    /* It should write chars to SRAM 0x22010600 */

                    /* Set up CPU state for iBoot entry */
                    cpu->env.regs[13] = sp;          /* SP */
                    cpu->env.regs[0] = 2;            /* r0 = boot stage 2 (iBEC->iBoot) */
                    cpu->env.regs[1] = boot_args_addr; /* r1 = boot_args */
                    cpu->env.regs[4] = device_addr;   /* r4 = device_info */
                    /* LR: return to safe SRAM loop after entry function completes */
                    cpu->env.regs[14] = S5L8900_RAM_BASE + 0xF920;
                    cpu->env.regs[15] = S5L8900_IBOOT_BASE + 0x4C01; /* Thumb entry */
                    cpu->env.thumb = 1;
                    cpu->env.uncached_cpsr = (cpu->env.uncached_cpsr & ~0x1F) | 0x13; /* SVC mode */
                    cpu->env.cp15.sctlr_s = 0;     /* MMU off */
                    cpu->env.cp15.sctlr_ns = 0;
                    queue_tb_flush(CPU(cpu));
                    cpu_reset_interrupt(CPU(cpu), CPU_INTERRUPT_EXITTB);
                    fprintf(stderr, "PERIODIC: LAUNCH iBoot entry 0x4C01 r0=2 sp=0x%08x boot_args=0x%08x\n",
                            sp, boot_args_addr);
                }
            } else if (upper_ram_state == 4) {
                /* Monitor uart_putchar SRAM buffer and forward to UART.
                 * uart_putchar writes char to SRAM 0x22010600.
                 * We read it here and forward to serial. */
                {
                    static uint8_t last_char = 0;
                    static uint8_t seq_counter = 0;
                    uint8_t sram_char;
                    uint32_t sram_uart = S5L8900_RAM_BASE + 0x10600;

                    cpu_physical_memory_read(sram_uart, &sram_char, 1);

                    if (sram_char >= 0x20 && sram_char <= 0x7E) {
                        /* Check if this is a new character (not stuck) */
                        if (sram_char != last_char || seq_counter > 100) {
                            last_char = sram_char;
                            seq_counter = 0;
                            if (s5l8900_serial_chr) {
                                uint8_t ch = sram_char;
                                qemu_chr_write(s5l8900_serial_chr, &ch, 1, false);
                                fprintf(stderr, "iBoot: '%c' (0x%02x) pc=0x%08x\n",
                                        sram_char, sram_char, pc);
                            }
                        }
                    } else if (sram_char == '\n' || sram_char == '\r') {
                        if (sram_char != last_char) {
                            last_char = sram_char;
                            if (s5l8900_serial_chr) {
                                uint8_t ch = sram_char;
                                qemu_chr_write(s5l8900_serial_chr, &ch, 1, false);
                            }
                        }
                    }
                    seq_counter++;

                    /* Check if CPU is still in valid execution range */
                    int in_iboot = (pc >= S5L8900_IBOOT_BASE && pc < S5L8900_IBOOT_BASE + S5L8900_IBOOT_SIZE);
                    int in_sram = (pc >= S5L8900_RAM_BASE && pc < S5L8900_RAM_BASE + 0x40000);
                    if (!in_iboot && !in_sram) {
                        fprintf(stderr, "PERIODIC: CPU escaped to 0x%08x mode=0x%x\n",
                                pc, (unsigned int)(cpu->env.uncached_cpsr & 0x1F));
                        upper_ram_state = 100;
                    }
                }
              } else if (upper_ram_state < 10) {
                   /* If CPU is in iBoot region OR SRAM (valid iBoot execution territory) */
                   int in_iboot = (pc >= S5L8900_IBOOT_BASE && pc < S5L8900_IBOOT_BASE + S5L8900_IBOOT_SIZE);
                   int in_sram = (pc >= S5L8900_RAM_BASE && pc < S5L8900_RAM_BASE + 0x0400000); /* SRAM 0x22000000-0x26000000 */
                   fprintf(stderr, "PERIODIC: state=%d pc=0x%08x in_iboot=%d in_sram=%d [0x%08x, 0x%08x)\n",
                           upper_ram_state, pc, in_iboot, in_sram, S5L8900_RAM_BASE, S5L8900_RAM_BASE + 0x0400000);
                   if (in_iboot || in_sram) {
                      /* Trace first 100 instructions to see execution path */
                      static int first_trace = 0;
                      if (first_trace < 100) {
                          first_trace++;
                          uint8_t dump[8];
                          cpu_physical_memory_read(pc & ~1, dump, 8);
                          fprintf(stderr, "PERIODIC: TRACE[%d] pc=0x%08x thumb=%d mode=0x%x lr=0x%08x bytes=%02x%02x%02x%02x%02x%02x%02x%02x\n",
                                  first_trace, pc, cpu->env.thumb,
                                  (unsigned int)(cpu->env.uncached_cpsr & 0x1F),
                                  (unsigned int)cpu->env.regs[14],
                                  dump[0], dump[1], dump[2], dump[3], dump[4], dump[5], dump[6], dump[7]);
                      }
                      upper_ram_state = 100; /* Stable in iBoot */
                      if (first_trace >= 100) {
                          fprintf(stderr, "PERIODIC: CPU in iBoot/SRAM at 0x%08x thumb=%d\n", pc, cpu->env.thumb);
                      }
                    } else {
                      /* CPU in invalid region (e.g., BSS gap, SecureROM, etc.).
                       * Apply critical stubs, write Thumb trampoline to SRAM, and jump to it.
                       * The trampoline calls puts("iBoot start") then loops. */
                       static int bss_gap_handled = 0;
                       uint32_t tramp_addr = S5L8900_RAM_BASE + 0x10500;
                       if (!bss_gap_handled) {
                           bss_gap_handled = 1;

                          fprintf(stderr, "PERIODIC: BSS-GAP CATCH pc=0x%08x thumb=%d lr=0x%08x cpsr=0x%08x mode=0x%x\n",
                                  pc, cpu->env.thumb, (unsigned int)cpu->env.regs[14],
                                  (unsigned int)cpu->env.uncached_cpsr,
                                  (unsigned int)(cpu->env.uncached_cpsr & 0x1F));

                           /* Apply ALL patches from s5l8900_jump_to_tramp.
                            * This callback is never invoked, so we apply them here. */
                           {
                               uint32_t base = S5L8900_IBOOT_BASE;

                               /* === RESET HANDLER PATCHES === */

                               /* Patch load address literal at 0x4F0 to match computed addr */
                               {
                                   uint32_t expected_addr = S5L8900_IBOOT_BASE + 0x400;
                                   cpu_physical_memory_write(base + 0x4F0, (uint8_t *)&expected_addr, 4);
                               }

                               /* Patch BSS literal pool addresses */
                               {
                                   uint32_t bss_start = 0x23021980;
                                   uint32_t bss_end = 0x23026000;
                                   cpu_physical_memory_write(base + 0x4F8, (uint8_t *)&bss_start, 4);
                                   cpu_physical_memory_write(base + 0x4FC, (uint8_t *)&bss_end, 4);
                               }

                               /* NOP LDR/ADD/B sequences at 0x4AC-0x4DC (12 ARM instructions) */
                               {
                                   uint32_t arm_nop = 0xE1A00000; /* MOV r0, r0 */
                                   for (int ni = 0; ni < 12; ni++) {
                                       cpu_physical_memory_write(base + 0x4AC + ni * 4, &arm_nop, 4);
                                   }
                               }

                               /* Patch stack setup literals at 0x500/0x504 to SRAM */
                               {
                                   uint32_t sram_sp = S5L8900_RAM_BASE + 0x20000;
                                   cpu_physical_memory_write(base + 0x500, &sram_sp, 4);
                                   cpu_physical_memory_write(base + 0x504, &sram_sp, 4);
                               }

                               /* === PUTS STUBS === */

                               /* 0x4FA0: called from puts. Return 1. */
                               {
                                   uint16_t stub[] = { 0x2001, 0x4770 };
                                   cpu_physical_memory_write(base + 0x4FA0, stub, sizeof(stub));
                               }

                               /* 0x4FC4: called from puts. Return 0. */
                               {
                                   uint16_t stub[] = { 0x2000, 0x4770 };
                                   cpu_physical_memory_write(base + 0x4FC4, stub, sizeof(stub));
                               }

                               /* 0x4F00: called from 0x7BFC. Return 0. */
                               {
                                   uint16_t stub[] = { 0x2000, 0x4770 };
                                   cpu_physical_memory_write(base + 0x4F00, stub, sizeof(stub));
                               }

                               /* 0x7BFC: memory management. Return 0. */
                               {
                                   uint16_t stub[] = { 0x2000, 0x4770 };
                                   cpu_physical_memory_write(base + 0x7BFC, stub, sizeof(stub));
                               }

                                /* 0x4A5C: uart_putchar - write char to UART MMIO 0xE0002000 */
                                {
                                    uint8_t patch[] = {
                                        0xB5, 0x02,     /* PUSH {r0,lr} */
                                        0x60, 0xF2, 0x00, 0x20,  /* MOVW r2, #0x2000 */
                                        0xE0, 0xF2, 0x00, 0xE0,  /* MOVT r2, #0xE000 */
                                        0x20, 0x40,     /* STRB r0, [r2] */
                                        0xBC, 0x02,     /* POP {r0,lr} */
                                        0x70, 0x47,     /* BX LR */
                                    };
                                    cpu_physical_memory_write(base + 0x4A5C, patch, sizeof(patch));
                                }

                                fprintf(stderr, "PERIODIC: BSS-GAP: applied reset handler + puts patches\n");
                                /* === EXCEPTION VECTORS === */
                               {
                                   uint32_t handler_addr = S5L8900_RAM_BASE + 0xF000;
                                   uint32_t handler_code[] = {
                                       0xE12FFF1F,  /* MOV pc, lr (exception return, preserves T-bit) */
                                       0xE1A00000,  /* NOP */
                                   };
                                   cpu_physical_memory_write(handler_addr, handler_code, sizeof(handler_code));
                                   uint32_t vec_base = base + 0x404;
                                   for (uint32_t vi = 0; vi < 7; vi++) {
                                       uint32_t vec_addr = vec_base + vi * 4;
                                       int32_t offset = (handler_addr - (vec_addr + 8)) / 4;
                                       uint32_t b_instr = 0xEA000000 | (offset & 0x00FFFFFF);
                                       cpu_physical_memory_write(vec_addr, &b_instr, 4);
                                   }
                               }

                               /* === BOOT ARGS + DEVICE INFO === */
                               {
                                   uint32_t ba = S5L8900_RAM_BASE + 0x10000;
                                   uint32_t ba_arr[16] = {0};
                                   ba_arr[0x18/4] = ba + 0x100;
                                   ba_arr[0x24/4] = ba + 0x200;
                                   ba_arr[0x2C/4] = ba + 0x100;
                                   ba_arr[0x38/4] = ba + 0x100;
                                   cpu_physical_memory_write(ba, ba_arr, sizeof(ba_arr));
                                   uint8_t zeros[0x200] = {0};
                                   cpu_physical_memory_write(ba + 0x100, zeros, sizeof(zeros));
                                   {
                                       const char *bid = "n45";
                                       cpu_physical_memory_write(ba + 0x200, (uint8_t *)bid, 4);
                                   }

                                   uint32_t di = S5L8900_RAM_BASE + 0x10100;
                                   uint32_t di_arr[8] = {0};
                                   di_arr[0x10/4] = ba + 0x200;
                                   cpu_physical_memory_write(di, di_arr, sizeof(di_arr));
                               }

                               /* === MEMORY RANGE GLOBALS === */
                               {
                                   uint32_t mem_end = S5L8900_RAM_BASE + 0x40000;
                                   uint32_t mem_start = S5L8900_RAM_BASE;
                                   cpu_physical_memory_write(base + 0x22FA0, &mem_end, 4);
                                   cpu_physical_memory_write(base + 0x22FA4, &mem_start, 4);
                                   cpu_physical_memory_write(base + 0x22FA8, &mem_end, 4);
                               }

                               /* === ENTRY FUNCTION GLOBALS === */
                               {
                                   uint32_t buf = S5L8900_RAM_BASE + 0x10100;
                                   cpu_physical_memory_write(base + 0x210E8, &buf, 4);
                                   cpu_physical_memory_write(base + 0x210E4, &buf, 4);
                               }

                                 /* === DISABLE MMU === */
                                 cpu->env.cp15.sctlr_s = 0;
                                 cpu->env.cp15.sctlr_ns = 0;

                                 /* === NOP ALL CP15 INSTRUCTIONS ===
                                  * QEMU doesn't emulate s5l8900 CP15. MRC/MCR instructions
                                  * cause undefined instruction exceptions. NOP the entire
                                  * range 0x528-0x804 plus scattered CP15 at 0x474, 0x478,
                                  * 0x1A858, 0x1D804. */
                                 {
                                     uint32_t arm_nop = 0xE1A00000; /* MOV r0, r0 */
                                     /* Main CP15 range: 0x528-0x804 */
                                     for (uint32_t a = 0x528; a <= 0x804; a += 4) {
                                         cpu_physical_memory_write(base + a, &arm_nop, 4);
                                     }
                                     /* Scattered CP15 instructions */
                                     cpu_physical_memory_write(base + 0x1A858, &arm_nop, 4);
                                     cpu_physical_memory_write(base + 0x1D804, &arm_nop, 4);
                                     fprintf(stderr, "PERIODIC: BSS-GAP: NOP'd CP15 (0x474, 0x478, 0x528-0x804, 0x1A858, 0x1D804)\n");
                                 }

                                  /* === ARM TRAMPOLINE at 0x580 ===
                                  * LDR r0, [pc, #0x18] -> pool at 0x5A0; BX r0
                                  * When LDR at 0x580 executes, ARM PC = 0x588, so literal = 0x588+0x18 = 0x5A0
                                  * Pool: address of main init function (0x5CA0 | 1) */
                                 {
                                     uint8_t tramp_code[] = {
                                         0x18, 0xF0, 0x06, 0xE5,   /* LDR r0, [pc, #0x18] (0xE506F018) */
                                         0x10, 0xF0, 0x2F, 0xE1,   /* BX r0 (0xE12FF010) */
                                         0xFE, 0xFF, 0xFF, 0xEA,   /* B . (fallback loop) */
                                         0xFE, 0xFF, 0xFF, 0xEA,
                                         0xFE, 0xFF, 0xFF, 0xEA,
                                         0xFE, 0xFF, 0xFF, 0xEA,
                                         0xFE, 0xFF, 0xFF, 0xEA,
                                         0xFE, 0xFF, 0xFF, 0xEA,
                                         /* Literal pool at 0x5A0 (offset 0x20 in array): main init (Thumb) */
                                         0xA0, 0x5C, 0x00, 0x23,   /* 0x23005CA0 LE (no thumb bit, set below) */
                                     };
                                     /* Set Thumb bit on literal: LE byte 0 of literal = array index 0x20 */
                                     tramp_code[0x20] |= 0x01;  /* Set Thumb bit: 0x23005CA1 */
                                     cpu_physical_memory_write(base + 0x580, tramp_code, sizeof(tramp_code));
                                     /* Verify trampoline bytes */
                                     {
                                         uint8_t verify[36];
                                         cpu_physical_memory_read(base + 0x580, verify, sizeof(verify));
                                         fprintf(stderr, "PERIODIC: BSS-GAP: trampoline verify: ");
                                         for (int vi = 0; vi < 12; vi++) fprintf(stderr, "%02x ", verify[vi]);
                                         fprintf(stderr, "literal=0x%02x%02x%02x%02x\n",
                                                 verify[0x20], verify[0x21], verify[0x22], verify[0x23]);
                                     }
                                 }

                                /* === INFINITE LOOP at 0x4EC === */
                                {
                                    uint8_t thumb_loop[] = { 0xFE, 0xE7, 0xFE, 0xE7 };
                                    cpu_physical_memory_write(base + 0x4EC, thumb_loop, sizeof(thumb_loop));
                                }

                                /* === HEAP STUBS (called by main init 0x5CA0) === */
                                {
                                    uint32_t heap_buf = S5L8900_RAM_BASE + 0x18000;
                                    uint8_t zeros[0x1000] = {0};
                                    cpu_physical_memory_write(heap_buf, zeros, sizeof(zeros));

                                    // 0x17F50: malloc -> return heap_buf
                                    {
                                        uint32_t v = heap_buf;
                                        uint8_t stub[] = {
                                            0x40, 0xF2, (uint8_t)v, (uint8_t)(v>>8),
                                            0xC0, 0xF2, (uint8_t)(v>>16), (uint8_t)(v>>24),
                                            0xF0, 0x47, 0x00, 0x46,
                                        };
                                        cpu_physical_memory_write(base + 0x17F50, stub, sizeof(stub));
                                    }
                                    // 0x7A34: allocator -> return heap_buf+8
                                    {
                                        uint32_t v = heap_buf + 8;
                                        uint8_t stub[] = {
                                            0x40, 0xF2, (uint8_t)v, (uint8_t)(v>>8),
                                            0xC0, 0xF2, (uint8_t)(v>>16), (uint8_t)(v>>24),
                                            0xF0, 0x47, 0x00, 0x46,
                                        };
                                        cpu_physical_memory_write(base + 0x7A34, stub, sizeof(stub));
                                    }
                                    // 0x4FA0: heap init -> return 0
                                    cpu_physical_memory_write(base + 0x4FA0, (uint16_t[]){0x2000, 0x4770}, 4);
                                    // 0x4FC4: heap mgmt -> return 0
                                    cpu_physical_memory_write(base + 0x4FC4, (uint16_t[]){0x2000, 0x4770}, 4);
                                    // 0x79DC: internal alloc -> return heap_buf+0x10
                                    {
                                        uint32_t v = heap_buf + 0x10;
                                        uint8_t stub[] = {
                                            0x40, 0xF2, (uint8_t)v, (uint8_t)(v>>8),
                                            0xC0, 0xF2, (uint8_t)(v>>16), (uint8_t)(v>>24),
                                            0xF0, 0x47, 0x00, 0x46,
                                        };
                                        cpu_physical_memory_write(base + 0x79DC, stub, sizeof(stub));
                                    }
                                    // 0x763C: init -> return 1
                                    cpu_physical_memory_write(base + 0x763C, (uint16_t[]){0x2001, 0x4770}, 4);
                                    // 0x5C00: comparison -> return 0,0
                                    cpu_physical_memory_write(base + 0x5C00, (uint16_t[]){0x2000, 0x2100, 0xF047}, 6);
                                    // 0x17BE0: free -> return 0
                                    cpu_physical_memory_write(base + 0x17BE0, (uint16_t[]){0x2000, 0x4770}, 4);
                                    // 0x7BFC: memory mgmt -> return 0
                                    cpu_physical_memory_write(base + 0x7BFC, (uint16_t[]){0x2000, 0x4770}, 4);
                                    fprintf(stderr, "PERIODIC: BSS-GAP: stubbed heap functions -> 0x%08x\n", heap_buf);
                                }

                                  /* === MAIN_INIT LITERAL POOL (0x5FD2-0x5FF8) ===
                                   * main_init loads literals for SP setup and global access.
                                   * Without patching, garbage values corrupt SP and cause crashes.
                                   * Must patch BEFORE mirror so runtime region gets correct values. */
                                 {
                                     uint32_t sp_literal = 0; /* ADD sp, 0 = no-op */
                                     uint32_t data_base = S5L8900_RAM_BASE + 0x18000; /* SRAM data area */

                                     /* SP base literal -> 0 (no-op for ADD sp, r4) */
                                     cpu_physical_memory_write(base + 0x5FD2, &sp_literal, 4);

                                     /* Data literals -> SRAM */
                                     { uint32_t v = data_base; cpu_physical_memory_write(base + 0x5FD6, &v, 4); }
                                     { uint32_t v = data_base + 4; cpu_physical_memory_write(base + 0x5FD8, &v, 4); }
                                     { uint32_t v = data_base + 8; cpu_physical_memory_write(base + 0x5FDC, &v, 4); }
                                     { uint32_t v = data_base + 0xC; cpu_physical_memory_write(base + 0x5FDE, &v, 4); }
                                     { uint32_t v = data_base + 0x10; cpu_physical_memory_write(base + 0x5FE2, &v, 4); }
                                     { uint32_t v = data_base + 0x14; cpu_physical_memory_write(base + 0x5FE4, &v, 4); }
                                     { uint32_t v = data_base + 0x18; cpu_physical_memory_write(base + 0x5FE8, &v, 4); }
                                     { uint32_t v = data_base + 0x1C; cpu_physical_memory_write(base + 0x5FEC, &v, 4); }
                                     { uint32_t v = data_base + 0x20; cpu_physical_memory_write(base + 0x5FFE, &v, 4); }
                                     { uint32_t v = data_base + 0x24; cpu_physical_memory_write(base + 0x5FF2, &v, 4); }
                                     { uint32_t v = data_base + 0x28; cpu_physical_memory_write(base + 0x5FF4, &v, 4); }
                                     cpu_physical_memory_write(base + 0x5FF6, &data_base, 4);
                                     { uint32_t v = data_base + 0x30; cpu_physical_memory_write(base + 0x5FF8, &v, 4); }

                                     /* Write test string at data_base for printf */
                                     {
                                         const char *test_str = "iBoot is running on emulated iPod Touch 1G!\n";
                                         uint8_t data_area[0x100] = {0};
                                         memcpy(data_area, test_str, strlen(test_str));
                                         cpu_physical_memory_write(data_base, data_area, sizeof(data_area));
                                     }

                                     fprintf(stderr, "PERIODIC: BSS-GAP: patched main_init literal pool 0x5FD2-0x5FF8 -> SRAM 0x%08x\n", data_base);
                                 }

                                   fprintf(stderr, "PERIODIC: BSS-GAP: full setup complete, MMU off\n");

                                  /* Patch function at 0x17F50 -> MOV r0,#1; POP {r4,r5,pc}
                                   * This function hangs early (CLZ + BL to unknown target).
                                   * Stub it to return success immediately. */
                                  {
                                      uint16_t stub[] = { 0x2001, 0xBD70 }; /* MOV r0,#1; POP {r4,r5,pc} */
                                      cpu_physical_memory_write(S5L8900_IBOOT_BASE + 0x17F50, stub, sizeof(stub));
                                      fprintf(stderr, "PERIODIC: BSS-GAP: stubbed func @ 0x17F50 -> MOV r0,#1; POP {r4,r5,pc}\n");
                                  }

                                  /* Mirror patched iBoot to USBOTG (0x18000000) so literal
                                   * references to 0x180xxxxx resolve correctly. */
                                {
                                    size_t mirror_size = S5L8900_IBOOT_SIZE;
                                    uint8_t *mirror_buf = g_malloc(mirror_size);
                                    cpu_physical_memory_read(S5L8900_IBOOT_BASE, mirror_buf, mirror_size);
                                    cpu_physical_memory_write(S5L8900_USBOTG_BASE, mirror_buf, mirror_size);
                                    g_free(mirror_buf);
                                    fprintf(stderr, "PERIODIC: BSS-GAP: mirrored iBoot to 0x%08x\n",
                                            S5L8900_USBOTG_BASE);
                                }

                                       /* REDIRECT CPU directly to main_init at 0x18005CA1 (Thumb mode).
                                        * Skip the reset handler entirely - it calls functions via BLX with
                                        * literals that point to unpatched code, causing exceptions that land
                                        * in evec RAM (0x00000000) where ARM NOPs decode as Thumb PC jumps.
                                        * All reset handler duties are done by QEMU: BSS is zeroed (QEMU memory),
                                        * MMU is off, SP is set, globals are patched, literals are fixed.
                                        * Jump to the runtime region (0x18000000) where iBoot expects to run. */
                                       {
                                           uint32_t main_init = S5L8900_IBOOT_RUNTIME + 0x5CA0 | 1; /* Thumb */
                                           /* CRITICAL: Stop periodic timer to prevent interference during redirect. */
                                           if (s5l8900_periodic_timer) {
                                               timer_del(s5l8900_periodic_timer);
                                               fprintf(stderr, "PERIODIC: BSS-GAP: stopped periodic timer\n");
                                           }

                                           /* Set SP to safe SRAM location (outside BSS region) */
                                           cpu->env.regs[13] = S5L8900_RAM_BASE + 0x30000;
                                           /* Set banked SPs for all exception modes */
                                           for (int si = 0; si < 8; si++) {
                                               cpu->env.banked_r13[si] = S5L8900_RAM_BASE + 0x30000;
                                           }

                                           /* Write stack frame for main_init POP {r4-r7,pc}.
                                            * main_init starts with PUSH {r4-r7,lr} (SP -= 0x14),
                                            * then ends with POP {r4-r7,pc}. Stack frame at SP+0x14: */
                                           {
                                               uint32_t sp = cpu->env.regs[13];
                                               uint32_t ret_addr = S5L8900_RAM_BASE + 0xF921; /* Thumb safe loop */
                                               /* PUSH {r4-r7,lr} writes at SP-0x14; POP reads from SP */
                                               /* Frame: r4=0, r5=0, r6=0, r7=0, pc=ret_addr */
                                               uint32_t frame[5] = {0, 0, 0, 0, ret_addr};
                                               cpu_physical_memory_write(sp + 0x14, frame, sizeof(frame));
                                           }

                                           /* Patch exception vectors at 0x00000000 (low vectors).
                                            * ALL vectors -> safe ARM loop at 0x2200F900.
                                            * This prevents exceptions from landing in evec RAM where ARM NOPs
                                            * (0xE1A00000) decode as Thumb ADD r0,pc,#0xC00 causing PC walk. */
                                           {
                                               /* Disable evec state to prevent evec_write handler from overwriting vectors */
                                               s5l8900_evec_state = NULL;

                                               uint32_t safe_arm_loop = S5L8900_RAM_BASE + 0xF900;
                                               /* Write ARM infinite loop at 0xF900 */
                                               uint32_t arm_loop = 0xEAFFFFFE; /* B #-4 */
                                               for (int li = 0; li < 16; li++)
                                                   cpu_physical_memory_write(safe_arm_loop + li * 4, &arm_loop, 4);

                                                 /* Common exception handler: saves faulting LR and CPSR, then loops.
                                                  * Layout at 0xF800:
                                                  *   0xF800: STR lr, [pc, #0x20]  -> 0xF828
                                                  *   0xF804: MRS r1, CPSR
                                                  *   0xF808: STR r1, [pc, #0x1c]  -> 0xF82C
                                                  *   0xF80C: B 0xF900
                                                  *   0xF828: faulting LR
                                                  *   0xF82C: faulting CPSR */
                                                uint32_t handler_base = S5L8900_RAM_BASE + 0xF800; /* Handler code base */
                                                uint32_t h[12] = {
                                                    0xE58FE020, /* STR lr, [pc, #0x20] */
                                                    0xE1000000, /* MRS r1, CPSR */
                                                    0xE58F101C, /* STR r1, [pc, #0x1c] */
                                                    0xEA00003B, /* B 0xF900 */
                                                    0, 0, 0, 0, 0, 0,
                                                    0,          /* 0xF828: faulting LR */
                                                    0           /* 0xF82C: faulting CPSR */
                                                };
                                                cpu_physical_memory_write(handler_base, h, sizeof(h));

                                                 /* Vector table: all low vectors branch to the common handler */
                                                 for (int i = 0; i < 8; i++) {
                                                     hwaddr vec_addr = i * 4;
                                                    int32_t offset = (handler_base - (vec_addr + 8)) / 4;
                                                    uint32_t b_instr = 0xEA000000 | (offset & 0x00FFFFFF);
                                                    cpu_physical_memory_write(vec_addr, &b_instr, 4);
                                                }

                                                fprintf(stderr, "PERIODIC: BSS-GAP: wrote common exception handler at 0x%08x (LR->0x%08x CPSR->0x%08x)\n",
                                                        handler_base, handler_base + 0x28, handler_base + 0x2c);

                                               /* Fill rest of evec RAM (0x24-0xFFF) with ARM loops */
                                               for (hwaddr a = 0x24; a < 0x1000; a += 4)
                                                   cpu_physical_memory_write(a, &arm_loop, 4);

                                               /* Write Thumb safe loop at 0xF920 (LR target for main_init return).
                                                * ARM bytes at this address would decode as Thumb ADD pc,pc,#0xC00
                                                * causing PC walk. Must use Thumb B #-4 (0xE7FE). */
                                               {
                                                   uint16_t thumb_loop[] = {0xE7FE, 0xE7FE, 0xE7FE, 0xE7FE};
                                                   cpu_physical_memory_write(S5L8900_RAM_BASE + 0xF920, thumb_loop, sizeof(thumb_loop));
                                               }

                                               /* Write Thumb safe loop at 0xFE00 (fallback) */
                                               {
                                                   uint16_t thumb_loop[] = {0xE7FE, 0xE7FE, 0xE7FE, 0xE7FE};
                                                   cpu_physical_memory_write(S5L8900_RAM_BASE + 0xFE00, thumb_loop, sizeof(thumb_loop));
                                               }

                                               fprintf(stderr, "PERIODIC: BSS-GAP: patched low vectors at 0x00000000 -> safe loop 0x%08x\n",
                                                       safe_arm_loop);
                                           }

                                             /* Jump directly to main_init via BX r0.
                                              * Set r0 in the callback to avoid LDR/MOVW issues.
                                              * Trampoline is just: BX r0 (one instruction). */
                                             {
                                                 uint32_t tramp = S5L8900_RAM_BASE + 0xFF00;
                                                 /* Write BX r0 trampoline (callback sets r0 to main_init) */
                                                 uint32_t bx_r0 = 0xE12FFF10;
                                                 cpu_physical_memory_write(tramp, &bx_r0, 4);
                                                 fprintf(stderr, "PERIODIC: BSS-GAP: wrote BX r0 at 0x%08x\n", tramp);

                                                 /* Schedule redirect via run_on_cpu */
                                                 run_on_cpu(CPU(cpu), s5l8900_cpu_redirect_cb, RUN_ON_CPU_HOST_ULONG(tramp));
                                                 fprintf(stderr, "PERIODIC: BSS-GAP: scheduled run_on_cpu redirect to BX r0 at 0x%08x\n", tramp);
                                             }

                                            /* Verify main_init bytes in runtime region */
                                            {
                                                uint32_t pc_before = cpu->env.regs[15];
                                                uint8_t verify[16];
                                                cpu_physical_memory_read(S5L8900_IBOOT_RUNTIME + 0x5CA0, verify, sizeof(verify));
                                                uint32_t pc_after = cpu->env.regs[15];
                                                fprintf(stderr, "PERIODIC: BSS-GAP: main_init @ 0x%08x: ", S5L8900_IBOOT_RUNTIME + 0x5CA0);
                                                for (int vi = 0; vi < 16; vi++) fprintf(stderr, "%02x ", verify[vi]);
                                                fprintf(stderr, " pc_before=0x%08x pc_after=0x%08x\n", pc_before, pc_after);
                                            }

                                            fprintf(stderr, "PERIODIC: BSS-GAP: SET pc=0x%08x sp=0x%08x thumb=%d cpsr=0x%08x (direct main_init)\n",
                                                   (unsigned int)cpu->env.regs[15],
                                                   (unsigned int)cpu->env.regs[13],
                                                   cpu->env.thumb,
                                                   (unsigned int)cpu->env.uncached_cpsr);

                                           /* Flush ALL TBs so new PC is used */
                                           queue_tb_flush(CPU(cpu));

                                            /* Force CPU to exit current exec loop */
                                            cpu_interrupt(CPU(cpu), CPU_INTERRUPT_EXITTB);

                                            /* Signal: iBoot launching, don't interfere */
                                            s5l8900_iboot_launched = 1;
                                            /* Delete step timer to prevent interference with TCG */
                                            if (s5l8900_step_timer) {
                                                timer_del(s5l8900_step_timer);
                                            }
                                      }
                              }

                               /* Redirect callback will handle PC, TB flush, and state transition.
                                * Do NOT call queue_tb_flush or cpu_reset_interrupt here - it interferes. */
                              upper_ram_state = 100;
                        }
                    }
                  } else if (upper_ram_state == 100) {
                     /* iBoot execution tracking + MMU monitoring + UART capture */
                     static int iboot_trace_cnt = 0;

                     /* LR validation on every BSS-GAP access: if LR is corrupted,
                      * fix it before a function return uses the garbage address. */
                     {
                         uint32_t lr = (unsigned int)cpu->env.regs[14];
                         int lr_valid = 0;
                         if (lr >= S5L8900_IBOOT_RUNTIME && lr < S5L8900_IBOOT_RUNTIME + 0x18000) lr_valid = 1;
                         if (lr >= S5L8900_IBOOT_BASE && lr < S5L8900_IBOOT_BASE + 0x18000) lr_valid = 1;
                         if (lr >= S5L8900_RAM_BASE && lr < S5L8900_RAM_BASE + S5L8900_RAM_SIZE) lr_valid = 1;
                         if (lr >= 0x20000000 && lr < 0x20010000) lr_valid = 1;
                         if (!lr_valid && lr != 0) {
                             static int bss_lr_cnt = 0;
                             if (++bss_lr_cnt <= 10) {
                                 fprintf(stderr, "BSS-GAP: CORRUPT LR #%d lr=0x%08x pc=0x%08x -> fixing\n",
                                         bss_lr_cnt, lr, pc);
                             }
                             cpu->env.regs[14] = S5L8900_RAM_BASE + 0xF920;
                         }
                     }

                     /* Dump SRAM buffer for trampoline UART output */
                    {
                        static int sram_dump_done = 0;
                        if (!sram_dump_done) {
                            sram_dump_done = 1;
                            uint8_t sram_buf[32];
                            cpu_physical_memory_read(S5L8900_RAM_BASE + 0x10000, sram_buf, sizeof(sram_buf));
                            fprintf(stderr, "PERIODIC: SRAM buffer @ 0x%08x: ", S5L8900_RAM_BASE + 0x10000);
                            for (int si = 0; si < 20; si++) {
                                if (sram_buf[si] >= 0x20 && sram_buf[si] <= 0x7E)
                                    fprintf(stderr, "%c", sram_buf[si]);
                                else
                                    fprintf(stderr, ".");
                            }
                            fprintf(stderr, " | hex: ");
                            for (int si = 0; si < 20; si++) fprintf(stderr, "%02x ", sram_buf[si]);
                            fprintf(stderr, "\n");
                        }
                    }

                    /* CRITICAL: If CPU is in reset handler area (0x400-0x580) in ARM mode,
                     * it's re-executing reset handler code due to exception return issues.
                     * Force redirect to safe ARM loop in SRAM.
                     * NOTE: Trampoline at 0x580 is excluded from this check. */
                    {
                        uint32_t reset_handler_start = S5L8900_IBOOT_BASE + 0x400;
                        uint32_t reset_handler_end = S5L8900_IBOOT_BASE + 0x580; /* Exclude trampoline at 0x580+ */
                        if (pc >= reset_handler_start && pc < reset_handler_end && !cpu->env.thumb) {
                            static int reset_reentry_cnt = 0;
                            if (++reset_reentry_cnt <= 3 || reset_reentry_cnt % 100 == 0) {
                                fprintf(stderr, "PERIODIC: RESET RE-ENTRY CATCH #%d pc=0x%08x cpsr=0x%08x -> redirect to safe loop\n",
                                        reset_reentry_cnt, pc, (unsigned int)cpu->env.uncached_cpsr);
                            }
                            cpu->env.uncached_cpsr = 0xD3; /* SVC mode, I+F */
                            cpu->env.regs[15] = S5L8900_RAM_BASE + 0xF920; /* ARM safe loop */
                            cpu->env.thumb = 0;
                            cpu_reset_interrupt(CPU(cpu), CPU_INTERRUPT_HARD | CPU_INTERRUPT_EXITTB);
                            queue_tb_flush(CPU(cpu));
                            cpu_reset_interrupt(CPU(cpu), CPU_INTERRUPT_EXITTB);
                         }
                    }

                     /* Safety net: if PC lands in data/BSS region (beyond code area),
                      * redirect to safe ARM loop in SRAM. */
                      {
                          uint32_t code_end = S5L8900_IBOOT_BASE + 0x20000; /* iBoot code extends far */
                          uint32_t data_end = S5L8900_IBOOT_BASE + S5L8900_IBOOT_SIZE;
                          if (pc >= code_end && pc < data_end) {
                              static int data_redirect_cnt = 0;
                              if (++data_redirect_cnt <= 3 || data_redirect_cnt % 100 == 0) {
                                  fprintf(stderr, "PERIODIC: DATA-REGION CATCH #%d pc=0x%08x -> redirect to safe loop\n",
                                          data_redirect_cnt, pc);
                              }
                              cpu->env.uncached_cpsr = 0xD3;
                              cpu->env.regs[15] = S5L8900_RAM_BASE + 0xF920;
                              cpu->env.thumb = 0;
                              cpu_reset_interrupt(CPU(cpu), CPU_INTERRUPT_HARD | CPU_INTERRUPT_EXITTB);
                              queue_tb_flush(CPU(cpu));
                              cpu_reset_interrupt(CPU(cpu), CPU_INTERRUPT_EXITTB);
                          }
                      }

                     /* Runtime region data/BSS execution catch.
                      * iBoot code ends well before the end of the binary.
                      * Executing from data section causes corruption. */
                     {
                          uint32_t rt_code_end = S5L8900_IBOOT_RUNTIME + 0x20000; /* iBoot code extends to ~0x1A000 */
                          uint32_t rt_data_end = S5L8900_IBOOT_RUNTIME + S5L8900_IBOOT_SIZE;
                         if (pc >= rt_code_end && pc < rt_data_end) {
                             static int rt_data_cnt = 0;
                             if (++rt_data_cnt <= 5) {
                                 fprintf(stderr, "PERIODIC: RT-DATA CATCH #%d pc=0x%08x (+0x%05x) lr=0x%08x -> safe loop\n",
                                         rt_data_cnt, pc, (uint32_t)(pc - S5L8900_IBOOT_RUNTIME),
                                         (unsigned int)cpu->env.regs[14]);
                             }
                             cpu->env.uncached_cpsr = 0xD3;
                             cpu->env.regs[15] = S5L8900_RAM_BASE + 0xF920;
                             cpu->env.thumb = 0;
                             cpu_reset_interrupt(CPU(cpu), CPU_INTERRUPT_HARD | CPU_INTERRUPT_EXITTB);
                             queue_tb_flush(CPU(cpu));
                             cpu_reset_interrupt(CPU(cpu), CPU_INTERRUPT_EXITTB);
                         }
                         /* Also catch execution beyond runtime binary (into zero region) */
                         uint32_t rt_beyond_end = S5L8900_IBOOT_RUNTIME + 0x30000;
                         if (pc >= rt_data_end && pc < rt_beyond_end) {
                             static int rt_beyond_cnt = 0;
                             if (++rt_beyond_cnt <= 5) {
                                 fprintf(stderr, "PERIODIC: RT-BEYOND CATCH #%d pc=0x%08x lr=0x%08x -> safe loop\n",
                                         rt_beyond_cnt, pc, (unsigned int)cpu->env.regs[14]);
                             }
                             cpu->env.uncached_cpsr = 0xD3;
                             cpu->env.regs[15] = S5L8900_RAM_BASE + 0xF920;
                             cpu->env.thumb = 0;
                             cpu_reset_interrupt(CPU(cpu), CPU_INTERRUPT_HARD | CPU_INTERRUPT_EXITTB);
                             queue_tb_flush(CPU(cpu));
                             cpu_reset_interrupt(CPU(cpu), CPU_INTERRUPT_EXITTB);
                         }
                     }

                     /* LR validation: if LR is corrupted (outside valid code ranges),
                      * prevent return to garbage address. */
                     {
                         uint32_t lr = (unsigned int)cpu->env.regs[14];
                         int lr_valid = 0;
                         /* Valid LR ranges: iBoot code, SRAM handlers, staging code */
                         if (lr >= S5L8900_IBOOT_RUNTIME && lr < S5L8900_IBOOT_RUNTIME + 0x18000) lr_valid = 1;
                         if (lr >= S5L8900_IBOOT_BASE && lr < S5L8900_IBOOT_BASE + 0x18000) lr_valid = 1;
                         if (lr >= S5L8900_RAM_BASE && lr < S5L8900_RAM_BASE + S5L8900_RAM_SIZE) lr_valid = 1;
                         if (lr >= 0x20000000 && lr < 0x20010000) lr_valid = 1; /* SecureROM */
                         if (!lr_valid && lr != 0) {
                             static int lr_corrupt_cnt = 0;
                             if (++lr_corrupt_cnt <= 5) {
                                 fprintf(stderr, "PERIODIC: CORRUPT LR #%d lr=0x%08x pc=0x%08x -> fixing to safe loop\n",
                                         lr_corrupt_cnt, lr, pc);
                             }
                             cpu->env.regs[14] = S5L8900_RAM_BASE + 0xF920; /* Safe return target */
                         }
                     }

                    /* BSS-EXEC catch: CPU trying to execute from BSS region.
                     * Force redirect to trampoline in SRAM. */
                    {
                        uint32_t bss_exec_start = S5L8900_IBOOT_BASE + 0x21980;
                        uint32_t bss_exec_end = S5L8900_IBOOT_BASE + 0x27000;
                        if (pc >= bss_exec_start && pc < bss_exec_end) {
                            static int bss_exec_cnt = 0;
                            if (++bss_exec_cnt <= 5) {
                                fprintf(stderr, "PERIODIC: BSS-EXEC CATCH #%d pc=0x%08x thumb=%d -> force trampoline\n",
                                        bss_exec_cnt, pc, cpu->env.thumb);
                                /* Verify redirect bytes */
                                uint8_t verify[12];
                                cpu_physical_memory_read(S5L8900_IBOOT_BASE + 0x26000, verify, sizeof(verify));
                                fprintf(stderr, "PERIODIC: bytes @ 0x23026000: ");
                                for (int vi = 0; vi < 12; vi++) fprintf(stderr, "%02x ", verify[vi]);
                                fprintf(stderr, "\n");
                            }
                            /* Force redirect to trampoline */
                            cpu->env.uncached_cpsr = 0xD3; /* SVC + I+F */
                            cpu->env.regs[15] = S5L8900_RAM_BASE + 0x10501; /* tramp_addr | 1 */
                            cpu->env.thumb = 1;
                            queue_tb_flush(CPU(cpu));
                            cpu_reset_interrupt(CPU(cpu), CPU_INTERRUPT_EXITTB);
                        }
                    }

                    if (++iboot_trace_cnt % 2 == 0) {
                        int in_iboot = (pc >= S5L8900_IBOOT_BASE && pc < S5L8900_IBOOT_BASE + S5L8900_IBOOT_SIZE);
                        int in_iboot_rt = (pc >= S5L8900_IBOOT_RUNTIME && pc < S5L8900_IBOOT_RUNTIME + S5L8900_IBOOT_SIZE);
                        int in_sram = (pc >= S5L8900_RAM_BASE && pc < S5L8900_RAM_BASE + 0x0400000);
                        if (in_iboot || in_iboot_rt || in_sram) {
                          /* Check if MMU got re-enabled */
                         uint64_t sctlr_s = cpu->env.cp15.sctlr_s;
                         uint64_t sctlr_ns = cpu->env.cp15.sctlr_ns;
                         static int last_mmu_s = -1, last_mmu_ns = -1;
                         int mmu_on_s = (sctlr_s & 1) ? 1 : 0;
                         int mmu_on_ns = (sctlr_ns & 1) ? 1 : 0;
                         if (mmu_on_s != last_mmu_s || mmu_on_ns != last_mmu_ns) {
                             fprintf(stderr, "PERIODIC: MMU state changed! SCTLR_s=0x%08x(M=%d) SCTLR_ns=0x%08x(M=%d) pc=0x%08x\n",
                                     (unsigned int)sctlr_s, mmu_on_s,
                                     (unsigned int)sctlr_ns, mmu_on_ns, pc);
                             last_mmu_s = mmu_on_s;
                             last_mmu_ns = mmu_on_ns;
                             /* Force-disable MMU if re-enabled */
                             if (mmu_on_s) {
                                 cpu->env.cp15.sctlr_s = 0;
                                 fprintf(stderr, "PERIODIC: FORCE-DISABLED secure MMU!\n");
                             }
                             if (mmu_on_ns) {
                                 cpu->env.cp15.sctlr_ns = 0;
                                 fprintf(stderr, "PERIODIC: FORCE-DISABLED non-secure MMU!\n");
                             }
                              queue_tb_flush(CPU(cpu));
                          }
                           /* Poll SRAM for UART characters written by patched uart_putchar */
                          {
                              hwaddr uart_sram = S5L8900_RAM_BASE + 0x10600; /* 0x22010600 */
                              uint8_t ch;
                              cpu_physical_memory_read(uart_sram, &ch, 1);
                              static uint8_t last_uart_ch = 0xFF; /* Start with impossible value */
                              if (ch != last_uart_ch && ch != 0x00) {
                                  last_uart_ch = ch;
                                  Chardev *chr = s5l8900_serial_chr;
                                  if (chr) {
                                      if (ch == '\n') {
                                          uint8_t cr = '\r';
                                          qemu_chr_write(chr, &cr, 1, false);
                                      }
                                      if (ch >= 0x08) {
                                          qemu_chr_write(chr, &ch, 1, false);
                                      }
                                  }
                                  fprintf(stderr, "PERIODIC: UART char: 0x%02x (%c)\n",
                                          ch, (ch >= 0x20 && ch < 0x7F) ? ch : '.');
                              }
                          }
                          fprintf(stderr, "PERIODIC: iBoot pc=0x%08x thumb=%d MMU_s=%d MMU_ns=%d\n",
                                  pc, cpu->env.thumb, mmu_on_s, mmu_on_ns);
                       } else {
                            /* Full register dump on escape - trace last valid PC vs crash PC */
                            static int escape_cnt = 0;
                            if (escape_cnt < 3) {
                                escape_cnt++;
                                fprintf(stderr, "\n=== ESCAPE #%d: pc=0x%08x thumb=%d (last_rt_pc=0x%08x) ===\n",
                                        escape_cnt, pc, cpu->env.thumb, s5l8900_last_valid_rt_pc);
                                fprintf(stderr, "  r0=0x%08x r1=0x%08x r2=0x%08x r3=0x%08x\n",
                                        (unsigned int)cpu->env.regs[0], (unsigned int)cpu->env.regs[1],
                                        (unsigned int)cpu->env.regs[2], (unsigned int)cpu->env.regs[3]);
                                fprintf(stderr, "  r4=0x%08x r5=0x%08x r6=0x%08x r7=0x%08x\n",
                                        (unsigned int)cpu->env.regs[4], (unsigned int)cpu->env.regs[5],
                                        (unsigned int)cpu->env.regs[6], (unsigned int)cpu->env.regs[7]);
                                fprintf(stderr, "  r8=0x%08x r9=0x%08x r10=0x%08x r11=0x%08x\n",
                                        (unsigned int)cpu->env.regs[8], (unsigned int)cpu->env.regs[9],
                                        (unsigned int)cpu->env.regs[10], (unsigned int)cpu->env.regs[11]);
                                fprintf(stderr, "  r12=0x%08x sp=0x%08x lr=0x%08x pc=0x%08x\n",
                                        (unsigned int)cpu->env.regs[12], (unsigned int)cpu->env.regs[13],
                                        (unsigned int)cpu->env.regs[14], pc);
                                fprintf(stderr, "  cpsr=0x%08x\n", (unsigned int)cpu->env.uncached_cpsr);
                                /* Read memory around SP to find return addresses */
                                uint32_t sp_val = (unsigned int)cpu->env.regs[13];
                                uint32_t stack_words[8];
                                if (sp_val >= S5L8900_RAM_BASE && sp_val < S5L8900_RAM_BASE + S5L8900_RAM_SIZE) {
                                    cpu_physical_memory_read(sp_val, stack_words, sizeof(stack_words));
                                    fprintf(stderr, "  stack@sp: ");
                                    for (int si = 0; si < 8; si++)
                                        fprintf(stderr, "0x%08x ", stack_words[si]);
                                    fprintf(stderr, "\n");
                                }
                                /* Read memory at last valid PC if in known region */
                                if (s5l8900_last_valid_rt_pc >= S5L8900_IBOOT_RUNTIME &&
                                    s5l8900_last_valid_rt_pc < S5L8900_IBOOT_RUNTIME + S5L8900_IBOOT_SIZE) {
                                    uint32_t lr_bytes[4];
                                    uint32_t lr_off = s5l8900_last_valid_rt_pc - S5L8900_IBOOT_RUNTIME;
                                    // Align to even boundary for Thumb
                                    lr_off &= ~1u;
                                    if (lr_off + 8 <= S5L8900_IBOOT_SIZE) {
                                        cpu_physical_memory_read(s5l8900_last_valid_rt_pc & ~1u, lr_bytes, 8);
                                        fprintf(stderr, "  bytes@last_rt_pc: %02x %02x %02x %02x %02x %02x %02x %02x\n",
                                                lr_bytes[0]&0xff, (lr_bytes[0]>>8)&0xff, (lr_bytes[0]>>16)&0xff, (lr_bytes[0]>>24)&0xff,
                                                lr_bytes[1]&0xff, (lr_bytes[1]>>8)&0xff, (lr_bytes[1]>>16)&0xff, (lr_bytes[1]>>24)&0xff);
                                    }
                                }
                            }
                            /* ALWAYS redirect to SRAM safe ARM loop - NOT staging area */
                            cpu->env.uncached_cpsr = 0xD3; /* SVC, I+F, ARM */
                            cpu->env.regs[15] = S5L8900_RAM_BASE + 0xF920; /* ARM safe loop */
                            cpu->env.thumb = 0;
                            cpu->env.regs[13] = S5L8900_RAM_BASE + 0x30000;
                            arm_rebuild_hflags(&cpu->env);
                            cpu_reset_interrupt(CPU(cpu), CPU_INTERRUPT_HARD | CPU_INTERRUPT_EXITTB);
                            queue_tb_flush(CPU(cpu));
                            cpu_reset_interrupt(CPU(cpu), CPU_INTERRUPT_EXITTB);
                        }
                  }
              }

          /* Granular PC tracing for iBoot runtime region (0x18000000).
           * Trace every PC while in iBoot code region to identify crash path. */
          {
              static uint32_t last_rt_pc = 0;
              static int rt_trace_cnt = 0;
              static uint32_t last_valid_rt_pc = 0;  /* For escape handler */
              if (pc >= S5L8900_IBOOT_RUNTIME && pc < S5L8900_IBOOT_RUNTIME + 0x20000) {
                  /* In iBoot code region - trace every unique PC */
                  s5l8900_last_valid_rt_pc = pc;
                  if (pc != last_rt_pc) {
                      last_rt_pc = pc;
                      rt_trace_cnt++;
                      fprintf(stderr, "RT_TRACE[%03d]: pc=0x%08x (+0x%05x) thumb=%d lr=0x%08x sp=0x%08x\n",
                              rt_trace_cnt, pc, (uint32_t)(pc - S5L8900_IBOOT_RUNTIME),
                              cpu->env.thumb, (unsigned int)cpu->env.regs[14],
                              (unsigned int)cpu->env.regs[13]);
                  }
               } else {
                   /* Reset tracing when leaving code region */
                   if (rt_trace_cnt > 0) {
                       last_rt_pc = 0;
                   }
               }
           }

          /* Check exception handler trace marker */
          {
              static uint8_t last_exc_marker = 0;
              uint8_t exc_marker;
              cpu_physical_memory_read(S5L8900_RAM_BASE + 0xED00, &exc_marker, 1);
              if (exc_marker != last_exc_marker && exc_marker == 0xFF) {
                  fprintf(stderr, "PERIODIC: EXCEPTION HANDLER INVOKED! pc=0x%08x cpsr=0x%08x\n",
                          pc, (unsigned int)cpu->env.uncached_cpsr);
                  /* Clear marker */
                  uint8_t clear = 0;
                  cpu_physical_memory_write(S5L8900_RAM_BASE + 0xED00, &clear, 1);
              }
              last_exc_marker = exc_marker;
          }
       }

 /* Post-redirect: guard against garbage execution and stuck loops.
          * When PC goes beyond valid iBEC/iBoot range, redirect to safe loop. */
         {
            static int post_redirect_cnt = 0;
            static int post_redirect_started = 0;
            static uint32_t last_traced_pc = 0;
            static int same_pc_count = 0;
            static int already_handled = 0;
            /* Skip if CPU is in iBoot region (valid execution) */
            int in_iboot_base = (pc >= S5L8900_IBOOT_BASE && pc < S5L8900_IBOOT_BASE + S5L8900_IBOOT_SIZE);
            int in_iboot_rt = (pc >= S5L8900_IBOOT_RUNTIME && pc < S5L8900_IBOOT_RUNTIME + S5L8900_IBOOT_SIZE);
            int in_sram_safe = (pc >= S5L8900_RAM_BASE && pc < S5L8900_RAM_BASE + 0x10000);
            if (in_iboot_base || in_iboot_rt || in_sram_safe) {
                /* Valid iBoot execution - nothing to do */
            } else {
                /* Start tracing once PC enters iBEC or iBoot region */
                if (!post_redirect_started &&
                    ((pc >= 0x18000000 && pc < 0x18200000) ||
                     (pc >= 0x23000000 && pc < 0x23100000))) {
                    post_redirect_started = 1;
                }
                if (post_redirect_started && !already_handled) {
                    /* Guard: if PC is beyond valid iBEC/iBoot range, redirect to safe loop */
                    if ((pc > 0x18200000 && pc < 0x22000000) ||
                        (pc >= 0x08000000 && pc < 0x09000000) ||
                        (pc >= 0x20000000 && pc < 0x21000000)) {
                        already_handled = 1;
                        fprintf(stderr, ">>> PC ESCAPED valid range: 0x%08x, redirecting to safe loop\n", pc);
                        uint32_t redirect_target = S5L8900_RAM_BASE + 0xF900;
                        run_on_cpu(CPU(cpu), s5l8900_cpu_redirect_cb, RUN_ON_CPU_HOST_ULONG(redirect_target));
                    } else if (pc != last_traced_pc) {
                        /* Normal: PC changed */
                        last_traced_pc = pc;
                        post_redirect_cnt++;
                        same_pc_count = 1;
                        if (post_redirect_cnt <= 5 || post_redirect_cnt % 10 == 0) {
                            fprintf(stderr, "POST-REDIRECT #%d: pc=0x%08x thumb=%d\n",
                                    post_redirect_cnt, pc, cpu->env.thumb);
                        }
                    } else {
                        /* Stuck: same PC for multiple callbacks */
                        same_pc_count++;
                        if (same_pc_count >= 3) {
                             already_handled = 1;
                             fprintf(stderr, ">>> STUCK at 0x%08x, redirecting to safe loop\n", pc);
                             uint32_t redirect_target = S5L8900_RAM_BASE + 0xF900;
                             run_on_cpu(CPU(cpu), s5l8900_cpu_redirect_cb, RUN_ON_CPU_HOST_ULONG(redirect_target));
                        }
                    }
                }
            }
        }

    /* Detect "config_board\0" in USB OTG RAM and trigger iBEC redirect.
     * This replaces the write-handler detection since USB OTG is now RAM. */
    {
        static int config_board_triggered = 0;
          if (!config_board_triggered) {
            uint8_t buf[16];
            cpu_physical_memory_read(S5L8900_USBOTG_BASE + 0x20, buf, sizeof(buf));
            if (memcmp(buf, "config_board", 12) == 0 && buf[12] == 0) {
                config_board_triggered = 1;
                fprintf(stderr, ">>> config_board detected! Triggering iBEC redirect\n");
                s5l8900_config_board_trigger(cpu);
            }
        }
    }

    /* Monitor scratch buffer for guest UART writes.
     * Guest CPU writes characters to 0x2200F800 onwards; we scan and forward to serial. */
    {
        static uint8_t seen[64]; /* Track which bytes we've already forwarded */
        static int initialized = 0;
        uint8_t buf[64];
        cpu_physical_memory_read(S5L8900_RAM_BASE + 0xF800, buf, sizeof(buf));
        if (!initialized) {
            memcpy(seen, buf, sizeof(buf));
            initialized = 1;
        }
        for (int i = 0; i < sizeof(buf); i++) {
            if (buf[i] >= 0x0A || buf[i] >= 0x20 && buf[i] <= 0x7E) {
                if (buf[i] != seen[i]) {
                    seen[i] = buf[i];
                    if (s5l8900_serial_chr) {
                        qemu_chr_write(s5l8900_serial_chr, &buf[i], 1, false);
                    }
                }
            }
        }
    }

  /* Re-arm timer for next 100ms. Stop after iBoot launches to avoid
    * catching valid iBoot execution in redirect catches. */
        if (s5l8900_periodic_timer && !s5l8900_iboot_launched) {
            timer_mod(s5l8900_periodic_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + 100ULL * 1000 * 1000);
        }
}

/* ---- UART stub (0xE0002000) ------------------------------------------------
 * Minimal UART that outputs to QEMU's serial character device.
 * Only the data register (offset 0) and line status (offset 5) are
 * implemented.  Writes to offset 0 are sent to the serial port.
 * Reads of offset 5 always return 0x60 (TH=1, TEMT=1, FE=0, PE=0, BE=0,
 * OE=0, DR=0, BI=0) meaning transmitter ready, no errors. */
static uint64_t s5l8900_uart_read(void *opaque, hwaddr offset, unsigned size)
{
    switch (offset) {
        case 5:  /* Line status: TH=1, TEMT=1 (tx ready), no errors */
            return 0x60;
        default:
            return 0;
    }
}

static void s5l8900_uart_write(void *opaque, hwaddr offset,
                                  uint64_t value, unsigned size)
{
    Chardev *chr = opaque;
    ARMCPU *cpu = ARM_CPU(qemu_get_cpu(0));
    fprintf(stderr, ">>> UART WRITE CB: offset=0x%"HWADDR_PRIx" val=0x%02x size=%u pc=0x%08x chr=%p\n",
            offset, (unsigned int)(value & 0xFF), size, (unsigned int)cpu->env.regs[15], chr);
    if (offset == 0 && chr) {
        uint8_t ch = (uint8_t)(value & 0xFF);
        qemu_chr_write(chr, &ch, 1, false);
    }
}

/* RAM-backed UART proxy: guest writes to SRAM region, callback forwards to serial.
 * This works without MMU since it's normal RAM. */
static void s5l8900_uartproxy_write(void *opaque, hwaddr offset,
                                      uint64_t value, unsigned size)
{
    Chardev *chr = opaque;
    ARMCPU *cpu = ARM_CPU(qemu_get_cpu(0));
    fprintf(stderr, ">>> UARTPROXY write: offset=0x%"HWADDR_PRIx" val=0x%02x size=%u pc=0x%08x\n",
            offset, (unsigned int)(value & 0xFF), size, (unsigned int)cpu->env.regs[15]);
    if (chr) {
        uint8_t ch = (uint8_t)(value & 0xFF);
        if (ch >= 0x20 && ch <= 0x7E) {
            qemu_chr_write(chr, &ch, 1, false);
        }
    }
}

static uint64_t s5l8900_uartproxy_read(void *opaque, hwaddr offset, unsigned size)
{
    return 0;
}

static const MemoryRegionOps s5l8900_uartproxy_ops = {
    .read  = s5l8900_uartproxy_read,
    .write = s5l8900_uartproxy_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static const MemoryRegionOps s5l8900_uart_ops = {
    .read  = s5l8900_uart_read,
    .write = s5l8900_uart_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

/* ---- Catch-all peripheral stub ------------------------------------------------
 * Returns 0 for all reads, ignores all writes. Prevents data aborts for
 * peripherals we haven't implemented yet. */
static uint64_t s5l8900_periph_read(void *opaque, hwaddr offset, unsigned size)
{
    static int cnt = 0;
    static uint32_t seen_regions[256];
    static int dumped_state = 0;

    if (offset == 0x6200080) {
        return 0x1;
    }
    if (offset == 0x6200084) {
        return 0x1;
    }
    /* iBSS polling loop at 0x22003720 reads 0x4c00010 and waits for value > 11.
     * Return 0x10 so the CMP r3, #11 + BLE exits the loop. */
    if (offset == 0x4c00010) {
        return 0x10;
    }

   ARMCPU *cpu = ARM_CPU(qemu_get_cpu(0));
    uint32_t pc = (unsigned int)cpu->env.regs[15];

   /* Track Thumb->ARM mode transitions and recover from ARM-mode crashes.
      * iBEC code is Thumb-32, but ARM-mode functions exist at 0x3e0+ and
      * 0x3c60+. When a BLX to even address enters ARM mode, the function may
      * not return properly or may jump to bad memory. Detect this and redirect
      * back to Thumb mode at the last known valid PC. */
     static int prev_thumb_track = 1;
     if (cpu->env.thumb) {
         prev_thumb_track = 1;
        /* Track last valid Thumb-mode PC in iBEC code ranges */
          if ((pc >= 0x18000000 && pc < 0x18200000) ||
              (pc >= 0x0A000000 && pc < 0x0A200000)) {
             s5l8900_last_thumb_pc = pc;
         }
     } else if (prev_thumb_track) {
         /* First detection of ARM mode after Thumb.
          * If PC is in bad memory, redirect back to Thumb mode immediately. */
        qemu_log_mask(LOG_UNIMP,
             "s5l8900.periph: *** THUMB->ARM TRANSITION *** pc=0x%08x lr=0x%08x cpsr=0x%08x last_thumb_pc=0x%08x\n",
             pc, (unsigned int)cpu->env.regs[14],
             (unsigned int)cpu->env.uncached_cpsr, s5l8900_last_thumb_pc);

       int pc_is_bad = !(pc >= 0x09000000 && pc < 0x09200000) &&  // iBSS
                          !(pc >= 0x0A000000 && pc < 0x0A200000) &&  // iBEC
                          !(pc >= 0x18000000 && pc < 0x20000000) &&   // USB OTG
                          !(pc >= 0x20000000 && pc < 0x20100000) &&   // ROM
                          !(pc >= 0x22000000 && pc < 0x22100000) &&   // SRAM
                          !(pc >= 0x00000000 && pc < 0x00001000) &&   // vector RAM
                          !(pc >= 0x30000000 && pc < 0x30010000) &&   // ROM stub 1
                          !(pc >= 0x03D00000 && pc < 0x03D10000);      // ROM stub 2

        if (pc_is_bad) {
               /* Redirect back to Thumb mode to prevent infinite spiraling.
                * The ARM function didn't return properly. Use LR as the return
                * address (set by the BLX that caused the crash), since redirecting
                * to last_thumb_pc would re-execute the same crashing instruction.
                * Set r0=1 so the caller sees "success" and continues. */
               uint32_t return_pc = cpu->env.regs[14]; /* LR from the BLX */
               /* Make sure LR points to valid Thumb code; if not, use fallback */
               if (return_pc < 0x18000000 || return_pc >= 0x18200000) {
                   /* LR is outside iBEC range; use last_thumb_pc or fallback.
                    * Skip past constant pool: Thumb entry at 0x4960 has constant
                    * pool data at 0x4964-0x497F, with actual code at 0x4980.
                    * If last_thumb_pc is at the entry, redirect past the pool. */
                   if (s5l8900_last_thumb_pc) {
                       return_pc = s5l8900_last_thumb_pc + 4;
                       /* Avoid landing in constant pool data (0x4964-0x497F) */
                       if (return_pc >= 0x18004964 && return_pc < 0x18004980) {
                           return_pc = 0x18004980;
                       }
                       if (return_pc % 2) return_pc++;
                   } else {
                       return_pc = 0x18004980; /* Skip past constant pool */
                   }
               }
           /* Avoid redirecting to ARM-mode callback functions (0x113b0-0x113ff).
                * These are ARM-mode functions that decode as garbage in Thumb mode,
                * leading to stack-based returns to bad memory. If LR points to this
                * range, skip to a safe fallback instead. */
               if (return_pc >= 0x180113b0 && return_pc < 0x18011400) {
                   return_pc = 0x18004980;
                   qemu_log_mask(LOG_UNIMP,
                       "s5l8900.periph: ARM crash -> LR in callback range, skipping to 0x%08x\n",
                       return_pc);
               }
           /* Infinite loop detection: if we keep crashing at the same bad PC,
                * we're stuck. Patch the iBEC Thumb entry to skip all init calls
                * that transition to ARM mode and crash. */
               static uint32_t last_crash_pc = 0;
               static int crash_loop_count = 0;
               if (pc == last_crash_pc) {
                   crash_loop_count++;
                   if (crash_loop_count >= 3 && !s5l8900_ibec_init_patched) {
                        s5l8900_ibec_init_patched = 1;
                        /* Patch BLs at corrected offsets 0x4968-0x4978 */
                        uint8_t mov_r0_1[] = { 0x01, 0x20, 0x00, 0xbf };
                        uint32_t bl_offsets[] = { 0x4968, 0x496c, 0x4970, 0x4974, 0x4978 };
                        for (int j = 0; j < 5; j++) {
                            cpu_physical_memory_write(0x18000000 + bl_offsets[j], mov_r0_1, 4);
                            cpu_physical_memory_write(0x0A000000 + bl_offsets[j], mov_r0_1, 4);
                        }
                        qemu_log_mask(LOG_UNIMP,
                            "s5l8900.periph: infinite crash loop! Patching iBEC entry BLs 0x4968-0x4978\n");
                        return_pc = 0x1800497c;
                       crash_loop_count = 0;
                   }
               } else {
                   last_crash_pc = pc;
                   crash_loop_count = 0;
               }
          qemu_log_mask(LOG_UNIMP,
                    "s5l8900.periph: ARM-mode crash in bad memory! Returning to 0x%08x\n",
                    return_pc);
               int is_thumb = (return_pc >= 0x18000000);
               /* Ensure PC LSB (T-bit) matches Thumb mode.
                * QEMU ARM TCG uses env.thumb, but the PC LSB should also
                * be consistent to avoid mis-decoding at TB boundaries. */
               if (is_thumb && !(return_pc & 1)) {
                   return_pc |= 1;
               }
           cpu->env.regs[15] = return_pc;
             cpu->env.thumb = is_thumb;
            cpu->env.regs[0] = 1; /* Return success */
           queue_tb_flush(CPU(cpu));
           CPU(cpu)->exit_request = 0; /* Let CPU continue */
           cpu_reset_interrupt(qemu_get_cpu(0), CPU_INTERRUPT_EXITTB);
         }

        prev_thumb_track = 0;
    }

    /* Log every access from iBEC region to track execution flow */
    static uint32_t last_ibec_pc = 0;
    if (pc >= 0x18010000 && pc < 0x18011000 && pc != last_ibec_pc) {
        last_ibec_pc = pc;
        qemu_log_mask(LOG_UNIMP,
            "s5l8900.periph: iBEC PC=0x%08x thumb=%d lr=0x%08x\n",
            pc, cpu->env.thumb, (unsigned int)cpu->env.regs[14]);
    }

    /* Log execution in the 0x18002fe0-0x18003200 range to trace post-init flow */
    static uint32_t last_init_pc = 0;
    if (pc >= 0x18002fe0 && pc < 0x18003200 && pc != last_init_pc) {
        last_init_pc = pc;
        qemu_log_mask(LOG_UNIMP,
            "s5l8900.periph: INIT PC=0x%08x thumb=%d lr=0x%08x sp=0x%08x r0=0x%08x r1=0x%08x r2=0x%08x r3=0x%08x\n",
            pc, cpu->env.thumb, (unsigned int)cpu->env.regs[14],
            (unsigned int)cpu->env.regs[13],
            (unsigned int)cpu->env.regs[0], (unsigned int)cpu->env.regs[1],
            (unsigned int)cpu->env.regs[2], (unsigned int)cpu->env.regs[3]);
    }

    /* Detect infinite loops: same PC visited too many times */
    static uint32_t loop_detect_pc = 0;
    static int loop_detect_cnt = 0;
    if (pc == loop_detect_pc) {
        loop_detect_cnt++;
        if (loop_detect_cnt == 100) {
            qemu_log_mask(LOG_UNIMP,
                "s5l8900.periph: *** INFINITE LOOP DETECTED *** pc=0x%08x thumb=%d lr=0x%08x sp=0x%08x r0=0x%08x r1=0x%08x r2=0x%08x r3=0x%08x r4=0x%08x r5=0x%08x r6=0x%08x r7=0x%08x r8=0x%08x r9=0x%08x r10=0x%08x r11=0x%08x r12=0x%08x\n",
                pc, cpu->env.thumb,
                (unsigned int)cpu->env.regs[14], (unsigned int)cpu->env.regs[13],
                (unsigned int)cpu->env.regs[0], (unsigned int)cpu->env.regs[1],
                (unsigned int)cpu->env.regs[2], (unsigned int)cpu->env.regs[3],
                (unsigned int)cpu->env.regs[4], (unsigned int)cpu->env.regs[5],
                (unsigned int)cpu->env.regs[6], (unsigned int)cpu->env.regs[7],
                (unsigned int)cpu->env.regs[8], (unsigned int)cpu->env.regs[9],
                (unsigned int)cpu->env.regs[10], (unsigned int)cpu->env.regs[11],
                (unsigned int)cpu->env.regs[12]);
            /* Dump bytes around PC */
            hwaddr pc_base = pc & ~0xF;
            uint8_t loop_bytes[32];
            cpu_physical_memory_read(pc_base, loop_bytes, 32);
            for (int i = 0; i < 32; i += 4) {
                qemu_log_mask(LOG_UNIMP,
                    "s5l8900.periph: *** LOOP MEM 0x%08x: %02x %02x %02x %02x\n",
                    (unsigned int)(pc_base + i),
                    loop_bytes[i], loop_bytes[i+1], loop_bytes[i+2], loop_bytes[i+3]);
            }
        }
    } else {
        loop_detect_pc = pc;
        loop_detect_cnt = 1;
    }

    /* Trap: detect first PC in unmapped region */
    static uint32_t first_bad_pc = 0;
    static int dumped_bad = 0;
    int in_valid = (pc >= 0x09000000 && pc < 0x09200000) ||  // iBSS
                    (pc >= 0x0A000000 && pc < 0x0A200000) ||  // iBEC
                    (pc >= 0x18000000 && pc < 0x20000000) ||   // USB OTG region
                    (pc >= 0x20000000 && pc < 0x20100000) ||   // ROM (1MB)
                    (pc >= 0x22000000 && pc < 0x22100000) ||   // SRAM
                    (pc >= 0x00000000 && pc < 0x00001000) ||   // vector RAM
                    (pc >= 0x24000000 && pc < 0x24010000);     // unknown region
    if (!in_valid && !first_bad_pc) {
        first_bad_pc = pc;
        qemu_log_mask(LOG_UNIMP,
            "s5l8900.periph: *** FIRST BAD PC *** pc=0x%08x lr=0x%08x sp=0x%08x cpsr=0x%08x thumb=%d\n",
            pc, (unsigned int)cpu->env.regs[14],
            (unsigned int)cpu->env.regs[13],
            (unsigned int)cpu->env.uncached_cpsr, cpu->env.thumb);
        qemu_log_mask(LOG_UNIMP,
            "s5l8900.periph: *** FIRST BAD PC *** r0=0x%08x r1=0x%08x r2=0x%08x r3=0x%08x r4=0x%08x r5=0x%08x r6=0x%08x r7=0x%08x\n",
            (unsigned int)cpu->env.regs[0], (unsigned int)cpu->env.regs[1],
            (unsigned int)cpu->env.regs[2], (unsigned int)cpu->env.regs[3],
            (unsigned int)cpu->env.regs[4], (unsigned int)cpu->env.regs[5],
            (unsigned int)cpu->env.regs[6], (unsigned int)cpu->env.regs[7]);
        qemu_log_mask(LOG_UNIMP,
            "s5l8900.periph: *** FIRST BAD PC *** r8=0x%08x r9=0x%08x r10=0x%08x r11=0x%08x r12=0x%08x\n",
            (unsigned int)cpu->env.regs[8], (unsigned int)cpu->env.regs[9],
            (unsigned int)cpu->env.regs[10], (unsigned int)cpu->env.regs[11],
            (unsigned int)cpu->env.regs[12]);
    }

    /* Dump instruction bytes around LR when PC is bad */
    if (!in_valid && !dumped_bad) {
        dumped_bad = 1;
        hwaddr lr_base = (cpu->env.regs[14]) & ~0x1F;
        uint8_t bytes[64];
        cpu_physical_memory_read(lr_base, bytes, 64);
        for (int i = 0; i < 64; i += 4) {
            qemu_log_mask(LOG_UNIMP,
                "s5l8900.periph: *** LR MEM 0x%08x: %02x %02x %02x %02x\n",
                (unsigned int)(lr_base + i),
                bytes[i], bytes[i+1], bytes[i+2], bytes[i+3]);
        }
    }

    if (pc >= 0x20000000 && pc < 0x30000000) {
        uint32_t region = (pc >> 20) & 0xFF;
        if (!seen_regions[region]) {
            seen_regions[region] = pc;
            qemu_log_mask(LOG_UNIMP,
                "s5l8900.periph: FIRST PC in 1MB region 0x%02x (pc=0x%x, lr=0x%x)\n",
                region, pc, (unsigned int)cpu->env.regs[14]);
        }
    }

    /* When PC first enters the USB OTG region (0x18000000), re-apply patches.
     * The iBEC self-copy to 0x18000000 overwrites patches applied by the
     * USB OTG write handler during iBSS config_board trigger. */
    if (pc >= 0x18000000 && pc < 0x18200000) { /* 2MB USB OTG region */
        s5l8900_usbotg_apply_patches();
    }

    /* Trap: detect when PC leaves iBEC code range to bad memory */
    static uint32_t prev_pc_in_code = 0;
    static int bad_jump_logged = 0;
    if (!bad_jump_logged && cnt > 20 && !in_valid && first_bad_pc) {
        /* Log the transition on first occurrence only */
        bad_jump_logged = 1;
    }
    /* Track last valid PC before entering bad region */
    if (in_valid && pc >= 0x18000000 && pc < 0x18200000) {
        prev_pc_in_code = pc;
    }

    /* Dump full CPU state once at 1M accesses to snapshot where execution is */
    if (!dumped_state && cnt == 999999) {
        dumped_state = 1;
        qemu_log_mask(LOG_UNIMP,
            "s5l8900.periph: STATE DUMP [cnt=%d] pc=0x%x lr=0x%x sp=0x%x r0=0x%x r1=0x%x r2=0x%x r3=0x%x cpsr=0x%x thumb=%d\n",
            cnt, pc, (unsigned int)cpu->env.regs[14],
            (unsigned int)cpu->env.regs[13],
            (unsigned int)cpu->env.regs[0], (unsigned int)cpu->env.regs[1],
            (unsigned int)cpu->env.regs[2], (unsigned int)cpu->env.regs[3],
            (unsigned int)cpu->env.uncached_cpsr, cpu->env.thumb);
    }

    if (++cnt <= 20 || cnt % 10000 == 0) {
        qemu_log_mask(LOG_UNIMP, "s5l8900.periph: read 0x%"HWADDR_PRIx" (size=%u) pc=0x%x [cnt=%d]\n",
                      offset, size, pc, cnt);
    }
    return 0;
}

 static void s5l8900_periph_write(void *opaque, hwaddr offset,
                                    uint64_t value, unsigned size)
 {
     static int cnt = 0;
     if (++cnt <= 20 || cnt % 10000 == 0) {
         ARMCPU *cpu = ARM_CPU(qemu_get_cpu(0));
         qemu_log_mask(LOG_UNIMP, "s5l8900.periph: write 0x%"HWADDR_PRIx"=0x%"PRIx64" (size=%u) pc=0x%x [cnt=%d]\n",
                       offset, value, size, (unsigned int)cpu->env.regs[15], cnt);
     }
 }

static const MemoryRegionOps s5l8900_periph_catchall_ops = {
    .read  = s5l8900_periph_read,
    .write = s5l8900_periph_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/* ---- USB OTG RAM (0x18000000) --------------------------------
 * On real S5L8900 this is SRAM. iBSS uses it for USB FIFO/register writes
 * during initialization. Later iBEC self-copies here and executes from it.
 *
 * This is a regular RAM region so code can execute from it.
 * config_board detection is done by the periodic callback. */

typedef struct {
    uint8_t ram[S5L8900_USBOTG_SIZE];  /* Backing store for the region */
} S5L8900USBOTGState;

static uint64_t s5l8900_usbotg_read(void *opaque, hwaddr offset, unsigned size)
{
    S5L8900USBOTGState *s = opaque;
    if (s && offset + size <= S5L8900_USBOTG_SIZE) {
        return ldn_le_p(s->ram + offset, size);
    }
    return 0;
}

   /* Apply patches to the USB OTG region (0x18000000) after iBEC self-copy.
  * The self-copy may overwrite patches applied earlier, so we re-apply here.
  * Called once when the first read from the self-copied iBEC is detected. */
 static void s5l8900_usbotg_apply_patches(void)
 {
     static int patched = 0;
     if (patched) return;
     patched = 1;

     /* Fix ARM/Thumb mode: BX target at 0xf0 in self-copied iBEC.
      * Original value 0x18000000 (even -> ARM mode) causes crash because
      * the code at that address is Thumb-32. Patch to 0x18000001 (odd -> Thumb). */
     uint32_t ibec_bx_thumb = 0x18000001;
     cpu_physical_memory_write(S5L8900_USBOTG_BASE + 0xf0,
                               &ibec_bx_thumb, 4);
     qemu_log_mask(LOG_UNIMP,
         "s5l8900.usbotg: patched iBEC BX target 0xf0 -> 0x18000001 (Thumb mode)\n");

    /* Restore POP {PC} at 0x113b4 (callback function, needs stack-based return) */
    uint32_t arm_poppc = 0xE49DF004;
    cpu_physical_memory_write(S5L8900_USBOTG_BASE + 0x113b4, &arm_poppc, 4);

    /* Replace POP {PC} with MOV r0,#1 at 0x115a8 and 0x116e4 */
    uint32_t arm_mov1 = 0xE3A00001;
    cpu_physical_memory_write(S5L8900_USBOTG_BASE + 0x115a8, &arm_mov1, 4);
    cpu_physical_memory_write(S5L8900_USBOTG_BASE + 0x116e4, &arm_mov1, 4);

    /* Replace BL #-64 at 0x113bc with MOV r0,#1 (skip nested 0x11380 loop) */
    cpu_physical_memory_write(S5L8900_USBOTG_BASE + 0x113bc, &arm_mov1, 4);

    /* Don't NOP 0x113ac-0x113af here. This range overlaps with ARM-mode
     * code used by the division function (0x180112c0). The callback loop
     * at 0x11380 is already patched to MOV r0,#1; BX LR, so the loop
     * entry is skipped. NOPing 0x113ac corrupts ARM-mode STR/MOV
     * instructions, causing SVC exceptions when the division path calls
     * through 0x180113a8 -> 0x180113ac. */

    /* Patch ARM-mode BL at 0x103c4 that calls into garbage memory.
     * The function at 0x103c0 is entered via BLX from Thumb mode (ARM1176
     * BLX to even address enters ARM mode). It does 'rsbs r0,r0,#0' then
     * 'bl 0x18013c6c'. The target 0x18013c6c is beyond the valid iBEC
     * payload and contains zeroed/garbage USB OTG RAM data. Replace the
     * entire function entry with MOV r0,#1; BX LR for a clean return.
     * Same pattern at 0x103d8 (rsbs + bl to another bad target). */
    uint32_t arm_mov1_init = 0xE3A00001; /* MOV r0, #1 */
    uint32_t arm_bxlr_init = 0xE12FFF1E; /* BX LR */
    cpu_physical_memory_write(S5L8900_USBOTG_BASE + 0x103c0, &arm_mov1_init, 4);
    cpu_physical_memory_write(S5L8900_USBOTG_BASE + 0x103c4, &arm_bxlr_init, 4);
    cpu_physical_memory_write(S5L8900_USBOTG_BASE + 0x103d8, &arm_mov1_init, 4);
    cpu_physical_memory_write(S5L8900_USBOTG_BASE + 0x103dc, &arm_bxlr_init, 4);
    qemu_log_mask(LOG_UNIMP,
        "s5l8900.usbotg: patched ARM BL at 0x103c4, 0x103dc -> MOV r0,#1/BX LR\n");

   /* Patch iBEC halt function at 0x153c (B #-2 infinite loop).
      * This halt is called when iBEC detects a fatal error (e.g. missing
      * hardware, failed peripheral init). Replace with BX LR to return
      * to caller and allow execution to continue. */
    uint16_t thumb_bxlr = 0x4770; /* BX LR (Thumb-16) */
    cpu_physical_memory_write(S5L8900_USBOTG_BASE + 0x153c, &thumb_bxlr, 2);
    qemu_log_mask(LOG_UNIMP,
        "s5l8900.usbotg: patched iBEC halt at 0x153c (B #-2 -> BX LR)\n");

    /* Patch ARM-mode function table placeholder at 0x40c.
     * Replace infinite loop with MOV r0,#1; BX LR. */
    uint32_t arm_func_ret[] = { 0xE3A00001, 0xE12FFF1E };
    cpu_physical_memory_write(S5L8900_USBOTG_BASE + 0x40c, arm_func_ret, 8);
    qemu_log_mask(LOG_UNIMP,
        "s5l8900.usbotg: patched ARM func at 0x40c (infinite loop -> MOV r0,#1/BX LR)\n");

    /* Re-apply ARM stub patches and allocation driver patch at self-copied iBEC.
     * These were applied at 0x0A000000 but overwritten by the self-copy. */
    {
        uint32_t arm_mov0 = 0xE3A00000; /* MOV r0, #0 */
        uint32_t arm_bxlr_stub = 0xE12FFF1E; /* BX LR */
        /* ARM stubs: replace with MOV r0,#0; BX LR (failure return) */
        cpu_physical_memory_write(S5L8900_USBOTG_BASE + 0x3c60, &arm_mov0, 4);
        cpu_physical_memory_write(S5L8900_USBOTG_BASE + 0x3c64, &arm_bxlr_stub, 4);
        cpu_physical_memory_write(S5L8900_USBOTG_BASE + 0x3c68, &arm_mov0, 4);
        cpu_physical_memory_write(S5L8900_USBOTG_BASE + 0x3c6c, &arm_bxlr_stub, 4);
        cpu_physical_memory_write(S5L8900_USBOTG_BASE + 0x3c74, &arm_mov0, 4);
        cpu_physical_memory_write(S5L8900_USBOTG_BASE + 0x3c78, &arm_bxlr_stub, 4);
        /* Thumb stub at 0x3e48: MOV r0,#0; BX LR */
        uint8_t thumb_fail[] = { 0x00, 0x20, 0x70, 0x47 };
        cpu_physical_memory_write(S5L8900_USBOTG_BASE + 0x3e48, thumb_fail, 4);
        /* Allocation driver at 0xfb26: MOV r0,#0; BX LR */
        cpu_physical_memory_write(S5L8900_USBOTG_BASE + 0xfb26, thumb_fail, 4);
        /* Bad BLX at 0x312e: replace with MOV r0,#1; BX LR to skip crash */
        uint8_t thumb_skip_blx[] = { 0x01, 0x20, 0x70, 0x47 };
        cpu_physical_memory_write(S5L8900_USBOTG_BASE + 0x312e, thumb_skip_blx, 4);

        /* Patch callback loop at 0x11380: replace entry with infinite loop.
          * This loop iterates over a table of function pointers and calls each
          * via BLX r3 (Thumb mode). The function pointers point to garbage.
          * The caller at 0x18001211 keeps re-calling this function in a loop,
          * so BX LR causes an infinite call loop. Infinite loop here is stable. */
         uint8_t thumb_infinite_loop[] = { 0x00, 0xEA }; /* B #-2 (Thumb-16 infinite loop) */
         cpu_physical_memory_write(S5L8900_USBOTG_BASE + 0x11380, thumb_infinite_loop, 2);
         qemu_log_mask(LOG_UNIMP,
             "s5l8900.usbotg: patched callback loop at 0x11380 (entry -> infinite loop)\n");

        /* Also patch the callback function at 0x113b0 directly.
         * If anything calls this ARM-mode function directly, return safely.
         * ARM: MOV r0,#1 (0xE3A00001); BX LR (0xE12FFF1E) */
        uint32_t arm_mov1_cb = 0xE3A00001;
        uint32_t arm_bxlr_cb = 0xE12FFF1E;
        cpu_physical_memory_write(S5L8900_USBOTG_BASE + 0x113b0, &arm_mov1_cb, 4);
        cpu_physical_memory_write(S5L8900_USBOTG_BASE + 0x113b4, &arm_bxlr_cb, 4);
        qemu_log_mask(LOG_UNIMP,
            "s5l8900.usbotg: patched callback at 0x113b0 (ARM: MOV r0,#1/BX LR)\n");

        /* Patch the write loop at 0x113c0: replace POP {r0-r3,PC} with
         * MOV r0,#1; BX LR to prevent stack-based PC return with garbage. */
        cpu_physical_memory_write(S5L8900_USBOTG_BASE + 0x113c0, &arm_mov1_cb, 4);
        cpu_physical_memory_write(S5L8900_USBOTG_BASE + 0x113c4, &arm_bxlr_cb, 4);
           qemu_log_mask(LOG_UNIMP,
                "s5l8900.usbotg: patched write loop at 0x113c0 (ARM: MOV r0,#1/BX LR)\n");

           /* Patch bit-manipulation function at 0x1660: ARM MOV r0,#1; BX LR.
              * This function reads/modifies/writes CLOCK1 register at offset 0x4C
              * (address 0x3c50004c). GDB stepping hangs on the STR to this MMIO
              * address. Bypass entirely to avoid the write. */
             {
                 uint32_t arm_mov1_bit = 0xE3A00001; /* MOV r0, #1 */
                 uint32_t arm_bxlr_bit = 0xE12FFF1E; /* BX LR */
                 cpu_physical_memory_write(S5L8900_USBOTG_BASE + 0x1660, &arm_mov1_bit, 4);
                 cpu_physical_memory_write(S5L8900_USBOTG_BASE + 0x1664, &arm_bxlr_bit, 4);
                 qemu_log_mask(LOG_UNIMP,
                     "s5l8900.usbotg: patched bit-manip function at 0x1660 (ARM: MOV r0,#1/BX LR)\n");
             }

           /* Patch iBEC Thumb entry BLs (0x4968-0x4978) to skip init calls
              * that transition to ARM mode and crash on unmapped ROM pointers.
              * These BLs call 0xfb26, 0x3b6c, 0x48a0, 0x6e0, 0x4904.
              * Corrected offsets: actual BL instructions start at 0x4968, not 0x4966.
              * The 0x4966 offset was misaligned and corrupted constant pool data. */
             {
                 uint8_t skip_bl[] = { 0x01, 0x20, 0x00, 0xbf }; /* MOV r0,#1; NOP */
                 uint16_t offsets[] = { 0x4968, 0x496c, 0x4970, 0x4974, 0x4978 };
                 for (int k = 0; k < 5; k++) {
                     cpu_physical_memory_write(S5L8900_USBOTG_BASE + offsets[k],
                                                skip_bl, 4);
                 }
            qemu_log_mask(LOG_UNIMP,
                      "s5l8900.usbotg: patched iBEC entry BLs 0x4968-0x4978 (skip init calls)\n");
              }

              /* Jump to iBoot at 0x4980 (past entry BLs).
                 * Replaces the safe infinite loop. When iBEC entry BLs return,
                 * execution falls through here and jumps to iBoot. */
                {
                    uint32_t iboot_entry = S5L8900_IBOOT_BASE + 0x400; /* ARM vectors */
                    uint8_t tramp_code[] = {
                        0xF8, 0xDF, 0x04, 0x00,  /* LDR r0, [pc, #4] */
                        0x00, 0x47,              /* BX r0 */
                        0x00, 0x00               /* padding */
                    };
                    cpu_physical_memory_write(S5L8900_USBOTG_BASE + 0x4980,
                                              tramp_code, sizeof(tramp_code));
                    cpu_physical_memory_write(S5L8900_USBOTG_BASE + 0x4988,
                                              &iboot_entry, 4);
                    qemu_log_mask(LOG_UNIMP,
                        "s5l8900.usbotg: patched 0x4980 -> jump to iBoot at 0x%08x\n",
                        iboot_entry);
                }

            /* Patch stuck function at 0x5080 (refcount/data init loop).
             * This function iterates over data structures and gets stuck
             * in a tight loop (0x50ae-0x50e4) due to hardware-dependent
             * memory accesses. Replace entry with MOV r0,#0; BX LR. */
            {
                uint8_t func_ret[] = { 0x00, 0x20, 0x70, 0x47 }; /* MOV r0,#0; BX LR */
                cpu_physical_memory_write(S5L8900_USBOTG_BASE + 0x5080, func_ret, 4);
                qemu_log_mask(LOG_UNIMP,
                    "s5l8900.usbotg: patched stuck func at 0x5080 -> MOV r0,#0; BX LR\n");
            }
        }

        qemu_log_mask(LOG_UNIMP,
            "s5l8900.usbotg: re-applied all iBEC patches at 0x18000000\n");
}

/* Trigger iBEC self-copy, patching, and redirect.
 * Called from periodic callback when config_board pattern is detected in RAM,
 * or from upper-RAM escalation when CPU is stuck executing garbage. */
static void s5l8900_config_board_trigger(ARMCPU *cpu)
{
    CPUState *cs = CPU(cpu);
    fprintf(stderr, ">>> s5l8900_config_board_trigger: starting\n");
    uint32_t cur_pc = (uint32_t)cpu->env.regs[15];

    /* Test UART: write "Hello from iPod!\n" to serial port */
    {
        const char *msg = "Hello from iPod!\n";
        for (int i = 0; msg[i]; i++) {
            uint32_t ch = (uint8_t)msg[i];
            cpu_physical_memory_write(S5L8900_UART_BASE, &ch, 1);
        }
    }
    fprintf(stderr, ">>> Current PC=0x%08x, thumb=%d, cpsr=0x%08x\n",
            cur_pc, cpu->env.thumb, (unsigned int)cpu->env.uncached_cpsr);

    /* Manual self-copy: iBEC from load address to USB OTG execution address */
    {
        uint8_t *ibec_src = g_malloc(S5L8900_IBEC_SIZE);
        cpu_physical_memory_read(S5L8900_IBEC_BASE, ibec_src, S5L8900_IBEC_SIZE);
        cpu_physical_memory_write(S5L8900_USBOTG_BASE, ibec_src, S5L8900_IBEC_SIZE);
        g_free(ibec_src);
        fprintf(stderr, ">>> config_board: copied iBEC to 0x18000000\n");
    }

    /* Apply all iBEC patches at USB OTG address */
     s5l8900_usbotg_apply_patches();

/* Write ARM infinite loop at 0xF900.
         * Periodic callback detects PC in this region and prints to serial. */
        {
            uint32_t loop = 0xEAFFFFFE; /* B #-4 */
            for (int i = 0; i < 64; i++)
                cpu_physical_memory_write(S5L8900_RAM_BASE + 0xF900 + i * 4, &loop, 4);
            fprintf(stderr, ">>> SRAM 0xF900: ARM infinite loop (serial via periodic)\n");
        }

     /* Patch BSS data at 0x12900: write pointer to safe stub.
      * The function at 0x4960 loads [0x18012900] as a function pointer. */
     {
         uint32_t safe_ptr = S5L8900_USBOTG_BASE + 0x4960 | 1; /* Thumb bit */
         cpu_physical_memory_write(S5L8900_USBOTG_BASE + 0x12900, &safe_ptr, 4);
     }

     /* Jump to iBoot at 0x4980 (past entry BLs) */
     {
         uint32_t iboot_entry = S5L8900_IBOOT_BASE + 0x400; /* ARM vectors */
         uint8_t tramp_code[] = {
             0xF8, 0xDF, 0x04, 0x00,  /* LDR r0, [pc, #4] */
             0x00, 0x47,              /* BX r0 */
             0x00, 0x00               /* padding */
         };
         cpu_physical_memory_write(S5L8900_USBOTG_BASE + 0x4980,
                                   tramp_code, sizeof(tramp_code));
         cpu_physical_memory_write(S5L8900_USBOTG_BASE + 0x4988,
                                   &iboot_entry, 4);
     }

    /* Break stuck loop at 0x50e4 (BNE->NOP) */
    {
        uint16_t thumb_nop = 0xBF00;
        cpu_physical_memory_write(S5L8900_USBOTG_BASE + 0x50e4, &thumb_nop, 2);
    }

    /* Re-patch exception vectors (ARM infinite loops, no mode switching) */
    s5l8900_evec_redirect_all();

        /* Write Thumb trampoline in SRAM.
         * Jumps directly to iBoot main function at 0x4C20 (Thumb mode).
         * Skips the ARM reset handler entirely (cache ops, copy loops,
         * CP15 access all crash in QEMU).
         * Thumb LDR r0,[pc,#imm] reads from (instr_addr+8) + imm*4.
         * At 0xFF00 with imm=0: reads from 0xFF08. */
         {
             uint16_t thumb_tramp[] = {
                 0x4800,       /* LDR r0, [pc, #0]  -> reads 0xFF08 */
                 0x4700,       /* BX r0              -> Thumb mode switch */
                 0x0000,       /* padding (0xFF04) */
                 0x0000,       /* padding (0xFF06) */
                 0x4C21,       /* low 16 of 0x23004C21 (at 0xFF08) */
                 0x2300,       /* high 16 of 0x23004C21 (at 0xFF0A) */
             };
             cpu_physical_memory_write(S5L8900_RAM_BASE + 0xFF00, thumb_tramp, sizeof(thumb_tramp));
             fprintf(stderr, ">>> config_board: trampoline at 0x%08x -> iBoot main Thumb 0x%08x\n",
                     S5L8900_RAM_BASE + 0xFF00, S5L8900_IBOOT_BASE + 0x4C20);
         }

       s5l8900_last_thumb_pc = S5L8900_RAM_BASE + 0xFF00; /* ARM trampoline */

   /* Set up stack and registers for iBEC entry.
        * The iBEC expects a valid stack pointer and the function at 0x4960
        * uses PUSH/POP. Set sp to end of SRAM and lr to safe return.
        * Set CPU to SVC mode with interrupts masked (required for CPSID i). */
       {
           uint32_t stack_top = S5L8900_RAM_BASE + 0x20000; /* End of SRAM */
           uint32_t safe_ret = S5L8900_USBOTG_BASE + 0x4981; /* Past entry, Thumb */
           cpu->env.regs[13] = stack_top; /* SP */
           cpu->env.regs[14] = S5L8900_RAM_BASE + 0xF910 | 1; /* LR -> safe Thumb infinite loop */
           cpu->env.regs[0] = 0; /* r0 = 0 (no device tree) */
           cpu->env.regs[1] = 0; /* r1 = 0 */
           cpu->env.regs[2] = 0; /* r2 = 0 */
           cpu->env.uncached_cpsr = (cpu->env.uncached_cpsr & ~0x1F) | 0xD3; /* SVC + I+F */
           cpu->env.thumb = true; /* iBEC is Thumb mode */
           fprintf(stderr, ">>> config_board: sp=0x%08x, lr=0x%08x, cpsr=0x%08x\n",
                   stack_top, safe_ret, (unsigned int)cpu->env.uncached_cpsr);
       }

    /* Disable MMU immediately so guest code doesn't data-abort on MMIO.
     * This must happen before the CPU runs any patched iBEC code that
     * might access unmapped peripherals. */
    {
        cpu->env.cp15.sctlr_s &= ~(1 | (1 << 2)); /* Clear M (MMU) + C (cache) */
        cpu->env.cp15.ttbr0_s = 0;
        cpu->env.cp15.ttbr1_s = 0;
        fprintf(stderr, ">>> config_board: disabled MMU (sctlr=0x%08x)\n",
                (unsigned int)cpu->env.cp15.sctlr_s);
    }

    /* Redirect CPU to iBoot by replacing the ARM loop at 0xF900.
            * Thumb LDR r0,[pc,#imm] reads from (instr_addr+8) + imm*4.
            * At 0xF900 with imm=0: reads from 0xF908. */
           {
               uint8_t redirect_bytes[] = {
                   0x00, 0x48,   /* 0xF900: LDR r0, [pc, #0]  -> reads 0xF908 */
                   0x00, 0x47,   /* 0xF902: BX r0              -> Thumb mode switch */
                   0x00, 0x00,   /* 0xF904: padding */
                   0x00, 0x00,   /* 0xF906: padding */
                   0x01, 0xFF,   /* 0xF908: low 16 of 0x2200FF01 */
                   0x00, 0x22,   /* 0xF90A: high 16 of 0x2200FF01 */
               };
               cpu_physical_memory_write(S5L8900_RAM_BASE + 0xF900, redirect_bytes, sizeof(redirect_bytes));

               /* Write safe Thumb infinite loop at 0xF910 (for iBoot return address).
                * If iBoot main function returns, it will loop here instead of crashing. */
                uint16_t safe_loop[] = { 0xE7FE, 0xE7FE }; /* B #-2; B #-2 */
                cpu_physical_memory_write(S5L8900_RAM_BASE + 0xF910, safe_loop, sizeof(safe_loop));

                /* ARM safe loop at 0xF920 (for exception vector fallback) */
                uint32_t arm_loop = 0xEAFFFFFE; /* B #-4 */
                cpu_physical_memory_write(S5L8900_RAM_BASE + 0xF920, &arm_loop, 4);

                fprintf(stderr, ">>> config_board_trigger: replaced ARM loop at 0xF900 with Thumb redirect\n");
               /* Directly jump to Thumb trampoline using run_on_cpu to break the tight loop */
               run_on_cpu(cpu, s5l8900_jump_to_tramp, RUN_ON_CPU_NULL);
           }

     /* Patch iBEC entry point (0x4960) to safe return.
      * CPU is now redirected to iBoot trampoline at 0x4980.
      * If any code calls 0x4960, just return safely. */
     {
         uint8_t thumb_ret[] = {
             0x01, 0x20,  /* MOV r0, #1 */
             0x70, 0x47,  /* BX LR */
         };
         cpu_physical_memory_write(S5L8900_USBOTG_BASE + 0x4960,
                                   thumb_ret, sizeof(thumb_ret));
         fprintf(stderr, ">>> config_board: patched iBEC entry 0x4960 -> MOV r0,#1; BX LR\n");
     }
}

static void s5l8900_usbotg_write(void *opaque, hwaddr offset,
                                    uint64_t value, unsigned size)
{
    S5L8900USBOTGState *s = opaque;
    if (s && offset + size <= S5L8900_USBOTG_SIZE) {
        stn_le_p(s->ram + offset, size, value);
    }
}

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
  uint32_t irq_pending;
} S5L8900USBCTRLState;

typedef struct {
  uint32_t ctrl;
  uint32_t destaddr;
  uint32_t copysize;
  uint8_t fifo[8];
  uint32_t unknown[5];
  S5L8900VICState *vic0;
  S5L8900USBCTRLState *usbctrl;
  QEMUTimer *deferred_irq;
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

static void s5l8900_deferred_irq_cb(void *opaque)
{
    S5L8900DMAState *s = opaque;
    uint32_t usb_struct_ptr, usb_struct;
    cpu_physical_memory_read(0x200031c0, &usb_struct_ptr, 4);
    cpu_physical_memory_read(usb_struct_ptr, &usb_struct, 4);
    uint8_t usb_state = 4;
    cpu_physical_memory_write(usb_struct + 0x50, &usb_state, 1);
    uint32_t ev8 = 8;
    cpu_physical_memory_write(usb_struct + 0x100, &ev8, 4);

    /* Simulate completed DFU DNLOAD: set download-complete flag and sub-struct flag
     * so the DFU main loop (0x20003a00) proceeds to call the image booter. */
    uint32_t sub_ptr = 0;
    cpu_physical_memory_read(usb_struct + 0x8ec, &sub_ptr, 4);
    if (sub_ptr) {
        uint8_t flag1 = 1;
        cpu_physical_memory_write(sub_ptr + 0x36, &flag1, 1);
        qemu_log_mask(LOG_UNIMP, "s5l8900.dma: wrote sub_ptr+0x36=1 (sub_ptr=0x%x)\n", sub_ptr);
    }
    uint32_t dnload_done = 0x40; /* 64-byte block done */
    cpu_physical_memory_write(usb_struct + 0x2c, &dnload_done, 4);

    qemu_log_mask(LOG_UNIMP, "s5l8900.dma: deferred IRQ fired usb_struct=0x%x +0x50=4 +0x100=8 +0x2c=0x40 sub_ptr=0x%x\n", usb_struct, sub_ptr);
    if (s5l8900_iboot_launched) {
        qemu_log_mask(LOG_UNIMP, "s5l8900.dma: skipping IRQ because iBoot launched\n");
        return;
    }
    s->usbctrl->irq_pending = 1;
    s->vic0->pending |= 1;
    s->vic0->vectaddr[0] = 0x200077a4;
    cpu_interrupt(qemu_get_cpu(0), CPU_INTERRUPT_HARD);
}

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

                // Defer IRQ until after USB init (0x200043b4) has completed
                timer_mod(s->deferred_irq,
                          qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 5000000);
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

static uint64_t s5l8900_usbctrl_read(void *opaque, hwaddr offset, unsigned size)
{
    S5L8900USBCTRLState *s = opaque;
    switch (offset) {
        case 0x14:
            if (s->irq_pending) {
                return 0x90080005;
            }
            else {
                return 1;
            }
        case 0x818:
            if (s->irq_pending) {
                return 0x10000;
            }
            else {
                return 0;
            }
        case 0xb08:
            return 0xffffffff;
        case 0x0:
            return s->irq_pending ? 0x10000 : 0;
        default:
            return 0;
    }
}

static void s5l8900_usbctrl_write(void *opaque, hwaddr offset,
                                uint64_t value, unsigned size)
{
    (void)opaque;
    switch (offset) {
        case 0x14:
            break;
        default:
            break;
    }
}

static const MemoryRegionOps s5l8900_usbctrl_ops = {
    .read  = s5l8900_usbctrl_read,
    .write = s5l8900_usbctrl_write,
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
    fprintf(stderr, ">>> s5l8900_init: START\n"); fflush(stderr);
    MemoryRegion *sysmem = get_system_memory();
    MemoryRegion *ram  = g_new0(MemoryRegion, 1);
    MemoryRegion *vrom = g_new0(MemoryRegion, 1);
    MemoryRegion *evec = g_new0(MemoryRegion, 1);
    MemoryRegion *unknown = g_new0(MemoryRegion, 1);
    MemoryRegion *ibss_mr = g_new0(MemoryRegion, 1);
    MemoryRegion *ibec_mr = g_new0(MemoryRegion, 1);
    MemoryRegion *global_catchall = g_new0(MemoryRegion, 1);

     /* Global catch-all for ALL unmapped memory.
      * Added FIRST with overlap priority 0 so specific regions (priority >= 1) take precedence.
      * Prevents data aborts from accessing unmapped addresses. */
     memory_region_init_io(global_catchall, NULL, &s5l8900_periph_catchall_ops,
                            NULL, "s5l8900.global_catchall", 0x100000000ULL);
     memory_region_add_subregion_overlap(sysmem, 0, global_catchall, 0);

   S5L8900VICState *vic0 = g_new0(S5L8900VICState, 1);
    S5L8900VICState *vic1 = g_new0(S5L8900VICState, 1);

    MemoryRegion *vic0_mr = g_new0(MemoryRegion, 1);
    vic0->vectaddr[0] = 0x200077a4;

    MemoryRegion *vic1_mr = g_new0(MemoryRegion, 1);

    /* ARM1176JZF-S */
    ARMCPU *cpu = ARM_CPU(cpu_create(machine->cpu_type));
    qemu_register_reset(s5l8900_cpu_reset, cpu);

   /* Internal SRAM */
    memory_region_init_ram(ram, NULL, "s5l8900.ram",
                            S5L8900_RAM_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, S5L8900_RAM_BASE, ram);

/* Safe Thumb-mode infinite loop at SRAM 0xFE00.
      * Exception vectors redirect here instead of infinite ARM loops,
      * preventing CPU traps from mode-switch issues.
      * Use B #-2 (0xE000) NOT CBZ (0x00EA) which falls through when r0!=0. */
    {
        uint8_t safe_loop[16];
        for (int i = 0; i < 16; i += 2) {
            safe_loop[i] = 0xFE;   /* B #-4 (Thumb-16: 0xE7FE) */
            safe_loop[i+1] = 0xE7;
        }
        cpu_physical_memory_write(S5L8900_SAFELOOP_ADDR, safe_loop, sizeof(safe_loop));
        qemu_log_mask(LOG_UNIMP,
            "s5l8900: safe Thumb loop at 0x%08x\n", S5L8900_SAFELOOP_ADDR);
    }

/* Fill SRAM gap region (0xFC00-0xFE00) with UART print + safe loops.
      * Use WFI (Wait For Interrupt) instead of B #0, because B #0 causes
      * TCG to generate host-level infinite loops that never return to the
      * main loop, preventing timer callbacks from firing. WFI yields to
      * the main loop, allowing timers to fire. */
    {
        uint8_t fill[0x200];
        /* WFI (Thumb-16: 0x3302) yields to main loop, allowing timer processing */
        for (int i = 0; i < sizeof(fill); i += 2) {
            fill[i] = 0x02;   /* WFI (Thumb-16) */
            fill[i+1] = 0x33;
        }

        /* At offset 0xFC00: Thumb routine to write 'i' to UART */
        /* 0xFC00: MOVW r0, #'i' (0x69) -> 0x0069F240 -> LE: 40 F2 69 00 */
        fill[0x00] = 0x40; fill[0x01] = 0xF2;
        fill[0x02] = 0x69; fill[0x03] = 0x00;
        /* 0xFC04: MOVW r1, #0x0002     -> 0x0002F241 -> LE: 41 F2 02 00 */
        fill[0x04] = 0x41; fill[0x05] = 0xF2;
        fill[0x06] = 0x02; fill[0x07] = 0x00;
        /* 0xFC08: MOVT r1, #0xE000     -> 0xE000F2C1 -> LE: C1 F2 00 E0 */
        fill[0x08] = 0xC1; fill[0x09] = 0xF2;
        fill[0x0A] = 0x00; fill[0x0B] = 0xE0;
        /* 0xFC0C: STR r0, [r1]         -> 0x6100 (Thumb-16) -> LE: 00 61 */
        fill[0x0C] = 0x00; fill[0x0D] = 0x61;
        /* 0xFC0E: B #-7 -> back to 0xFC00
         * imm8 = -7 = 0xF9, encoding = 0xEF9E -> LE: 9E EF */
        fill[0x0E] = 0x9E; fill[0x0F] = 0xEF;

        cpu_physical_memory_write(S5L8900_RAM_BASE + 0xFC00, fill, sizeof(fill));
        qemu_log_mask(LOG_UNIMP,
            "s5l8900: SRAM 0xFC00 = UART print 'i' loop, rest = WFI\n");
    }

    /* Unknown region */
    memory_region_init_ram(unknown, NULL, "s5l8900.unknownregion",
                           0x10000, &error_fatal);
    memory_region_add_subregion(sysmem, 0x24000000, unknown);

  /* iBSS RAM region - separate from SRAM, used for DFU bootloader */
    memory_region_init_ram(ibss_mr, NULL, "s5l8900.ibss",
                            S5L8900_IBSS_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, S5L8900_IBSS_BASE, ibss_mr);

  /* iBEC RAM region - recovery bootloader */
    memory_region_init_ram(ibec_mr, NULL, "s5l8900.ibec",
                              S5L8900_IBEC_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, S5L8900_IBEC_BASE, ibec_mr);

    /* iBoot RAM region - main bootloader (loaded after iBEC) */
    {
        MemoryRegion *iboot_mr = g_new0(MemoryRegion, 1);
        memory_region_init_ram(iboot_mr, NULL, "s5l8900.iboot",
                                S5L8900_IBOOT_SIZE, &error_fatal);
        memory_region_add_subregion(sysmem, S5L8900_IBOOT_BASE, iboot_mr);
    }

   /* Large RAM region for A-bit translated addresses.
     * iBEC sets A-bit (CPSR bit 24) for heap/stack/data accesses.
     * With A-bit set, 0x60000000+ becomes 0x70000000+ on access.
     * This catches writes to 0x7ffffxxx that would otherwise
     * go to the catch-all black hole and corrupt function pointers. */
    {
        MemoryRegion *abit_ram = g_new0(MemoryRegion, 1);
        memory_region_init_ram(abit_ram, NULL, "s5l8900.abit_ram",
                                S5L8900_ABIT_SIZE, &error_fatal);
        memory_region_add_subregion(sysmem, S5L8900_ABIT_BASE, abit_ram);
    }

   /* Large heap RAM region for iBEC dynamic allocations.
      * iBEC uses gmalloc for linked-list nodes and data structures
      * at addresses 0x1xxxxxx and 0x2xxxxxx. Without this region,
      * those writes go to the catch-all peripheral stub, reads return
      * 0, and the allocation loop never terminates.
      * Extended to cover 0x08000000-0x37FFFFFF (768MB) to also catch
      * high-address heap allocations and function pointers.
      * The ROM region at 0x20000000 (added later) takes priority for its 1MB.
      * The peripheral region at 0x38000000+ is not affected. */
    {
        MemoryRegion *heap_ram = g_new0(MemoryRegion, 1);
        memory_region_init_ram(heap_ram, NULL, "s5l8900.heap_ram",
                                 0x30000000, &error_fatal); /* 768MB */
        memory_region_add_subregion(sysmem, 0x08000000, heap_ram);
    }

    /* ROM function stub regions - filled with ARM BX LR (0xE12FFF1E).
     * iBEC constant pool contains pointers to ROM functions at these
     * addresses. Any call to these addresses will safely return. */
    {
        MemoryRegion *romstub1 = g_new0(MemoryRegion, 1);
        memory_region_init_ram(romstub1, NULL, "s5l8900.romstub1",
                                S5L8900_ROMSTUB1_SIZE, &error_fatal);
        memory_region_add_subregion(sysmem, S5L8900_ROMSTUB1_BASE, romstub1);
        /* Fill with ARM BX LR */
        uint8_t *stub_buf = memory_region_get_ram_ptr(romstub1);
        uint32_t bx_lr = 0xE12FFF1E; /* ARM BX LR */
        for (gsize i = 0; i < S5L8900_ROMSTUB1_SIZE; i += 4) {
            stl_p(stub_buf + i, bx_lr);
        }
        qemu_log_mask(LOG_UNIMP,
            "s5l8900: ROM stub region 0x%08x (%d bytes) filled with BX LR\n",
            S5L8900_ROMSTUB1_BASE, S5L8900_ROMSTUB1_SIZE);
    }
    {
        MemoryRegion *romstub2 = g_new0(MemoryRegion, 1);
        memory_region_init_ram(romstub2, NULL, "s5l8900.romstub2",
                                S5L8900_ROMSTUB2_SIZE, &error_fatal);
        memory_region_add_subregion(sysmem, S5L8900_ROMSTUB2_BASE, romstub2);
        uint8_t *stub_buf = memory_region_get_ram_ptr(romstub2);
        uint32_t bx_lr = 0xE12FFF1E; /* ARM BX LR */
        for (gsize i = 0; i < S5L8900_ROMSTUB2_SIZE; i += 4) {
            stl_p(stub_buf + i, bx_lr);
        }
        qemu_log_mask(LOG_UNIMP,
            "s5l8900: ROM stub region 0x%08x (%d bytes) filled with BX LR\n",
            S5L8900_ROMSTUB2_BASE, S5L8900_ROMSTUB2_SIZE);
    }
    /* Upper address space RAM catch-all.
     * Covers 0x40000000-0xFFFFFFFF to catch function pointers and
     * heap addresses that land above the peripheral region.
     * Prevents instruction fetches from hitting the catch-all IO
     * handler (which triggers crash recovery loops). */
    {
        MemoryRegion *upper_ram = g_new0(MemoryRegion, 1);
        memory_region_init_ram(upper_ram, NULL, "s5l8900.upper_ram",
                                0xC0000000, &error_fatal); /* 3GB */
        memory_region_add_subregion(sysmem, 0x40000000, upper_ram);
        qemu_log_mask(LOG_UNIMP,
            "s5l8900: upper RAM region 0x40000000-0xFFFFFFFF (3GB)\n");
    }

    /* ROM stub 3: catch iBoot beyond-code fill pattern (0x47704770)
     * being loaded as function pointers. MUST be after upper_ram so it
     * overrides the zero-filled catch-all for this range. Fill with ARM BX LR. */
    {
        MemoryRegion *romstub3 = g_new0(MemoryRegion, 1);
        memory_region_init_ram(romstub3, NULL, "s5l8900.romstub3",
                                S5L8900_ROMSTUB3_SIZE, &error_fatal);
        memory_region_add_subregion(sysmem, S5L8900_ROMSTUB3_BASE, romstub3);
        uint8_t *stub_buf = memory_region_get_ram_ptr(romstub3);
        uint32_t bx_lr = 0xE12FFF1E; /* ARM BX LR */
        for (gsize i = 0; i < S5L8900_ROMSTUB3_SIZE; i += 4) {
            stl_p(stub_buf + i, bx_lr);
        }
        qemu_log_mask(LOG_UNIMP,
            "s5l8900: ROM stub3 region 0x%08x (%d bytes) filled with BX LR\n",
            S5L8900_ROMSTUB3_BASE, S5L8900_ROMSTUB3_SIZE);
    }

    /* SecureROM as RAM region to allow runtime patching.
     * ROM is loaded into a buffer, patched, then written to memory.
     * This avoids TCG translation cache issues with in-place ROM patching. */
    memory_region_init_ram(vrom, NULL, "s5l8900.vrom",
                           S5L8900_VROM_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, S5L8900_VROM_BASE, vrom);

    if (machine->firmware) {
        gsize rom_size;
        guint8 *rom_buf = NULL;
        GError *gerr = NULL;
        size_t rom_len = S5L8900_VROM_SIZE;

        if (g_file_get_contents(machine->firmware,
                                (gchar **)&rom_buf, &rom_size, &gerr)) {
            if (rom_size > rom_len) {
                rom_size = rom_len;
            }

            /* ---- ROM patches to skip polling loops and jump to iBSS ---- */
            /* 0x208-0x214: CLOCK1 PLL lock polling loop.
             * LDR r1,[pc,#0x50] -> r1=CLOCK1+0x40
             * LDR r0,[r1,#0]   -> r0=read(CLOCK1+0x40)
             * MOV r5,#1
             * BNE 0x208         -> loop if r0 != 1
             * Stub returns 0x3, not 1, so BNE always taken.
             * NOP the BNE to skip the infinite loop. */
            {
                uint32_t patch_rom_clock = 0xE1A00000; /* NOP */
                memcpy(rom_buf + 0x214, &patch_rom_clock, 4);
            }

            /* ---- ARM/Thumb mode fix for iBEC re-entry ----
              * When iBSS redirects back to iBEC at 0x0A000000, the ARM prologue
              * checks if self-copy is needed. Since code is already at 0x18000000,
              * the self-copy is skipped and execution falls through to BX r1 at
              * offset 0x80, which reads its target from offset 0xf0.
              * The value 0x18000000 is even -> ARM mode. But the code at that
              * address is Thumb-32. Patch to 0x18000001 (odd -> Thumb mode).
              * ARM hardware ignores LSB for data access, so self-copy loop
              * (which reads the same value) still works correctly. */
             {
                 uint32_t ibec_bx_thumb = 0x18000001; /* Odd address -> Thumb mode */
                 cpu_physical_memory_write(S5L8900_IBEC_BASE + 0xf0,
                                           &ibec_bx_thumb, 4);
                 qemu_log_mask(LOG_UNIMP,
                     "s5l8900: patched iBEC BX target 0xf0 -> 0x18000001 (Thumb mode)\n");
             }

             /* ---- ROM vector table patches to prevent exception cascade ----
              * The ROM copies its vector table from 0x20000000 to 0x00000000
              * during BEGIN_HARDWARE_INIT. When iBSS triggers exceptions, the
              * ROM's handlers run but eventually return to 0x00000000, which
              * branches to 0x00000040 (a memory copy loop). This copy loop
              * overwrites itself mid-execution, corrupting instructions into
              * 'msr cpsr_c, r1' which triggers an illegal AArch64 mode switch.
              *
              * Fix: Patch the ROM's vector table so safe code is copied to 0x0.
              * 1. Replace reset vector (0x00) with jump to iBSS.
              * 2. Replace copy loop at 0x40 with infinite loop.
              * 3. Replace handler data entries with safe loop addresses. */
            {
                   /* Replace ROM entry with LDR r0,[pc,#0x3C]; BX r0 to jump to iBSS.
                    * ARM LDR uses PC = instruction_addr + 8 (guaranteed by ARM spec).
                    * Instruction at 0x00, PC at exec = 0x08.
                    * Offset 0x3C means LDR reads from 0x08 + 0x3C = 0x44.
                    * Constant placed at 0x44, safely beyond vector table and patches. */
                   uint32_t jump_ldr = 0xE59F003C; /* LDR r0, [pc, #0x3C] -> reads 0x44 */
                   uint32_t jump_bx = 0xE12FFF10;   /* BX r0 */
                   uint32_t jump_addr = S5L8900_IBSS_BASE; /* constant at 0x44 */
                   memcpy(rom_buf + 0x00, &jump_ldr, 4);
                   memcpy(rom_buf + 0x04, &jump_bx, 4);
                   memcpy(rom_buf + 0x44, &jump_addr, 4);

                   /* Verify ROM bytes at key offsets */
                   uint32_t verify_ldr, verify_bx, verify_addr;
                   memcpy(&verify_ldr, rom_buf + 0x00, 4);
                   memcpy(&verify_bx, rom_buf + 0x04, 4);
                   memcpy(&verify_addr, rom_buf + 0x44, 4);
                   qemu_log_mask(LOG_UNIMP,
                       "s5l8900: ROM verify: LDR=0x%08X at 0x00, BX=0x%08X at 0x04, addr=0x%08X at 0x44\n",
                       verify_ldr, verify_bx, verify_addr);

                  /* Replace copy loop start at 0x40 with infinite loop. */
                  uint32_t patch_copy_loop = 0xEAFFFFFE;
                  memcpy(rom_buf + 0x40, &patch_copy_loop, 4);

                 /* Replace all vector instructions with infinite loops.
                  * The ROM's vector table has 'ldr pc,[pc,#offset]' at 0x08/0x0c/0x10
                  * that load handler addresses from data entries. Instead of trying to
                  * find the correct data entry offsets, just replace every vector with
                  * a self-loop so any exception traps safely. */
                 uint32_t safe_vec = 0xEAFFFFFE; /* ARM B #-2 (infinite loop) */
                  /* SWI vector at 0x08 is our BX r0 from the jump sequence, leave it */
                  /* Prefetch Abort vector at 0x0c */
                  memcpy(rom_buf + 0x0c, &safe_vec, 4);
                 /* Data Abort vector at 0x10 */
                 memcpy(rom_buf + 0x10, &safe_vec, 4);
                 /* IRQ vector at 0x18 */
                 memcpy(rom_buf + 0x18, &safe_vec, 4);

                 qemu_log_mask(LOG_UNIMP,
                     "s5l8900: patched ROM vector table (reset->iBSS, handlers)\n");
             }

             /* 0x37c0: B 0x2000F000 (skip USB/DFU, go to trampoline)
             * PC at exec = 0x200037c0+8 = 0x200037c8
             * Offset = (0x2000F000 - 0x200037c8)/4 = 0x2E0D */
            uint32_t patch1 = 0xEA002E0E;
            memcpy(rom_buf + 0x37c0, &patch1, 4);

            /* 0x3a0c: B 0x20003a10 (skip USB DMA status wait) */
            uint32_t patch2 = 0xEA000001;
            memcpy(rom_buf + 0x3a0c, &patch2, 4);

            /* 0x3a80: NOP NOP (skip flag check loop) */
            uint32_t patch3 = 0x1a000000;
            memcpy(rom_buf + 0x3a80, &patch3, 4);

            /* 0x3a8c: NOP NOP (skip state check loop) */
            uint32_t patch4 = 0x1a000000;
            memcpy(rom_buf + 0x3a8c, &patch4, 4);

            /* 0x3a90: NOP NOP (skip USB polling branch) */
            uint32_t patch5 = 0x1a000000;
            memcpy(rom_buf + 0x3a90, &patch5, 4);

            /* 0x3a9c: B 0x20003aa8 (skip bx lr, go to sub_4aa8) */
            uint32_t patch6 = 0xEA000002;
            memcpy(rom_buf + 0x3a9c, &patch6, 4);

            /* 0x3aa8: B 0x20003ab0 (skip mov/bx, go to jump point) */
            uint32_t patch7 = 0xEA000001;
            memcpy(rom_buf + 0x3aa8, &patch7, 4);

            /* 0x3ab0: LDR r0, [pc, #8]; BX r0 (load iBSS addr, jump) */
            uint32_t patch8a = 0xE59F0008; /* Rn=15=pc */
            uint32_t patch8b = 0xE12FFF10;
            uint32_t patch8c = S5L8900_IBSS_BASE;
            memcpy(rom_buf + 0x3ab0, &patch8a, 4);
            memcpy(rom_buf + 0x3ab4, &patch8b, 4);
            memcpy(rom_buf + 0x3ab8, &patch8c, 4);

            /* Trampoline at 0xF000: LDR r0,[pc,#0]; BX r0; <iBSS addr> */
            uint32_t tramp0 = 0xE59F0000; /* Rn=15=pc */
            uint32_t tramp1 = 0xE12FFF10;
            uint32_t tramp2 = S5L8900_IBSS_BASE;
            memcpy(rom_buf + 0xF000, &tramp0, 4);
            memcpy(rom_buf + 0xF004, &tramp1, 4);
            memcpy(rom_buf + 0xF008, &tramp2, 4);

            qemu_log_mask(LOG_UNIMP,
                "s5l8900: ROM loaded (%zu bytes), patched, at 0x%x\n",
                rom_size, S5L8900_VROM_BASE);
        } else {
            error_report("s5l8900: failed to load ROM: %s", gerr->message);
            g_error_free(gerr);
            rom_size = 0;
        }

        /* Write patched ROM to memory region */
        if (rom_size > 0) {
            cpu_physical_memory_write(S5L8900_VROM_BASE, rom_buf, rom_size);

            /* Trap patterns at ROM boundaries.
             * Real ROM is 64KB (0x20000000-0x20010000). Region is expanded to 1MB.
             * If execution falls past ROM end, trap and log. */
            {
                /* ARM B #-2 at 0x2000FFFC (4 bytes, catches fallthrough from 0xFFC/0xFFE) */
                uint32_t trap_arm = 0xEAFFFFFE;
                cpu_physical_memory_write(0x2000FFFC, &trap_arm, 4);

                /* ARM B #-2 at 0x20100000 (1MB boundary) */
                cpu_physical_memory_write(0x20100000, &trap_arm, 4);

                /* Thumb B #-2 at 0x20100002 (2 bytes, catches Thumb fallthrough) */
                uint16_t trap_thumb = 0xDEBE;
                cpu_physical_memory_write(0x20100002, &trap_thumb, 2);

                qemu_log_mask(LOG_UNIMP,
                    "s5l8900: wrote trap patterns at 0x2000FFFC and 0x20100000\n");
            }
        }
        g_free(rom_buf);
    }

    /* Load iBSS from -kernel: skip 0x800-byte img2 header, load payload */
    fprintf(stderr, ">>> s5l8900_init: checking kernel_filename\n"); fflush(stderr);
    if (machine->kernel_filename) {
        gsize img_size;
        guint8 *img_data = NULL;
        GError *gerr = NULL;
        if (g_file_get_contents(machine->kernel_filename,
                                (gchar **)&img_data, &img_size, &gerr)) {
            if (img_size > IMG2_HDR_SIZE) {
                size_t payload_size = img_size - IMG2_HDR_SIZE;
                /* Load iBEC from default path if available.
                 * iBEC is the next stage after iBSS in the DFU boot chain. */
                {
                    gsize ibec_size;
                    guint8 *ibec_data = NULL;
                    GError *ibec_err = NULL;
                    const char *ibec_path = "/Users/chris/dev/ipod-touch-1g/work/iPod1,1_1.1_3A101a_Restore/Firmware/dfu/iBEC.n45ap.RELEASE.dfu";
                    if (g_file_get_contents(ibec_path,
                                            (gchar **)&ibec_data, &ibec_size, &ibec_err)) {
                        if (ibec_size > IMG2_HDR_SIZE) {
                            size_t ibec_payload = ibec_size - IMG2_HDR_SIZE;
                            cpu_physical_memory_write(S5L8900_IBEC_BASE,
                                                      ibec_data + IMG2_HDR_SIZE,
                                                      ibec_payload);
                            qemu_log_mask(LOG_UNIMP,
                                "s5l8900: loaded iBEC (%zu bytes payload) at 0x%x\n",
                                ibec_payload, S5L8900_IBEC_BASE);

                            /* Patch iBEC ARM entry point: replace with trampoline.
                              * The ARM prologue checks if the iBEC is at the expected
                              * load address and fails in QEMU. Replace with a simple
                              * LDR/BX trampoline that jumps to Thumb code at 0x18004960.
                              * The constant 0x18004961 (with Thumb bit) comes from the
                              * iBEC's own constant pool at offset 0xe8. */
                             {
                                 uint32_t tramp[5] = {
                                     0xE59F0008,  /* LDR r0, [pc, #8] -> loads from 0x50 */
                                     0xE12FFF10,  /* BX r0 -> Thumb mode */
                                     0x00000000,  /* padding */
                                     0x00000000,  /* padding */
                                     0x18004961   /* Thumb address (0x18004960 + thumb bit) */
                                 };
                                 cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x40,
                                                           tramp, sizeof(tramp));
                                 qemu_log_mask(LOG_UNIMP,
                                     "s5l8900: patched iBEC ARM entry -> trampoline to 0x18004961\n");
                             }

                             /* Patch iBEC crypto verification loop.
                             * iBEC gets stuck in an ARM-mode crypto loop at offset 0x11530-0x1155E
                             * (executing at 0x18011530+ after self-copy to USB OTG region).
                             * The loop does CMP/MOVCS on R0-R3 with a backward branch at 0x1155C.
                             *
                             * Patch strategy: replace the backward branch at 0x1155C-0x1155F
                             * with a forward branch that exits the loop and continues execution.
                             * Also patch the BL at 0x10F04 that calls into the crypto chain
                             * to skip it entirely (MOV r0,#1 to indicate "success"). */
                            {
                                /* Patch 1: Replace BL at offset 0x10F04 with MOV r0,#1 + NOP.
                                 * This skips the crypto function call entirely.
                                 * Thumb: MOV r0,#1 = 0x2001; NOP = 0x4600 */
                                uint8_t skip_bl[] = { 0x01, 0x20, 0x00, 0x46 };
                                cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x10F04,
                                                          skip_bl, 4);
                                qemu_log_mask(LOG_UNIMP,
                                    "s5l8900: patched iBEC BL at 0x%lx (skip crypto call)\n",
                                    (unsigned long)(S5L8900_IBEC_BASE + 0x10F04));

                               /* Patch 2: Replace backward B at offset 0x1155C with BX LR.
                                  * If execution reaches the primary crypto loop, make it
                                  * return immediately.
                                  * Thumb-32 BX LR: 0x18 0x46 0x00 0x47 */
                                uint8_t break_loop[] = { 0x18, 0x46, 0x00, 0x47 };
                                cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x1155C,
                                                          break_loop, 4);
                                qemu_log_mask(LOG_UNIMP,
                                    "s5l8900: patched iBEC primary crypto loop at 0x%lx (B->BX LR)\n",
                                    (unsigned long)(S5L8900_IBEC_BASE + 0x1155C));

                                /* Patch 3: Secondary crypto function at offset 0x11300.
                                  * This function contains two backward branches (0x11310,
                                  * 0x11354) with nested CMP/SUBCS/ORCS bit-comparison loops.
                                  * Replace entry point with MOV r0,#1 + BX LR to skip entirely
                                  * and return "success". Also patch both backward branches
                                  * with NOPs, since execution may jump directly into the
                                  * function body (bypassing the patched entry). */
                                uint8_t skip_secondary[] = { 0x01, 0x20, 0x70, 0x47 };
                                cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x11300,
                                                          skip_secondary, 4);
                                qemu_log_mask(LOG_UNIMP,
                                    "s5l8900: patched iBEC secondary crypto func at 0x%lx (entry->MOV r0,#1/BX LR)\n",
                                    (unsigned long)(S5L8900_IBEC_BASE + 0x11300));

                                /* Patch 3b: NOP backward branch at 0x11310 (outer loop).
                                 * ARM disasm: 'bcc 0' -> branches to function entry.
                                 * Replace with two Thumb-16 NOPs. */
                                uint8_t nop_outer[] = { 0x00, 0xBF, 0x00, 0xBF };
                                cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x11310,
                                                          nop_outer, 4);
                                qemu_log_mask(LOG_UNIMP,
                                    "s5l8900: patched iBEC outer loop branch at 0x%lx (B->NOP NOP)\n",
                                    (unsigned long)(S5L8900_IBEC_BASE + 0x11310));

                                /* Patch 3c: NOP backward branch at 0x11354 (inner loop).
                                 * ARM disasm: 'bne 0x18' -> branches to 0x11318.
                                 * Replace with MOV r0,#1 + BX LR to exit function cleanly. */
                                uint8_t exit_inner[] = { 0x01, 0x20, 0x70, 0x47 };
                                cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x11354,
                                                          exit_inner, 4);
                                qemu_log_mask(LOG_UNIMP,
                                     "s5l8900: patched iBEC inner loop branch at 0x%lx (B->MOV r0,#1/BX LR)\n",
                                     (unsigned long)(S5L8900_IBEC_BASE + 0x11354));

 /* Patch 4-6: Three hardware-dependent functions at 0x05220,
                                  * 0x05248, 0x05270. These functions make BLX calls to addresses
                                  * beyond the iBEC payload boundary, causing instruction fetch
                                  * aborts and infinite looping. Replace each function entry with
                                  * MOV r0,#1 + BX LR to return "success" immediately.
                                  * Thumb: MOV r0,#1 = 0x2001; BX LR = 0x4770 */
                                 uint8_t skip_hw_func[] = { 0x01, 0x20, 0x70, 0x47 };
                                 cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x05220,
                                                           skip_hw_func, 4);
                                 qemu_log_mask(LOG_UNIMP,
                                     "s5l8900: patched iBEC hw func at 0x%lx (entry->MOV r0,#1/BX LR)\n",
                                     (unsigned long)(S5L8900_IBEC_BASE + 0x05220));
                                 cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x05248,
                                                           skip_hw_func, 4);
                                 qemu_log_mask(LOG_UNIMP,
                                     "s5l8900: patched iBEC hw func at 0x%lx (entry->MOV r0,#1/BX LR)\n",
                                     (unsigned long)(S5L8900_IBEC_BASE + 0x05248));
                                 cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x05270,
                                                           skip_hw_func, 4);
                                 qemu_log_mask(LOG_UNIMP,
                                     "s5l8900: patched iBEC hw func at 0x%lx (entry->MOV r0,#1/BX LR)\n",
                                     (unsigned long)(S5L8900_IBEC_BASE + 0x05270));

                                /* Patch 7: BLX within function at 0x05220 that targets address
                                   * beyond the iBEC payload (offset 0x27898). Causes fetch abort.
                                   * Replace with MOV r0,#1 + NOP to return "success" without calling. */
                                  uint8_t blx_nop[] = { 0x01, 0x20, 0x00, 0xbf };
                                  cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x05236,
                                                            blx_nop, 4);
                                  qemu_log_mask(LOG_UNIMP,
                                      "s5l8900: patched iBEC BLX at 0x%lx (BLX->MOV r0,#1/NOP)\n",
                                      (unsigned long)(S5L8900_IBEC_BASE + 0x05236));

                                  /* Patch 8-12: Five BL instructions in looping functions at
                                    * 0x524c, 0x5274, 0x527c. Each function calls a hardware-dependent
                                    * function (0x5218) in a tight retry loop with backward bhi.n
                                    * branches (0x529c -> 0x528c, 0x52a4 -> 0x528c).
                                    *
                                    * Fix: Replace all BL instructions with MOV r0,#1 + NOP to skip
                                    * the hardware calls. This breaks all retry paths.
                                    *
                                    * 0x525a: f7ff ffe1 (BL in 0x524c func)
                                    * 0x5286: f7ff ffcb (BL in 0x5274 func)
                                    * 0x528e: f7ff ffc7 (BL in 0x527c func, CPU stuck here)
                                    * 0x52d6: f00a ff6b (BL in retry loop)
                                    * 0x52f4: f7ff ff94 (BL in retry loop)
                                    * Thumb: MOV r0,#1 = 0x2001 (0x01,0x20); NOP = 0xBF00 (0x00,0xbf) */
                                   uint8_t skip_hw_bl[] = { 0x01, 0x20, 0x00, 0xbf };
                                   cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x0525a,
                                                             skip_hw_bl, 4);
                                   qemu_log_mask(LOG_UNIMP,
                                       "s5l8900: patched iBEC looping func BL at 0x%lx\n",
                                       (unsigned long)(S5L8900_IBEC_BASE + 0x0525a));
                                   cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x05286,
                                                             skip_hw_bl, 4);
                                   qemu_log_mask(LOG_UNIMP,
                                       "s5l8900: patched iBEC looping func BL at 0x%lx\n",
                                       (unsigned long)(S5L8900_IBEC_BASE + 0x05286));
                                   cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x0528e,
                                                             skip_hw_bl, 4);
                                   qemu_log_mask(LOG_UNIMP,
                                       "s5l8900: patched iBEC looping func BL at 0x%lx\n",
                                       (unsigned long)(S5L8900_IBEC_BASE + 0x0528e));
                                   cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x052d6,
                                                             skip_hw_bl, 4);
                                   qemu_log_mask(LOG_UNIMP,
                                       "s5l8900: patched iBEC looping func BL at 0x%lx\n",
                                       (unsigned long)(S5L8900_IBEC_BASE + 0x052d6));
                                   cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x052f4,
                                                             skip_hw_bl, 4);
                                   qemu_log_mask(LOG_UNIMP,
                                       "s5l8900: patched iBEC looping func BL at 0x%lx\n",
                                       (unsigned long)(S5L8900_IBEC_BASE + 0x052f4));

                                   /* NOP backward bhi.n branches that retry the patched calls.
                                    * With BL replaced by MOV r0,#1, the subs/cmp logic at
                                    * 0x5296-0x52a4 always finds the values unchanged and
                                    * retries forever. Break the retry loops:
                                    * 0x529c: bhi.n #-6 (0xd8f7) -> back to 0x528e
                                    * 0x52a4: bhi.n #-10 (0xd8f3) -> back to 0x528e
                                    * Replace with Thumb-16 NOP (0xBF00 -> bytes 0x00, 0xBF) */
                                   uint16_t thumb_nop = 0xBF00;
                                   cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x0529c,
                                                             &thumb_nop, 2);
                                   qemu_log_mask(LOG_UNIMP,
                                       "s5l8900: NOPed iBEC bhi.n retry at 0x%lx\n",
                                       (unsigned long)(S5L8900_IBEC_BASE + 0x0529c));
                                   cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x052a4,
                                                             &thumb_nop, 2);
                                   qemu_log_mask(LOG_UNIMP,
                                       "s5l8900: NOPed iBEC bhi.n retry at 0x%lx\n",
                                       (unsigned long)(S5L8900_IBEC_BASE + 0x052a4));

                               /* Patch source iBEC function at 0x4960: Thumb MOV r0,#1; BX LR.
                                      * This function loads null function pointer from BSS.
                                      * Patch propagates to 0x18000000 on self-copy. */
                                     uint8_t thumb_mov1_bxlr[] = { 0x01, 0x20, 0x70, 0x47 };
                                     cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x4960,
                                                               thumb_mov1_bxlr, 4);
                                     qemu_log_mask(LOG_UNIMP,
                                         "s5l8900: patched iBEC func at 0x4960 -> MOV r0,#1/BX LR\n");

                                /* Patch source iBEC at 0x0A000000 with MOV r0,#1 / BX LR
                                     * at the 3 POP {PC} offsets. If the self-copy reads from
                                     * 0x0A000000, these patches will propagate to 0x18000000.
                                     * The USB OTG handler also patches 0x18000000 directly as
                                     * a fallback, since self-copy may read from flash instead.
                                     * Use MOV r0,#1 + BX LR instead of POP {PC} to avoid
                                     * reading corrupted stack, and instead of bare BX LR to
                                     * avoid infinite loops when LR = function entry. */
                                    uint32_t arm_mov1_bxlr = 0xE3A00001; /* MOV r0, #1 */
                                    /* Only patch at 0x115a8 and 0x116e4 where BL calls are used.
                                     * At 0x113b4, the function is a callback with no valid return. */
                                    cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x115a8,
                                                              &arm_mov1_bxlr, 4);
                                    cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x116e4,
                                                               &arm_mov1_bxlr, 4);
                                     qemu_log_mask(LOG_UNIMP,
                                         "s5l8900: patched 2 iBEC POP {PC} -> MOV r0,#1 at 0x115a8, 0x116e4\n");

                                     /* Patch iBEC halt function at 0x153c.
                                       * B #-2 infinite loop -> BX LR to return to caller. */
                                      uint16_t thumb_bxlr = 0x4770; /* BX LR (Thumb-16) */
                                      cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x153c,
                                                                &thumb_bxlr, 2);
                                      qemu_log_mask(LOG_UNIMP,
                                          "s5l8900: patched iBEC halt at 0x153c (B #-2 -> BX LR)\n");

                                      /* Patch ARM-mode function table placeholder at 0x40c.
                                        * The iBEC has ARM-mode utility functions at 0x3e0 (VFP
                                        * config), 0x3f0 (FPExc enable), 0x400 (D-cache invalidate),
                                        * followed by 0x40c which is an infinite loop (B #-2). Some
                                        * code path calls this function via BLX and gets trapped.
                                        * Replace with MOV r0,#1; BX LR to return success. */
                                       uint32_t arm_func_return[] = { 0xE3A00001, 0xE12FFF1E }; /* MOV r0,#1; BX LR */
                                       cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x40c,
                                                                 arm_func_return, 8);
                                       qemu_log_mask(LOG_UNIMP,
                                           "s5l8900: patched iBEC ARM func at 0x40c (infinite loop -> MOV r0,#1/BX LR)\n");

   /* Patch ARM-mode BL instructions at 0x103c4 and 0x103dc
                     * that call into garbage memory beyond the iBEC payload.
                     * The function at 0x103c0 is entered via BLX from Thumb
                     * code (ARM mode). It does rsbs r0,r0,#0 then bl to
                     * 0x18013c6c which is beyond valid code. Same at 0x103d8.
                     * Replace with MOV r0,#1; BX LR for clean return. */
                    uint32_t arm_mov1_ibec = 0xE3A00001;
                    uint32_t arm_bxlr_ibec = 0xE12FFF1E;
                    cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x103c0,
                                              &arm_mov1_ibec, 4);
                    cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x103c4,
                                              &arm_bxlr_ibec, 4);
                    cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x103d8,
                                              &arm_mov1_ibec, 4);
                    cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x103dc,
                                              &arm_bxlr_ibec, 4);
                    qemu_log_mask(LOG_UNIMP,
                        "s5l8900: patched iBEC ARM BL at 0x103c4, 0x103dc -> MOV r0,#1/BX LR\n");

                    /* Patch infinite linked-list allocation loop.
                      * iBEC enters an infinite loop creating linked list nodes.
                      * The loop calls ARM-mode stub functions at 0x3c60, 0x3c64,
                      * 0x3c78 (each: PUSH all; MOV r0,SP; BLX target; B #-2 loop)
                      * and a Thumb-mode function at 0x3e48 (PUSH {r4,lr}; ...).
                      * Each ARM stub ends with an infinite loop (B #-2).
                      * Replace each stub with MOV r0,#0; BX LR (failure/NULL return)
                      * to make the allocation appear to fail, so the caller exits. */
                     {
                         uint32_t arm_mov0_stub = 0xE3A00000; /* MOV r0, #0 */
                         uint32_t arm_bxlr_stub = 0xE12FFF1E; /* BX LR */
                         /* ARM stub at 0x3c60: replace with MOV r0,#0; BX LR */
                         cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x3c60,
                                                   &arm_mov0_stub, 4);
                         cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x3c64,
                                                   &arm_bxlr_stub, 4);
                         /* ARM stub at 0x3c68: replace with MOV r0,#0; BX LR */
                         cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x3c68,
                                                   &arm_mov0_stub, 4);
                         cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x3c6c,
                                                   &arm_bxlr_stub, 4);
                         /* ARM stub at 0x3c74: replace with MOV r0,#0; BX LR */
                         cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x3c74,
                                                   &arm_mov0_stub, 4);
                         cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x3c78,
                                                   &arm_bxlr_stub, 4);
                        /* Thumb function at 0x3e48: replace PUSH+next with MOV r0,#0; BX LR */
                         uint8_t thumb_fail[] = { 0x00, 0x20, 0x70, 0x47 }; /* MOV r0,#0; BX LR */
                         cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x3e48,
                                                   thumb_fail, 4);
                         /* Allocation driver at 0xfb26: replace prologue with MOV r0,#0; BX LR
                          * to make gmalloc return NULL, causing the caller to exit its loop. */
                         uint8_t thumb_alloc_fail[] = { 0x00, 0x20, 0x70, 0x47 }; /* MOV r0,#0; BX LR */
                        cpu_physical_memory_write(S5L8900_IBEC_BASE + 0xfb26,
                                                   thumb_alloc_fail, 4);
                         qemu_log_mask(LOG_UNIMP,
                             "s5l8900: patched iBEC linked-list loop (stubs->fail, 0xfb26->fail, 0x3e48->fail)\n");
                     }

                     /* Patch bad BLX at 0x312E that calls 0x016e3600 (bad memory).
                       * The function at this pointer is in unmapped address space.
                       * Replace the 4-byte BLX (Thumb-32) with MOV r0,#1; BX LR (2x Thumb-16)
                       * to return success and skip the crash entirely. */
                      {
                          uint8_t thumb_skip_blx[] = { 0x01, 0x20, 0x70, 0x47 }; /* MOV r0,#1; BX LR */
                          cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x312e,
                                                    thumb_skip_blx, 4);
                          qemu_log_mask(LOG_UNIMP,
                              "s5l8900: patched iBEC bad BLX at 0x312e (skip 0x016e3600 -> MOV r0,#1/BX LR)\n");
                      }

                     /* Patch callback loop at 0x11380: replace entry with infinite loop.
                        * The caller at 0x18001211 keeps re-calling this function.
                        * BX LR causes infinite call loop. Infinite loop here is stable. */
                       {
                           uint8_t thumb_infinite_cb[] = { 0x00, 0xEA }; /* B #-2 */
                           cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x11380,
                                                     thumb_infinite_cb, 2);
                           qemu_log_mask(LOG_UNIMP,
                               "s5l8900: patched iBEC callback loop at 0x11380 -> infinite loop\n");
                       }

                      /* Patch ARM-mode callback at 0x113b0: MOV r0,#1; BX LR.
                       * Prevents crashes if called directly from non-loop code paths. */
                      {
                          uint32_t arm_mov1_cb = 0xE3A00001; /* MOV r0, #1 */
                          uint32_t arm_bxlr_cb = 0xE12FFF1E; /* BX LR */
                          cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x113b0,
                                                    &arm_mov1_cb, 4);
                          cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x113b4,
                                                    &arm_bxlr_cb, 4);
                          qemu_log_mask(LOG_UNIMP,
                              "s5l8900: patched iBEC callback at 0x113b0 -> ARM MOV r0,#1/BX LR\n");
                      }

                      /* Patch write loop at 0x113c0: replace POP {r0-r3,PC} with
                       * MOV r0,#1; BX LR to prevent garbage stack returns. */
                      {
                          uint32_t arm_mov1_wl = 0xE3A00001; /* MOV r0, #1 */
                          uint32_t arm_bxlr_wl = 0xE12FFF1E; /* BX LR */
                          cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x113c0,
                                                    &arm_mov1_wl, 4);
                          cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x113c4,
                                                    &arm_bxlr_wl, 4);
                      qemu_log_mask(LOG_UNIMP,
                           "s5l8900: patched iBEC write loop at 0x113c0 -> ARM MOV r0,#1/BX LR\n");
                       }

                       /* Patch bit-manipulation function at 0x1660: ARM MOV r0,#1; BX LR.
                        * This function reads/modifies/writes CLOCK1 register 0x4C
                        * (0x3c50004c). GDB step hangs on the STR to this MMIO.
                        * Bypass to prevent stall during iBEC init dispatch. */
                       {
                           uint32_t arm_mov1_bit = 0xE3A00001; /* MOV r0, #1 */
                           uint32_t arm_bxlr_bit = 0xE12FFF1E; /* BX LR */
                           cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x1660,
                                                     &arm_mov1_bit, 4);
                           cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x1664,
                                                     &arm_bxlr_bit, 4);
                           qemu_log_mask(LOG_UNIMP,
                               "s5l8900: patched iBEC bit-manip at 0x1660 -> ARM MOV r0,#1/BX LR\n");
                       }

                        /* Bypass crypto/hash processing loop at 0x10ed0.
                         * Original: CBZ r3, #0x10f1a (conditional exit on null pointer).
                         * The loop iterates over crypto operations that depend on
                         * hardware data we can't emulate. Replace with unconditional
                         * branch to exit path at 0x10f1a. */
                        {
                            uint8_t thumb_b_cryptoloop[] = { 0x01, 0xD0, 0x75, 0x00 };
                            cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x10ed0,
                                                      thumb_b_cryptoloop, 4);
                            qemu_log_mask(LOG_UNIMP,
                                "s5l8900: patched iBEC crypto loop at 0x10ed0 (CBZ->B exit)\n");
                        }

                    /* Patch iBEC Thumb entry BLs (0x4968-0x4978) to skip init calls
                        * that transition to ARM mode and crash on unmapped ROM pointers.
                        * Corrected offsets: actual BLs at 0x4968, 0x496c, 0x4970, 0x4974, 0x4978. */
                        {
                            uint8_t skip_bl[] = { 0x01, 0x20, 0x00, 0xbf }; /* MOV r0,#1; NOP */
                            uint16_t offsets[] = { 0x4968, 0x496c, 0x4970, 0x4974, 0x4978 };
                            for (int k = 0; k < 5; k++) {
                                cpu_physical_memory_write(S5L8900_IBEC_BASE + offsets[k],
                                                           skip_bl, 4);
                            }
                           qemu_log_mask(LOG_UNIMP,
                                  "s5l8900: patched iBEC entry BLs 0x4968-0x4978 (skip init calls)\n");

                            /* Patch code at 0x4980 (past BLs) with safe Thumb infinite loop.
                             * The original code at 0x4980+ makes many more BL calls to
                             * bad/unmapped addresses that crash immediately. Replace with
                             * a halt loop to prove we can reach this point. The trampoline
                             * points to 0x18004981 (odd), which enters Thumb mode at 0x4980. */
                        {
                            uint8_t safe_loop[] = { 0xFE, 0xE7 }; /* B #-4 (infinite loop) */
                            cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x4980,
                                                      safe_loop, sizeof(safe_loop));
                            qemu_log_mask(LOG_UNIMP,
                                "s5l8900: patched iBEC safe loop at 0x4980 (B #-2)\n");
                        }
                        }  /* End BL+safe_loop patch block */

                            /* CRITICAL: Redirect ARM entry point to Thumb entry.
                             * When iBSS copies iBEC from 0x0A000000 to 0x18000000,
                             * it copies this patched ARM entry. The ARM prologue
                             * enables the MMU which causes a prefetch abort at
                             * 0x18003fd6. By redirecting at the entry point,
                             * we skip the ARM prologue entirely.
                             *
                             * Patch: LDR r0, [pc, #8]; BX r0; NOP; .word 0x18004960
                             * This loads the Thumb entry address and branches to it.
                             * The BX with odd target (0x18004960 | 1 = 0x18004961)
                             * would switch to Thumb mode, but 0x18004960 is even.
                             * Instead, we use the SRAM trampoline (ARM->Thumb). */
                        {
                            /* Trampoline at iBEC offset 0x0:
                             * 0x00: LDR r0, [pc, #8]  -> loads 0x2200FF00 from pool at 0x0C
                             *  0x04: BX r0             -> jumps to SRAM trampoline
                             *  0x08: NOP (ARM)         -> padding
                             *  0x0C: .word 0x2200FF00  -> SRAM trampoline address
                             */
                            uint32_t arm_entry_patch[5] = {
                                0xE59F0008,  /* LDR r0, [pc, #8]  -> r0 = 0x2200FF00 */
                                0xE12FFF10,  /* BX r0              -> jump to trampoline */
                                0xE1A00000,  /* NOP                -> padding */
                                0x2200FF00   /* constant: trampoline address */
                            };
                            cpu_physical_memory_write(S5L8900_IBEC_BASE,
                                                      arm_entry_patch, sizeof(arm_entry_patch));
                            qemu_log_mask(LOG_UNIMP,
                                 "s5l8900: patched iBEC ARM entry -> redirect to SRAM trampoline\n");
                         }

                        /* Patch stuck function at 0x5080 (refcount/data init loop).
                         * This function iterates over data structures and gets stuck
                         * in a tight loop (0x50ae-0x50e4) due to hardware-dependent
                         * memory accesses. Replace entry with MOV r0,#0; BX LR. */
                        {
                            uint8_t func_ret[] = { 0x00, 0x20, 0x70, 0x47 }; /* MOV r0,#0; BX LR */
                            cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x5080, func_ret, 4);
                            qemu_log_mask(LOG_UNIMP,
                                "s5l8900: patched iBEC stuck func at 0x5080 -> MOV r0,#0; BX LR\n");
                        }
                  }
                        }
                           fprintf(stderr, ">>> s5l8900_init: about to load iBoot\n"); fflush(stderr);
                          /* Load decrypted iBoot.
                            * iBoot is the next stage after iBEC in the DFU boot chain.
                            * On real hardware, iBEC receives iBoot via USB DFU and
                            * jumps to it. In QEMU, we pre-load the decrypted payload
                            * and patch iBEC to jump directly to iBoot. */
                          {
                              fprintf(stderr, ">>> iBoot: attempting to load from %s\n",
                                      "/Users/chris/dev/ipod-touch-1g/work/iBoot.decrypted");
                              fflush(stderr);
                              gsize iboot_size;
                              guint8 *iboot_data = NULL;
                              GError *iboot_err = NULL;
                              const char *iboot_path = "/Users/chris/dev/ipod-touch-1g/work/iBoot.decrypted";
                               if (g_file_get_contents(iboot_path,
                                                       (gchar **)&iboot_data, &iboot_size, &iboot_err)) {
                                   fprintf(stderr, ">>> iBoot: loaded %zu bytes\n", iboot_size);
                                   size_t iboot_payload = iboot_size;
                                   if (iboot_payload > S5L8900_IBOOT_SIZE) {
                                       iboot_payload = S5L8900_IBOOT_SIZE;
                                   }

                                    /* iBoot was compiled for load address 0x18000000.
                                    * Its literal pools contain absolute 0x180xxxxx addresses.
                                    * Aggressive word-by-word patching corrupts Thumb instructions
                                    * that happen to match the 0x180xxxxx pattern. Instead, we
                                    * will write iBoot to BOTH 0x18000000 and 0x23000000 so
                                    * all literal references resolve correctly. */

                                   /* Zero BSS section in buffer (will be written to both regions).
                                    * The reset handler also zeros BSS at runtime, but pre-zeroing
                                    * ensures the USBOTG mirror at 0x18000000 is also clean. */
                                   {
                                       size_t bss_start = 0x21980;
                                       size_t bss_end = 0x26000;
                                       if (bss_end <= iboot_payload) {
                                           memset(iboot_data + bss_start, 0, bss_end - bss_start);
                                           fprintf(stderr, ">>> iBoot: pre-zeroed BSS 0x%zx-0x%zx\n",
                                                   bss_start, bss_end);
                                       }
                                   }

                                     /* Patch reset handler for QEMU emulation.
                                       * - Force load address check to pass (skip self-copy loop)
                                       * - NOP CP15 cache operations (crash in QEMU)
                                       * The reset handler then zeros BSS, sets up mode stack
                                       * pointers, and BX to main (Thumb mode). */
                                     {
                                         uint32_t *buf = (uint32_t *)iboot_data;
                                         buf[0x454/4] = 0xEA00000A;       /* B #0x484 (was BEQ, force skip copy) */
                                         buf[0x474/4] = 0xE1A00000;       /* NOP (was MCR p15, clean D cache) */
                                         buf[0x478/4] = 0xE1A00000;       /* NOP (was MCR p15, invalidate I cache) */
                                         qemu_log_mask(LOG_UNIMP,
                                             "s5l8900: patched iBoot reset handler (skip copy, stub CP15)\n");
                                     }

                                       /* Leave entry function at 0x4C20 as-is (B #0x4C90).
                                        * The original branch skips device detection and returns -1.
                                        * The return address on the stack points to 0x5CA0 (main init).
                                        * This lets 0x5CA0 run without the device detection setup. */
                                      {
                                          qemu_log_mask(LOG_UNIMP,
                                              "s5l8900: left iBoot entry 0x4C20 as-is (B -> 0x4C90, returns to 0x5CA0)\n");
                                      }

                                      /* Patch iBoot exception vectors to proper handler that
                                        * restores CPSR to User mode before returning.
                                        * Simple BX LR leaves CPU in exception mode (e.g., UND=0x1B),
                                        * causing cascading exceptions when privileged instructions
                                        * are executed in the wrong mode.
                                        *
                                        * Handler at 0x420: MOV r0,#0xDF; MSR cpsr_c,r0; BX lr
                                        * Each vector: B handler + NOP (4 bytes each)
                                        * Vectors: 0x404-0x41C (UND,SWI,PABT,DABT,RES,IRQ,FIQ) */
                                      {
                                          uint8_t *buf8 = (uint8_t *)iboot_data;
                                          /* Vector patches: B to 0x420 + NOP */
                                          uint8_t vec_patches[][4] = {
                                              {0x68, 0xE0, 0x00, 0xBF},  /* 0x404: B #0x1a -> 0x420 */
                                              {0x58, 0xE0, 0x00, 0xBF},  /* 0x408: B #0x16 */
                                              {0x48, 0xE0, 0x00, 0xBF},  /* 0x40C: B #0x12 */
                                              {0x38, 0xE0, 0x00, 0xBF},  /* 0x410: B #0x0e */
                                              {0x28, 0xE0, 0x00, 0xBF},  /* 0x414: B #0x0a */
                                              {0x18, 0xE0, 0x00, 0xBF},  /* 0x418: B #0x06 */
                                              {0x08, 0xE0, 0x00, 0xBF},  /* 0x41C: B #0x02 */
                                          };
                                          for (size_t i = 0; i < 7; i++) {
                                              size_t v = 0x404 + i * 4;
                                              buf8[v] = vec_patches[i][0];
                                              buf8[v+1] = vec_patches[i][1];
                                              buf8[v+2] = vec_patches[i][2];
                                              buf8[v+3] = vec_patches[i][3];
                                          }
                                          /* Exception handler at 0x420 */
                                          uint8_t handler[] = {
                                              0xDF, 0x20,     /* MOV r0, #0xDF (User+N+I+F) */
                                              0x80, 0xF3, 0x00, 0x81, /* MSR cpsr_c, r0 */
                                              0x70, 0x47      /* BX lr */
                                          };
                                          for (size_t i = 0; i < sizeof(handler); i++) {
                                              buf8[0x420 + i] = handler[i];
                                          }
                                          qemu_log_mask(LOG_UNIMP,
                                              "s5l8900: patched iBoot exception vectors -> handler at 0x420 (MSR CPSR User; BX lr)\n");
                                      }

                                     /* Stub hardware-dependent functions called from 0x5CA0.
                                       * These functions depend on uninitialized globals and
                                       * hardware state. Patch each to: MOV r0,#0; BX LR (Thumb).
                                       * This lets 0x5CA0 skip hardware init and continue.
                                       * Thumb encoding: 0x2000 (MOVS r0,#0) 0x4770 (BX LR) */
                                     {
                                         uint16_t *buf16 = (uint16_t *)iboot_data;
                                         uint16_t stub[2] = {0x2000, 0x4770};  /* MOV r0,#0; BX LR */
                                         uint16_t stub1[2] = {0x2001, 0x4770}; /* MOV r0,#1; BX LR */

                                         /* 0x763C: device detection linked-list traversal
                                          * Returns 0 -> BEQ at 0x5CE6 skips to 0x5D02 */
                                         buf16[0x763C/2] = stub[0];
                                         buf16[0x763C/2+1] = stub[1];

                                         /* 0x57A8: hardware init (memory alloc + setup) */
                                         buf16[0x57A8/2] = stub[0];
                                         buf16[0x57A8/2+1] = stub[1];

                                         /* 0x595E: hardware init (linked-list operations) */
                                         buf16[0x595E/2] = stub[0];
                                         buf16[0x595E/2+1] = stub[1];

                                         /* 0x7A34: memory allocation function */
                                         buf16[0x7A34/2] = stub[0];
                                         buf16[0x7A34/2+1] = stub[1];

                                         /* 0x17F50: malloc-like function */
                                         buf16[0x17F50/2] = stub[0];
                                         buf16[0x17F50/2+1] = stub[1];

                                         /* 0x5C00: hash/comparison function */
                                         buf16[0x5C00/2] = stub[0];
                                         buf16[0x5C00/2+1] = stub[1];

                                         /* 0x7BFC: memory management (called from 0x7688) */
                                         buf16[0x7BFC/2] = stub[0];
                                         buf16[0x7BFC/2+1] = stub[1];

                                          /* 0x4FA0: called from puts (0x17D96).
                                           * Must NOT return 0, or puts skips the print loop.
                                           * Return 1 to let puts continue to putchar loop. */
                                          buf16[0x4FA0/2] = 0x2001;  /* MOVS r0, #1 */
                                          buf16[0x4FA0/2+1] = 0x4770;  /* BX LR */

                                         /* 0x4FC4: called from 0x595E */
                                         buf16[0x4FC4/2] = stub[0];
                                         buf16[0x4FC4/2+1] = stub[1];

                                         /* 0x4F00: called from 0x7BFC */
                                         buf16[0x4F00/2] = stub[0];
                                         buf16[0x4F00/2+1] = stub[1];

                                         /* 0x17D96: puts-like function - DO NOT STUB.
                                          * This function prints strings via 0x4A5C (UART putchar).
                                          * Its dependencies (0x4FA0, 0x4FC4) are stubbed, but
                                          * the core loop (LDRB + BL 0x4A5C) will work. */

                                         /* 0x17D6A: getchar - return 0x0A (newline) to trigger
                                          * the newline handler at 0x5D86, which calls 0x17D96 (puts)
                                          * to print a prompt/menu. Thumb: MOV r0,#0xa; BX LR */
                                         buf16[0x17D6A/2] = 0x0020;  /* MOVS r0, #0 (low byte) */
                                         buf16[0x17D6A/2+1] = 0x4770;  /* BX LR */
                                         /* Need to set r0 = 0x0A, use MOVW or MOVS+ORR */
                                         /* Actually use: MOVS r0, #0x0A (16-bit Thumb: 0x200A) */
                                         buf16[0x17D6A/2] = 0x200A;  /* MOVS r0, #0x0A */
                                         buf16[0x17D6A/2+1] = 0x4770;  /* BX LR */

                                         qemu_log_mask(LOG_UNIMP,
                                             "s5l8900: stubbed iBoot hardware functions -> MOV r0,#0; BX LR\n");
                                     }

                                    /* Replace infinite loop at 0x4EC with branch to
                                      * custom Thumb-1 stub at 0x510. The stub:
                                      * 1. Prints "iBoot start\n" directly to UART MMIO
                                      * 2. Branches to boot function at 0x05CA0
                                      * This bypasses the broken 0x4C20 entry point and
                                      * the putchar function's dependency on unmapped
                                      * iBEC data (0x1C04B5B0).
                                      *
                                      * Thumb B: PC = instr+4, target = PC + (imm11<<1)
                                      * 0x4EC -> 0x510: imm11 = (0x510-0x4F0)/2 = 0x10 */
                                    {
                                        uint16_t *buf16 = (uint16_t *)iboot_data;
                                        buf16[0x4EC/2] = 0xF020;         /* B 0x510 (stub) */

                                        /* Thumb-1 stub at 0x510 (38 bytes, overwrites
                                         * ARM safe loops at 0x526+ which aren't needed).
                                         * Prints "iBoot start\n" to UART at 0xE0002000,
                                         * then B to boot function at 0x05CA0. */
                                        uint8_t stub[] = {
                                            0x00, 0xE0,          /* 0x510: B 0x512 */
                                            0xF8, 0xDF, 0x10, 0x00, /* 0x512: LDR r0,[pc,#0x10] -> str addr */
                                            0x60, 0xF2, 0x20, 0x5E, /* 0x516: MOV r4,#0xE0,ROR#24 -> 0xE0000000 */
                                            0x84, 0xF2, 0x00, 0x20, /* 0x51A: ORR r4,r4,#0x2000 -> 0xE0002000 */
                                            0x00, 0x21,          /* 0x51E: MOV r1,#0 */
                                            0x10, 0x78,          /* 0x520: LDRB r1,[r0],#1 */
                                            0xFA, 0xD0,          /* 0x522: BLE 0x51E */
                                            0x04, 0x60,          /* 0x524: STRB r1,[r4,#0] UART write */
                                            0x01, 0x30,          /* 0x526: ADD r0,r0,#1 */
                                            0x1A, 0x42,          /* 0x528: CMP r0,r2 */
                                            0xF4, 0xD1,          /* 0x52A: BNE 0x520 */
                                            0x00, 0xF3,          /* 0x52C: B 0x05CA0 */
                                            0x4D, 0xAE, 0x01, 0x23, /* 0x52E: "iBoot start\n" addr */
                                            0x59, 0xAE, 0x01, 0x23, /* 0x532: string end addr (r2) */
                                        };
                                        memcpy(iboot_data + 0x510, stub, sizeof(stub));
                                        qemu_log_mask(LOG_UNIMP,
                                            "s5l8900: patched iBoot infinite loop -> UART print stub at 0x510\n");
                                    }

                                      cpu_physical_memory_write(S5L8900_IBOOT_BASE,
                                                               iboot_data,
                                                               iboot_payload);
                                      qemu_log_mask(LOG_UNIMP,
                                          "s5l8900: loaded decrypted iBoot (%zu bytes) at 0x%x\n",
                                          iboot_payload, S5L8900_IBOOT_BASE);

                                      /* Mirror iBoot at original load address (0x18000000).
                                       * iBoot literal pools reference 0x180xxxxx addresses.
                                       * Writing to both locations avoids corrupting Thumb
                                       * code with aggressive address patching. */
                                      cpu_physical_memory_write(S5L8900_USBOTG_BASE,
                                                               iboot_data,
                                                               iboot_payload);
                                      fprintf(stderr, ">>> iBoot: mirrored %zu bytes to 0x%08x\n",
                                              iboot_payload, S5L8900_USBOTG_BASE);




                                    /* Fill beyond-code region with ARM BX LR (0xE12FFF1E).
                                     * In ARM mode: BX LR returns safely to caller.
                                     * In Thumb mode: bytes 1E FF 2F E1 decode as:
                                     *   offset+0: 0xFF1E = PUSH {r0-r7,LR} (stack smash but safe)
                                     *   offset+2: 0x2FE1 = (invalid, but CPU may not reach)
                                     * Better: use pattern 70 47 70 47 (Thumb BX LR repeated).
                                     * ARM reads 0x47704770 = BXEQ r0; BXEQ r0 (no-ops if !Z).
                                     * Thumb reads 0x4770 = BX LR (safe return). */
                                    {
                                        size_t fill_sz = S5L8900_IBOOT_SIZE - iboot_payload;
                                        uint8_t *fill = g_malloc0(fill_sz);
                                        uint32_t pat = 0x47704770; /* Thumb BX LR x2, ARM BXEQ r0 x2 */
                                        for (gsize i = 0; i + 4 <= fill_sz; i += 4) {
                                            memcpy(fill + i, &pat, 4);
                                        }
                                         cpu_physical_memory_write(S5L8900_IBOOT_BASE + iboot_payload,
                                                                   fill, fill_sz);
                                         /* Also fill the mirror region */
                                         cpu_physical_memory_write(S5L8900_USBOTG_BASE + iboot_payload,
                                                                   fill, fill_sz);
                                         g_free(fill);
                                         fprintf(stderr, ">>> iBoot: filled 0x%x-0x%x with 0x47704770 pattern\n",
                                                 (unsigned)(S5L8900_IBOOT_BASE + iboot_payload),
                                                 (unsigned)(S5L8900_IBOOT_BASE + S5L8900_IBOOT_SIZE));
                                     }

                                    /* Verify iBoot loaded correctly */
                                    {
                                        uint8_t verify[16];
                                        cpu_physical_memory_read(S5L8900_IBOOT_BASE + 0x3ca0, verify, 16);
                                        fprintf(stderr, ">>> iBoot: verify @ 0x3ca0: %02x %02x %02x %02x ...\n",
                                                verify[0], verify[1], verify[2], verify[3]);
                                        cpu_physical_memory_read(S5L8900_IBOOT_BASE + 0x1ede6, verify, 16);
                                        fprintf(stderr, ">>> iBoot: verify @ 0x1ede6: %02x %02x %02x %02x ...\n",
                                                verify[0], verify[1], verify[2], verify[3]);
                                    }

                                     /* Patch iBEC safe loop at 0x18004980 to jump to iBoot.
                                     * The safe loop (B #-2) is reached after iBEC entry BLs
                                     * are patched to MOV r0,#1; NOP. Replace with Thumb-32
                                     * LDR r0, [pc, #offset]; BX r0 to jump to iBoot.
                                     * iBoot entry: offset 0x40 is ARM prologue, but we try
                                     * offset 0x0 first (direct payload start).
                                     * Use Thumb bit (LSB=1) for BX to enter Thumb mode. */
                                    {
                                         uint32_t iboot_entry = S5L8900_IBOOT_BASE + 0x400; /* ARM vectors */

                                        // Fix: constant should be at PC+8 when LDR executes.
                                        // LDR is at 0x4980, PC during exec = 0x4984.
                                        // PC+8 = 0x498C. So constant at offset 0x0C from 0x4980.
                                        // Let me recalculate with proper Thumb LDR encoding.
                                        // Actually let me use a simpler approach: write ARM instructions
                                        // at 0x4980 since the region might be entered in ARM mode.
                                        // No, the safe loop is Thumb mode. Let me use proper Thumb.
                                        //
                                        // Thumb-32: LDR r0, [PC, #imm12]
                                        // Encoding: 1111 X 100 H 1111 | imm8 0000 RnRn
                                        // For LDR r0, [pc, #8]:
                                        // First halfword: 1111 1000 1111 1101 = 0xF8FD
                                        // No wait, let me use a different approach.
                                        //
                                        // Simplest: use two Thumb-16 instructions + constant
                                        // ADR r0, const (Thumb-32) + BX r0
                                        // Or just: LDR r0, [pc, #0] (reads from pc+4+0 = pc+4)
                                        //
                                        // At 0x4980: LDR r0, [pc, #4] -> reads from 0x498C
                                        // Thumb-32 LDR r0, [pc, #4]:
                                        // 0xF8DF 0x0004 -> bytes: F8 DF 04 00
                                        // At 0x4984: BX r0 -> 00 47
                                        // At 0x4986: padding -> 00 00
                                        // At 0x4988: padding -> 00 00
                                        // At 0x498C: constant (iboot_entry)
                                        //
                                        // Wait, PC during LDR exec = instruction_addr + 4 = 0x4984.
                                        // LDR r0, [pc, #4] reads from 0x4984 + 4 = 0x4988.
                                        // So constant at 0x4988.
                                        //
                                        // Layout:
                                        // 0x4980: F8 DF 04 00 (LDR r0, [pc, #4])
                                        // 0x4984: 00 47 (BX r0)
                                        // 0x4986: 00 00 (padding)
                                        // 0x4988: iboot_entry (constant)
                                        // Total: 8 bytes from 0x4980 to 0x498F.
                                        uint8_t tramp_code[] = {
                                            0xF8, 0xDF, 0x04, 0x00,  /* LDR r0, [pc, #4] */
                                            0x00, 0x47,              /* BX r0 */
                                            0x00, 0x00               /* padding */
                                        };
                                        cpu_physical_memory_write(S5L8900_USBOTG_BASE + 0x4980,
                                                                  tramp_code, sizeof(tramp_code));
                                        cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x4980,
                                                                  tramp_code, sizeof(tramp_code));
                                        uint32_t entry_const = iboot_entry;
                                        cpu_physical_memory_write(S5L8900_USBOTG_BASE + 0x4988,
                                                                  &entry_const, 4);
                                        cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x4988,
                                                                  &entry_const, 4);
                                        qemu_log_mask(LOG_UNIMP,
                                            "s5l8900: patched iBEC safe loop -> jump to iBoot at 0x%08x\n",
                                            iboot_entry);
                                     }
                                 g_free(iboot_data);
                            } else {
                                qemu_log_mask(LOG_UNIMP,
                                    "s5l8900: could not load iBoot: %s\n",
                                    iboot_err ? iboot_err->message : "unknown");
                                if (iboot_err) g_error_free(iboot_err);
                            }
                        }
                        g_free(ibec_data);
                     } else {
                        qemu_log_mask(LOG_UNIMP,
                            "s5l8900: could not load iBEC: %s\n",
                            ibec_err ? ibec_err->message : "unknown");
                        if (ibec_err) g_error_free(ibec_err);
                    }
                }

               /* CRITICAL: Patch iBSS buffer BEFORE writing to memory.
                  * The iBSS self-copies from 0x09000000 to 0x22000000.
                  * By patching the buffer, both the source AND the self-copied
                  * destination will have the patches. This is essential because
                  * the self-copy overwrites any patches applied to RAM after load. */
                  if (s5l8900_ibss_clean()) {
                      fprintf(stderr, "s5l8900: CLEAN BASELINE - skipping ALL iBSS buffer patches (real image)\n");
                  } else
                  {
                      uint8_t *buf = img_data + IMG2_HDR_SIZE;

                      /* Patch 0x3cdc-0x4fff: fill region with ARM-mode safe return.
                      * CRITICAL: The iBSS is Thumb-mode code. When executed in ARM mode
                      * (e.g., via BLX to even address), Thumb bytes decode as ARM LDR/STR
                      * instructions that read data from PC-relative addresses and use them
                      * as jump targets, causing PC to jump to 0xe* range garbage addresses.
                      * Start from 0x3cdc (aligned to 0x3c00) to cover the function that
                      * crashes at 0x22003cde. Fill through 0x4fff.
                      * ARM: MOV r0,#1 (0xE3A00001); BX LR (0xE12FFF1E) */
                      {
                          /* EXPERIMENT: This fill destroys ~5KB of valid iBSS Thumb
                           * code (16 real functions, incl. the one at 0x3f4c that the
                           * crash lands in). It is only correct if iBSS jumps here in
                           * ARM mode; iBSS is a Thumb binary, so the fill is harmful.
                           * Gate it behind S5L8900_IBSS_FILL_3C00 (default: SKIP). */
                          if (getenv("S5L8900_IBSS_FILL_3C00")) {
                              uint32_t arm_ret[] = { 0xE3A00001, 0xE12FFF1E };
                              size_t region_start = 0x3c00;
                              size_t region_size = 0x4400 - region_start; /* Cover 0x3c00 to 0x4fff */
                              for (size_t i = 0; i < region_size && (region_start + i) < payload_size; i += 8) {
                                  memcpy(buf + region_start + i, arm_ret, sizeof(arm_ret));
                              }
                              fprintf(stderr, "s5l8900: patched iBSS buffer 0x%zx-0x4fff -> ARM MOV r0,#1/BX LR (%zu bytes)\n",
                                  region_start, region_size);
                          } else {
                              fprintf(stderr, "s5l8900: SKIPPED iBSS fill 0x3c00-0x4fff (S5L8900_IBSS_FILL_3C00 unset; real iBSS code preserved)\n");
                          }
                      }

                   /* Patch BL callers OUTSIDE the 0x3c00-0x4fff region.
                     * Only patch callers at 0x5006 and beyond. */
                     {
                         uint32_t arm_ret[] = { 0xE3A00001, 0xE12FFF1E };
                         uint32_t callers[] = { 0x5006, 0x5020, 0x5c66, 0x950e, 0x966c };
                         for (size_t i = 0; i < sizeof(callers) / sizeof(callers[0]); i++) {
                             if (callers[i] < payload_size) {
                                 memcpy(buf + callers[i], arm_ret, sizeof(arm_ret));
                             }
                         }
                         fprintf(stderr, "s5l8900: patched iBSS buffer BL callers -> ARM MOV r0,#1/BX LR\n");
                     }

                    /* Skip iBSS polling loop at 0x3720.
                     * This loop reads peripheral 0x4c00010 and waits for value > 11.
                     * The peripheral stub always returns 0, so the loop never exits.
                     * Replace BLE #-4 (0xD0FC) at offset 0x3724 with NOP (0xBF00). */
                    if (0x3724 < payload_size) {
                        uint8_t thumb_nop[] = { 0x00, 0xBF }; /* NOP */
                        memcpy(buf + 0x3724, thumb_nop, sizeof(thumb_nop));
                        fprintf(stderr,
                            "s5l8900: skipped iBSS polling loop at 0x3724 (BLE->NOP)\n");
                    }

                    /* Patch stuck function at 0x5bf0: Thumb MOV r0,#1; BX LR.
                     * This function loops and calls functions that return bad addresses,
                     * causing PC to jump through 0x22* range garbage. */
                    if (0x5bf0 < payload_size) {
                        uint8_t thumb_ret[] = { 0x01, 0x20, 0x70, 0x47 }; /* MOV r0,#1; BX LR */
                        memcpy(buf + 0x5bf0, thumb_ret, sizeof(thumb_ret));
                        fprintf(stderr,
                            "s5l8900: patched iBSS stuck func at 0x5bf0 -> MOV r0,#1/BX LR\n");
                    }

                     /* Patch 0x5400: Thumb MOV r0,#0; BX LR */
                     {
                         uint8_t ret_0[] = { 0x00, 0x20, 0x70, 0x47 };
                         memcpy(buf + 0x5400, ret_0, sizeof(ret_0));
                     }
                 }

                cpu_physical_memory_write(S5L8900_IBSS_BASE,
                                            img_data + IMG2_HDR_SIZE,
                                            payload_size);

                 /* Patch iBSS polling loops and exception handlers.
                  * iBSS self-copies to SRAM (0x22000000) at boot.
                  * Patch source at 0x09000000 so self-copy propagates patches.
                  * Write bytes explicitly to avoid host endianness issues. */
                 if (s5l8900_ibss_clean()) {
                     fprintf(stderr, "s5l8900: CLEAN BASELINE - skipping iBSS post-write code patches\n");
                 } else
                 {
                     /* 1. CLOCK1 PLL lock poll at offset 0x1154 (Thumb-16).
                      * Loop: LDR r3,[r2] / TST r1,r3 / BEQ #-4
                      * Replace BEQ (0xD0FC) with NOP (0xBF00). */
                     uint8_t ibss_nop[] = { 0x00, 0xBF };
                    cpu_physical_memory_write(S5L8900_IBSS_BASE + 0x1154,
                                              ibss_nop, 2);
                    qemu_log_mask(LOG_UNIMP,
                        "s5l8900: patched iBSS CLOCK1 poll at 0x%lx (BEQ->NOP)\n",
                        (unsigned long)(S5L8900_IBSS_BASE + 0x1154));

                    /* 2. Restore corrupted Thumb-32 ADD at offset 0x10540.
                     * Previous NOP patch (2 bytes) overwrote only the first halfword
                     * of the 32-bit 'add r0, r0, #1' (0xFA 0xFF 0xFF 0x0A),
                     * creating a malformed instruction that triggers undefined-instruction
                     * or prefetch abort. Restore original bytes. */
                    uint8_t ibss_restore_add[] = { 0xfa, 0xff, 0xff, 0x0a };
                    cpu_physical_memory_write(S5L8900_IBSS_BASE + 0x10540,
                                              ibss_restore_add, 4);
                    qemu_log_mask(LOG_UNIMP,
                        "s5l8900: restored iBSS ADD at 0x%lx\n",
                        (unsigned long)(S5L8900_IBSS_BASE + 0x10540));

  /* 3. Prevent fallthrough to zeros at offset 0x10564.
                       * After the scan loop counter expires (r1=12), the Thumb-32
                       * B.ne instruction at 0x10564 (0x1EFF 0x2FE1) is not taken,
                       * and execution falls into zero padding at 0x10568.
                       * Replace entire Thumb-32 instruction with two NOPs (0xBF00 0xBF00)
                       * so execution falls through safely. The zeros at 0x10568 decode as
                       * YIELD hints in Thumb mode, which are harmless no-ops.
                       * Eventually the catch-all stub handles accesses to unmapped data,
                       * and the USB init code at 0x10598 writes "config_board" to trigger
                       * the iBEC redirect. */
                      uint8_t ibss_loop_patch[] = { 0x00, 0xBF, 0x00, 0xBF }; /* Two Thumb NOPs */
                      cpu_physical_memory_write(S5L8900_IBSS_BASE + 0x10564,
                                                ibss_loop_patch, 4);
                      qemu_log_mask(LOG_UNIMP,
                          "s5l8900: patched iBSS fallthrough at 0x%lx (two NOPs)\n",
                          (unsigned long)(S5L8900_IBSS_BASE + 0x10564));

                    /* 3b. Patch stuck iBSS function at 0x10580 that calls bad
                         * function pointers (0x22010500) and loops in upper RAM.
                         * The function is called in ARM mode by the caller
                         * (lr bit[0]=0), so ARM-mode encodings are required.
                         * Patch with MOV r0,#1; BX LR (ARM) to return success.
                         * Also patch the two BLX #0x22010500 at 0x10592 and
                         * 0x105ac as a fallback, in case execution reaches them. */
                        {
                            uint8_t ibss_arm_ret[] = {
                                0x01, 0x00, 0xA0, 0xE3,  /* MOV r0, #1 (ARM) */
                                0x1E, 0xFF, 0x2F, 0xE1   /* BX LR (ARM, return to Thumb) */
                            };
                            cpu_physical_memory_write(S5L8900_IBSS_BASE + 0x10580,
                                                      ibss_arm_ret, sizeof(ibss_arm_ret));
                            cpu_physical_memory_write(S5L8900_RAM_BASE + 0x10580,
                                                      ibss_arm_ret, sizeof(ibss_arm_ret));
                            /* Fallback: NOP the BLX #0x22010500 calls at 0x10592 and 0x105ac */
                            uint8_t blx_nop[] = { 0x00, 0xBF, 0x00, 0xBF }; /* Two Thumb NOPs */
                            cpu_physical_memory_write(S5L8900_IBSS_BASE + 0x10592,
                                                      blx_nop, sizeof(blx_nop));
                            cpu_physical_memory_write(S5L8900_RAM_BASE + 0x10592,
                                                      blx_nop, sizeof(blx_nop));
                            cpu_physical_memory_write(S5L8900_IBSS_BASE + 0x105ac,
                                                      blx_nop, sizeof(blx_nop));
                            cpu_physical_memory_write(S5L8900_RAM_BASE + 0x105ac,
                                                      blx_nop, sizeof(blx_nop));

                            /* 3c. Patch iBSS linked-list comparison loop at 0x702a.
                             * The loop at 0x702c-0x7048 iterates a linked list,
                             * calling 0x104e0 to compare each entry. The comparison
                             * always fails (hardware-specific data), causing an
                             * infinite or very long loop. Skip the entire loop by
                             * redirecting B at 0x702a to the exit at 0x7050.
                             * Target: 0x7050, Source: 0x702c (PC after B).
                             * Offset: (0x7050 - 0x702c) / 2 = 0x13.
                             * Encoding: 0xE000 | 0x13 = 0xE013. */
                            {
                                uint8_t skip_loop[] = { 0x13, 0xE0 }; /* B #0x7050 */
                                cpu_physical_memory_write(S5L8900_IBSS_BASE + 0x702a,
                                                          skip_loop, sizeof(skip_loop));
                                cpu_physical_memory_write(S5L8900_RAM_BASE + 0x702a,
                                                          skip_loop, sizeof(skip_loop));
                            }

                            /* Also patch 0x6f60 which has the same comparison loop.
                             * Redirect B at 0x6f70 to the exit at 0x6faa.
                             * Target: 0x6faa, Source: 0x6f72 (PC after B).
                             * Offset: (0x6faa - 0x6f72) / 2 = 0x1C.
                             * Encoding: 0xE000 | 0x1C = 0xE01C. */
                            {
                                uint8_t skip_6f60_loop[] = { 0x1C, 0xE0 }; /* B #0x6faa */
                                cpu_physical_memory_write(S5L8900_IBSS_BASE + 0x6f70,
                                                          skip_6f60_loop, sizeof(skip_6f60_loop));
                                cpu_physical_memory_write(S5L8900_RAM_BASE + 0x6f70,
                                                          skip_6f60_loop, sizeof(skip_6f60_loop));
                            }

                           /* 3d. Patch the function at 0x4ea0 entirely.
                              * This function reads bad function pointers from its constant pool
                              * and jumps to garbage addresses (0xe* range). Replace with
                              * Thumb infinite loop. Also patch BL callers to skip the call. */
                             {
/* Fill 0x4ea0-0x4ebf with Thumb infinite loop */
                                  uint8_t safe_loop[32];
                                  for (int i = 0; i < 32; i += 2) {
                                       safe_loop[i] = 0xFE;  /* B #-4 (0xE7FE) */
                                       safe_loop[i+1] = 0xE7;
                                   }
                                 cpu_physical_memory_write(S5L8900_IBSS_BASE + 0x4ea0,
                                                           safe_loop, sizeof(safe_loop));
                                 cpu_physical_memory_write(S5L8900_RAM_BASE + 0x4ea0,
                                                           safe_loop, sizeof(safe_loop));

                                 /* Patch BL callers: replace with MOV r0,#1; BX LR */
                                 uint8_t skip_call[] = { 0x01, 0x20, 0x70, 0x47 };
                                 uint32_t callers[] = {
                                     0x4eaa, 0x4efb, 0x4fba, 0x5006, 0x5020, 0x5c66, 0x950e, 0x966c
                                 };
                                 for (size_t i = 0; i < sizeof(callers) / sizeof(callers[0]); i++) {
                                     cpu_physical_memory_write(S5L8900_IBSS_BASE + callers[i],
                                                               skip_call, sizeof(skip_call));
                                     cpu_physical_memory_write(S5L8900_RAM_BASE + callers[i],
                                                               skip_call, sizeof(skip_call));
                                 }
                             }

                            /* 3e. Also patch 0x5400 entry itself as a fallback. */
                            {
                                uint8_t func_5400_safe[] = {
                                    0x00, 0x20,  /* MOV r0, #0 (Thumb) */
                                    0x70, 0x47   /* BX LR (Thumb) */
                                };
                                cpu_physical_memory_write(S5L8900_IBSS_BASE + 0x5400,
                                                          func_5400_safe, sizeof(func_5400_safe));
                            }

                            /* Flush TB cache so patched instructions take effect */
                            queue_tb_flush(CPU(cpu));
                            fprintf(stderr,
                                "s5l8900: patched iBSS stuck func at 0x10580 -> MOV r0,#1; BX LR (ARM)\n");
                            fprintf(stderr,
                                "s5l8900: NOPed BLX at 0x10592 and 0x105ac (bad function pointer)\n");
                            fprintf(stderr,
                                "s5l8900: skipped iBSS comparison loop at 0x702a -> B exit\n");
                            fprintf(stderr,
                                "s5l8900: skipped iBSS 0x6f60 comparison loop -> B exit\n");
                      fprintf(stderr,
                                 "s5l8900: NOPed 7 callers of 0x5400 (bad data deref)\n");
                             fprintf(stderr,
                                 "s5l8900: patched iBSS func at 0x5400 -> MOV r0,#0; BX LR\n");
                         }
                 }

               /* 4. Patch iBSS exception handlers to redirect to iBEC.
                  * The iBSS sets up its own exception handlers at 0x22004040,
                  * 0x22004068, and 0x2200408c. Instead of infinite loops,
                  * redirect to the ARM->Thumb trampoline so we skip to iBEC
                  * and avoid the undefined-instruction crash in iBSS code.
                  * Use: LDR pc, [pc, #0] with constant = trampoline address. */
                  if (!s5l8900_ibss_clean()) {
                      /* Each handler: LDR pc, [pc, #0]; .word tramp_addr */
                      uint32_t arm_to_tramp = 0xE59FF000; /* LDR pc, [pc, #0] */
                       uint32_t tramp_addr = S5L8900_RAM_BASE + 0xF900;

                     /* Undefined Instruction handler at offset 0x4040 */
                     cpu_physical_memory_write(S5L8900_IBSS_BASE + 0x4040,
                                               &arm_to_tramp, 4);
                     cpu_physical_memory_write(S5L8900_IBSS_BASE + 0x4044,
                                               &tramp_addr, 4);
                     /* Prefetch Abort handler at offset 0x4068 */
                     cpu_physical_memory_write(S5L8900_IBSS_BASE + 0x4068,
                                               &arm_to_tramp, 4);
                     cpu_physical_memory_write(S5L8900_IBSS_BASE + 0x406C,
                                               &tramp_addr, 4);
                     /* Data Abort handler at offset 0x408c */
                     cpu_physical_memory_write(S5L8900_IBSS_BASE + 0x408c,
                                               &arm_to_tramp, 4);
                     cpu_physical_memory_write(S5L8900_IBSS_BASE + 0x4090,
                                               &tramp_addr, 4);

                     /* Also patch SRAM copies directly. The iBSS self-copies to
                      * 0x22000000, and its exception handlers may already be set up
                      * in SRAM at 0x22004040, 0x22004068, 0x2200408c. */
                     cpu_physical_memory_write(S5L8900_RAM_BASE + 0x4040,
                                               &arm_to_tramp, 4);
                     cpu_physical_memory_write(S5L8900_RAM_BASE + 0x4044,
                                               &tramp_addr, 4);
                     cpu_physical_memory_write(S5L8900_RAM_BASE + 0x4068,
                                               &arm_to_tramp, 4);
                     cpu_physical_memory_write(S5L8900_RAM_BASE + 0x406C,
                                               &tramp_addr, 4);
                     cpu_physical_memory_write(S5L8900_RAM_BASE + 0x408c,
                                               &arm_to_tramp, 4);
                     cpu_physical_memory_write(S5L8900_RAM_BASE + 0x4090,
                                               &tramp_addr, 4);

                     qemu_log_mask(LOG_UNIMP,
                         "s5l8900: patched iBSS exception handlers -> ARM UART loop\n");
                 }

               qemu_log_mask(LOG_UNIMP,
                     "s5l8900: loaded iBSS %s (%zu bytes payload) at 0x%x\n",
                     machine->kernel_filename,
                     payload_size, S5L8900_IBSS_BASE);

                      /* Write ARM trampoline to SRAM early.
                       * This trampoline is used by exception handlers and
                       * stuck-loop detection to redirect to iBoot.
                       * Must exist before iBSS jumps to iBEC at 0x18000000. */
                  {
                      uint32_t iboot_entry = S5L8900_IBOOT_BASE + 0x400; /* ARM vectors */
                      uint32_t tramp[3] = {
                          0xE59F0000,  /* LDR r0, [pc, #0] */
                          0xE12FFF10,  /* BX r0            -> Thumb mode (LSB=1) */
                          iboot_entry  /* constant: iBoot entry */
                      };
                      cpu_physical_memory_write(S5L8900_RAM_BASE + 0xFF00, tramp, sizeof(tramp));
                      qemu_log_mask(LOG_UNIMP,
                          "s5l8900: wrote ARM trampoline at 0x%08x -> iBoot 0x%08x\n",
                          S5L8900_RAM_BASE + 0xFF00, iboot_entry);
                  }
            } else {
                error_report("s5l8900: iBSS image too small: %zu bytes", img_size);
            }
            g_free(img_data);
        } else {
            error_report("s5l8900: failed to load iBSS: %s", gerr->message);
            g_error_free(gerr);
        }
    }

    /* === Load iBoot (unconditional - must run even without -kernel) === */
                               fprintf(stderr, ">>> s5l8900_init: about to load iBoot\n"); fflush(stderr);
                              /* Load decrypted iBoot.
                                * iBoot is the next stage after iBEC in the DFU boot chain.
                                * On real hardware, iBEC receives iBoot via USB DFU and
                                * jumps to it. In QEMU, we pre-load the decrypted payload
                                * and patch iBEC to jump directly to iBoot. */
                              {
                                  fprintf(stderr, ">>> iBoot: attempting to load from %s\n",
                                          "/Users/chris/dev/ipod-touch-1g/work/iBoot.decrypted");
                                  fflush(stderr);
                                  gsize iboot_size;
                                  guint8 *iboot_data = NULL;
                                  GError *iboot_err = NULL;
                                  const char *iboot_path = "/Users/chris/dev/ipod-touch-1g/work/iBoot.decrypted";
                                   if (g_file_get_contents(iboot_path,
                                                           (gchar **)&iboot_data, &iboot_size, &iboot_err)) {
                                       fprintf(stderr, ">>> iBoot: loaded %zu bytes\n", iboot_size);
                                       size_t iboot_payload = iboot_size;
                                       if (iboot_payload > S5L8900_IBOOT_SIZE) {
                                           iboot_payload = S5L8900_IBOOT_SIZE;
                                       }
    
                                        /* iBoot was compiled for load address 0x18000000.
                                        * Its literal pools contain absolute 0x180xxxxx addresses.
                                        * Aggressive word-by-word patching corrupts Thumb instructions
                                        * that happen to match the 0x180xxxxx pattern. Instead, we
                                        * will write iBoot to BOTH 0x18000000 and 0x23000000 so
                                        * all literal references resolve correctly. */
    
                                       /* Zero BSS section in buffer (will be written to both regions).
                                        * The reset handler also zeros BSS at runtime, but pre-zeroing
                                        * ensures the USBOTG mirror at 0x18000000 is also clean. */
                                       {
                                           size_t bss_start = 0x21980;
                                           size_t bss_end = 0x26000;
                                           if (bss_end <= iboot_payload) {
                                               memset(iboot_data + bss_start, 0, bss_end - bss_start);
                                               fprintf(stderr, ">>> iBoot: pre-zeroed BSS 0x%zx-0x%zx\n",
                                                       bss_start, bss_end);
                                           }
                                       }
    
                                         /* Patch reset handler for QEMU emulation.
                                           * - Force load address check to pass (skip self-copy loop)
                                           * - NOP CP15 cache operations (crash in QEMU)
                                           * The reset handler then zeros BSS, sets up mode stack
                                           * pointers, and BX to main (Thumb mode). */
                                         {
                                             uint32_t *buf = (uint32_t *)iboot_data;
                                             buf[0x454/4] = 0xEA00000A;       /* B #0x484 (was BEQ, force skip copy) */
                                             buf[0x474/4] = 0xE1A00000;       /* NOP (was MCR p15, clean D cache) */
                                             buf[0x478/4] = 0xE1A00000;       /* NOP (was MCR p15, invalidate I cache) */
                                             qemu_log_mask(LOG_UNIMP,
                                                 "s5l8900: patched iBoot reset handler (skip copy, stub CP15)\n");
                                         }
    
                                           /* Leave entry function at 0x4C20 as-is (B #0x4C90).
                                            * The original branch skips device detection and returns -1.
                                            * The return address on the stack points to 0x5CA0 (main init).
                                            * This lets 0x5CA0 run without the device detection setup. */
                                          {
                                              qemu_log_mask(LOG_UNIMP,
                                                  "s5l8900: left iBoot entry 0x4C20 as-is (B -> 0x4C90, returns to 0x5CA0)\n");
                                          }
    
                                          /* Patch iBoot exception vectors to proper handler that
                                            * restores CPSR to User mode before returning.
                                            * Simple BX LR leaves CPU in exception mode (e.g., UND=0x1B),
                                            * causing cascading exceptions when privileged instructions
                                            * are executed in the wrong mode.
                                            *
                                            * Handler at 0x420: MOV r0,#0xDF; MSR cpsr_c,r0; BX lr
                                            * Each vector: B handler + NOP (4 bytes each)
                                            * Vectors: 0x404-0x41C (UND,SWI,PABT,DABT,RES,IRQ,FIQ) */
                                          {
                                              uint8_t *buf8 = (uint8_t *)iboot_data;
                                              /* Vector patches: B to 0x420 + NOP */
                                              uint8_t vec_patches[][4] = {
                                                  {0x68, 0xE0, 0x00, 0xBF},  /* 0x404: B #0x1a -> 0x420 */
                                                  {0x58, 0xE0, 0x00, 0xBF},  /* 0x408: B #0x16 */
                                                  {0x48, 0xE0, 0x00, 0xBF},  /* 0x40C: B #0x12 */
                                                  {0x38, 0xE0, 0x00, 0xBF},  /* 0x410: B #0x0e */
                                                  {0x28, 0xE0, 0x00, 0xBF},  /* 0x414: B #0x0a */
                                                  {0x18, 0xE0, 0x00, 0xBF},  /* 0x418: B #0x06 */
                                                  {0x08, 0xE0, 0x00, 0xBF},  /* 0x41C: B #0x02 */
                                              };
                                              for (size_t i = 0; i < 7; i++) {
                                                  size_t v = 0x404 + i * 4;
                                                  buf8[v] = vec_patches[i][0];
                                                  buf8[v+1] = vec_patches[i][1];
                                                  buf8[v+2] = vec_patches[i][2];
                                                  buf8[v+3] = vec_patches[i][3];
                                              }
                                              /* Exception handler at 0x420 */
                                              uint8_t handler[] = {
                                                  0xDF, 0x20,     /* MOV r0, #0xDF (User+N+I+F) */
                                                  0x80, 0xF3, 0x00, 0x81, /* MSR cpsr_c, r0 */
                                                  0x70, 0x47      /* BX lr */
                                              };
                                              for (size_t i = 0; i < sizeof(handler); i++) {
                                                  buf8[0x420 + i] = handler[i];
                                              }
                                              qemu_log_mask(LOG_UNIMP,
                                                  "s5l8900: patched iBoot exception vectors -> handler at 0x420 (MSR CPSR User; BX lr)\n");
                                          }
    
                                         /* Stub hardware-dependent functions called from 0x5CA0.
                                           * These functions depend on uninitialized globals and
                                           * hardware state. Patch each to: MOV r0,#0; BX LR (Thumb).
                                           * This lets 0x5CA0 skip hardware init and continue.
                                           * Thumb encoding: 0x2000 (MOVS r0,#0) 0x4770 (BX LR) */
                                         {
                                             uint16_t *buf16 = (uint16_t *)iboot_data;
                                             uint16_t stub[2] = {0x2000, 0x4770};  /* MOV r0,#0; BX LR */
                                             uint16_t stub1[2] = {0x2001, 0x4770}; /* MOV r0,#1; BX LR */
    
                                             /* 0x763C: device detection linked-list traversal
                                              * Returns 0 -> BEQ at 0x5CE6 skips to 0x5D02 */
                                             buf16[0x763C/2] = stub[0];
                                             buf16[0x763C/2+1] = stub[1];
    
                                             /* 0x57A8: hardware init (memory alloc + setup) */
                                             buf16[0x57A8/2] = stub[0];
                                             buf16[0x57A8/2+1] = stub[1];
    
                                             /* 0x595E: hardware init (linked-list operations) */
                                             buf16[0x595E/2] = stub[0];
                                             buf16[0x595E/2+1] = stub[1];
    
                                             /* 0x7A34: memory allocation function */
                                             buf16[0x7A34/2] = stub[0];
                                             buf16[0x7A34/2+1] = stub[1];
    
                                             /* 0x17F50: malloc-like function */
                                             buf16[0x17F50/2] = stub[0];
                                             buf16[0x17F50/2+1] = stub[1];
    
                                             /* 0x5C00: hash/comparison function */
                                             buf16[0x5C00/2] = stub[0];
                                             buf16[0x5C00/2+1] = stub[1];
    
                                             /* 0x7BFC: memory management (called from 0x7688) */
                                             buf16[0x7BFC/2] = stub[0];
                                             buf16[0x7BFC/2+1] = stub[1];
    
                                              /* 0x4FA0: called from puts (0x17D96).
                                               * Must NOT return 0, or puts skips the print loop.
                                               * Return 1 to let puts continue to putchar loop. */
                                              buf16[0x4FA0/2] = 0x2001;  /* MOVS r0, #1 */
                                              buf16[0x4FA0/2+1] = 0x4770;  /* BX LR */
    
                                             /* 0x4FC4: called from 0x595E */
                                             buf16[0x4FC4/2] = stub[0];
                                             buf16[0x4FC4/2+1] = stub[1];
    
                                             /* 0x4F00: called from 0x7BFC */
                                             buf16[0x4F00/2] = stub[0];
                                             buf16[0x4F00/2+1] = stub[1];
    
                                             /* 0x17D96: puts-like function - DO NOT STUB.
                                              * This function prints strings via 0x4A5C (UART putchar).
                                              * Its dependencies (0x4FA0, 0x4FC4) are stubbed, but
                                              * the core loop (LDRB + BL 0x4A5C) will work. */
    
                                             /* 0x17D6A: getchar - return 0x0A (newline) to trigger
                                              * the newline handler at 0x5D86, which calls 0x17D96 (puts)
                                              * to print a prompt/menu. Thumb: MOV r0,#0xa; BX LR */
                                             buf16[0x17D6A/2] = 0x0020;  /* MOVS r0, #0 (low byte) */
                                             buf16[0x17D6A/2+1] = 0x4770;  /* BX LR */
                                             /* Need to set r0 = 0x0A, use MOVW or MOVS+ORR */
                                             /* Actually use: MOVS r0, #0x0A (16-bit Thumb: 0x200A) */
                                             buf16[0x17D6A/2] = 0x200A;  /* MOVS r0, #0x0A */
                                             buf16[0x17D6A/2+1] = 0x4770;  /* BX LR */
    
                                             qemu_log_mask(LOG_UNIMP,
                                                 "s5l8900: stubbed iBoot hardware functions -> MOV r0,#0; BX LR\n");
                                         }
    
                                        /* Replace infinite loop at 0x4EC with branch to
                                          * custom Thumb-1 stub at 0x510. The stub:
                                          * 1. Prints "iBoot start\n" directly to UART MMIO
                                          * 2. Branches to boot function at 0x05CA0
                                          * This bypasses the broken 0x4C20 entry point and
                                          * the putchar function's dependency on unmapped
                                          * iBEC data (0x1C04B5B0).
                                          *
                                          * Thumb B: PC = instr+4, target = PC + (imm11<<1)
                                          * 0x4EC -> 0x510: imm11 = (0x510-0x4F0)/2 = 0x10 */
                                        {
                                            uint16_t *buf16 = (uint16_t *)iboot_data;
                                            buf16[0x4EC/2] = 0xF020;         /* B 0x510 (stub) */
    
                                            /* Thumb-1 stub at 0x510 (38 bytes, overwrites
                                             * ARM safe loops at 0x526+ which aren't needed).
                                             * Prints "iBoot start\n" to UART at 0xE0002000,
                                             * then B to boot function at 0x05CA0. */
                                            uint8_t stub[] = {
                                                0x00, 0xE0,          /* 0x510: B 0x512 */
                                                0xF8, 0xDF, 0x10, 0x00, /* 0x512: LDR r0,[pc,#0x10] -> str addr */
                                                0x60, 0xF2, 0x20, 0x5E, /* 0x516: MOV r4,#0xE0,ROR#24 -> 0xE0000000 */
                                                0x84, 0xF2, 0x00, 0x20, /* 0x51A: ORR r4,r4,#0x2000 -> 0xE0002000 */
                                                0x00, 0x21,          /* 0x51E: MOV r1,#0 */
                                                0x10, 0x78,          /* 0x520: LDRB r1,[r0],#1 */
                                                0xFA, 0xD0,          /* 0x522: BLE 0x51E */
                                                0x04, 0x60,          /* 0x524: STRB r1,[r4,#0] UART write */
                                                0x01, 0x30,          /* 0x526: ADD r0,r0,#1 */
                                                0x1A, 0x42,          /* 0x528: CMP r0,r2 */
                                                0xF4, 0xD1,          /* 0x52A: BNE 0x520 */
                                                0x00, 0xF3,          /* 0x52C: B 0x05CA0 */
                                                0x4D, 0xAE, 0x01, 0x23, /* 0x52E: "iBoot start\n" addr */
                                                0x59, 0xAE, 0x01, 0x23, /* 0x532: string end addr (r2) */
                                            };
                                            memcpy(iboot_data + 0x510, stub, sizeof(stub));
                                            qemu_log_mask(LOG_UNIMP,
                                                "s5l8900: patched iBoot infinite loop -> UART print stub at 0x510\n");
                                        }
    
                                          cpu_physical_memory_write(S5L8900_IBOOT_BASE,
                                                                   iboot_data,
                                                                   iboot_payload);
                                          qemu_log_mask(LOG_UNIMP,
                                              "s5l8900: loaded decrypted iBoot (%zu bytes) at 0x%x\n",
                                              iboot_payload, S5L8900_IBOOT_BASE);
    
                                          /* Mirror iBoot at original load address (0x18000000).
                                           * iBoot literal pools reference 0x180xxxxx addresses.
                                           * Writing to both locations avoids corrupting Thumb
                                           * code with aggressive address patching. */
                                          cpu_physical_memory_write(S5L8900_USBOTG_BASE,
                                                                   iboot_data,
                                                                   iboot_payload);
                                          fprintf(stderr, ">>> iBoot: mirrored %zu bytes to 0x%08x\n",
                                                  iboot_payload, S5L8900_USBOTG_BASE);
    
    
    
    
                                        /* Fill beyond-code region with ARM BX LR (0xE12FFF1E).
                                         * In ARM mode: BX LR returns safely to caller.
                                         * In Thumb mode: bytes 1E FF 2F E1 decode as:
                                         *   offset+0: 0xFF1E = PUSH {r0-r7,LR} (stack smash but safe)
                                         *   offset+2: 0x2FE1 = (invalid, but CPU may not reach)
                                         * Better: use pattern 70 47 70 47 (Thumb BX LR repeated).
                                         * ARM reads 0x47704770 = BXEQ r0; BXEQ r0 (no-ops if !Z).
                                         * Thumb reads 0x4770 = BX LR (safe return). */
                                        {
                                            size_t fill_sz = S5L8900_IBOOT_SIZE - iboot_payload;
                                            uint8_t *fill = g_malloc0(fill_sz);
                                            uint32_t pat = 0x47704770; /* Thumb BX LR x2, ARM BXEQ r0 x2 */
                                            for (gsize i = 0; i + 4 <= fill_sz; i += 4) {
                                                memcpy(fill + i, &pat, 4);
                                            }
                                             cpu_physical_memory_write(S5L8900_IBOOT_BASE + iboot_payload,
                                                                       fill, fill_sz);
                                             /* Also fill the mirror region */
                                             cpu_physical_memory_write(S5L8900_USBOTG_BASE + iboot_payload,
                                                                       fill, fill_sz);
                                             g_free(fill);
                                             fprintf(stderr, ">>> iBoot: filled 0x%x-0x%x with 0x47704770 pattern\n",
                                                     (unsigned)(S5L8900_IBOOT_BASE + iboot_payload),
                                                     (unsigned)(S5L8900_IBOOT_BASE + S5L8900_IBOOT_SIZE));
                                         }
    
                                        /* Verify iBoot loaded correctly */
                                        {
                                            uint8_t verify[16];
                                            cpu_physical_memory_read(S5L8900_IBOOT_BASE + 0x3ca0, verify, 16);
                                            fprintf(stderr, ">>> iBoot: verify @ 0x3ca0: %02x %02x %02x %02x ...\n",
                                                    verify[0], verify[1], verify[2], verify[3]);
                                            cpu_physical_memory_read(S5L8900_IBOOT_BASE + 0x1ede6, verify, 16);
                                            fprintf(stderr, ">>> iBoot: verify @ 0x1ede6: %02x %02x %02x %02x ...\n",
                                                    verify[0], verify[1], verify[2], verify[3]);
                                        }
    
                                         /* Patch iBEC safe loop at 0x18004980 to jump to iBoot.
                                         * The safe loop (B #-2) is reached after iBEC entry BLs
                                         * are patched to MOV r0,#1; NOP. Replace with Thumb-32
                                         * LDR r0, [pc, #offset]; BX r0 to jump to iBoot.
                                         * iBoot entry: offset 0x40 is ARM prologue, but we try
                                         * offset 0x0 first (direct payload start).
                                         * Use Thumb bit (LSB=1) for BX to enter Thumb mode. */
                                        {
                                             uint32_t iboot_entry = S5L8900_IBOOT_BASE + 0x400; /* ARM vectors */
    
                                            // Fix: constant should be at PC+8 when LDR executes.
                                            // LDR is at 0x4980, PC during exec = 0x4984.
                                            // PC+8 = 0x498C. So constant at offset 0x0C from 0x4980.
                                            // Let me recalculate with proper Thumb LDR encoding.
                                            // Actually let me use a simpler approach: write ARM instructions
                                            // at 0x4980 since the region might be entered in ARM mode.
                                            // No, the safe loop is Thumb mode. Let me use proper Thumb.
                                            //
                                            // Thumb-32: LDR r0, [PC, #imm12]
                                            // Encoding: 1111 X 100 H 1111 | imm8 0000 RnRn
                                            // For LDR r0, [pc, #8]:
                                            // First halfword: 1111 1000 1111 1101 = 0xF8FD
                                            // No wait, let me use a different approach.
                                            //
                                            // Simplest: use two Thumb-16 instructions + constant
                                            // ADR r0, const (Thumb-32) + BX r0
                                            // Or just: LDR r0, [pc, #0] (reads from pc+4+0 = pc+4)
                                            //
                                            // At 0x4980: LDR r0, [pc, #4] -> reads from 0x498C
                                            // Thumb-32 LDR r0, [pc, #4]:
                                            // 0xF8DF 0x0004 -> bytes: F8 DF 04 00
                                            // At 0x4984: BX r0 -> 00 47
                                            // At 0x4986: padding -> 00 00
                                            // At 0x4988: padding -> 00 00
                                            // At 0x498C: constant (iboot_entry)
                                            //
                                            // Wait, PC during LDR exec = instruction_addr + 4 = 0x4984.
                                            // LDR r0, [pc, #4] reads from 0x4984 + 4 = 0x4988.
                                            // So constant at 0x4988.
                                            //
                                            // Layout:
                                            // 0x4980: F8 DF 04 00 (LDR r0, [pc, #4])
                                            // 0x4984: 00 47 (BX r0)
                                            // 0x4986: 00 00 (padding)
                                            // 0x4988: iboot_entry (constant)
                                            // Total: 8 bytes from 0x4980 to 0x498F.
                                            uint8_t tramp_code[] = {
                                                0xF8, 0xDF, 0x04, 0x00,  /* LDR r0, [pc, #4] */
                                                0x00, 0x47,              /* BX r0 */
                                                0x00, 0x00               /* padding */
                                            };
                                            cpu_physical_memory_write(S5L8900_USBOTG_BASE + 0x4980,
                                                                      tramp_code, sizeof(tramp_code));
                                            cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x4980,
                                                                      tramp_code, sizeof(tramp_code));
                                            uint32_t entry_const = iboot_entry;
                                            cpu_physical_memory_write(S5L8900_USBOTG_BASE + 0x4988,
                                                                      &entry_const, 4);
                                            cpu_physical_memory_write(S5L8900_IBEC_BASE + 0x4988,
                                                                      &entry_const, 4);
                                            qemu_log_mask(LOG_UNIMP,
                                                "s5l8900: patched iBEC safe loop -> jump to iBoot at 0x%08x\n",
                                                iboot_entry);
                                         }
                                     g_free(iboot_data);
                                } else {
                                    qemu_log_mask(LOG_UNIMP,
                                        "s5l8900: could not load iBoot: %s\n",
                                        iboot_err ? iboot_err->message : "unknown");
                                    if (iboot_err) g_error_free(iboot_err);
                                }
                            }


  /* Exception vector RAM at 0x0 (SCTLR.V=0 on reset).
       * ROM copies its vector table here during BEGIN_HARDWARE_INIT.
       * Must NOT overlap with ROM alias - evec takes priority.
       * Use RAM region so CPU can actually execute the redirect code. */
     s5l8900_evec_state = g_new0(S5L8900EvecState, 1);
     memory_region_init_ram(evec, NULL, "s5l8900.evec", S5L8900_EVEC_SIZE, &error_fatal);
     memory_region_add_subregion(sysmem, S5L8900_EVEC_BASE, evec);
     /* Initial vector redirect - write directly to RAM */
     s5l8900_evec_redirect_all();

   /* Pre-patch vector RAM handled by IO write handler (s5l8900_evec_redirect_all).
      * The evec redirect function was called during evec init above. */

 /* Catch-all peripheral stub FIRST (broad 0x38000000 region).
     * All specific peripheral stubs below must come after this,
     * so they take priority at their addresses. */
    MemoryRegion *periph_catchall = g_new0(MemoryRegion, 1);
    memory_region_init_io(periph_catchall, NULL, &s5l8900_periph_catchall_ops,
                           NULL, "s5l8900.periph_catchall", S5L8900_PERIPH_SIZE);
    memory_region_add_subregion(sysmem, S5L8900_PERIPH_BASE, periph_catchall);

    /* CLOCK1 stub - stateful PLL lock status for ROM and iBSS */
     MemoryRegion *clock1 = g_new0(MemoryRegion, 1);
     S5L8900ClockState *clock1_state = g_new0(S5L8900ClockState, 1);
     memory_region_init_io(clock1, NULL, &s5l8900_clock_ops, clock1_state,
                            "s5l8900.clock1", 0x1000);
     memory_region_add_subregion(sysmem, 0x3c500000, clock1);

    /* VIC0 stub (0x38e00000) - Interrupt controller */
    memory_region_init_io(vic0_mr, NULL, &s5l8900_vic_ops, vic0,
                              "s5l8900.vic0", 0x1000);
    memory_region_add_subregion(sysmem, 0x38e00000, vic0_mr);

    /* VIC1 stub (0x38e01000) - Second interrupt controller */
    memory_region_init_io(vic1_mr, NULL, &s5l8900_vic_ops, vic1,
                              "s5l8900.vic1", 0x1000);
    memory_region_add_subregion(sysmem, 0x38e01000, vic1_mr);

    /* CLOCK0 stub - same handler as CLOCK1 (generic pass-through) */
    MemoryRegion *clock0 = g_new0(MemoryRegion, 1);
    memory_region_init_io(clock0, NULL, &s5l8900_clock_ops, NULL,
                           "s5l8900.clock0", 0x1000);
    memory_region_add_subregion(sysmem, 0x38100000, clock0);

    /* EDGEIC stub (0x38e02000) - edge interrupt controller */
    MemoryRegion *edgeic = g_new0(MemoryRegion, 1);
    memory_region_init_io(edgeic, NULL, &s5l8900_clock_ops, NULL,
                           "s5l8900.edgeic", 0x1000);
    memory_region_add_subregion(sysmem, 0x38e02000, edgeic);

    /* GPIOIC stub (0x39a00000) - GPIO interrupt controller */
    MemoryRegion *gpioic = g_new0(MemoryRegion, 1);
    memory_region_init_io(gpioic, NULL, &s5l8900_clock_ops, NULL,
                           "s5l8900.gpioic", 0x1000);
    memory_region_add_subregion(sysmem, 0x39a00000, gpioic);

    /* WDT_CTRL stub (0x3e300000) - watchdog timer */
    MemoryRegion *wdt = g_new0(MemoryRegion, 1);
    memory_region_init_io(wdt, NULL, &s5l8900_clock_ops, NULL,
                           "s5l8900.wdt", 0x1000);
    memory_region_add_subregion(sysmem, 0x3e300000, wdt);

    /* USB PHY stub (0x3c400000) - analog USB front-end */
    MemoryRegion *usphy = g_new0(MemoryRegion, 1);
    memory_region_init_io(usphy, NULL, &s5l8900_clock_ops, NULL,
                           "s5l8900.usbphy", 0x1000);
    memory_region_add_subregion(sysmem, 0x3c400000, usphy);

    /* TIMER stub (0x3e400000) - hardware timer */
    MemoryRegion *timer = g_new0(MemoryRegion, 1);
    memory_region_init_io(timer, NULL, &s5l8900_clock_ops, NULL,
                           "s5l8900.timer", 0x1000);
    memory_region_add_subregion(sysmem, 0x3e400000, timer);

    /* PMU/Sleep controller stub (0x3e500000) - iBSS polls 0x3e500004
     * for power management status. Returns 0x8 (bit 3 set) to indicate
     * power is stable/ready. */
    S5L8900PMUState *pmu_state = g_new0(S5L8900PMUState, 1);
    MemoryRegion *pmu = g_new0(MemoryRegion, 1);
    memory_region_init_io(pmu, NULL, &s5l8900_pmu_ops, pmu_state,
                           "s5l8900.pmu", 0x10000);
    memory_region_add_subregion(sysmem, 0x3e500000, pmu);

   /* NAND controller stub (0x3c300000) - handles polling accesses */
    MemoryRegion *nand = g_new0(MemoryRegion, 1);
    memory_region_init_io(nand, NULL, &s5l8900_clock_ops, NULL,
                           "s5l8900.nand", 0x10000);
    memory_region_add_subregion(sysmem, 0x3c300000, nand);

    /* Unknown controller stubs (discovered during ROM execution tracing) */
    MemoryRegion *unk1 = g_new0(MemoryRegion, 1);
    memory_region_init_io(unk1, NULL, &s5l8900_clock_ops, NULL,
                           "s5l8900.unk3c400000", 0x10000);
    memory_region_add_subregion(sysmem, 0x3c400000, unk1);

MemoryRegion *unk2 = g_new0(MemoryRegion, 1);
    memory_region_init_io(unk2, NULL, &s5l8900_clock_ops, NULL,
                            "s5l8900.unk38200000", 0x10000);
    memory_region_add_subregion(sysmem, 0x38200000, unk2);

    /* Keystore/KASLR peripheral (0x38620000) - iBEC polls 0x80 for status */
    S5L8900KeystoreState *keystore_state = g_new0(S5L8900KeystoreState, 1);
    MemoryRegion *keystore = g_new0(MemoryRegion, 1);
    memory_region_init_io(keystore, NULL, &s5l8900_keystore_ops, keystore_state,
                            "s5l8900.keystore", 0x1000);
    memory_region_add_subregion(sysmem, 0x38620000, keystore);

 /* USB OTG RAM region (0x18000000).
     * Regular RAM so iBEC code can execute from this region.
     * config_board detection moved to periodic callback. */
    S5L8900USBOTGState *usbotg_state = g_new0(S5L8900USBOTGState, 1);
    MemoryRegion *usbotg = g_new0(MemoryRegion, 1);
    memory_region_init_ram_ptr(usbotg, NULL, "s5l8900.usbotg",
                                S5L8900_USBOTG_SIZE, usbotg_state->ram);
    memory_region_add_subregion(sysmem, S5L8900_USBOTG_BASE, usbotg);

    /* High-priority read hook on the dispatch literal-pool window
     * 0x18006000..0x18006008. Overrides the USBOTG RAM for these 8 bytes so
     * every read of the dispatch table base is logged with the PC. */
     s5l8900_poolbase_ram = usbotg_state->ram;
     {
         MemoryRegion *poolbase_io = g_new0(MemoryRegion, 1);
         memory_region_init_io(poolbase_io, NULL, &s5l8900_poolbase_ops, NULL,
                               "s5l8900.poolbase", 8);
         memory_region_add_subregion(sysmem, 0x18006000, poolbase_io);
     }

     /* Write-hooks on the ARM code region AND the Thumb helper 0x18002e84-0x18002ee0
      * to catch the clobbering writer (iBoot inits write data into code). The base
      * address is passed as opaque so each region forwards to the right RAM offset. */
     {
         MemoryRegion *codehook = g_new0(MemoryRegion, 1);
         memory_region_init_io(codehook, NULL, &s5l8900_codehook_ops,
                               (void *)(uintptr_t)S5L8900_CODEHOOK_BASE,
                               "s5l8900.codehook", S5L8900_CODEHOOK_SIZE);
         memory_region_add_subregion_overlap(sysmem, S5L8900_CODEHOOK_BASE, codehook, 100);
         fprintf(stderr, ">>> CODE HOOK at 0x%08x-0x%08x (ARM code-region writer tracker)\n",
                 S5L8900_CODEHOOK_BASE, S5L8900_CODEHOOK_BASE + S5L8900_CODEHOOK_SIZE);
     }
     {
         MemoryRegion *thhook = g_new0(MemoryRegion, 1);
         memory_region_init_io(thhook, NULL, &s5l8900_codehook_ops,
                               (void *)(uintptr_t)0x18002e80,
                               "s5l8900.thhook", 0x80);
         memory_region_add_subregion_overlap(sysmem, 0x18002e80, thhook, 100);
         fprintf(stderr, ">>> THUMB HOOK at 0x18002e80-0x18002f00 (Thumb code-region writer tracker)\n");
     }

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

    /* USB CTRL stub */
    S5L8900USBCTRLState *usbctrl_state = g_new0(S5L8900USBCTRLState, 1);
    MemoryRegion *usbctrl = g_new0(MemoryRegion, 1);
memory_region_init_io(usbctrl, NULL, &s5l8900_usbctrl_ops, usbctrl_state,
                            "s5l8900.usbctrl", 0x10000);
                    memory_region_add_subregion(sysmem, 0x38400000, usbctrl);

  /* UART at 0xE0002000 - outputs to QEMU serial port.
       * Use overlap with high priority to beat upper_ram (0x40000000-0xFFFFFFFF). */
      {
         MemoryRegion *uart = g_new0(MemoryRegion, 1);
          s5l8900_serial_chr = serial_hd(0);
          memory_region_init_io(uart, NULL, &s5l8900_uart_ops, s5l8900_serial_chr,
                                   "s5l8900.uart", S5L8900_UART_SIZE);
          memory_region_add_subregion_overlap(sysmem, S5L8900_UART_BASE, uart, 100);
          fprintf(stderr, ">>> UART stub at 0x%08x -> serial port (priority 100)\n", S5L8900_UART_BASE);
      }

      /* UART proxy at 0xE0003000: simpler IO region for guest CPU writes.
       * Every byte write goes to serial. Works without MMU. */
      {
         MemoryRegion *uartproxy = g_new0(MemoryRegion, 1);
          memory_region_init_io(uartproxy, NULL, &s5l8900_uartproxy_ops,
                                  s5l8900_serial_chr, "s5l8900.uartproxy", 0x1000);
          memory_region_add_subregion_overlap(sysmem, 0xE0003000, uartproxy, 100);
          fprintf(stderr, ">>> UART proxy at 0xE0003000 -> serial port (priority 100)\n");
      }

    dma_state->vic0 = vic0;
    dma_state->usbctrl = usbctrl_state;
    dma_state->deferred_irq = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                             s5l8900_deferred_irq_cb, dma_state);

    /* Periodic CPU state dump timer */
    s5l8900_periodic_timer = timer_new_ns(QEMU_CLOCK_REALTIME,
                                             s5l8900_periodic_dump_cb, NULL);
    timer_mod(s5l8900_periodic_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + 100ULL * 1000 * 1000);

    /* Immediate post-redirect tracing timer */
    s5l8900_step_timer = timer_new_ns(QEMU_CLOCK_REALTIME,
                                        s5l8900_step_trace_cb, NULL);

    /* Fine-grained (2us) dispatch-pool watcher. Armed from init so it is
     * running well before the iBoot init relocation (which rewrites the
     * dispatch table base in the 0x18005FD0 literal pool) and can catch the
     * writer PC within a few instructions of the store. */
    s5l8900_poolwatch_timer = timer_new_ns(QEMU_CLOCK_REALTIME,
                                            s5l8900_poolwatch_cb, NULL);
    s5l8900_poolwatch_armed = 1;
    s5l8900_poolwatch_prev_valid = 0;
    s5l8900_poolwatch_changes = 0;
    timer_mod(s5l8900_poolwatch_timer,
              qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + 2ULL * 1000);

   /* Catch and log all peripheral accesses we haven't implemented yet */
    /* TEMPORARILY DISABLED for debugging */
    // create_unimplemented_device("s5l8900.periph",
    //                              S5L8900_PERIPH_BASE, S5L8900_PERIPH_SIZE);
}

static void s5l8900_machine_init(MachineClass *mc)
{
    mc->desc             = "Apple iPod Touch 1G (S5L8900)";
    mc->init             = s5l8900_init;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("arm1176");
    mc->default_ram_size = S5L8900_RAM_SIZE;
}

DEFINE_MACHINE_ARM("s5l8900", s5l8900_machine_init)
