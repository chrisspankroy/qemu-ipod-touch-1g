#ifndef HW_ARM_XNU_MEM_H
#define HW_ARM_XNU_MEM_H

#include "qemu-common.h"
#include "hw/arm/boot.h"
#include "target/arm/cpu.h"

extern uint32_t g_virt_base;
extern uint32_t g_phys_base;

uint32_t align_64k_low(uint32_t addr);
uint32_t align_64k_high(uint32_t addr);

uint8_t get_highest_different_bit_index(uint32_t addr1, uint32_t addr2);
uint8_t get_lowest_non_zero_bit_index(uint32_t addr);
uint32_t get_low_bits_mask_for_bit_index(uint8_t bit_index);

uint32_t vtop_bases(uint32_t va, uint32_t phys_base, uint32_t virt_base);

void allocate_ram(MemoryRegion *top, const char *name, uint32_t addr, uint32_t size);

#endif