/*
 * Copyright (c) 2019 Jonathan Afek <jonyafek@me.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu-common.h"
#include "hw/arm/boot.h"
#include "exec/address-spaces.h"
#include "hw/misc/unimp.h"
#include "sysemu/sysemu.h"
#include "qemu/error-report.h"
#include "hw/platform-bus.h"
#include "exec/memory.h"
#include "qemu-common.h"
#include "hw/boards.h"
#include "hw/arm/boot.h"
#include "cpu.h"
#include "hw/arm/xnu_mem.h"

uint32_t g_virt_base = 0;
uint32_t g_phys_base = 0;

uint32_t vtop_bases(uint32_t va, uint32_t phys_base, uint32_t virt_base)
{
    if ((0 == virt_base) || (0 == phys_base)) {
        abort();
    }
    return va - virt_base + phys_base;
}

uint8_t get_highest_different_bit_index(uint32_t addr1, uint32_t addr2)
{
    if ((addr1 == addr2) || (0 == addr1) || (0 == addr2)) {
        printf("get_highest_different_bit_index - precondition failed\n");
        abort();
    }
    return (64 - __builtin_clzll(addr1 ^ addr2));
}

uint32_t align_64k_low(uint32_t addr)
{
    return addr & ~0xffffull;
}

uint32_t align_64k_high(uint32_t addr)
{
    return (addr + 0xffffull) & ~0xffffull;
}

uint8_t get_lowest_non_zero_bit_index(uint32_t addr)
{
    if (0 == addr) {
        printf("get_lowest_non_zero_bit_index - address must be non-zero\n");
        abort();
    }
    return __builtin_ctzll(addr);
}

uint32_t get_low_bits_mask_for_bit_index(uint8_t bit_index)
{
    if (bit_index >= 64) {
        printf("get_low_bits_mask_for_bit_index - bit index should not exceed 64\n");
        abort();
    }
    return (1 << bit_index) - 1;
}

void allocate_ram(MemoryRegion *top, const char *name, uint32_t addr, uint32_t size)
{
        MemoryRegion *sec = g_new(MemoryRegion, 1);
        memory_region_init_ram(sec, NULL, name, size, &error_fatal);
        memory_region_add_subregion(top, addr, sec);
}