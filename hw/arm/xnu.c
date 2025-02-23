#include "qemu/osdep.h"
#include "qemu-common.h"
#include "hw/arm/xnu.h"
#include "hw/arm/xnu_mem.h"

static void allocate_and_copy(MemoryRegion *mem, AddressSpace *as, const char *name, uint32_t pa, uint32_t size, void *buf)
{
    if (mem) {
        allocate_ram(mem, name, pa, align_64k_high(size));
    }
    address_space_rw(as, pa, MEMTXATTRS_UNSPECIFIED, (uint8_t *)buf, size, 1);
}

void arm_load_securerom(char *filename, AddressSpace *as, MemoryRegion *mem,
                    const char *name, uint32_t load_addr)
{
    uint8_t *data = NULL;
    gsize len;

    if (!g_file_get_contents(filename, (char **)&data, &len, NULL)) {
        printf("Could not get content of SecureROM file: %s\n", filename);
        abort();
    }

    printf("Loaded SecureROM file: %s (size: %llu bytes)\n", filename, len);

    allocate_and_copy(mem, as, name, load_addr, align_64k_high(len), data);

    printf("SecureROM loaded into memory at: 0x%08x\n", load_addr);

    if (data) {
        g_free(data);
    }
}
