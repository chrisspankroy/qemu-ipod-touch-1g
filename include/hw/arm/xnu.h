#ifndef HW_ARM_XNU_H
#define HW_ARM_XNU_H

#include "qemu-common.h"

#define	LC_SEGMENT	0x1	/* segment of this file to be mapped */
#define LC_UNIXTHREAD   0x5

struct segment_command { /* for 32-bit architectures */
	uint32_t	cmd;		/* LC_SEGMENT */
	uint32_t	cmdsize;	/* includes sizeof section structs */
	char		segname[16];	/* segment name */
	uint32_t	vmaddr;		/* memory address of this segment */
	uint32_t	vmsize;		/* memory size of this segment */
	uint32_t	fileoff;	/* file offset of this segment */
	uint32_t	filesize;	/* amount to map from the file */
	uint32_t	maxprot;	/* maximum VM protection */
	uint32_t	initprot;	/* initial VM protection */
	uint32_t	nsects;		/* number of sections in segment */
	uint32_t	flags;		/* flags */
};

struct load_command {
	uint32_t cmd;		/* type of load command */
	uint32_t cmdsize;	/* total size of command in bytes */
};

struct mach_header {
	uint32_t	magic;		/* mach magic number identifier */
	uint32_t	cputype;	/* cpu specifier */
	uint32_t	cpusubtype;	/* machine specifier */
	uint32_t	filetype;	/* type of file */
	uint32_t	ncmds;		/* number of load commands */
	uint32_t	sizeofcmds;	/* the size of all the load commands */
	uint32_t	flags;		/* flags */
};

void macho_file_highest_lowest_base(const char *filename, uint32_t phys_base, uint32_t *virt_base, uint32_t *lowest, uint32_t *highest);
void arm_load_securerom(char *filename, AddressSpace *as, MemoryRegion *mem, const char *name, uint32_t load_addr);

#endif
