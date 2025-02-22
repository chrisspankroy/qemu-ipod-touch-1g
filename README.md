# QEMU for iPod Touch (1st gen)
This is QEMU forked from upstream commit `89f3bfa3265554d1d591ee4d7f1197b6e3397e84`

It will emulate the 1st-generation iPod Touch. It is heavily based on `devos50`'s work in `https://devos50.github.io/blog/2022/ipod-touch-qemu/`. I am simply trying to follow along as a learning project

## Building

	mkdir build
	cd build
	CFLAGS="-Wno-error" ../configure --target-list=arm-softmmu
	make -j16

## Running

	./arm-softmmu/qemu-system-arm -M iPod-Touch -serial mon:stdio -cpu max -m 1G -d unimp

## Getting raw kernelcache

	# strip 8900 file header from kernelcache (first 2048 bytes)
	dd if=kernelcache.release.s5l8900xrb of=kernelcache.lzss.enc bs=512 skip=4 conv=sync

	# decrypt (using 0x837 key with no IV, thx iphone wiki)
	openssl enc -d -in kernelcache.lzss.enc -out kernelcache.lzss -aes-128-cbc -K 188458a6d15034dfe386f23b61d43774 -iv 0

	# decompress (skipping 0x180 bytes to 0xfeedface header)
	lzssdec -o 384 < kernelcache.lzss > kernelcache
	
## `pwndbg-lldb` cheatsheet

	pwndbg-lldb path/to/binary
	run -- arg1 arg2
	context // redraw screen

## Helpful binaries

	xpwntool
	lzssdec
