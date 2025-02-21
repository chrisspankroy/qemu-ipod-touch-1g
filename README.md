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

## `pwndbg-lldb` cheatsheet

	pwndbg-lldb path/to/binary
	run -- arg1 arg2
	context // redraw screen
