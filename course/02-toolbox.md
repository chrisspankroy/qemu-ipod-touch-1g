# Lesson 02: The Toolbox

**Goal:** Set up a working environment — QEMU that builds, GDB that attaches,
a disassembler you can operate, and Python+capstone one-liners — and learn
the address↔file-offset arithmetic that will appear constantly.

**Prerequisites:** a machine with Homebrew (macOS) or apt (Linux), git, and a
C compiler toolchain.

---

## 1. Install the basics

macOS:

```sh
brew install ninja meson glib pixman gdb python3 pkg-config
```

Linux (Debian/Ubuntu):

```sh
sudo apt install ninja-build meson libglib2.0-dev libpixman-1-dev \
  gdb python3 python3-pip
pip3 install --user capstone
```

Verify:

```sh
ninja --version && meson --version && gdb --version | head -1
python3 -c "import capstone; print(capstone.__version__)"
```

## 2. Build QEMU

This repo *is* a QEMU fork, already built into `build/`. Verify the existing
build works:

```sh
ninja -C build qemu-system-arm
./build/qemu-system-arm -M s5l8900 -machine help 2>&1 | head -5
```

If you ever start over from stock QEMU, the recipe is:

```sh
git clone https://github.com/qemu/qemu.git
cd qemu
mkdir build && cd build
../configure --target-list=arm-softmmu
ninja
```

That's the whole emulator we're modifying. When in doubt, upstream docs are
good: `docs/` inside the tree.

**Checkpoint A:** `ninja -C build` succeeds and `qemu-system-arm` runs.

## 3. The debug runner

`work/run.sh` is the canonical way to launch with a debugger. Read it
top to bottom — it's 55 lines and teaches you the whole pattern:

- builds,
- kills stale QEMU,
- launches `qemu-system-arm` with `-S -gdb tcp::1234` (halted, waiting for
  GDB) and `-d unimp,guest_errors` (log unimplemented-device hits and guest
  errors to a file),
- waits for the GDB port to open,
- attaches GDB with `set architecture arm`.

```sh
sh work/run.sh
```

**Checkpoint B:** GDB attaches. Run `info registers` — you should see an ARM
register set with PC at a reset vector address. `quit` to exit.

GDB notes for ARM:

- `info registers` shows r0–r15 **and** `$cpsr`.
- `x/20i $pc` disassembles 20 instructions from PC.
- `hbreak *0x200000c4` — *hardware* breakpoint. Use `hbreak`, not `break`,
  for code in ROM (read-only memory): GDB can't write a trap byte there.
- `p/x $cpsr`, `x/xw 0x2202bff8` (read a word), `set {int}0x38000000 = 0`
  (write a word) all work.

## 4. Pick a disassembler

You need a GUI disassembler for the "look at the whole function" tasks.
Options, all acceptable:

### Hopper (used for this project)

`work/ROM BOOT, S5L8900 Rev.hop` is an existing Hopper project — open it and
you'll see the ROM already annotated (that annotation was produced by
`work/annotate_rom_full.py`). To open a *raw binary* yourself:

1. File → New, choose "From file", pick `work/ROM BOOT, S5L8900 Rev.2`.
2. Architecture: **ARM**, little-endian, no symbols, entry point `0x20000000`.
3. Select the whole thing, `⌘N` (disassemble) — or let Hopper auto-decode.

Key Hopper skills (these map 1:1 to the RE tasks ahead):
- double-click an address to set a name (label). **Name everything you
  understand** — your future self depends on it.
- `⌘G` / "Go to address".
- Right-click → "Show xrefs to" / "Show xrefs from" — the single most used
  feature in RE.
- The pseudocode view (⌘⇧P) is a fast way to *get the idea*, but always drop
  to the assembly to verify — pseudocode lies about control flow and about
  MMIO side effects.

### Ghidra (free)

File → Import, "Binary file", processor **ARM:LE:32:v5:default**. Set the
image base to `0x20000000` for the ROM. Analysis → "Analyze" — then use
Cross References (X for "xrefs to", Shift+X for "xrefs from").

### radare2 (free, CLI)

```sh
r2 -a arm -b 32 -B 0x20000000 "work/ROM BOOT, S5L8900 Rev.2"
:aaa        # analyze
pdf @ 0x200000c4   # disassemble function at address
axt @ 0x200007a8   # xrefs to
```

### capstone (free, scriptable — the workhorse)

For scripting — scanning for patterns, batch-disassembling ranges, resolving
targets — use capstone. Save this as `work/dis.py`; you'll reuse it all course:

```python
#!/usr/bin/env python3
"""dis.py [thumb|arm] <base_addr_hex> <file_offset_hex> <count>
Disassemble a range of a raw file at a given link base."""
import sys
import capstone

mode = capstone.CS_MODE_THUMB if sys.argv[1] == 'thumb' else capstone.CS_MODE_ARM
base = int(sys.argv[2], 16)
off  = int(sys.argv[3], 16)
n    = int(sys.argv[4], 16) if len(sys.argv) > 4 else 0x40
data = open(sys.argv[5] if len(sys.argv) > 5 else 'work/iBoot.decrypted',
            'rb').read()
md = capstone.Cs(capstone.CS_ARCH_ARM, mode)
md.detail = True
for i in md.disasm(data[off:off+n], base + off):
    print(f"0x{i.address:08x}: {i.mnemonic:8s} {i.op_str}")
```

Example — the iBoot reset handler (file offset 0x400 + 0x440, ARM mode):

```sh
python3 work/dis.py arm 0x18000000 0x840 0x20
```

**Checkpoint C:** You can open the SecureROM in your disassembler of choice,
navigate to `0x20000000`, and see `b 0x200000c4` as the first instruction.

## 5. Hex inspection

```sh
xxd work/iBoot.decrypted | head          # first 256 bytes
xxd -s 0x400 -l 64 work/iBoot.decrypted  # first 64 bytes of the payload
```

What you should see at offset 0: `32 67 6d 49` = "IMG2", then `74 6f 62 69`
= "ibot" (the image type tag). Lesson 04 unpacks this.

## 6. The arithmetic that never goes away

Firmware files are not loaded at their file offset. Every image has a **link
base** (the virtual/physical address it expects to run at) and usually a
**header** (bytes before the actual code). The two conversions you will use
hundreds of times:

### iBoot (`work/iBoot.decrypted`, 140288 bytes)

```
file offset f  =  link address L - 0x18000000 + 0x400
link address L =  0x18000000 + f - 0x400
```

The `0x400` is the img2 header size for this image. So the instruction at
link `0x1800465C` (iBoot's `putchar`) lives at file offset
`0x465C + 0x400 = 0x4A5C`.
**Checkpoint D:** verify with xxd: `xxd -s 0x4A5C -l 8 work/iBoot.decrypted`
should show real Thumb code (the `putchar` body), well past the `0x400`
header. (Note the `+0x400`: link and file offsets for iBoot *differ* by the
header size. This is the single most-recurring source of off-by-`0x400`
confusion in the whole project — see lesson 10.)

### iBSS (at runtime, self-copied to SRAM at `0x22000000`)

```
file offset f  =  runtime SRAM address R - 0x22000000 + 0x800
```

The `0x800` is the `.dfu`/img2 header size in `iBSS.n45ap.RELEASE.dfu`. So
iBSS's halt loop at runtime `0x22001360` is at file offset `0x1360 + 0x800 =
0x1B60` in the .dfu file. (Use this to confirm a "halt" address is genuine
iBSS code and not something we wrote — a skill used in lesson 09.)

### SecureROM

No header: file offset = address − `0x20000000`. Simple.

Write these three formulas in your notes with a worked example for each.
Every "which file offset does this runtime address map to?" question in the
rest of the course is one of these three lines.

## 7. Git hygiene

Before you start modifying QEMU, make sure you have a branch to play in:

```sh
git status        # see what's dirty (build/ and work/ churn is normal)
git branch course 2>/dev/null || git checkout -b course
```

Commit early, commit often, with messages that say *what you learned*, not
just what you changed. Example: "s5l8900: clock stub must return 0xFFFFFFFF
at offset 0x40 — iBSS polls bit 3 for PLL lock."

---

## Pitfalls

- **GDB says "no symbol" / wrong architecture.** Always `set architecture arm`
  right after `target remote` (run.sh does this for you).
- **Hopper decodes garbage at the top of a file.** You probably gave it the
  wrong base address or endianness, or you're looking at the header region.
  For the ROM the code starts at file offset 0; for iBoot it starts at 0x400.
- **capstone silently stops mid-way.** A 16-bit Thumb boundary or an undefined
  instruction ends the disassembly. That's a feature: it often points at data
  or a mode switch.
- **You forgot the header offset** and your xrefs are off by 0x400/0x800.
  This is the most common beginner error in the whole course. If an xref
  lands in the middle of a string, check the header arithmetic.

## Exercises

1. Using `work/dis.py`, disassemble the first 0x20 bytes of the SecureROM at
   `0x20000000` (file offset 0). What is the first instruction? What address
   does it jump to?
2. Find the iBSS halt loop: disassemble 16 bytes of
   `iBSS.n45ap.RELEASE.dfu` at file offset `0x1B60`. Do you see a branch to
   self (`b .`)?
3. In your disassembler, set a name `reset_vector` at `0x20000000` in the ROM.
   What does the instruction at `0x200000c4` do? (Don't worry if you can't
   fully decode it yet — lesson 05 walks it line by line.)

Next: [Lesson 03: ARM From Zero](03-arm-basics.md)
