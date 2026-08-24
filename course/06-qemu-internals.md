# Lesson 06: QEMU Internals

**Goal:** Understand just enough of QEMU's architecture to write a new machine
model — which is exactly what lesson 07 does. You don't need to know all of
QEMU; you need the machine-model API, the memory model, the timer API, and
the logging/GDB toolbox.

**Prerequisites:** Lessons 01–05. C comfort.

**In-tree references** (read while doing this lesson):
- `hw/arm/s5l8900.c` — our deliverable (read it *after* reading this lesson)
- `hw/arm/vexpress.c` or `hw/arm/integratorcp.c` — stock ARM machines, good
  patterns to copy
- `include/hw/boards.h` — `MachineState`/`MachineClass`
- `hw/core/machine.c` — what happens at `-M` time
- `CONCEPTS.md` section 12 — the project's own QEMU-internals summary

---

## 1. What QEMU actually is

QEMU is a **system emulator**: it makes a whole machine exist in a process.
Three cooperating pieces:

1. **TCG (Tiny Code Generator)** — a JIT. It takes the guest's ARM basic
   blocks and compiles them to host machine code, which it then runs. So
   "emulating" a CPU here means *translating* its instructions, not
   interpreting them. Consequence: guest code can run very fast, and it can
   also get *stuck* in a hot loop that never returns control to QEMU's main
   loop (a "translation block" that never exits). You'll meet this in lesson
   08; it's why our timers use the realtime clock.
2. **The main loop** — QEMU's event dispatcher (GLib-based). It runs host
   callbacks: timer expirations, chardev I/O, GDB commands, HMP monitor
   commands. The vCPU runs as a thread; when guest code touches a device,
   execution of the translated block exits and the device callback runs in
   the main loop.
3. **Devices** — C objects that register **memory regions** (address ranges
   with read/write callbacks). When translated guest code does `str r0,
   [r1]` and `r1` hits a device region, QEMU calls your callback. **This is
   the entire device-model API: read callbacks, write callbacks, memory
   regions.** Everything else in this project is built from that.

## 2. The machine model

A "machine" in QEMU = a `MachineClass` plus an init function. The whole
registration is ~10 lines at the bottom of `s5l8900.c`:

```c
static void s5l8900_machine_init(MachineClass *mc)
{
    mc->desc             = "Apple iPod Touch 1G (S5L8900)";
    mc->init             = s5l8900_init;              /* the real work */
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("arm1176");
    mc->default_ram_size = S5L8900_RAM_SIZE;
}

DEFINE_MACHINE_ARM("s5l8900", s5l8900_machine_init)
```

When you run `-M s5l8900`, QEMU calls `s5l8900_init(machine)`. That one
function is the machine: create the CPU, wire up memory, attach devices,
load firmware, set the reset state. **Everything in this project lives
inside (or is called from) `s5l8900_init`.**

### Making the file build

A new machine needs three wiring points (all done in this repo already —
find them to verify):

1. `hw/arm/Kconfig` (line ~244):
   ```
   config S5L8900
       bool
   ```
2. `configs/devices/arm-softmmu/default.mak` (line ~26):
   ```
   CONFIG_S5L8900=y
   ```
3. `hw/arm/meson.build` (line ~109):
   ```
   arm_common_ss.add(when: 'CONFIG_S5L8900', if_true: files('s5l8900.c'))
   ```

Then `ninja -C build` picks it up. If your build doesn't see your machine,
you broke one of these three lines.

## 3. The memory model

QEMU's physical address space is a tree of **MemoryRegions** rooted at
`get_system_memory()`. You create regions and attach them at addresses:

```c
MemoryRegion *sysmem = get_system_memory();

/* RAM: QEMU allocates real host memory backing this range */
MemoryRegion *ram = g_new0(MemoryRegion, 1);
memory_region_init_ram(ram, NULL, "s5l8900.ram", S5L8900_RAM_SIZE, &error_fatal);
memory_region_add_subregion(sysmem, S5L8900_RAM_BASE, ram);
```

Key rules you will live and die by:

- **A guest access to an address with no region is a fault** — the guest CPU
  takes a data abort. (On real silicon, unmapped addresses might return
  garbage or hang; in QEMU they're hard faults.) This is why we need a
  catch-all (below), and why *missing the evec region at `0x0`* produces an
  instant abort spiral (lesson 03: the ROM copies its vector table to `0x0`).
- **Overlapping regions: the one added last (or with higher priority) wins.**
  `memory_region_add_subregion_overlap(sysmem, addr, mr, priority)` lets you
  put a specific device *on top of* a broad region. Our catch-all is added
  with priority 0 over the *entire* 4 GB space, so every specific device
  (priority ≥ 1) automatically shadows it. Get the order wrong and your
  UART disappears behind the catch-all.
- **IO regions don't consume RAM.** `memory_region_init_io(mr, NULL, &ops,
  opaque, name, size)` creates a range whose accesses call your callbacks
  instead of touching memory.

### MemoryRegionOps — the device API

```c
static uint64_t s5l8900_uart_read(void *opaque, hwaddr offset,
                                  unsigned size)
{
    return 0;  /* console is output-only in this project */
}

static void s5l8900_uart_write(void *opaque, hwaddr offset,
                               uint64_t value, unsigned size)
{
    char chr = value & 0xff;
    if (s5l8900_serial_chr) {
        qemu_chr_write(s5l8900_serial_chr, &chr, 1);
    }
}

static const MemoryRegionOps s5l8900_uart_ops = {
    .read = s5l8900_uart_read,
    .write = s5l8900_uart_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};
```

Notes:
- `opaque` is whatever pointer you passed to `memory_region_init_io` — your
  device state struct.
- `offset` is relative to the region's base (a guest write to `0xE0002000`
  with a region at `0xE0002000` arrives as `offset = 0`).
- `size` is the access width (1/2/4).
- The callback runs in the main loop context — you may not call arbitrary
  vCPU APIs from it; defer with timers/AIO (lesson 08 shows the pattern).

## 4. Loading firmware

QEMU's standard paths are `-bios` → `machine->firmware` and `-kernel` →
`machine->kernel_filename`. The standard ROM machinery (`rom_add_file`) is
designed for simple cases; **this project deliberately reads the files
itself** with `g_file_get_contents()` inside `s5l8900_init` and writes the
bytes into memory with `cpu_physical_memory_write()`. Why? Because the real
loading involves header-skipping (0x800 for .dfu), decryption (iBoot),
staging at multiple addresses, and patching — all much easier when *you*
hold the buffer.

```c
gsize sz; guint8 *buf; GError *err = NULL;
if (g_file_get_contents(machine->kernel_filename,
                        (gchar **)&buf, &sz, &err)) {
    /* skip the 0x800 img2 wrapper header, load the payload */
    cpu_physical_memory_write(S5L8900_IBSS_BASE,
                              buf + IMG2_HDR_SIZE, sz - IMG2_HDR_SIZE);
    g_free(buf);
}
```

`cpu_physical_memory_write(addr, buf, len)` is your hammer for "put these
bytes at this physical address, right now, regardless of any device
mapping." You'll use it constantly: to load firmware, to write page tables,
to hand-encode exception handlers into SRAM, to plant preseeded input.

## 5. Timers

```c
QEMUTimer *t = timer_new_ns(QEMU_CLOCK_REALTIME, my_cb, my_state);
timer_mod(t, qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + 100 * 1000 * 1000);
/* later: timer_del(t) to cancel */
```

- `QEMU_CLOCK_VIRTUAL` advances only while the vCPU is running. **If the
  guest is parked in an infinite `b .` loop that TCG never yields from,
  virtual time freezes** — a virtual timer will never fire.
- `QEMU_CLOCK_REALTIME` is wall-clock. It fires no matter what the guest is
  doing. **This is why every watchdog/polling timer in `s5l8900.c` uses
  REALTIME** — the project switched after virtual timers stopped firing
  exactly when the guest was stuck (which is precisely when you need them).

The pattern used throughout the project: a 100 ms REALTIME timer callback
that inspects the vCPU's state ("where is the PC? what is it stuck on?") and
acts — triggering the iBSS→iBoot handoff, dumping registers, redirecting
execution. That timer is the project's *conscience*.

## 6. Touching the guest CPU from C

You'll want to read and write guest CPU state (PC, CPSR, CP15). The pieces:

```c
ARMCPU *cpu = ARM_CPU(cpu_create(machine->cpu_type));   /* create */
CPUState *cs = CPU(cpu);

/* Read PC: */
uint64_t pc = cpu->env.regs[15];

/* Force a redirect: */
cpu->env.regs[15] = new_pc;
queue_tb_flush(cs, new_pc, 0);      /* drop translation cache for that PC */
cpu_interrupt(cs, CPU_INTERRUPT_EXITTB);  /* make the vCPU re-fetch */
```

(`queue_tb_flush` + `CPU_INTERRUPT_EXITTB` is the safe, main-loop-friendly
way to say "the code at that address changed, stop running the old version."
The project learned — the hard way, per `progress.md` — that calling
`tb_flush` variants from the wrong context crashes QEMU.)

For CP15 (MMU enable, TTBR0, …) see lesson 09 — the fields live in
`cpu->env.cp15.*` and `arm_rebuild_hflags(cpu)` must be called after
touching them.

## 7. Logging — the observability toolbox

You will *live* in logs. Know each tool:

| Tool | What it gives you |
|---|---|
| `fprintf(stderr, ">>> ...\n")` in your device code | your own trace lines; the project prefixes them `>>>` and flushes |
| `qemu_log_mask(LOG_UNIMP, "...")` | printed when `-d unimp` is active |
| `-d unimp -D /tmp/q.log` | every access to `unimplemented`/catch-all regions, logged — your roadmap for "what peripheral is the ROM poking that I haven't built?" |
| `-d in_asm -D /tmp/t.log` | **every translated basic block** with the guest disassembly — the ground truth for "what is executing" |
| `-d guest_errors -D /tmp/e.log` | guest exceptions (aborts, undef) with register dumps |
| `-singlestep` | translate one instruction at a time (very slow; for pinpoint work) |
| GDB (`-S -gdb tcp::1234`) | interactive: registers, memory, breakpoints (lesson 08) |

`-d` flags are comma-separated; `-D file` redirects them (otherwise stderr).

## 8. GDB integration

```sh
./build/qemu-system-arm -M s5l8900 ... -S -gdb tcp::1234
gdb
(gdb) set architecture arm
(gdb) target remote :1234
```

- `-S` starts the vCPU **halted**, so GDB owns the CPU until `continue`.
- `hbreak *0x200000c4` — hardware breakpoint (required for ROM code: the
  memory is read-only in the guest's eyes, so GDB can't plant a software
  trap).
- `info registers`, `x/20i $pc`, `p/x $cpsr`, `x/xw 0x2202bff8`,
  `set {int}0x3c500040 = 1` (write a guest memory word — *this* is how you
  test the hypothesis "does the guest unblock if this register has this
  value?").

`work/run.sh` automates the whole launch+attach; read it.

## 9. Before you write the machine: watch a machine boot

Do this exercise *before* lesson 07 so the in_asm log stops being scary:

```sh
# Build and run a stock ARM machine with logging
ninja -C build qemu-system-arm
./build/qemu-system-arm -M virt -nographic \
  -d in_asm,unimp,guest_errors -D /tmp/virt_trace.log &
sleep 3; kill %1
head -40 /tmp/virt_trace.log
```

Look at a few lines. Each in_asm entry is: the guest PC range, then the
disassembled instructions QEMU just translated. Find a block, read its
instructions, and confirm you can tell what it does (with your lesson 03
skills). Then in a second GDB session, watch the vCPU's PC walk through the
same blocks.

**Checkpoint:** You can answer:
1. Which function does QEMU call when `-M s5l8900` is given, and what are
   the five things that function must do?
2. Why does a guest store to an unmapped address produce a guest exception
   rather than just "doing nothing"?
3. Why are the project's timers REALTIME rather than VIRTUAL?
4. What does `-d in_asm` log, one line at a time?
5. Why `hbreak`, not `break`, for code in ROM?

Next: [Lesson 07: Build The Machine](07-machine-model.md)
