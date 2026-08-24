# Lesson 08: The Debug Loop

**Goal:** Install the *method*. Everything else in this course is specific to
the S5L8900; this lesson is the transferable skill. When firmware does
something you didn't expect — hangs, faults, corrupts memory — you will have
a repeatable procedure to find *why*, not just *where*.

**Prerequisites:** Lessons 03–07 (you have a machine that boots iBSS).

---

## 1. The loop

Every single problem in this project was solved by the same five steps. Do
not skip steps. Do not reorder them.

```
        ┌─────────────┐
        │   1. RUN     │  get it to fail, reproducibly
        └──────┬──────┘
        ┌──────▼──────┐
        │  2. OBSERVE  │  where is it? what did it just do?
        └──────┬──────┘
        ┌──────▼──────┐
        │ 3. HYPOTHESIZE│ "it hangs because the stub returns the wrong value"
        └──────┬──────┘
        ┌──────▼──────┐
        │ 4. INSTRUMENT│  add ONE probe that confirms/refutes the hypothesis
        └──────┬──────┘
        ┌──────▼──────┐
        │   5. FIX     │  make the smallest change; rebuild; re-run; verify
        └─────────────┘
```

The two most common failure modes are: **fixing without a confirmed
hypothesis** (random patching — you'll "fix" it and break something else two
steps later), and **not rebuilding** (the #1 false lead in this whole
project; see Pitfalls).

## 2. Observability: what QEMU gives you for free

| Flag / tool | What you see | When |
|---|---|---|
| `-d in_asm -D /tmp/t.log` | every translated basic block + guest disasm | "what is executing, in order" |
| `-d unimp -D /tmp/u.log` | every catch-all/stub access | "what peripheral is it poking?" |
| `-d guest_errors -D /tmp/e.log` | guest exceptions with registers | "did it fault, and with what state?" |
| your own `fprintf(stderr, ">>> ...")` + `fflush` | device-side ground truth | "what did my stub do?" |
| the periodic timer (lesson 07) | PC + mode + CPSR every 100 ms | "where is it stuck?" |

Reading an `in_asm` line: it shows a guest PC range and the instructions in
that block. Cross-reference with the file (lesson 04's offset formulas) to
know *which firmware function* that block belongs to.

**Rule: one log at a time.** `in_asm` for a busy guest produces megabytes in
a second. Run with a timeout, then `grep` the log for the window you care
about:

```sh
timeout 5 ./build/qemu-system-arm -M s5l8900 -bios "$ROM" -kernel "$IBSS" \
  -nographic -d in_asm,unimp -D /tmp/trace.log 2>/tmp/stderr.log
grep -n "2200136\|PERIODIC\|CLOCK1" /tmp/trace.log /tmp/stderr.log | head
```

## 3. Instrumentation: probes you add

Three probes did the heavy lifting in this project. Learn all three.

### P1. PC sampling (stuck-PC detection)

The periodic timer already gives you a PC every 100 ms. The skill is in
*reading* it:

- **PC bouncing in a small range** (`0x22001358`–`0x22001364`) = a polling
  loop. Disassemble that range; find the condition it's waiting on; the
  condition is a register you control.
- **PC perfectly still** = `b .` halt (intentional park) or a taken-forever
  loop.
- **PC walking downward / to wild addresses** = stack corruption or a bad
  literal.

### P2. The step timer (immediate post-event tracing)

A one-shot REALTIME timer fired a few ms after a known event (e.g. "right
after we redirected to iBoot"). It dumps PC + bytes at PC for N consecutive
fires. Use it to watch the first few dozen instructions after a jump, when
the periodic 100 ms cadence is far too coarse.

### P3. The write-watcher (find the culprit)

When a value in guest memory changes and you don't know *who* wrote it:
poll the address on a fast timer (this project used ~2–3 µs). When the value
changes, immediately read the vCPU's PC — that's your writer. The project
used this to catch iBoot's init relocating a dispatch-table literal pool,
pinpointing the exact instruction responsible.

```c
static void poolwatch_cb(void *opaque)
{
    uint32_t v; cpu_physical_memory_read(WATCH_ADDR, &v, 4);
    if (watch_prev_valid && v != watch_prev) {
        ARMCPU *cpu = ARM_CPU(qemu_get_cpu(0));
        fprintf(stderr, "POOLWATCH: 0x%08x changed 0x%08x->0x%08x writer pc=0x%08x\n",
                WATCH_ADDR, watch_prev, v, cpu->env.regs[15]);
    }
    watch_prev = v; watch_prev_valid = 1;
    timer_mod(poolwatch_timer, qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + 3*1000);
}
```

## 4. GDB: poke-and-see

Beyond breakpoints, GDB's most powerful feature here is **writing guest
memory from the debugger** to test a hypothesis live:

```sh
./build/qemu-system-arm -M s5l8900 ... -S -gdb tcp::1234
# in GDB:
(gdb) set architecture arm
(gdb) target remote :1234
(gdb) hbreak *0x22001360
(gdb) continue
(gdb) info registers            # what state is iBSS in, parked?
(gdb) x/8i $pc
(gdb) p/x 0x3c500040            # (won't hit a stub directly, but...)
(gdb) set {int}0x3e500004 = 8   # "what if PMU+0x4 read 8? does it unblock?"
(gdb) continue
```

The `set {int}0xADDR = value` trick is the fastest hypothesis tester you
have. If the guest unblocks, you've found a required register value without
reading a single line of firmware.

(For ROM code use `hbreak` — the guest can't write its own ROM, so a software
trap can't be planted. `work/run.sh` automates the launch+attach.)

## 5. Fault capture — the signature technique

A guest **data abort** tells you the faulting *virtual* address (DFAR) and
the faulting instruction, but only if you catch it before the firmware's own
(crappy) exception handler runs and destroys the scene. The project's
solution: **plant your own exception handler in SRAM and point the vector
table at it.**

### 5.1. The handler (nine hand-encoded ARM instructions)

```
  MOV r0, #0xF824      ; storage base (in SRAM 0x2200F800 block)
  STR lr, [r0, #0xC]   ; store return LR (-> faulting PC, via -4/-2 below)
  MRS r1, cpsr
  STR r1, [r0, #0x8]   ; store CPSR (mode + T bit)
  MRC p15, 0, r1, c6, c0, 0   ; DFAR (faulting data address)
  STR r1, [r0, #0x4]   ; store DFAR
  MRC p15, 0, r1, c5,  c0, 0   ; IFAR (faulting instr address)
  STR r1, [r0, #0x0]   ; store IFAR
  B   .                ; park forever; QEMU reads the storage block
```

You write these bytes into SRAM with `cpu_physical_memory_write` (the
project does this at `0x2200F800`, storage at `0x2200F820..`), then point
the guest's vector table (at `0x0` or wherever the firmware put it) at this
handler. Any fault now lands here, dumps its context, and parks — and QEMU
reads the block and prints it.

> **Why not just `MRC` the DFSR (fault status) inside the handler?** Reading
> certain CP15 registers *during* an exception handler can itself fault
> (nested exception), destroying everything. So the handler captures only
> DFAR/IFAR/CPSR/LR (safe), and QEMU reads the DFSR from the *C side*
> (`cpu->env.cp15.dfsr_s`) where it's always safe.

### 5.2. Reading the captured context

| Field | Meaning |
|---|---|
| `CPSR & 0x1F` | mode: `0x12`=ABT (data/prefetch abort), `0x16`=UND (undef) |
| `CPSR & 0x20` | T bit: was it Thumb when it faulted? |
| `DFAR` | the faulting **virtual** address |
| `LR` | faulting PC + 8 (ARM) or +4 (Thumb) → **faulting instruction = LR − 8 or LR − 4** |

Then: decode `CPSR` mode → you know it's a data abort. Take `DFAR` = the
address. Take `LR − 4` (if Thumb) = the faulting instruction address. Map
that back to a **file offset** with lesson 04's formula. Disassemble that
one instruction with capstone. You now know: *which instruction, touching
which address, faulted.* That's the whole diagnosis.

### 5.3. The DFSR fault-type table (read from the C side)

The low 4 bits of DFSR tell you *why*:

| DFSR[3:0] | Type | Usual cause here |
|---|---|---|
| `0b0010` | Translation fault (section) | page-table entry missing / `[1:0]=00` (the 0x30-vs-0x32 bug, lesson 09) |
| `0b0011` | Translation fault (page) | — |
| `0b0110` | Permission fault | domain/AP mis-set |
| `0b1111` | External/abort | access hit a region QEMU treats as invalid |

## 6. Worked example A — "the ROM hangs" (a hang, not a fault)

1. **Run** with the periodic timer. PC is stuck in `0x20000208`–`0x20000214`.
2. **Observe:** `x/6i 0x20000208` → `LDR r1,[pc,#0x24]; LDR r0,[r1]; CMP r0,#1; B -4`.
3. **Hypothesize:** it polls a register waiting for the value to be `1`; my
   stub returns something else.
4. **Instrument:** which register? `r1` = `0x3c500040` (CLOCK1+0x40). Add a
   log to the CLOCK1 stub printing `r1`/offset and the value it returns.
5. **Fix:** the ROM wants `1`; iBSS wants `0xFFFFFFFF` (single-bit tst's).
   One value can't do both → NOP the ROM's `BNE` (a 4-byte patch) and let the
   stub return `0xFFFFFFFF`. Rebuild. Re-run. The ROM proceeds.

Notice: the fix was a **4-byte firmware patch + a stub change**, and it was
chosen *because* the two consumers of the same register need different
values. That kind of "two masters, one register" conflict recurs constantly.

## 7. Worked example B — "iBoot data-aborts" (a fault)

1. **Run**; the fault-capture handler parks. QEMU prints: `CPSR mode=ABT, T=1,
   DFAR=0x805, LR=0x18005edE`.
2. **Read it:** Thumb data abort. Faulting VA = `0x805`. Faulting instruction
   = `LR − 4 = 0x18005EDA`.
3. **Map to file:** iBoot runtime base `0x18000000`, header `0x400` → file
   offset = `0x18005EDA − 0x18000000 + 0x400 = 0x61DA`. Disassemble that
   instruction: it's loading from a global that iBoot never initialized
   (because QEMU skipped its init).
4. **Hypothesize:** the load at `0x805` dereferences an uninitialized pointer
   (which happens to be a small offset). 
5. **Fix:** pre-zero/seed that global in the QEMU-side setup, or stub the
   function that reads it. Rebuild, re-run, no fault.

The whole diagnosis was: *catch the fault → DFAR + LR → one instruction → one
global.* No guessing.

## 8. Pitfalls (the ones that actually cost time)

- **Stale binary.** You changed the stub, QEMU still behaves old. Rebuild
  *every single time* before re-running. This is the #1 false lead.
- **TCG infinite translation block.** A guest `b .` can burn 100% of a host
  core and never yield to the main loop, so your timers "stop." That's why
  the timers are REALTIME and why you sometimes need a `WFI`/redirect to
  force a yield. If your periodic lines suddenly stop, suspect this.
- **Watchpoints that silently don't fire.** GDB hardware watchpoints on QEMU
  ARM can be flaky. Prefer P3 (the C-side write-watcher) for guest memory.
- **Log flooding.** `in_asm` without a timeout fills your disk. Always
  `timeout` + `grep`.
- **Reading a log out of order.** `-D` and stderr interleave; tag your own
  lines with a `>>>` prefix so you can `grep '>>> '`.
- **Trusting the firmware's own exception handler.** It will mask the real
  fault. Always install your own (section 5).

## 9. Checkpoint

Given a hung or faulting run, you can produce a **fault report**:
1. faulting PC (from DFAR/LR or a stuck PC),
2. the faulting instruction (disassembled, with its file offset),
3. the fault type (from CPSR mode + DFSR),
4. a one-sentence cause,
5. the minimal fix + how you verified it.

If you can write that report for a failing run, you can do this whole
project. Everything else is just more of the same loop.

## 10. Exercises

- Reproduce Worked Example A from scratch: revert the CLOCK1 stub to
  `return 0`, rebuild, and run the diagnosis *before* looking at the fix.
- Build the fault-capture handler from scratch: hand-encode the 8
  instructions (use the encodings in `appendix.md` C), write them to SRAM,
  and point a vector at them. Force a fault by reading a bad address from a
  planted stub and confirm you capture DFAR/LR/CPSR.
- Use P3 to find who writes a given SRAM address during iBSS's self-copy.

Next: [Lesson 09: iBSS and the Handoff](09-ibss-handoff.md)
