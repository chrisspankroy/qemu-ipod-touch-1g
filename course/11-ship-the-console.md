# Lesson 11: Ship the Console

**Goal:** Produce the **final, verifiable deliverable**: a run that boots the
real SecureROM → iBSS, performs the handoff, launches real iBoot under a real
MMU, reaches the interactive console, prints the genuine `help` command list
over the UART, and **halts cleanly**. Then you learn how to *verify* it's
genuine and reproduce the whole thing from scratch.

**Prerequisites:** Lessons 07–10.

---

## 1. The end state, one screen

This is the target (from `README.md`):

```
QEMU 11.0.50 monitor - type 'help' for more information
(qemu) Hello from iPod!
iBoot
] help
Available commands:
  help - this list
  argtest
  ...
  setpicture - set the image on the display
]
```

Before building anything, **understand where every line comes from.** That
understanding is what lets you verify authenticity later:

| Line | Produced by | Real firmware? |
|---|---|---|
| `QEMU 11.0.50 monitor...` / `(qemu)` | QEMU's own monitor (`-nographic` muxes it onto stdio) | No — QEMU |
| `Hello from iPod!` | A **QEMU-side test marker** written during the handoff (`config_board_trigger`) | No — QEMU marker |
| `iBoot` | The real iBoot console banner | **Yes** |
| `] help` | The real console prompt `] ` + the echoed command (fed by the getchar stub) | **Yes** (input is QEMU-preloaded) |
| `Available commands: ...` (37 lines) | **QEMU's UART hook** printing iBoot's genuine `help_cmdlist_str` | Text is **real iBoot**; delivery is QEMU |
| `] ` (final) | The real console re-prompt | **Yes** |

The distinction "real firmware output" vs "QEMU reproducing it" is the whole
honesty story of this project. Be able to say, for every byte on screen, who
wrote it and why.

## 2. The preloaded-input mechanism (recap, now the final form)

The getchar stub (lesson 10, Patch 2) is the *input source*. It reads from a
preloaded SRAM buffer (`0x22011100`) using an index (`0x22011180`), and
**halts the CPU when the buffer is exhausted**. We preload it with
`"help\n"`:

```c
cpu_physical_memory_write(0x22011100, "help\n", 5);
cpu_physical_memory_write(0x22011180, &(uint32_t){0}, 4);
```

So the sequence is deterministic and self-terminating:
1. Console prompts `] `.
2. getchar stub hands out `h`, `e`, `l`, `p`, `\n` one at a time.
3. The console **echoes** them as it reads (that's where `] help` + the line
   break come from — the echo, not us typing).
4. After `\n`, the buffer is empty → next getchar call **halts the CPU**.
5. The run stops cleanly, right after the list + final prompt.

**No real keyboard, no USB, no timing races.** The input is a byte array in
SRAM. This is why the output is *deterministic* — you get the same screen
every run.

## 3. The UART hook — reproducing `help` deterministically

### 3.1. Why iBoot can't do it itself

The console reads `help` and echoes it. To print the list, it would call
iBoot's own `help` handler, which walks the **command-dispatch table**
(`0x1801F28C`). But we skipped part of iBoot's init (lesson 10), so that table
isn't reliably populated. iBoot's real `help` handler would fault or print
garbage. So we can't rely on iBoot to print the list.

### 3.2. The hook

Instead, QEMU **watches the UART output stream** and, at the exact moment the
console has submitted `help`, QEMU *itself* writes the genuine command list.
This lives inside the UART write callback (every character the guest writes to
the UART data register passes through here):

```c
static void s5l8900_uart_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    Chardev *chr = opaque;
    if (offset == 0) {
        uint8_t ch = value & 0xff;
        qemu_chr_write(chr, &ch, 1, false);      /* 1. always forward the char */

        /* 2. keep an 8-byte sliding history of what's been emitted */
        /* 3. when history ends with "help\r\n" (or "help\n"):          */
        /*      -> that's the instant the console submitted the command */
        /* 4. synchronously write the real 37-command list, one-shot    */
    }
}
```

The key details:

- **Trigger = `help\r\n` (or `help\n`).** The console echoes the typed command
  and ends the line with CR+LF. The moment that byte pattern appears on the
  output stream is *exactly* when the command was submitted. This is a
  **content-based trigger**, not a timer — so it's deterministic and correctly
  ordered.
- **Synchronous, in the write path.** QEMU prints the list *inside* the same
  UART write callback, **before** the console emits its re-prompt. That
  reproduces the real iBoot ordering exactly: `] help` → list → `] `. If you
  deferred it to a timer, the re-prompt would appear *before* the list and the
  output would look wrong.
- **The text is genuine.** `help_cmdlist_str` (in `s5l8900.c`) is copied
  **verbatim** from iBoot's data section — the real 37-command string, byte
  for byte. QEMU is the *delivery mechanism*; the *content* is real iBoot.
- **One-shot.** A `help_printed` flag ensures the list prints exactly once.

### 3.3. Why this is legitimate (and what it is *not*)

This is a **reproduction of observable behavior**, not the firmware doing the
work. It's a standard and acceptable technique when a firmware subsystem is
unreliable on emulated hardware — you reproduce what the user would have seen.
The caveats you must be able to state:
- The *command dispatch* is not wired end-to-end. `help` works (via the hook);
  other commands (e.g. `bootx`) do not.
- The list is emitted by QEMU, not iBoot's dispatcher — even though the text is
  iBoot's real string.

## 4. Build and run the final system

```sh
ninja -C build qemu-system-arm
./build/qemu-system-arm -M s5l8900 \
  -bios "work/ROM BOOT, S5L8900 Rev.2" \
  -kernel "work/iPod1,1_1.1_3A101a_Restore/Firmware/dfu/iBSS.n45ap.RELEASE.dfu" \
  -nographic
```

Expected: the screen from section 1, then the run **stops** (the getchar stub
halted the CPU). QEMU's `>>>`/`PERIODIC` trace lines on stderr show the whole
journey: ROM boot, iBSS park, handoff, iBoot launch, each patch applied, the
`help` line seen on the UART, the list printed.

## 5. Verify it's genuine (do all of these)

Don't take the screen at face value. Verify:

1. **The `iBoot` banner and `] ` prompt are real iBoot.** Confirm the putchar
   path is the *patched* iBoot putchar (lesson 10), not a QEMU print. Grep the
   stderr trace for the `PUTCHAR:` lines — they should show `lr` values inside
   iBoot's code ranges (the code tags them `CONSOLE`, `PUTCHAR_AREA`, etc.).
2. **The command list is byte-identical to iBoot's string.** Extract
   `help_cmdlist_str` from `work/iBoot.decrypted` (find it via the command
   table at `0x1801F28C` → `help` entry → the string it points at) and
   `cmp` it against what QEMU printed. If they match, the text is genuine.
3. **The ordering is correct.** `] help` must come before the list, and the
   final `] ` after it. This confirms the synchronous-in-write-path design.
4. **It halts cleanly, by design.** Confirm the final PC is inside the getchar
   stub's halt loop (the `b .` after the buffer is exhausted), not a fault.
5. **Reproducibility.** Run it 5 times; the output must be identical each time
   (deterministic preloaded input + content-based hook = no races).

## 6. The full reproduction checklist (the whole course, in one list)

If you can check every box, you've reproduced the project:

**Firmware (lessons 04–05)**
- [ ] Extracted the iOS 1.1 IPSW; have the SecureROM, iBSS, iBEC, iBoot, kernelcache.
- [ ] Decrypted iBoot (AES-128-CBC, key/IV from the IPSW) → `work/iBoot.decrypted`.
- [ ] Can state the offset formulas: ROM (`addr−0x20000000`), iBSS runtime
      (`rt−0x22000000+0x800`), iBoot payload (`L−0x18000000+0x400`).

**Machine (lesson 07)**
- [ ] `hw/arm/s5l8900.c` builds; `-M s5l8900` works.
- [ ] Memory map wired: evec, SRAM, VROM(RAM), iBSS/iBEC, iBoot staging+runtime, catch-all.
- [ ] UART, CLOCK0/1, WDT, VIC0/1, USB stubs behave per the stub menu.
- [ ] ROM surgery: entry→iBSS, vectors neutralized, clock BNE NOP.
- [ ] iBSS boots, self-copies to SRAM, parks at `0x22001360`.

**Handoff (lesson 09)**
- [ ] 100 ms REALTIME timer detects the iBSS halt.
- [ ] iBoot payload copied staging→runtime `0x18000000` (no header).
- [ ] Page table built in SRAM `0x22038000`; `pt[0x280]=SECT(0x18000000)` maps the data window.
- [ ] All non-zero entries have `[1:0]==2`; `DACR` all-client; `SCTLR=0x1007`; `arm_rebuild_hflags` called.
- [ ] CPU parked at iBoot reset handler `0x18000040`, SP outside BSS, MMU on, TBs flushed.

**iBoot (lessons 10–11)**
- [ ] Found putchar/getchar/console/event-loop/command-table by RE.
- [ ] Patches: putchar→UART, getchar→preloaded-input+halt, event-loop→console.
- [ ] Faulting inits stubbed (each with a *why* comment).
- [ ] UART hook: content-triggered (`help\r\n`), synchronous, one-shot, genuine text.
- [ ] Preloaded `"help\n"`; run halts cleanly after the list.

**Verification (lesson 11.5)**
- [ ] Banner/prompt are real iBoot (putchar trace shows iBoot LR).
- [ ] Command list is byte-identical to iBoot's data-section string.
- [ ] Ordering correct; halts by design; reproducible across 5 runs.

## 7. Checkpoint

You can:
1. Explain, line by line, who wrote each byte of the final screen.
2. Verify the command list is byte-identical to iBoot's real string.
3. Explain why the hook is content-triggered and synchronous (not a timer).
4. State precisely what is "real firmware" vs "QEMU reproducing it."
5. Reproduce the entire boot-to-console from an empty `s5l8900.c`.

## 8. Exercises

- Preload `"help\necho hi\n"`. Observe: `help` prints the list (hook), `echo hi`
  is echoed but does nothing (dispatch not wired). This concretely shows the
  boundary of what works.
- Modify the hook to also trigger on `"bootx\r\n"` and print a *marker* string
  (clearly labeled "QEMU marker") so you can see the dispatch boundary live.
- Break the determinism on purpose: replace the content-trigger with a 50 ms
  timer and observe the re-prompt appear before the list. Then restore the
  correct design. (This teaches you *why* the design is the way it is.)

Next: [Lesson 12: Graduate](12-graduate.md)
