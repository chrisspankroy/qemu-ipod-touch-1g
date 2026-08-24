# Lesson 04: Get The Firmware

**Goal:** Obtain, extract, and understand the raw firmware images — the
SecureROM, iBSS, iBEC, iBoot, and kernelcache — and learn Apple's **img2**
container format (the "89001.0" variant used by this SoC). By the end you can
decrypt iBoot yourself from a pristine IPSW and prove the result is real.

**Prerequisites:** Lessons 02–03.

---

## 1. Where the firmware comes from

All boot firmware ships inside Apple's **IPSW** restore files. For this
project: **iOS 1.1, build 3A101a, for board `n45ap`** (the 1G iPod Touch's
internal name). It's already in the repo:

- `work/iPod1,1_1.1_3A101a_Restore.ipsw` (158 MB) — the archive
- `work/iPod1,1_1.1_3A101a_Restore/` — already extracted

If you're recreating from scratch: old IPSWs are still downloadable from
Apple's firmware servers (search "iPod1,1 3A101a ipsw"; community mirrors
exist). An `.ipsw` is a plain **ZIP file**:

```sh
unzip "work/iPod1,1_1.1_3A101a_Restore.ipsw" -d extracted/
find extracted -type f | sort
```

The important files:

| File | What it is |
|---|---|
| `Firmware/dfu/iBSS.n45ap.RELEASE.dfu` | iBSS — first DFU-stage image (103,562 B) |
| `Firmware/dfu/iBEC.n45ap.RELEASE.dfu` | iBEC — second DFU-stage image (103,562 B) |
| `Firmware/dfu/WTF.s5l8900xall.RELEASE.dfu` | WTF — the tiny DFU "wait for this" image (9,354 B) |
| `Firmware/all_flash/all_flash.n45ap.production/iBoot.n45ap.RELEASE.img2` | iBoot, **payload encrypted** (145,546 B) |
| `Firmware/all_flash/.../LLB.n45ap.RELEASE.img2` | LLB (the NOR-flash first stage — not used in our DFU path) |
| `Firmware/all_flash/.../DeviceTree.n45ap.img2` | Device tree (needed later for `bootx`) |
| `kernelcache.release.s5l8900xrb` | The XNU kernel + kexts (3,324,650 B) |
| `022-3604-4.dmg`, `022-3605-4.dmg` | HFS+ filesystem images (userland, ramdisks) |
| `Restore.plist` | Manifest: contents, signatures |

The `n45ap` suffix is the board name; `s5l8900xrb` is the SoC family tag in
the kernelcache name. iOS 1.x firmware predates SHSH blobs, so nothing here
is device-locked for our purposes.

## 2. The container format ("89001.0" img2)

Every boot image on this SoC is wrapped in a container whose 8-byte magic is
the SoC id + version: `38 39 30 30 31 2e 30` = **"89001.0"** (s5l8900,
format v1.0). Verified layout (little-endian fields):

```
offset 0x000   8-byte magic "89001.0" + version byte
offset 0x00C   payload size (e.g. iBoot: 0x22400 = 140288)
offset 0x014   0x800 + payload size   (end-of-payload offset)
offset 0x800   *** payload starts here ***
after payload  RSA-2048 signature + Apple certificate chain (~3,210 bytes)
```

Two crucial facts, verified by actually doing it:

1. **DFU images (iBSS, iBEC): the payload is plaintext.** File offset 0x800
   of `iBSS.n45ap.RELEASE.dfu` starts with `0e 00 00 ea` =
   `mov r0, pc` — real ARM code. QEMU just skips the 0x800 header and loads
   the payload (this is literally what `s5l8900.c` does).
2. **all_flash images (iBoot, LLB): the payload is AES-128-CBC encrypted.**
   The key for `s5l8900x` iBoot is public knowledge from the jailbreak
   community:

   ```
   key: 188458a6d15034dfe386f23b61d43774     (AES-128, IV = all zeros, no PKCS padding)
   ```

The ROM (on real hardware) always **verifies the RSA signature** before
jumping; it decrypts where the payload is encrypted. We replicate only what
we need: skip the header for iBSS/iBEC, decrypt iBoot.

## 3. Decrypt iBoot — do it yourself

Work in a scratch dir; the repo's `work/iBoot.decrypted` is your
byte-for-byte reference.

```sh
cd work
IB="iPod1,1_1.1_3A101a_Restore/Firmware/all_flash/all_flash.n45ap.production/iBoot.n45ap.RELEASE.img2"

# 1. Look at the wrapper header
xxd "$IB" | head -4
#    3839 3030 312e 30  "89001.0"...
#    0x0C: 00 24 02 00  -> payload size 0x22400 = 140288 bytes

# 2. Slice out the ciphertext (starts at 0x800 = 2048)
dd if="$IB" bs=1 skip=2048 count=140288 of=payload.enc

# 3. Decrypt
openssl enc -d -aes-128-cbc -nopad \
  -K 188458a6d15034dfe386f23b61d43774 \
  -iv 00000000000000000000000000000000 \
  -in payload.enc -out iBoot.mine

# 4. PROVE it: must be byte-identical to the reference
cmp iBoot.mine iBoot.decrypted && echo "MATCH"
```

Notes:
- `-nopad` is required: Apple does not PKCS#7-pad; OpenSSL's default
  padding check will fail with "bad decrypt" on the final block.
- If you ever get "bad decrypt" on a range that *should* be right, your
  offset or length is wrong, not your key.

**Checkpoint A:** `cmp` reports no differences.

## 4. What's inside the decrypted iBoot?

The 140,288 decrypted bytes are themselves an image with a **0x400-byte
"IMG2" sub-header**, then code:

```sh
xxd iBoot.mine | head -2
# 0x000: "IMG2"   0x004: "ibot"
xxd -s 0xc -l 4 iBoot.mine
# 00 00 00 18  -> load address 0x18000000 (little-endian)
```

| Offset | Content |
|---|---|
| `0x000` | `IMG2` magic |
| `0x004` | `ibot` type tag |
| `0x00C` | **load address: `0x18000000`** — where iBoot expects to run |
| `0x400` | **firmware code starts** (0x22000 = 139,264 bytes) |

So the file↔address arithmetic for iBoot (from lesson 02) is now derived,
not memorized:

```
file offset f = link address L − 0x18000000 + 0x400
```

**Checkpoint B:**
1. `iBoot.decrypted` = 140288 bytes = 0x400 sub-header + 0x22000 code.
2. Load address at sub-header offset 0x0C is `0x18000000`.
3. File offset 0x400 of the decrypted image begins with
   `0e 00 00 ea` (`mov r0, pc`).
4. `xxd -s 0x1B5C -l 12 iBSS.n45ap.RELEASE.dfu` shows a `str r2, [r3]`
   (`52 03 1a 60` → `0x601A0352`) immediately followed by `fe ff ff ea`
   (`b .`). That's iBSS's final watchdog kick + halt loop at runtime
   `0x2200135e`/`0x22001360` — verify the mapping with lesson 02's formula.

## 5. The SecureROM — no container, no decryption

The SecureROM is burned into the chip; it's obtained from **ROM dumps of
real hardware** (community collections exist — `securerom.fun`). The repo
has one:

```sh
ls -l "work/ROM BOOT, S5L8900 Rev.2"     # exactly 65536 bytes = 64 KB
xxd "work/ROM BOOT, S5L8900 Rev.2" | head -2
```

First word `ea 00 00 2f` = `b 0x200000c4` (from `0x20000000`, where the PC
starts: target = 0x20000000 + 8 + 0x2F×4 = 0x200000C4). No header, no
encryption: file offset = address − 0x20000000.

## 6. The kernelcache and the DMGs (context for later)

- `kernelcache.release.s5l8900xrb` — same "89001.0" wrapper; contents are
  the big-endian Mach-O XNU kernel with kexts. Our emulator preloads it at
  `0x60000000` for a future `bootx` (lesson 12). No need to analyze it now.
- The `.dmg` files are HFS+ filesystem images.
  `work/3A101a_root_filesystem_key.txt` holds the key for decrypting the root
  filesystem DMG. Userland is a different rabbit hole — out of scope.

## 7. What this means for the QEMU side (preview)

When we build the machine model (lesson 07), the `-kernel` argument receives
the **iBSS .dfu** file; the machine code reads it, skips the 0x800 header,
and writes the payload into an iBSS RAM region at `0x09000000`. The iBoot
image (already decrypted, like `work/iBoot.decrypted`) is loaded from a file
path into staging at `0x23000000`. iBEC is loaded the same way as iBSS into
`0x0A000000`. All of this mirrors what the real hardware does with the
container format — except we do the header-skip/decrypt in C instead of in
the ROM's crypto engine.

## Pitfalls

- **Wrong payload offset.** The wrapper header is 0x800, but the *decrypted*
  iBoot image has its own 0x400 sub-header. Mixing the two up shifts all your
  xrefs by 0x400. Keep the two constants separate in your head:
  **0x800 = wrapper**, **0x400 = sub-header**.
- **Omitting `-nopad`** in OpenSSL → "bad decrypt" even with the right key.
- **Assuming the .dfu payloads are encrypted.** They aren't (verified above).
  If you "decrypt" iBSS you'll produce garbage.

Next: [Lesson 05: Read The SecureROM](05-read-the-rom.md)
