# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a hardware research and reverse engineering repository for the **iPod Touch 1st Generation (iPod1,1, model n45ap)**.

## Device Hardware

- **SoC**: S5L8900 (ARM1176JZF-S core)
  - CPU datasheet: ARM DDI0301H
- **Storage**: 8GB (two 4GB NAND chips)
- **Supported iOS**: 1.1–3.1.3
- **No SHSH blob / downgrade protection**

## Memory Map (S5L8900)

| Region | Address Range |
|---|---|
| SecureROM (VROM) | `0x20000000`–`0x20010000` |
| CLOCK0 | `0x38100000` |
| VIC0 | `0x38e00000`–`0x38e00fff` |
| VIC1 | `0x38e01000`–`0x38e01fff` |
| EDGEIC | `0x38e02000` |
| GPIOIC | `0x39a00000` |
| CLOCK1 | `0x3c500000` |
| WDT_CTRL | `0x3e300000` |
| Peripheral base | `0x38000000` |

DFU mode is implemented in the SecureROM. SecureROM dumps are available at securerom.fun.

## Repository Contents

- `iPod1,1_1.1_3A101a_Restore/` — Extracted IPSW for iOS 1.1 (build 3A101a), board config `n45ap`
  - `Firmware/all_flash/` — Boot images (iBoot, LLB, DeviceTree, logos) in img2 format
  - `Firmware/dfu/` — DFU-mode images (iBSS, iBEC, WTF)
  - `kernelcache.release.s5l8900xrb` — Release kernelcache
  - `022-3601-4.dmg` — System (root) filesystem image
  - `022-3604-4.dmg` — User restore ramdisk
  - `022-3605-4.dmg` — Update ramdisk
- `3A101a_root_filesystem_key.txt` — AES key for decrypting the root filesystem DMG
- `backup_files/` — Original IPSW archive and ROM boot dump
- `connector_pinout.pdf` — Physical connector pinout reference
- `notes.txt` — Hardware notes and memory map

## Common Tools for This Domain

- **xpwntool / img2tool** — Decrypt/extract img2 firmware images
- **dmg / hdiutil** — Mount decrypted DMG filesystem images
- **ibootim** — Work with iBoot image format
- **iBoot/LLB/iBEC/iBSS** — Boot chain stages (SecureROM → LLB → iBoot → kernel)
- **libimobiledevice / irecovery** — Communicate with device in DFU/recovery mode

## Boot Chain

```
SecureROM (ROM) → LLB (NOR) → iBoot → kernelcache → iOS userland
```

DFU chain uses: WTF → iBSS → iBEC → restore ramdisk
