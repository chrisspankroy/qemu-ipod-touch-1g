#!/usr/bin/env python3
"""ARM/Thumb xref scanner for the iBoot payload.

work/iBoot.decrypted = 0x400-byte img2 header + 0x22000 payload.
link L <-> file f = (L - 0x18000000) + 0x400.

Thumb BL/BLX are found by opcode prefix (robust anywhere) and the target is
resolved with capstone on the 4-byte chunk. ARM BL/BLX likewise. Data words
matching a target are also reported.
"""
import struct, sys
import capstone

IMG = 'work/iBoot.decrypted'
BASE = 0x18000000
HDR  = 0x400

def load():
    return open(IMG, 'rb').read()

MDT = capstone.Cs(capstone.CS_ARCH_ARM, capstone.CS_MODE_THUMB)
MDT.detail = True
MDA = capstone.Cs(capstone.CS_ARCH_ARM, capstone.CS_MODE_ARM)
MDA.detail = True

def chunk_target(data, f, link, md):
    for ins in md.disasm(data[f:f+4], link):
        if ins.mnemonic in ('bl', 'blx'):
            try:
                return ins.operands[0].imm
            except Exception:
                return None
    return None

def scan(data):
    blxrefs = {}
    n = len(data)
    f = HDR
    while f + 4 <= n:
        hw1 = data[f] | (data[f+1] << 8)
        if (hw1 & 0xF800) in (0xF000, 0xF800):
            hw2 = data[f+2] | (data[f+3] << 8)
            if (hw2 & 0xF800) == 0xF800:
                link = BASE + (f - HDR)
                tgt = chunk_target(data, f, link, MDT)
                if tgt is not None:
                    blxrefs.setdefault(tgt & ~1, []).append(link)
        f += 2
    f = HDR
    while f + 4 <= n:
        w = struct.unpack_from('<I', data, f)[0]
        cond = (w >> 28) & 0xF
        if cond != 0xE and (w & 0x0F000000) == 0x0B000000:  # ARM BL/BLX: cond,101,1,imm24
            link = BASE + (f - HDR)
            tgt = chunk_target(data, f, link, MDA)
            if tgt is not None:
                blxrefs.setdefault(tgt & ~1, []).append(link)
        f += 4
    return blxrefs

def data_refs(data, target):
    out = []
    f = HDR
    while f + 4 <= len(data):
        if struct.unpack_from('<I', data, f)[0] == target:
            out.append(BASE + (f - HDR))
        f += 1
    return out

def main():
    data = load()
    blxrefs = scan(data)
    if not sys.argv[1:]:
        print("usage: xref.py <target_link_hex> [more...]")
        return
    for q in sys.argv[1:]:
        tgt = int(q, 16) & ~1
        hits = blxrefs.get(tgt, [])
        print(f"BL/BLX -> 0x{tgt:08x}: {len(hits)} caller(s)")
        for h in sorted(set(hits)):
            print(f"    from 0x{h:08x}")
        for d in data_refs(data, tgt):
            print(f"    data  0x{d:08x}")

if __name__ == '__main__':
    main()
