#!/usr/bin/env python3
import sys
import capstone

if len(sys.argv) < 6:
  print('dis.py [thumb|arm] <base_addr_hex> <file_offset_hex> <count> <file_name>')
  exit(0)

mode = capstone.CS_MODE_THUMB if sys.argv[1] == 'thumb' else capstone.CS_MODE_ARM
base = int(sys.argv[2], 16)
off  = int(sys.argv[3], 16)
n    = int(sys.argv[4], 16)
data = open(sys.argv[5], 'rb').read()
md = capstone.Cs(capstone.CS_ARCH_ARM, mode)
md.detail = True
for i in md.disasm(data[off:off+n], base + off):
    print(f"0x{i.address:08x}: {i.mnemonic:8s} {i.op_str}")
