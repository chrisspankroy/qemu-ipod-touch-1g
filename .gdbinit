# GDB may have ./.gdbinit loading disabled by default.  In that case you can
# follow the instructions it prints.  They boil down to adding the following to
# your home directory's ~/.gdbinit file:
#
#   add-auto-load-safe-path /path/to/qemu/.gdbinit

# Load QEMU-specific sub-commands and settings
source scripts/qemu-gdb.py

# Decode $cpsr (QEMU unified A32 layout: A=bit 8, E=bit 9).
# NOTE: python lines must start at column 0 inside a GDB define block.
define cpsr
  python
v = int(gdb.parse_and_eval("$cpsr"))
modes = {0x10:"USR",0x11:"FIQ",0x12:"IRQ",0x13:"SVC",0x17:"ABT",0x1B:"UND",0x1F:"SYS"}
print("N={} Z={} C={} V={}  E={} A={}  I={} F={} T={}  mode={} ({})".format((v>>31)&1,(v>>30)&1,(v>>29)&1,(v>>28)&1,(v>>9)&1,(v>>8)&1,(v>>7)&1,(v>>6)&1,(v>>5)&1,hex(v&0x1f),modes.get(v&0x1f,"?")))
  end
end
