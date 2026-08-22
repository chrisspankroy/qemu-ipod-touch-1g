#!/bin/sh
set -e

QEMU_BUILD=/Users/chris/dev/ipod-touch-1g/build
WORKDIR="/Users/chris/dev/ipod-touch-1g/work"
ROM="$WORKDIR/ROM BOOT, S5L8900 Rev.2"
IBSS="$WORKDIR/iPod1,1_1.1_3A101a_Restore/Firmware/dfu/iBSS.n45ap.RELEASE.dfu"
GDB_PORT=1234

echo "==> Building QEMU..."
ninja -C "$QEMU_BUILD" qemu-system-arm

echo "==> Starting QEMU (GDB port $GDB_PORT)..."
pkill -f "qemu-system-arm.*s5l8900" 2>/dev/null || true
python3 -c "
import os, sys
os.setsid()
os.execvp(sys.argv[1], sys.argv[1:])
" "$QEMU_BUILD/qemu-system-arm" \
    -M s5l8900 \
    -bios "$ROM" \
    -kernel "$IBSS" \
    -nographic \
    -S -gdb tcp::$GDB_PORT \
    -d unimp,guest_errors \
    > /tmp/qemu_out.txt 2>&1 &
QEMU_PID=$!

# Wait for QEMU to open the GDB port (up to 5 seconds)
for i in $(seq 1 10); do
    sleep 0.5
    if kill -0 $QEMU_PID 2>/dev/null && nc -z localhost $GDB_PORT 2>/dev/null; then
        echo "   QEMU pid $QEMU_PID (ready)"
        break
    fi
    if ! kill -0 $QEMU_PID 2>/dev/null; then
        echo "ERROR: QEMU exited early. Output:"
        cat /tmp/qemu_out.txt
        exit 1
    fi
    if [ $i -eq 10 ]; then
        echo "ERROR: QEMU did not open port $GDB_PORT in time. Output:"
        cat /tmp/qemu_out.txt
        kill $QEMU_PID 2>/dev/null || true
        exit 1
    fi
done

echo "==> Attaching GDB (Ctrl-D or 'quit' to exit GDB, then QEMU stops too)"
gdb \
    -ex "set architecture arm" \
    -ex "target remote :$GDB_PORT" \
    -ex "set pagination off"

kill $QEMU_PID 2>/dev/null || true
