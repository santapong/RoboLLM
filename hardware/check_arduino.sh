#!/usr/bin/env bash
# check_arduino.sh — one-command health check for the Arduino Uno R3.
# Run it with the Uno plugged into USB:   hardware/check_arduino.sh
# It verifies: toolchain -> port -> permissions -> compile -> flash -> talk.
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FW="$HERE/firmware/arm_firmware"
PASS=0; FAIL=0
ok()   { echo "  ✓ $1"; PASS=$((PASS+1)); }
bad()  { echo "  ✗ $1"; FAIL=$((FAIL+1)); }

echo "[1/6] toolchain"
if command -v avr-gcc >/dev/null && command -v avrdude >/dev/null \
   && [ -f /usr/share/arduino/Arduino.mk ]; then
  ok "avr-gcc + avrdude + arduino-mk installed"
else
  bad "toolchain missing — run:
      sudo apt install -y gcc-avr avr-libc avrdude arduino-core-avr arduino-mk"
  exit 1
fi

echo "[2/6] board detection"
PORT="$(ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null | head -1 || true)"
if [ -n "$PORT" ]; then
  ok "serial port: $PORT"
  lsusb | grep -iE 'arduino|2341|1a86|ch340|qinheng' | sed 's/^/      /' || true
else
  bad "no /dev/ttyACM* or /dev/ttyUSB* — is the Uno plugged in? Try another
      cable (charge-only USB cables have no data lines) and 'dmesg | tail'."
  exit 1
fi

echo "[3/6] permissions"
if [ -r "$PORT" ] && [ -w "$PORT" ]; then
  ok "you can read/write $PORT"
elif id -nG | grep -qw dialout; then
  bad "$PORT not accessible even though you're in dialout — check 'ls -l $PORT'"
else
  bad "not in the 'dialout' group — run: sudo usermod -aG dialout $USER
      then log out and back in (or: newgrp dialout)"
  exit 1
fi

echo "[4/6] compile firmware"
if make -C "$FW" >/tmp/arm_fw_build.log 2>&1; then
  ok "arm_firmware.ino compiles ($(grep -oE '[0-9]+ bytes.*maximum' -m1 /tmp/arm_fw_build.log || echo 'see /tmp/arm_fw_build.log'))"
else
  bad "compile failed — see /tmp/arm_fw_build.log"
  exit 1
fi

echo "[5/6] flash to board"
if make -C "$FW" upload MONITOR_PORT="$PORT" >/tmp/arm_fw_upload.log 2>&1; then
  ok "uploaded over $PORT"
else
  bad "upload failed — see /tmp/arm_fw_upload.log (wrong port? board busy?)"
  exit 1
fi

echo "[6/6] talk to firmware (PING + LED blink)"
if python3 "$HERE/arm_serial.py" ping && python3 "$HERE/arm_serial.py" led 1 \
   && sleep 1 && python3 "$HERE/arm_serial.py" led 0; then
  ok "firmware answers — the Uno works fine"
else
  bad "no reply from firmware — check baud/port, see hardware/README.md"
  exit 1
fi

echo
echo "RESULT: $PASS passed, $FAIL failed — Arduino is ready for the arm."
echo "Next: wire servos (EXTERNAL 5-6V supply, common GND) and try:"
echo "  python3 $HERE/arm_serial.py set 0 120"
