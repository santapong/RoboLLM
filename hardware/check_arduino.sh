#!/usr/bin/env bash
# check_arduino.sh — one-command health check for the Arduino Uno R3.
# Run it with the Uno plugged into USB:   hardware/check_arduino.sh
# It verifies: toolchain -> port -> permissions -> compile -> flash -> talk.
#
# Toolchain = arduino-cli (rootless, in ~/.local/bin + ~/.arduino15).
# Reinstall any time with:  arduino-cli core install arduino:avr && arduino-cli lib install Servo
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FW="$HERE/firmware/arm_firmware"
CLI="${ARDUINO_CLI:-$HOME/.local/bin/arduino-cli}"
FQBN=arduino:avr:uno
PASS=0; FAIL=0
ok()   { echo "  ✓ $1"; PASS=$((PASS+1)); }
bad()  { echo "  ✗ $1"; FAIL=$((FAIL+1)); }

echo "[1/6] toolchain"
if [ -x "$CLI" ] && "$CLI" core list 2>/dev/null | grep -q arduino:avr; then
  ok "arduino-cli + arduino:avr core installed"
else
  bad "arduino-cli or the AVR core missing — install (no sudo needed):
      curl -fsSLO https://github.com/arduino/arduino-cli/releases/latest
      then: arduino-cli core install arduino:avr && arduino-cli lib install Servo"
  exit 1
fi

echo "[2/6] board detection"
PORT="$(ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null | head -1 || true)"
if [ -n "$PORT" ]; then
  ok "serial port: $PORT"
  "$CLI" board list 2>/dev/null | grep -v '^$' | sed 's/^/      /'
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
  exit 1
else
  bad "not in the 'dialout' group — run: sudo usermod -aG dialout $USER
      then log out and back in (or: newgrp dialout)"
  exit 1
fi

echo "[4/6] compile firmware"
if "$CLI" compile --fqbn "$FQBN" "$FW" >/tmp/arm_fw_build.log 2>&1; then
  ok "arm_firmware.ino compiles ($(grep -oE 'Sketch uses [0-9]+ bytes \([0-9]+%\)' /tmp/arm_fw_build.log || echo 'see /tmp/arm_fw_build.log'))"
else
  bad "compile failed — see /tmp/arm_fw_build.log"
  exit 1
fi

echo "[5/6] flash to board"
if "$CLI" upload -p "$PORT" --fqbn "$FQBN" "$FW" >/tmp/arm_fw_upload.log 2>&1; then
  ok "uploaded over $PORT"
else
  bad "upload failed — see /tmp/arm_fw_upload.log (wrong port? board busy?
      close the IDE serial monitor if it's open)"
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
