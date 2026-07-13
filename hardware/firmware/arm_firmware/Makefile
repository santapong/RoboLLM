# Build/upload arm_firmware.ino with the Ubuntu apt toolchain (arduino-mk).
#   make            -> compile (no board needed)
#   make upload     -> compile + flash over USB
#   make monitor    -> serial monitor (screen, 115200)
# Requires: sudo apt install gcc-avr avr-libc avrdude arduino-core-avr arduino-mk
BOARD_TAG    = uno
ARDUINO_LIBS = Servo
ARDUINO_DIR  = /usr/share/arduino
MONITOR_PORT ?= /dev/ttyACM0
MONITOR_BAUDRATE = 115200
include /usr/share/arduino/Arduino.mk
