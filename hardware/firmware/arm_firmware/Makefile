# Build/upload arm_firmware.ino with arduino-cli (rootless, ~/.local/bin).
#   make            -> compile (no board needed)
#   make upload     -> compile + flash over USB (PORT=/dev/ttyACM0 by default)
#   make monitor    -> serial monitor at 115200
CLI  ?= $(HOME)/.local/bin/arduino-cli
FQBN ?= arduino:avr:uno
PORT ?= /dev/ttyACM0

all:
	$(CLI) compile --fqbn $(FQBN) .

upload: all
	$(CLI) upload -p $(PORT) --fqbn $(FQBN) .

monitor:
	$(CLI) monitor -p $(PORT) --config 115200

.PHONY: all upload monitor
