# Build/upload arm_firmware.ino with arduino-cli (rootless, ~/.local/bin).
#   make            -> compile (no board needed)
#   make upload     -> compile + flash over USB (PORT=/dev/ttyACM0 by default)
#   make monitor    -> serial monitor at 115200
CLI  ?= $(HOME)/.local/bin/arduino-cli
FQBN ?= arduino:avr:mega
PORT ?= /dev/ttyACM0

all: check-config
	$(CLI) compile --fqbn $(FQBN) .

config:
	python3 ../../generate_firmware_config.py

check-config:
	python3 ../../generate_firmware_config.py --check

upload: all
	$(CLI) upload -p $(PORT) --fqbn $(FQBN) .

monitor:
	$(CLI) monitor -p $(PORT) --config 115200

.PHONY: all config check-config upload monitor
