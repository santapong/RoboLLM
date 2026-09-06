CLI ?= arduino-cli
PYTHON ?= python3
PROFILE ?= ../../../ros2/robo_arm_driver/config/elexhub.yaml
FQBN ?= arduino:avr:mega

all: check-config
	$(CLI) compile --fqbn $(FQBN) .

config:
	$(PYTHON) ../../generate_firmware_config.py --config $(PROFILE) --output arm_config.h

check-config:
	$(PYTHON) ../../generate_firmware_config.py --config $(PROFILE) --output arm_config.h --check

# Upload is deliberately a separate, operator-run command after bench review.
.PHONY: all config check-config
