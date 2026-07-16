# hardware/ — DIY robot arm (Raspberry Pi 5 + Arduino Uno R3)

The real-hardware path of this repo. Same philosophy as the sim side:
**the LLM/ROS layer plans, a dumb fast layer actuates.**

```
Claude / ROS 2 (laptop or Pi 5)          Arduino Uno R3            arm
  arm_bridge_node.py ── USB serial ──▶ arm_firmware.ino ──▶ up to 6 servos
        /arm/command  115200, text        50 Hz slew-limited PWM
        /arm/joint_states ◀──────────  current angles
```

Why serial and not micro-ROS: the Uno R3 (AVR, 2 KB RAM) cannot run
micro-ROS. A tiny text protocol is robust, debuggable with any serial
monitor, and the Pi 5 easily runs the actual ROS 2 node.

## Files
| File | What |
|------|------|
| `firmware/arm_firmware/` | Uno sketch + Makefile (arduino-cli, `make upload`) |
| `arm_serial.py` | Python driver + CLI (`ping`, `joints`, `set <ch> <deg>`, `led`) |
| `arm_bridge_node.py` | ROS 2 node: `/arm/command` (deg) → servos, publishes `/arm/joint_states` |
| `check_arduino.sh` | **One-command health check**: toolchain→port→perms→compile→flash→PING |
| `sim_uno.py` | **Fake Uno** on a virtual port — develop/demo the whole stack with no hardware: `python3 sim_uno.py`, then `ARM_PORT=/dev/pts/N …` |
| `pi5_setup.sh` | Run on the Pi 5: dialout, toolchain, ROS 2 Jazzy, udev rule |

## Toolchain (already installed on the laptop, rootless)
- **arduino-cli 1.5.1** at `~/.local/bin/arduino-cli`, AVR core 1.8.8 +
  Servo 1.3.0 in `~/.arduino15` — compiles/flashes with **no sudo**.
- **Arduino IDE 2.3.10** at `~/.local/opt/arduino-ide` — launch with
  `arduino-ide` or from the app menu. (Wrapper passes `--no-sandbox`:
  Ubuntu 24.04 AppArmor blocks Electron's sandbox outside apt/snap.)
- Reinstall anywhere: `arduino-cli core install arduino:avr && arduino-cli lib install Servo`.

```bash
sudo usermod -aG dialout $USER    # ONE root step: serial-port permission,
                                  # then log out & back in once
# plug the Uno in over USB, then:
hardware/check_arduino.sh         # answers "does the Arduino work fine?"
```

## Protocol (115200 baud, line-based)
`PING`→`PONG arm-fw 1.0` · `E 1|0` torque on/off · `S <ch> <deg>` move ·
`G`→`A d0..d5` read · `LED 1|0`. Unknown → `ERR …`.
Note: opening the port **resets the Uno** (DTR) — wait ~2 s (ArmSerial does).

## Wiring — read before powering servos ⚠️
- Servo **signal** wires → Uno pins **3, 5, 6, 9, 10, 11** (ch 0–5).
- Servo **power** → an **external 5–6 V supply** (≥1 A per moving servo).
  **Never** the Uno's 5V pin — 6 servos will brown-out/kill the board.
- **Common ground**: external supply GND ↔ Uno GND ↔ servo GND.
- Set per-joint limits in `MIN_DEG[]`/`MAX_DEG[]` in the sketch so the arm
  can't drive into itself, then `make upload` again.

## Pi 5 deployment
```bash
scp -r hardware/ <user>@<pi-ip>:~/arm/
ssh <user>@<pi-ip> 'bash ~/arm/pi5_setup.sh'   # Ubuntu 24.04 on the Pi
# then on the Pi:
bash ~/arm/check_arduino.sh
python3 ~/arm/arm_bridge_node.py               # ROS 2 node
```
Laptop and Pi on the same LAN + same `ROS_DOMAIN_ID` → the laptop's ROS 2
(and the MCP bridge) sees `/arm/joint_states` and can publish `/arm/command`
over the network. On Raspberry Pi OS instead of Ubuntu, run the node in
Docker: `docker run -it --device=/dev/ttyACM0 -v ~/arm:/arm ros:jazzy-ros-base`.

## Smoke test without any servos connected
The firmware boot-blinks the onboard LED 3×, and `arm_serial.py led 1`
lights it — proves USB + firmware + protocol with zero wiring.
