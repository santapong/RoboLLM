# hardware/ — Technical notes: the real DIY arm stack

This directory is the real-hardware path of RoboLLM: a DIY robot arm built from
an Arduino Uno R3 (servo controller) and a Raspberry Pi 5 or laptop (the ROS 2
brain). The Uno runs a tiny line-based text protocol over USB serial at 115200
baud — it cannot run micro-ROS (AVR, 2 KB RAM), so plain serial is intentional.
All planning and kinematics stay on the host; the Uno just slew-limits servo
targets at 50 Hz. `sim_uno.py` emulates the firmware on a pty so the entire
stack is developable and testable with zero hardware plugged in.

![Hardware architecture](docs/hardware-architecture.svg)

## Component walkthrough

- **`firmware/arm_firmware/arm_firmware.ino`** — the Uno sketch. Drives up to
  6 hobby servos on PWM pins 3, 5, 6, 9, 10, 11 (channels 0–5). Movement is
  slew-rate limited to `MAX_STEP_DEG = 2` per 20 ms tick (100 deg/s) so a big
  command never slams the arm. Per-channel limits live in `MIN_DEG[]` /
  `MAX_DEG[]` (default 0–180 — tighten them for your arm, then `make upload`).
  Boot: 3 fast LED blinks, then prints `READY arm-fw 1.0`.
- **`arm_serial.py`** — host-side pyserial driver + CLI. Auto-detects the port
  by USB VID (genuine Uno + CH340/CP210x clones), falling back to any
  ttyACM/ttyUSB; `ARM_PORT` env overrides (this is how sim_uno plugs in).
  Opening the port DTR-resets the Uno, so the constructor waits ~2.5 s and
  swallows the `READY` banner. API: `ping()`, `enable(bool)`, `set_joint(ch,
  deg)`, `get_joints()`, `led(bool)`.
- **`arm_bridge_node.py`** — the ROS 2 node (`arm_bridge`). Subscribes
  `/arm/command`, streams each element to a servo channel, publishes
  `/arm/joint_states` at 10 Hz (degrees converted to radians). Disables torque
  on shutdown. This is the real-hardware twin of the sim arm path in
  `robot_bridge.py`.
- **`sim_uno.py`** — fake Uno on a pty (`pty.openpty()`, raw mode). Speaks the
  identical protocol including the 100 deg/s slew limit, so motion takes real
  time. Prints its port (e.g. `/dev/pts/3`) and logs every command/reply.
- **`check_arduino.sh`** — 6-step health check for the real board:
  toolchain → port → permissions → compile → flash → PING + LED blink.
- **`pi5_setup.sh`** — run on the Pi 5 (Ubuntu 24.04): dialout group,
  arduino-cli (ARM64), ROS 2 Jazzy, and a udev rule that symlinks the Uno to
  `/dev/arm_uno`.

## Serial protocol (115200 baud, one command per line, replies end with `\n`)

| Command | Reply | Meaning |
|---------|-------|---------|
| `PING` | `PONG arm-fw 1.0` | liveness (sim answers `PONG arm-fw 1.0 (SIM)`) |
| `E 1` / `E 0` | `OK` | attach / detach all servos (0 = torque off) |
| `S <ch> <deg>` | `OK` | move servo `<ch>` (0–5) to `<deg>` (clamped to limits) |
| `G` | `A d0 d1 d2 d3 d4 d5` | read current angles (deg) |
| `LED 1` / `LED 0` | `OK` | onboard LED — smoke test with no servos wired |
| anything else | `ERR …` | error with hint |

## Key files

| File | Role |
|------|------|
| `firmware/arm_firmware/arm_firmware.ino` | Uno sketch: protocol + 50 Hz slew-limited servo control |
| `firmware/arm_firmware/Makefile` | `make` compile, `make upload`, `make monitor` (arduino-cli, `PORT=/dev/ttyACM0`) |
| `arm_serial.py` | Python driver + CLI: `ping` \| `joints` \| `set <ch> <deg>` \| `led <0|1>` |
| `arm_bridge_node.py` | ROS 2 bridge node (`/arm/command` → servos, `/arm/joint_states` out) |
| `sim_uno.py` | firmware emulator on a pty — dev with no hardware |
| `check_arduino.sh` | 6-step Uno health check |
| `pi5_setup.sh` | Pi 5 provisioning script |

## Topics and parameters (arm_bridge_node.py)

| Name | Type | Direction | Notes |
|------|------|-----------|-------|
| `/arm/command` | `std_msgs/Float64MultiArray` | sub | target angles in **degrees**, index = channel |
| `/arm/joint_states` | `sensor_msgs/JointState` | pub | 10 Hz, positions in **radians** |
| `port` (param) | string | — | serial device; empty = auto-detect |
| `joint_names` (param) | string[] | — | default `joint0`…`joint5` |
| `enable_on_start` (param) | bool | — | default true (torque on at startup) |

## Run + verify

Dev without hardware (two terminals):

```bash
python3 hardware/sim_uno.py                       # prints e.g. /dev/pts/3
ARM_PORT=/dev/pts/3 python3 hardware/arm_serial.py ping
ARM_PORT=/dev/pts/3 python3 hardware/arm_serial.py set 0 120
```

Real Uno (needs `sudo usermod -aG dialout $USER`, then re-login):

```bash
hardware/check_arduino.sh                         # toolchain→…→PING, all 6 green
```

ROS 2 bridge (laptop or Pi 5; `ARM_PORT` works here too):

```bash
bash -c 'source /opt/ros/jazzy/setup.bash && python3 hardware/arm_bridge_node.py'
ros2 topic pub -1 /arm/command std_msgs/msg/Float64MultiArray \
    "{data: [90, 45, 120, 90, 90, 90]}"
ros2 topic echo /arm/joint_states                 # angles converge over ~1 s
```

Pi deployment: `scp -r hardware/ <user>@<pi-ip>:~/arm/` then
`ssh <user>@<pi-ip> 'bash ~/arm/pi5_setup.sh'`. Same LAN + same
`ROS_DOMAIN_ID` lets the laptop's ROS 2 see the arm topics.

## Gotchas

- **Servo power**: external 5–6 V supply (≥1 A per moving servo), **never** the
  Uno's 5V pin; tie all grounds together (supply ↔ Uno ↔ servos).
- **DTR reset**: opening the serial port reboots the Uno — `ArmSerial` waits
  ~2.5 s; don't "optimize" that away.
- No `/dev/ttyACM*`? Charge-only USB cables have no data lines; check
  `dmesg | tail`. Permission denied → you're not in `dialout`.
- Upload fails while a serial monitor is open — close the IDE monitor first.
- Toolchain is rootless (`arduino-cli` in `~/.local/bin`, core in
  `~/.arduino15`); never `apt install arduino`.
- The Uno R3 can't run micro-ROS — don't try to replace the text protocol.
