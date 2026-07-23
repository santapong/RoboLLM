#!/usr/bin/env bash
# pi5_setup.sh — run this ON the Raspberry Pi 5 to prepare it as the arm's brain.
# Assumes Ubuntu 24.04 (server or desktop) on the Pi — the same OS as the
# laptop, so ROS 2 Jazzy installs the same way. (On Raspberry Pi OS, use
# Docker: ros:jazzy-ros-base — see README.)
#
#   scp -r hardware/ pi@<pi-ip>:~/arm/ && ssh pi@<pi-ip> 'bash ~/arm/pi5_setup.sh'
set -e

echo "== [1/5] serial permissions =="
sudo usermod -aG dialout "$USER"

echo "== [2/5] base tools + arduino-cli (flash the Uno from the Pi too) =="
sudo apt update
sudo apt install -y python3-pip python3-serial git curl
if ! [ -x "$HOME/.local/bin/arduino-cli" ]; then
  ARCH=ARM64 # Pi 5
  VER=$(curl -fsSL https://api.github.com/repos/arduino/arduino-cli/releases/latest | grep -F tag_name | awk -F'"' '{print $4}')
  mkdir -p "$HOME/.local/bin" /tmp/acli && cd /tmp/acli
  curl -fsSLO "https://github.com/arduino/arduino-cli/releases/download/${VER}/arduino-cli_${VER#v}_Linux_${ARCH}.tar.gz"
  tar xzf "arduino-cli_${VER#v}_Linux_${ARCH}.tar.gz"
  install -m755 arduino-cli "$HOME/.local/bin/"
  cd -
fi
"$HOME/.local/bin/arduino-cli" core update-index
"$HOME/.local/bin/arduino-cli" core install arduino:avr
"$HOME/.local/bin/arduino-cli" lib install Servo

echo "== [3/5] ROS 2 Jazzy (skip if already installed) =="
if ! [ -f /opt/ros/jazzy/setup.bash ]; then
  sudo apt install -y software-properties-common curl
  sudo add-apt-repository -y universe
  export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F tag_name | awk -F'"' '{print $4}')
  curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${VERSION_CODENAME})_all.deb"
  sudo apt install -y /tmp/ros2-apt-source.deb
  sudo apt update
  sudo apt install -y ros-jazzy-ros-base python3-colcon-common-extensions
  echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
else
  echo "ROS 2 Jazzy already present"
fi

echo "== [3b/5] webcam publisher (v4l2_camera -> /camera/image_raw) =="
# lets run_camera.sh stream the real arm into the same dashboard/MCP camera path
if [ -f /opt/ros/jazzy/setup.bash ]; then
  sudo apt install -y ros-jazzy-v4l2-camera \
    || echo "  (install later: sudo apt install ros-jazzy-v4l2-camera)"
fi

echo "== [4/5] udev rule: Uno always shows up as /dev/arm_uno =="
sudo tee /etc/udev/rules.d/99-arm-uno.rules >/dev/null <<'EOF'
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", SYMLINK+="arm_uno", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", SYMLINK+="arm_uno", MODE="0666"
EOF
sudo udevadm control --reload-rules

echo "== [5/5] done =="
echo "Log out/in for the dialout group, plug in the Uno, then:"
echo "  bash ~/arm/check_arduino.sh"
echo "  source /opt/ros/jazzy/setup.bash && python3 ~/arm/arm_bridge_node.py"
echo "  bash ~/arm/run_camera.sh          # optional: stream the webcam"
