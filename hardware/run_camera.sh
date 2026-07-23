#!/usr/bin/env bash
# run_camera.sh — publish the Pi's USB webcam as /camera/image_raw so the WHOLE
# existing pipeline lights up on the real arm with zero new capture code:
#   robot_bridge._on_image → web /api/camera (dashboard) and MCP get_camera_image.
#
# Needs the v4l2_camera ROS 2 node (pi5_setup.sh installs it):
#   sudo apt install ros-jazzy-v4l2-camera
#
# Find your device first:  ls /dev/video*   (usually /dev/video0)
#
#   bash hardware/run_camera.sh                 # /dev/video0 @ 640x480
#   VIDEO_DEVICE=/dev/video2 CAM_W=1280 CAM_H=720 bash hardware/run_camera.sh
#
# NOTE: Gazebo (sim) also publishes /camera/image_raw — run the sim OR this,
# not both, or the dashboard will interleave real and simulated frames.
set -e

DEV="${VIDEO_DEVICE:-/dev/video0}"
W="${CAM_W:-640}"
H="${CAM_H:-480}"

if [ ! -e "$DEV" ]; then
  echo "No camera at $DEV. Plug in a USB webcam and check: ls /dev/video*" >&2
  exit 1
fi

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash

echo "Publishing $DEV (${W}x${H}) -> /camera/image_raw  (Ctrl-C to stop)"
exec ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:="$DEV" \
  -p image_size:="[$W, $H]" \
  -r image_raw:=/camera/image_raw
