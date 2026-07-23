# Lean robot-arm image: ROS2 Jazzy + MoveIt2 + ros2_control
FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-moveit \
    ros-jazzy-moveit-resources \
    ros-jazzy-moveit-setup-assistant \
    ros-jazzy-moveit-servo \
    ros-jazzy-moveit-visual-tools \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    ros-jazzy-xacro \
    && rm -rf /var/lib/apt/lists/*

# Fast DDS — CycloneDDS discovery breaks on this multi-NIC host
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc

# MediaPipe hand tracking, isolated in a venv (system numpy/cv2 stay untouched;
# PEP-668 blocks pip on the system python otherwise). --system-site-packages
# lets rclpy / trajectory_msgs still import from inside the venv.
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-venv \
    curl \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/* \
    && python3 -m venv --system-site-packages /opt/mpvenv \
    && /opt/mpvenv/bin/pip install --no-cache-dir --ignore-installed mediapipe==0.10.35

# Bake the hand landmarker model at build time so runtime needs no network.
RUN mkdir -p /opt/models \
    && curl -fL -o /opt/models/hand_landmarker.task \
       https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task \
    && sha256sum /opt/models/hand_landmarker.task > /opt/models/hand_landmarker.task.sha256 \
    && cat /opt/models/hand_landmarker.task.sha256

# ---- Gen3-lite pick-and-place (gen3_pick_place) ----------------------------
# This network blocks plain-HTTP to archive/security.ubuntu.com (port 80
# connection failed) while HTTPS works — flip the Ubuntu sources to https or
# universe deps (freeglut3-dev, qml-*, libzmq3-dev) resolve to "not installable".
RUN sed -i 's|http://archive.ubuntu.com|https://archive.ubuntu.com|; s|http://security.ubuntu.com|https://security.ubuntu.com|' \
    /etc/apt/sources.list.d/ubuntu.sources

# Kinova Gen3 lite description + MoveIt config (binary, ROS build farm).
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-kortex-description \
    ros-jazzy-kinova-gen3-lite-moveit-config \
    && rm -rf /var/lib/apt/lists/*

# Released kortex_description 0.2.6 ships a broken gen3_lite macro (load_arm
# lacks the 'gripper' param -> "Invalid parameter gripper" on xacro). Overwrite
# it with the fixed jazzy-branch copy from Kinovarobotics/ros2_kortex, PINNED
# to commit f0a2d4c39bcd3e80da22d725a8b8936fd875b267 (jazzy HEAD for this file,
# 2026-06-08). File sha256 is verified so a silent upstream rewrite fails loud.
RUN curl -fL -o /opt/ros/jazzy/share/kortex_description/arms/gen3_lite/6dof/urdf/gen3_lite_macro.xacro \
       https://raw.githubusercontent.com/Kinovarobotics/ros2_kortex/f0a2d4c39bcd3e80da22d725a8b8936fd875b267/kortex_description/arms/gen3_lite/6dof/urdf/gen3_lite_macro.xacro \
    && echo "fc57029d94145cb4ad93c55bf98b69069e660f826d4a45eed5c1a7958193ec67  /opt/ros/jazzy/share/kortex_description/arms/gen3_lite/6dof/urdf/gen3_lite_macro.xacro" | sha256sum -c -

# Gesture recognizer model (Closed_Fist / Open_Palm), sha256-verified at build.
RUN curl -fL -o /opt/models/gesture_recognizer.task \
       https://storage.googleapis.com/mediapipe-models/gesture_recognizer/gesture_recognizer/float16/1/gesture_recognizer.task \
    && echo "97952348cf6a6a4915c2ea1496b4b37ebabc50cbbf80571435643c455f2b0482  /opt/models/gesture_recognizer.task" | sha256sum -c -
