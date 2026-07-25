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
    && /opt/mpvenv/bin/pip install --no-cache-dir --ignore-installed \
           mediapipe==0.10.35 "numpy==1.26.4" "opencv-contrib-python==4.10.0.84"

# WHY BOTH PINS: mediapipe 0.10.35 declares numpy AND opencv-contrib-python
# unpinned. Left alone, pip resolves opencv-contrib-python to 5.x, which
# hard-requires numpy>=2 — so the venv silently ends up on numpy 2.x while the
# system stays 1.26.4, violating the repo's numpy law from inside the very
# interpreter the launch files use. Measured symptom: cv_bridge's C extension
# was compiled against numpy 1.x, so `CvBridge().cv2_to_imgmsg(...)` raises
# `KeyError: 16` and no node can publish an annotated sensor_msgs/Image.
# This MUST stay in the build layer — downgrading numpy inside an already
# populated venv leaves it corrupted ("numpy._core.multiarray failed to
# import"). See ../../../constraints.txt for the matching native-route pins.
# cv_bridge lives in /opt/ros/jazzy/lib/python3.12/site-packages, which only
# joins PYTHONPATH once setup.bash is sourced — and setup.bash is bash-only,
# so this cannot be a bare `RUN /opt/mpvenv/bin/python -c ...`.
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/jazzy/setup.bash \
    && /opt/mpvenv/bin/python -c "import numpy, cv2; \
    assert numpy.__version__ == '1.26.4', 'numpy law broken: ' + numpy.__version__; \
    assert cv2.__version__.startswith('4.'), 'opencv 5.x breaks cv_bridge: ' + cv2.__version__; \
    print('venv numpy', numpy.__version__, '/ cv2', cv2.__version__)" \
    && /opt/mpvenv/bin/python -c "from cv_bridge import CvBridge; import numpy as np; \
    CvBridge().imgmsg_to_cv2(CvBridge().cv2_to_imgmsg(np.zeros((8,8,3), np.uint8), 'bgr8'), 'bgr8'); \
    print('cv_bridge roundtrip OK')"
SHELL ["/bin/sh", "-c"]

# Bake the hand landmarker model at build time so runtime needs no network.
# Pinned to the versioned float16/1 URL (NOT the mutable /latest/ path) and
# sha256-verified with `sha256sum -c` at build time — exactly like the gesture
# model layer — so a silent upstream rewrite fails the build loud. The expected
# hash is written to the .sha256 file first, so the recorded pin doubles as the
# build-time check and stays in the image for runtime re-verification.
RUN mkdir -p /opt/models \
    && curl -fL -o /opt/models/hand_landmarker.task \
       https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task \
    && echo "fbc2a30080c3c557093b5ddfc334698132eb341044ccee322ccf8bcf3607cde1  /opt/models/hand_landmarker.task" > /opt/models/hand_landmarker.task.sha256 \
    && sha256sum -c /opt/models/hand_landmarker.task.sha256

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

# ---- FFW / AI Worker semi-humanoid (humanoid_mirror) -----------------------
# ROBOTIS FFW is the only apt-installable ROS 2 Jazzy robot whose MoveIt config
# already ships arm_l / arm_r / HEAD planning groups (418 real disable_collisions
# pairs, Apache-2.0). MoveIt itself ships no humanoid: moveit_resources is
# Panda + Fanuc + a PR2 that is description-only, whose SRDF is a 75-line stub
# with one disable_collisions pair, no head group, and <test_depend> status.
#
# ffw-bringup and realsense2-description are MANDATORY but are NOT declared as
# dependencies of ffw-description — without them, xacro on every follower
# .urdf.xacro dies with PackageNotFoundError.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ffw-description \
    ros-jazzy-ffw-moveit-config \
    ros-jazzy-ffw-bringup \
    ros-jazzy-realsense2-description \
    ros-jazzy-pick-ik \
    && rm -rf /var/lib/apt/lists/*

# Pose landmarker (33 body landmarks). The "full" variant, not "lite": measured
# 24.8 ms vs 18.1 ms on the i3-9100, but 2-3x less jitter (wrist sigma_z 1.52 mm
# vs 10.88 mm). Inference cost is resolution-independent — the model resizes
# internally — so there is nothing to win by downscaling the capture.
# Same pinning discipline as the hand/gesture layers: versioned float16/1 URL
# (never the mutable /latest/ path) + sha256 checked at build, so a silent
# upstream rewrite fails the build loud instead of shipping a changed model.
RUN curl -fL -o /opt/models/pose_landmarker_full.task \
       https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_full/float16/1/pose_landmarker_full.task \
    && echo "5134a3aad27a58b93da0088d431f366da362b44e3ccfbe3462b3827a839011b1  /opt/models/pose_landmarker_full.task" | sha256sum -c -
