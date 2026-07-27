# Humanoid-mirror image + Pinocchio/Pink differential IK, for the Talos
# full-body humanoid. FROM ros2-arm:jazzy (built from
# examples/humanoid_mirror/docker/Dockerfile) rather than re-deriving from
# osrf/ros:jazzy-desktop: it already contains everything humanoid_mirror
# needs (MoveIt2, moveit-setup-assistant, ros2_control, FFW description +
# MoveIt config + bringup, Kinova gen3-lite (patched xacro), pick_ik,
# realsense2-description, and /opt/mpvenv with mediapipe 0.10.35 + the
# hand/gesture/pose landmarker models, numpy==1.26.4 pinned and verified).
# Rebuild ros2-arm:jazzy first (docker build -t ros2-arm:jazzy
# ../humanoid_mirror/docker/) if this base has drifted.
FROM ros2-arm:jazzy

ENV DEBIAN_FRONTEND=noninteractive

# ---- Pinocchio + Pink differential IK, into the SAME /opt/mpvenv ----------
# Talos is a full-body (~30+ DOF) humanoid with no chain-solver-friendly
# single kinematic chain to hand to KDL/TRAC-IK/pick-ik (same "one node is
# one group is one chain solver" limitation documented in
# ../../humanoid_mirror/TECHNICAL.md's MoveIt-Servo section) — so, like every
# published humanoid teleop system (Open-TeleVision, Unitree xr_teleoperate,
# ACE), this uses Pinocchio's whole-body forward kinematics + Pink's
# task-space differential IK (QP-based damped least squares) instead.
#
# pip package names differ from the import names: "pin" -> `import
# pinocchio`, "pin-pink" -> `import pink`. Both are cmeel-packaged wheels
# (prebuilt native libs under a vendored cmeel.prefix/, no compiler needed).
#
# THE NUMPY TRAP HERE: pin-pink's newest releases (>=1.0) depend on
# numpy>=2, and pin's newest releases (>=3.x) pull in cmeel-boost/eigenpy
# builds that also drift to numpy>=2 transitively. Installing bare `pin
# pin-pink` against an unconstrained resolver upgrades numpy exactly like
# the mediapipe/opencv-contrib-python trap this repo already hit once (see
# ../../humanoid_mirror/TECHNICAL.md, "The numpy fix that shipped with M0").
# Pinning via -c constraints.txt (numpy==1.26.4, opencv-contrib-python<5,
# copied from repo-root ../../../constraints.txt so this docker/ build
# context stays self-contained) makes pip's resolver back off to versions
# that keep numpy 1.26.4: pin==2.7.0 + pin-pink==4.3.0, verified empirically
# against this exact base image before this Dockerfile was written.
#
# `quadprog` is the QP backend solve_ik() needs — pink/qpsolvers ship no
# solver by default and raise `SolverNotFound` without one. quadprog has no
# numpy>=2 requirement and needs no extra native deps beyond a C compiler
# already present in the base image's toolchain-free wheel install (it too
# ships as a manylinux wheel, so no compiler is actually invoked).
#
# This MUST be a build-layer install into the venv exactly as it already is
# (numpy already correctly pinned at 1.26.4 from the humanoid_mirror layer)
# — never a runtime pip step, and never a downgrade of an already-drifted
# venv (that corrupts numpy per the repo's numpy law).
COPY constraints.txt /tmp/constraints.txt
RUN /opt/mpvenv/bin/pip install --no-cache-dir -c /tmp/constraints.txt \
        pin pin-pink quadprog \
    && rm -f /tmp/constraints.txt

# Build-layer verify: import pinocchio + pink, build a real (if generic)
# articulated model, run exactly ONE pink differential-IK solve_ik() step
# against a QP solver, and hard-assert the numpy law. Any failure here fails
# `docker build` — this repo ships no broken IK layer (see house law: if
# pin/pink don't work with numpy 1.26.4, report the conflict instead of
# shipping this layer; verified compatible before writing this file).
# Same SHELL dance as the cv_bridge check below: none of this actually needs
# setup.bash (pinocchio/pink are pure /opt/mpvenv, no ROS import), but the
# bash SHELL is kept for consistency with the numpy/cv2 verify step and any
# future check that does need `source`.
SHELL ["/bin/bash", "-c"]
RUN /opt/mpvenv/bin/python - <<'PYEOF'
import numpy
assert numpy.__version__ == "1.26.4", "numpy law broken after pin/pink install: " + numpy.__version__

import pinocchio as pin
import pink
from pink import Configuration
from pink.tasks import FrameTask
from pink.solve_ik import solve_ik

# A generic 6-DOF sample manipulator is enough to exercise the real
# build->configure->task->solve_ik call chain; it does not need to be Talos
# itself (Talos's URDF/SRDF lands in a later task, and this Dockerfile must
# build standalone before that).
model = pin.buildSampleModelManipulator()
data = model.createData()
q0 = pin.neutral(model)
configuration = Configuration(model, data, q0)

ee_frame = model.frames[-1].name
task = FrameTask(ee_frame, position_cost=1.0, orientation_cost=1.0)
task.set_target(configuration.get_transform_frame_to_world(ee_frame))

dt = 0.01
velocity = solve_ik(configuration, [task], dt, solver="quadprog")
assert velocity.shape == (model.nv,), f"unexpected solve_ik output shape: {velocity.shape}"

print(f"pinocchio {pin.__version__} / pink {pink.__version__} OK")
print(f"solve_ik OK: {ee_frame} task, dt={dt}, |dq|={float(numpy.linalg.norm(velocity)):.6f}")
print("numpy law OK:", numpy.__version__)
PYEOF
SHELL ["/bin/sh", "-c"]
