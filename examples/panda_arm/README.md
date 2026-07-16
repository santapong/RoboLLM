# panda_arm — 7-DOF arm: FK/IK → serial → Arduino → vision pick & place

A progressive series on the Franka Panda that teaches the full manipulation
pipeline, ending at the same place `../../hardware` starts — joint angles
over serial to a microcontroller:

```
camera pixels → object pose → grasp pose (6-DOF) → trajectory → IK → joint
angles → serial → Arduino → servos
```

All simulation (RViz2 + a virtual Arduino on a pty) — no hardware needed.

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch examples/panda_arm/05_vision_sort.launch.py    # the full pipeline
```

## The demos, in order

| Launch | You learn |
|--------|-----------|
| `01_sliders.launch.py` | URDF in RViz, `robot_state_publisher`, joint sliders |
| `02_kinematics.launch.py` | **FK & IK with the math visible**: 4×4 transform chain, geometric Jacobian, damped-least-squares IK with a per-iteration convergence log, live cross-check against TF |
| `03_pick_place.launch.py` | Full pick & place; every joint target streamed over serial to a **virtual Arduino** — a GUI serial monitor shows the exact bytes a real board would get |
| `04_sort.launch.py` | Grasp poses **computed from detections** (not hardcoded): random cube poses, yaw-aligned grasps, color bins, stacking |
| `05_vision_sort.launch.py` | **The camera solves positions from pixels**: synthetic camera image → OpenCV (HSV → contours → minAreaRect) → back-project centroid ray onto the table plane → grasp; trapezoidal velocity trajectories |

## Key files

- `panda_kinematics.py` — FK/IK library parsed from the URDF (chain-product
  FK, geometric Jacobian, DLS IK). `python3 panda_kinematics.py` self-tests.
- `table_camera.py` — pinhole camera model + real OpenCV detection; documents
  intrinsics/extrinsics and the pixel→ray→plane solve.
  `python3 table_camera.py` self-tests (sub-mm accuracy).
- `arduino/panda_arm_servo.ino` — minimal educational firmware:
  receives `S,<j1°>,…,<j7°>,<grip°>\n`, replies `ACK`, rate-limits servos.
  The Arduino never does kinematics — it only gets target joint angles.
  (For the project's real-arm serial stack see `../../hardware/`.)
- `arduino_sim.py` — that firmware emulated on `/tmp/ttyPANDA`. With real
  hardware: flash the `.ino` and run demos with `PANDA_SERIAL=/dev/ttyUSB0`.

## Concepts

- **FK**: `T = Π Trans(xyzᵢ)·Rot(rpyᵢ)·Rot(axisᵢ, qᵢ)` down the URDF chain
- **IK**: `dq = Jᵀ(JJᵀ + λ²I)⁻¹ e`, error = position + rotation vector
- **Grasp pose**: a 6-DOF target derived from the detected object pose —
  position from vision, orientation top-down yaw-aligned to the object,
  plus approach/lift/drop poses (drop height depends on bin stack level)
- **Camera position solve**: one pixel = a ray, not a point; intersect the
  ray with the known table plane to recover (x, y, z)
- **Trajectory**: straight-line Cartesian path + trapezoidal velocity
  profile `v = min(v_max, √(2·a·d_remaining))`, IK at 50 Hz

Requires `moveit_resources_panda_description` (apt, comes with the
`ros-jazzy-moveit-resources` family), plus numpy / PyQt5 / pyserial / OpenCV.
