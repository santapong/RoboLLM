# humanoid_mirror — webcam body teleop of a humanoid

Your **left arm**, **right arm** and **head** drive a humanoid robot in MoveIt,
live from one USB webcam. CPU-only, no GPU, no Gazebo, no hardware.

> **Status: M0–M2 complete and verified. The humanoid moves.**
> `ros2-arm mirror synthetic` drives both arms, the head and the lift through a
> scripted whole-body sweep in RViz — no camera, and MediaPipe is never even
> imported. Camera tracking (M3–M4) is next; see [Build plan](#build-plan).

---

## MoveIt does not ship a humanoid

Worth stating plainly, because it is the first thing you will look for.
`moveit_resources` on Jazzy contains exactly three robots — Franka Panda,
Fanuc M-10iA, and **PR2, which is description-only**.

The PR2 URDF *is* humanoid-shaped (95 links, dual 7-DOF arms, a 2-DOF
`head_pan`/`head_tilt`, prismatic torso) and it is tempting. Don't take it: its
shipped SRDF is a deliberately truncated 75-line teaching stub with **one**
`disable_collisions` pair, literal `<!-- and many more ... -->` placeholders,
and **no head group at all**. `moveit_core` and `moveit_planners_ompl` both
declare it `<test_depend>` — it is a unit-test fixture, not a robot. There is
no `pr2_moveit_config` on ROS 2 anywhere. Every MoveIt-1-era humanoid asset
(`moveit_robots` with Atlas/Robonaut 2, `moveit_whole_body_ik`) is ROS 1, last
touched in 2019, never ported.

## So we use ROBOTIS FFW ("AI Worker")

The only robot that is **apt-installable on ROS 2 Jazzy with a MoveIt 2 config
that already has `arm_l`, `arm_r` and `head` planning groups.** Apache-2.0, so
it is clean for this public repo.

| | |
|---|---|
| Variant | `ffw_bg2_rev4_follower` (stationary — no swerve base) |
| Arms | 2 × 7-DOF |
| Head | 2-DOF neck |
| Torso | 1 prismatic lift (−0.5 … 0.0 m) |
| Grippers | 2 (present, not driven by mirroring) |
| Planning groups | `arm_l`, `arm_r`, `head`, `lift` — **418** real `disable_collisions` pairs |
| Geometry | 25 meshes, 26.8 MB |
| Licence | Apache-2.0 |

**It is a *semi-humanoid*: torso + dual arms + head on a lift column. There are
no legs.** For mirroring two arms and a head that is irrelevant, but don't call
it a biped.

Runner-up was a DIY MoveIt Setup Assistant config on the Unitree G1
(`g1_dual_arm.urdf`, BSD-3, clean 2×7-DOF arms). It lost because **the G1 has
no neck** — its `head_joint` is a *fixed* sensor mount — plus no SRDF exists
anywhere and its URDFs use relative mesh paths with zero `package://` URIs.
PAL's TALOS and TIAGo++ are the classic answer and are dead on Jazzy
(`pal_urdf_utils` has no Jazzy release, and neither SRDF defines a head group
anyway). NAO/Pepper meshes are CC BY-NC-ND with a click-through EULA —
disqualifying for a public repo.

---

## Run it

```bash
./docker/ros2-arm mirror synthetic         # 🎬 the humanoid MOVES in RViz
./docker/ros2-arm mirror-accept            # M2 acceptance (needs the above running)

./docker/ros2-arm humanoid                 # the robot alone, no motion
./docker/ros2-arm humanoid-check           # M1 acceptance (needs the above running)
```

`mirror synthetic` accepts every node parameter as `key:=value` — e.g.
`rviz:=false`, `latency_probe:=true`, `sweep_period_s:=8.0`, `use_lift:=false`.

Freeze and resume it live (the deadman — no jump on resume, verified):

```bash
ros2 service call /mirror_enable std_srvs/srv/SetBool "{data: false}"   # hold
ros2 service call /mirror_enable std_srvs/srv/SetBool "{data: true}"    # go
```

Native (Ubuntu 24.04 + ROS 2 Jazzy):

```bash
sudo apt install ros-jazzy-ffw-description ros-jazzy-ffw-moveit-config \
                 ros-jazzy-ffw-bringup ros-jazzy-realsense2-description
colcon build --packages-select humanoid_mirror
ros2 launch humanoid_mirror mock_bringup.launch.py
```

> `ffw-bringup` and `realsense2-description` are **mandatory but not declared
> as dependencies** of `ffw-description`. Without them, `xacro` on every
> follower `.urdf.xacro` dies with `PackageNotFoundError`.

`humanoid-check` output on a healthy stack — all 18 checks pass:

```
descriptions
  [PASS] robot_description is non-empty — 52663 chars
  [PASS] URDF is the bg2_rev4 variant (name matches the SRDF)
  [PASS] SRDF defines planning group 'arm_l' / 'arm_r' / 'head' / 'lift'
joint_states
  [PASS] all 19 mock joints present
controllers
  [PASS] arm_l / arm_r / head / lift controllers active, disjoint joint sets
inverse kinematics
  [PASS] /compute_ik SUCCESS for group 'arm_l' (7-DOF) — error_code=1
  [PASS] /compute_ik SUCCESS for group 'arm_r' (7-DOF) — error_code=1
```

---

## Three things that will bite you

**1. `ffw_moveit_config`'s own launch file crashes.** It calls
`.robot_description_semantic()` but never `.robot_description()`, and the
package neither ships a config URDF nor depends on `ffw_description`. You get
`Could not find parameter robot_description` → `XML_ERROR_EMPTY_DOCUMENT` →
`[FATAL] Unable to configure planning scene monitor` → SIGABRT. The bug is in
jazzy-branch HEAD too. `humanoid_mirror/ffw_config.py` is the corrected chain.

**2. Use `bg2_rev4`, not `sg2_rev1`.** `bg2_rev4`'s `<robot name>` is
`ffw_bg2_follower`, which **matches the SRDF**; `sg2_rev1` is
`ffw_sg2_follower`, which does not, and MoveIt then logs *"Semantic description
is not specified for the same robot as the URDF"*. `sg2_rev1` also has 3 broken
`${swerve_meshes_dir}` wheel meshes.

**3. The head barely moves.** Measured from the expanded URDF — and note the
axes are the **opposite** of the "pan/tilt" reading ROBOTIS's own docs suggest:

| Joint | Axis | Actual meaning | Range |
|---|---|---|---|
| `head_joint1` | Y | **PITCH** (nod) — positive = looking **down** | −0.2317 … +0.6951 rad (−13° … +40°) |
| `head_joint2` | Z | **YAW** (pan) | ±0.35 rad (**±20° only**) |

So head mirroring will be a nod and a glance, **not** a look-around. Retargeting
gains must be well below 1 (≈0.33 yaw, ≈0.6 pitch), because human yaw
comfortably reaches ±60°.

Also expect 7 benign `Link 'lift_link' / '*_wheel_*_link' is not known to URDF`
warnings — those are `disable_collisions` entries for links that exist only on
the SG2 variant. Harmless. Do not "fix" them by switching to SG2.

---

## Build plan

| | Milestone | Status |
|---|---|---|
| **M0** | numpy law fix + FFW and pose model baked into the image | **done, verified** |
| **M1** | humanoid loads, renders, plans; 4 controllers active; dual-arm IK | **done, verified** |
| **M2** | 🎬 humanoid *moving* in RViz — scripted sweep, no camera | **done, verified** |
| M3 | body tracking: TF + skeleton markers, robot parked | next |
| M4 | arms mirror live | |
| M5 | head + lift gains tuned against a real body | |
| M6 | *(optional)* Cartesian hands via a `pink` QP | |

`/mirror_enable` (planned for M5) landed early in M2 — the control loop needed
a freeze path anyway, and it is verified: frozen publishes **nothing**, and
resume re-seeds from `/joint_states` so there is no jump.

**M2 measured** (`mirror-accept`, 10 s window): 50.8 Hz on all four controller
topics, 0 joint-limit violations, 0 per-tick slew violations, 11/11 swept
joints moved, and the mock hardware tracked every command.

Every milestone has a no-camera synthetic test, per house convention.

Design decisions already locked in for M2+: **mirror mode** (your left arm →
the robot's right, as if it faces you), **direct joint-angle retargeting** (not
IK — link-length invariant, no singularities, and it makes simultaneous
arms+head a non-problem), and **`/mirror_enable` service as the deadman**
rather than fist/palm gestures.

See [TECHNICAL.md](TECHNICAL.md) for the component walkthrough and the full
retargeting math.
