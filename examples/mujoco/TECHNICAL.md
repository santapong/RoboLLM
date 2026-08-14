# RoboLLM · MuJoCo technical notes

> **RoboLLM field guide** · Build → Observe → Measure → Learn<br>
> [Home](../../README.md) · [Examples](../README.md) · [Documentation](../../docs/README.md) · [Diagram](docs/mujoco-architecture.svg)

`examples/mujoco/` has two bounded CPU examples. `hello_mujoco.py` teaches the
model/data/step API with a one-hinge pendulum. `arm_dataset.py` adds the A3
path: an inline 6-DOF arm plus gripper, a smooth scripted policy, an offscreen
front camera, and direct LeRobot v3 episode recording. Neither path uses ROS.
The model is intentionally simple; it proves the data pipeline without
pretending to be a measured digital twin of the physical arm.

![hello_mujoco.py pipeline](docs/mujoco-architecture.svg)

## Component walkthrough

- **`MODEL_XML` (MJCF string).** A ground plane, a light, and a body `arm` at
  height 1 m with a `hinge` joint (axis `0 1 0`) and a 0.5 m capsule geom;
  gravity `0 0 -9.81`. MJCF is MuJoCo's native format (not URDF) — this is the
  smallest useful example of it.
- **`model = mujoco.MjModel.from_xml_string(MODEL_XML)`.** Parses and compiles
  the MJCF into the immutable model description (geoms, joints, masses,
  timestep — the default MJCF timestep is 2 ms).
- **`data = mujoco.MjData(model)`.** The time-varying state (`qpos`, `qvel`,
  `time`). The script sets `data.qpos[0] = 1.2` so the pendulum starts raised
  1.2 rad and has something to do. The model/data split is *the* core MuJoCo
  API idea — one compiled model can drive many independent data instances
  (that is exactly what MJX parallelizes on GPU).
- **Headless loop (default).** `mujoco.mj_step(model, data)` 1000 times
  (= 2.0 s of sim time at 2 ms/step), printing `t` and the hinge angle every
  100 steps, then `done.`.
- **Viewer loop (`--view`).** `mujoco.viewer.launch_passive(model, data)` opens
  a GLFW window; the script owns the physics loop (`mj_step` + `viewer.sync()`
  per frame) while the viewer handles camera/mouse. Needs a display.

## Key files

| File | Role |
|---|---|
| `hello_mujoco.py` | The whole example: MJCF model, compile, headless loop, `--view` viewer loop |
| `arm_dataset.py` | 6-DOF + gripper MJCF, scripted targets, offscreen camera, LeRobot v3 writer |
| `../../requirements-extra.txt` | Supplies the `mujoco` pip package (installed into the project venv) |
| `../../requirements-lerobot.txt` | Isolated NumPy-2 dataset environment; never install in the ROS venv |

## Interfaces

No ROS topics, services, or parameters — these are standalone Python scripts.

| CLI flag | Effect |
|---|---|
| *(none)* | Headless: 1000 steps, prints angle every 100 steps, exits |
| `--view` | Interactive 3-D viewer via `mujoco.viewer.launch_passive` (display required) |

`arm_dataset.py` accepts `--root`, `--repo-id`, `--task`, `--episodes`,
`--frames`, `--fps`, and `--validate-only`. Its saved vectors are ordered
`joint1` … `joint6`, `gripper`, matching the hardware logger's learning
boundary.

## Run + verify

```bash
# one-time: mujoco comes from requirements-extra.txt (numpy stays pinned)
.venv/bin/python -m pip install -c constraints.txt -r requirements-extra.txt

.venv/bin/python examples/mujoco/hello_mujoco.py            # prints the swing
.venv/bin/python examples/mujoco/hello_mujoco.py --view     # 3D viewer window

# Fast A3 policy check (no LeRobot install or dataset write)
.venv-lerobot/bin/python examples/mujoco/arm_dataset.py --validate-only

# One 5-second, 20 Hz LeRobot v3 episode with rendered front-camera video
.venv-lerobot/bin/python examples/mujoco/arm_dataset.py \
  --task "move every arm joint smoothly" --episodes 1 --frames 100 --fps 20
```

Expected headless output (verified with mujoco 3.10.0) — the angle starts at
+1.200 rad and swings back and forth under gravity:

```
t= 0.00s  hinge angle=+1.200 rad
t= 0.20s  hinge angle=+0.690 rad
t= 0.40s  hinge angle=-0.436 rad
...
t= 1.80s  hinge angle=-0.865 rad
done.
```

Exit code 0 and `done.` on the last line = pass. Headless mode needs no
display, no GPU, and no ROS environment, so it doubles as a smoke test that
the `mujoco` wheel is healthy in the venv.

## Gotchas

- **Install with the constraint file.** `mujoco` must not drag numpy to 2.x —
  always `pip install -c constraints.txt` (ROS Jazzy ABI law; see repo
  `CLAUDE.md`).
- **Keep the dataset environment separate.** LeRobot 0.6 needs NumPy 2.x.
  Follow `hardware/README.md` to install CPU-only PyTorch before
  `requirements-lerobot.txt`; the latter pins TorchCodec to the PyTorch-2.10
  compatible line.
- **A3 is pipeline evidence, not sim-to-real evidence.** The inline arm has
  generic links and actuators. Replace it with measured geometry and dynamics
  only after the physical worksheet exists.
- **`--view` needs a display** (GLFW). Over SSH/headless it will fail to open a
  window — use the default headless mode there.
- **MJCF ≠ URDF.** MuJoCo's native format is MJCF; the URDFs produced by
  `cad/` and `scan3d/` target Gazebo/PyBullet. (MuJoCo can compile URDF too,
  but this example deliberately teaches MJCF.)
- The `t= 0.00s` first line is printed *after* the first step (t = 0.002 s,
  rounded) — the loop steps first, then prints on `step % 100 == 0`.
- `launch_passive` means the *script* drives time: forget `mj_step` in the
  loop and the viewer shows a frozen scene; forget `viewer.sync()` and nothing
  updates on screen.
- Heavy RL / massively parallel sim belongs on cloud GPU via MuJoCo MJX — this
  laptop (no NVIDIA GPU) is for exactly this kind of small CPU run.
