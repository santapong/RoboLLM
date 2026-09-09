# RoboLLM Humanoid Lab

A local, interactive **Unitree G1 in MuJoCo** with scripted reaching, waving,
draggable targets, and optional webcam palm targeting. Open the viewer to see
real joint dynamics under gravity. The pelvis is fixed to a visible stand;
this experiment focuses on arm control, not walking or balance.

## Run on Santapong's machine

```bash
cd /home/santapong/RoboLLM-harness-wam
examples/humanoid_harness/run.sh
```

Open **http://127.0.0.1:8846**. The demo starts with scripted reaching and the
camera off. `run.sh` downloads the pinned model only if the local provenance
manifest is absent. Every launch verifies the asset hashes. No ROS install is
needed for this MuJoCo example.

1. **Run reaching example** moves the orange target; the selected hand follows.
2. **Wave** shows a joint-space pose example. **Hold robot** captures the current
   joint pose and settles the position servos; it does not freeze physics.
3. Select **Left/Right** to change the controlled arm. Enable **Drag the orange
   target** to use its three translation handles. Targets stay inside the arm's
   configured workspace.
4. **Start camera** opens `/dev/video0`. Raise one open hand into the preview;
   move sideways/up/down to guide the robot hand. **Stop camera** releases it.
   Only the preview is mirrored; inference uses the raw image.
5. Use **Session → Shut down demo**, or Ctrl+C in its terminal, to stop the server.
   Closing every viewer stops camera capture after five seconds.

The local viewer can display camera frames in memory. The program does not
record frames or send them to a cloud API. Runtime status contains operational
metadata only. With no hand, a stale frame, or invalid coordinates, the harness
holds the joints. The latest target is never treated as a fresh observation.

If the camera reports **missing or busy**, close camera capture in the other
application (for example, Elexhub) and click **Start camera** again. The demo
does not take the device away from another process.

## What the example teaches

```text
USB camera → local MediaPipe hand landmarks → normalized palm target
                                            ↓
                  freshness gate → workspace clamp → target slew limit
                                            ↓
                         classical IK → joint position servos
                                            ↓
                      MuJoCo G1 → live 3D viewer + status
```

- **Harness:** the code around a controller: observations, timing, constraints,
  hold behavior, visualization and evaluation. Here it bounds target movement
  to 12 mm per 40 ms update and joint control setpoint movement to 1.1 rad/s.
  The latter bounds commanded setpoints, not measured physical joint velocity.
- **VLM:** a vision-language model interprets images and language. MediaPipe is
  a hand-landmark detector; this demo does not run a VLM.
- **WAM:** the world-action model direction studied in the branch's research
  brief learns predictive representations and actions. This humanoid controller
  is classical IK; no humanoid WAM has been trained.
- **Learning policy:** the separate [CPU learning pilot](../../sim/vla-bed/experiments/harness_wam/README.md)
  compares behavior cloning with auxiliary future prediction and shuffled-future
  controls. Its UR5e-style action space and results cannot be transferred to G1
  by relabeling coordinates.

The paper reading order and evidence are in the
[research/experiment brief](../../sim/vla-bed/experiments/harness_wam/README.md).
A next controlled G1 learning experiment would collect simulated reaches,
split trajectories by seed/target region, fit BC and future-prediction variants,
then compare held-out rollouts under the same harness, including dropped or
delayed observations. That training experiment is **planned, not executed**.

## Environment and model

The existing environments are reused without installing or merging packages:

| Role | Verified interpreter | Key versions |
| --- | --- | --- |
| Browser + simulation | `/home/santapong/RoboLLM/.venv-vla-bed/bin/python` | MuJoCo 3.10.0, NumPy 2.5.2, Viser 1.1.0, mjviser 0.0.14, Pillow 12.3.0 |
| Camera worker | `/mnt/data/RoboLLM-elexhub/.venv/bin/python` | NumPy 1.26.4, OpenCV 4.10.0, MediaPipe 0.10.35 |
| Dynamics tests | `/home/santapong/RoboLLM/.venv-lerobot/bin/python` | MuJoCo 3.10.0, NumPy 2.2.6, pytest 9.1.1 |

Set `ROBOLLM_SIM_PYTHON`, `ROBOLLM_CAMERA_PYTHON`, and `ROBOLLM_HAND_MODEL` to
override the machine-specific defaults. `--port`, `--camera`, `--camera-python`,
`--hand-model`, and `--duration` are also available. Camera and viewer communicate
through an owned local subprocess pipe because their NumPy/vision environments
differ. The local hand model is reused from
`/mnt/data/RoboLLM-elexhub/.cache/perception/hand_landmarker.task` (SHA256
`fbc2a30080c3c557093b5ddfc334698132eb341044ccee322ccf8bcf3607cde1`).
If hand tracking cannot initialize, `auto` mode labels a **red-marker heuristic**
fallback in the preview/status; it must not be interpreted as hand tracking.

The public G1 assets are pinned to Google DeepMind Menagerie revision
`8161bba264d7fa7c99ca301e91e7fb44737676ad`; 38 files, about 19.7 MB.
See [model provenance and license](MODEL_NOTICE.md). Downloads, renders and
runtime metadata live in ignored `assets/`, `artifacts/`, and `.runtime/`.
Viewer library: [Viser](https://viser.studio/main/).

## Reproduce the checks

```bash
# Three five-second reaches and PNG frames (EGL headless rendering).
MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 \
  /home/santapong/RoboLLM/.venv-lerobot/bin/python \
  examples/humanoid_harness/simulation.py

# Dynamics and camera-target adapter (no camera opened).
OPENBLAS_NUM_THREADS=1 /home/santapong/RoboLLM/.venv-lerobot/bin/python -m pytest -q \
  tests/unit/test_humanoid_harness_simulation.py \
  tests/unit/test_humanoid_harness_adapter.py

# Camera and subprocess freshness tests (synthetic frames/packets only).
/mnt/data/RoboLLM-elexhub/.venv/bin/python -m pytest -q \
  tests/unit/test_humanoid_harness_camera.py \
  tests/unit/test_humanoid_harness_bridge.py

# Viewer event regressions with real MuJoCo and fake GUI/camera handles.
/home/santapong/RoboLLM/.venv-vla-bed/bin/python -m pytest -q \
  tests/unit/test_humanoid_harness_app.py

# Bounded viewer smoke check; camera stays off.
examples/humanoid_harness/run.sh --duration 8
```

See [verification record](verification-2026-09-09.md) for the measured results
and the distinction between verified capture and human-controlled motion.
