# Left-hand camera simulation

This local mode lets a left hand steer the photo-referenced Elexhub model in
MuJoCo. It is available at `http://127.0.0.1:8767/mirror` and uses five positioning
joints plus one gripper actuator. The physical profile is still unmeasured; the
model's dimensions, joint zeros, masses and collision shapes remain estimates.

## Run and use

From `/mnt/data/RoboLLM-elexhub`, using the already prepared environment:

```sh
MUJOCO_GL=egl .venv/bin/python examples/elexhub/run.py serve --mode sim --port 8767
```

Keep that terminal running. For a console that stays available independently of
the terminal or Codex command session, the local desktop also supports:

```sh
systemd-run --user --unit=robollm-elexhub-console --collect \
  --working-directory=/mnt/data/RoboLLM-elexhub --setenv=MUJOCO_GL=egl \
  /mnt/data/RoboLLM-elexhub/.venv/bin/python \
  /mnt/data/RoboLLM-elexhub/examples/elexhub/run.py serve --mode sim --port 8767
```

Check it with `systemctl --user status robollm-elexhub-console`. Stop it with
`systemctl --user stop robollm-elexhub-console`. This is a temporary user service,
not automatic startup after reboot. If **Start camera** is disabled and the page
says **Console disconnected**, restore the server first; a disconnected page
cannot start a camera. Avoid starting another process on the same port.

The console loads existing model assets without rewriting CAD, meshes, GLB or
review hashes. If the profile or appearance revision changes, rebuild the assets
explicitly with the existing model workflow before starting the console.

1. Open `/mirror` and click **Start camera**. Camera index 0 is the default.
2. Hold your **left palm** toward the camera with your whole hand visible. Green
   landmarks and **Left hand detected** indicate usable input. Right hands are
   ignored; ambiguous or uncertain detections hold movement.
3. Hold a comfortable position and click **Start mirroring**. The first valid
   hand observation becomes the neutral point at the robot's current position.
4. Move left/right or up/down to steer the gripper. A small change in apparent
   palm size controls forward/back movement. Pinch thumb and index finger to
   close the claw; separate them to open it.
5. **Hold position** pauses the arm while detection continues. **Recenter**
   chooses a new neutral point. **Camera off** stops movement and releases the
   capture device. **Reset pose** is available while mirroring is paused.

The preview is mirrored once, before detection. The left-hand label follows that
selfie-image convention. Keep only one left hand visible and face your palm
toward the lens during initial verification. A brief loss of tracking resets the
neutral reference on reacquisition, avoiding a jump to an old target.

## What the controls represent

Image displacement maps to relative base-frame Y/Z motion. Apparent palm-size
change is a bounded X control signal; it is **not measured depth**. A monocular
camera does not provide calibrated absolute hand position here. This mode
steers the robot gripper and does not estimate human shoulder/elbow angles.

The controller consumes only the latest sample at 20 Hz, limits target speed to
50 mm/s, applies the configured joint and gripper command rates, and rejects
unreachable or colliding simulation steps. Camera capture/inference can run
slower than the controller. Frames older than 350 ms cannot drive movement;
camera images older than one second disappear. Missing camera images return
HTTP 204 and never fall back to the simulation image.

Hand loss clears delayed arm commands and holds the current simulated pose. If
the page stops sending control heartbeats for 1.5 seconds, following stops and
requires **Start mirroring** again. Hiding or leaving the page sends a camera-stop
request. All frames are processed locally and kept in memory; this mode does not
record photos or video.

The photographed gripper linkage is a visual approximation. The simulation
still uses two coupled sliding jaws. This is a model interaction experiment,
not validation of physical servo dynamics or physical hand-to-robot alignment.

## Integration and verification

- `hand_camera.py`: local pinned MediaPipe Tasks model, handedness, normalized
  landmarks, pinch, fresh samples, camera ownership and annotated JPEG preview.
- `hand_control.py` and `SimArm.teleop_step`: neutral mapping, bounded IK and
  tentative physics, freshness guards and cancellation before committing motion.
- `web.py` and `hand_mirror.html`: separate camera and motion start/stop controls,
  simulation-only endpoints, same-origin mutations and task mutual exclusion.
- `camera_ownership.py`: ownership shared with the ChArUco calibration page.

The hand model is the already cached `hand_landmarker.task` with SHA-256
`fbc2a30080c3c557093b5ddfc334698132eb341044ccee322ccf8bcf3607cde1`.
The mirror service never downloads a model on startup. The existing NumPy and
OpenCV pins remain unchanged.

```sh
MUJOCO_GL=egl .venv/bin/python -m pytest \
  tests/unit/test_elexhub_hand_camera.py \
  tests/unit/test_elexhub_hand_control.py \
  tests/unit/test_elexhub_hand_web.py \
  tests/unit/test_elexhub_camera_setup.py \
  tests/unit/test_elexhub.py -q
```

Synthetic landmark tests establish software behavior, while a live user test is
needed to establish anatomical left/right labeling and comfortable control for
the current camera orientation. Dated live observations and the model-integrity
check are recorded in `output/elexhub-hand-mirror-review-2026-09-09/`.
