# Guided webcam setup — 7 September 2026

## Delivered behavior

The local console now links to `/camera`, with an operator-started preview, ChArUco corner count, actual image resolution, explicit photo capture, calibration and JSON download. No device opens during service construction, page loading or status reads. Only **Start camera** opens the selected local camera. The page is available for camera capture only in simulation mode without another webcam owner.

Captures are unannotated PNGs in separate `captures/camera-setup/session-…/` directories. Each session records the requested device/resolution, actual resolution, detected corners, capture time and SHA-256 of every saved view. Calibration verifies those hashes, uses the existing ChArUco solver and records the image hashes in the calibration identity. Exact duplicate frames are rejected; a session accepts up to 40 views. Passing requires at least 12 usable views, at least ten corners per view, varied board tilt and RMS no greater than 1 px. Twenty views are recommended for collection; the view count alone does not establish quality.

The download contains lens intrinsics at the recorded resolution. Base/mat and tool-marker/TCP transforms remain unset. It does not enable hardware or attach an unregistered preview to the experiment's webcam observations. Starting another session keeps prior files and clears the current result. Stop releases the device from its owning thread; stale/disconnected previews become HTTP 204 and show **Cannot detect**. Leaving the page requests a stop, but this is best-effort; use Stop camera or exit the local console if that request does not reach the server.

Follow [the commissioning runbook](COMMISSIONING.md#print-and-calibrate) for print scale, capture steps and subsequent measured registration.

## Checks actually run

Environment: existing isolated Python 3.12 environment, OpenCV contrib 4.10.0.84, NumPy 1.26.4, MuJoCo 3.10.0; branch `experiment/elexhub-webcam`, base HEAD `295f050`, with the earlier photo-model work retained uncommitted.

```sh
MUJOCO_GL=egl .venv/bin/python -m pytest tests/unit/test_elexhub_camera_setup.py tests/unit/test_elexhub.py tests/unit/test_arm_driver_foundation.py -q
.venv/bin/ruff check --ignore EXE002 src/robollm/elexhub tests/unit/test_elexhub_camera_setup.py tests/unit/test_elexhub.py examples/elexhub/bundle.py
```

- **64 tests passed**, including eight new camera test cases. Two existing Starlette/httpx/AnyIO dependency deprecation warnings remain. Logs are under `output/elexhub-camera-setup-review-2026-09-07/`.
- Fake devices verify no camera access from reads, validation failures or cross-origin starts; ownership exclusion in shadow/hardware modes; failed-open release; loss/recovery/staleness; and separation from the registered experiment feed.
- A synthetic 12-view board sequence passes the actual OpenCV solver. The test checks focal lengths against known 700 px intrinsics with a 12 px tolerance, RMS below 0.6 px, identity integrity and exact capture-file hashes. This is software evidence, not a calibration of the owner's lens.
- The synthetic test downloads a valid result, confirms registration is absent, rejects modified/missing saved images and preserves earlier sessions. A front-facing sequence without varied tilt is rejected.
- The inline JavaScript passed `node --check`. Visible in-app browser review at 666 × 726 px confirmed the responsive layout, no horizontal overflow, camera-off state, available Start control, disabled capture/calibration controls and readable steps. Live capture/result interactions were exercised through the API tests, not a physical browser camera session.
- Read-only HTTP checks passed for `/camera`, `/model`, the printable PDFs and camera state. Both absent setup and experiment webcam feeds returned 204. The real setup state remained idle with no capture session. The already-owned, idle local simulation server was restarted to load these changes.
- The read-only doctor still reports provisional geometry and hardware readiness false. No physical camera, serial device, PWM, firmware upload, Pi service or Kaggle operation was used. The frozen 6 September evaluation is unchanged.

## Remaining experiment work

The owner still needs to print the board, connect and calibrate their webcam, supply arm dimensions and servo labels, measure base/tool registration and verify held-out projection error. Wiring, power/cutoff checks, actuator calibration, shadow observations and physical trials remain pending. The software addition does not advance these physical acceptance gates.
