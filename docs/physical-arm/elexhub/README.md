# Elexhub tabletop experiment

This is an isolated, CPU-only Elexhub experiment workspace. It contains a working MuJoCo demonstrator, editable provisional CAD, a webcam interface, a PCA9685 firmware backend, and reproducible trial tools. **It does not access Kaggle or modify the UR5e VLA-bed experiment.**

The hardware layout is **five arm joints plus one gripper: six motors total**. The appearance now follows Elexhub's photographs of the black metal kit, which the owner confirmed matches their arm. Dimensions, masses, joint zeros and actuator parameters remain provisional. See [the photo-model review](MODEL-REVIEW-2026-09-07.md) for sources, estimates and validation, and [the dimension/component research](DIMENSIONS-AND-COMPONENTS-2026-09-07.md) for published parts and their limits.

## Open the demonstrator

From this worktree:

```sh
uv venv --python 3.12 .venv
uv pip install --python .venv/bin/python -c requirements/ros-constraints.txt -r requirements/elexhub.txt
.venv/bin/python examples/elexhub/run.py prepare-perception
MUJOCO_GL=egl .venv/bin/python examples/elexhub/run.py serve
```

Open **http://127.0.0.1:8767**. Click a destination, type `reach here`, preview, check the clear-workspace box, and confirm. The default box starts at mat coordinates `(90, 80)` mm. For the checked pick/place example, select `(100, 80)` mm and enter `move this red box here`.

Open **http://127.0.0.1:8767/model** to rotate and zoom the reconstructed arm, or use **Inspect 3D model** in the console. The viewer also works by opening `assets/elexhub/provisional/model.html` locally; it contains its model data and has no external JavaScript dependencies. The live view shows **Cannot detect** when its selected feed is unavailable.

```sh
MUJOCO_GL=egl .venv/bin/python examples/elexhub/run.py demo --task reach
MUJOCO_GL=egl .venv/bin/python examples/elexhub/run.py demo --task pick_place
.venv/bin/python examples/elexhub/run.py doctor
```

The service binds only to loopback. No Pi deployment, physical USB connection, camera recording, firmware upload or model training occurs in these simulation commands. For a future Pi-hosted console, use an SSH tunnel to loopback after a separate Pi deployment; this implementation creates no public route. Browser microphone permission is needed only when the operator clicks **Record instruction**. Vosk transcribes locally and returns text for another explicit preview.

## Guided webcam preparation

Open [Set up webcam](http://127.0.0.1:8767/camera) in the local simulation console. Print the supplied ChArUco sheet, click **Start camera**, capture 12–20 varied views, then **Calibrate camera** and download the intrinsics JSON. The page shows the actual resolution, detected corners, capture count and reprojection RMS. Opening the page does not start the camera. Missing/stale frames show **Cannot detect**.

Captured photos stay in the ignored `captures/camera-setup/` directory and are excluded from the bundle. This step calibrates the lens; measured camera/base/tool registration and real hardware checks remain pending. Follow [the commissioning guide](COMMISSIONING.md#print-and-calibrate) for the next steps. Software validation uses fake camera devices and synthetic board images, not the owner's webcam. See [the camera setup review](CAMERA-SETUP-2026-09-07.md).

## Deliverables

| Item | Location | Meaning |
|---|---|---|
| Editable FreeCAD | `assets/elexhub/provisional/elexhub.FCStd` | 283 valid solids; editable plate sketches, extrusions, boxes and cylinders; colors and reopen verified |
| STEP | `assets/elexhub/provisional/elexhub.step` | Reimported; solid count, volume and global placements verified |
| MuJoCo | `assets/elexhub/provisional/elexhub.xml` | Five arm hinge joints, coupled jaws, free red box |
| URDF and OBJ | `assets/elexhub/provisional/` | Same joint frames and geometry source |
| Interactive model / GLB | `assets/elexhub/provisional/model.html`, `elexhub.glb` | Rotate, zoom, inspect and download; GLB uses Y-up |
| Photo-model review | `output/elexhub-model-review-2026-09-07/` | Four MuJoCo renders, hashes, test and CAD evidence; no physical evidence |
| Reference pack | `output/pdf/elexhub-reference-sheets.pdf` | Three A4 pages: ChArUco, workspace mat, tool/object markers |
| Wiring/worksheet | `output/pdf/elexhub-wiring-and-measurements.pdf` | Two A4 pages; unknown ratings remain blank |
| Editable print graphics | `output/pdf/*.svg` | Metric SVG versions |
| Explorable wiring | `docs/physical-arm/elexhub/wiring.html` | Self-contained architecture diagram |
| Trial evidence | `output/elexhub-evaluation-2026-09-06/` | Frozen 100 goals × A/B/C; no physical evidence |
| Physical trial sheets | `docs/physical-arm/elexhub/trials/` | 108 reach, 30 intent, 20 pick/place rows; results blank |

Read [the implementation plan](PLAN.md), [the commissioning runbook](COMMISSIONING.md), [the paper-to-project brief](RESEARCH.md), and [validation evidence](VALIDATION.md).

## Editable hardware profile and visual construction

`ros2/robo_arm_driver/config/elexhub.yaml` is the source of dimensions, joint mapping, limits, power evidence and measured servo specifications. `elexhub.sim.yaml` is generated from it with an explicit simulation runtime. Regenerate both downstream artifacts after changing the source:

```sh
.venv/bin/python examples/elexhub/run.py sync-sim-profile
.venv/bin/python examples/elexhub/run.py build
.venv/bin/python hardware/generate_firmware_config.py --config ros2/robo_arm_driver/config/elexhub.yaml --output hardware/firmware/elexhub_pca9685/arm_config.h
```

To regenerate editable CAD, install FreeCAD and pass its command executable:

```sh
.venv/bin/python examples/elexhub/run.py cad --freecad /path/to/freecadcmd
```

For CAD colors, use the GUI executable offscreen (the verified portable runtime is shown here):

```sh
.venv/bin/python examples/elexhub/run.py cad --freecad /mnt/data/.cache/elexhub-freecad/squashfs-root/AppRun --gui-colors
MUJOCO_GL=egl .venv/bin/python examples/elexhub/render_model.py --output output/model-review
```

`src/robollm/elexhub/appearance.py` defines the estimated sheet-metal outlines, holes, servo cases, fasteners and cable details. Its shared solid description feeds every visual export. Edit joint dimensions in YAML and visual construction details there; edits made directly in FreeCAD must be transferred back before regeneration. No vendor CAD or product images are bundled.

The FreeCAD 1.1.3 portable runtime used for validation is cached locally outside this repository. Its official AppImage SHA-256 was `3a853eb69ee595f779f2255dbf80a765926981d8ff68903cefee4dfb03a8f5ef`. For the extracted runtime, pass its `AppRun`; the CLI supplies `freecadcmd`. The runtime itself is not bundled. FreeCAD spreadsheets edit the geometry in the document, but regeneration reads the YAML; transfer final measurements back into YAML to preserve one source.

## Limits of this release

The CAD is a parameterized approximation, not manufacturer CAD, a scan, a fit-checked replacement part or manufacturing geometry. Link masses use box inertias and simplified collision proxies. The photographed claw linkage is represented visually at its open pose; its simulated contact mechanism still uses two coupled sliding jaws. The decorative links do not reproduce the real four-bar motion, and the visual meshes do not define measured collision envelopes. The wrist zero in the demonstration is a provisional 80-degree raw datum; it is not a physical calibration instruction.

The 7 September photo model uses estimated 55 mm claw opening and 65 mm finger length. Its provisional mat origin is `(0.23, -0.09, 0)` m, with 52 mm transit height. Grasp height accounts for the longer fingertips, and closure accounts for the nominal box width. A consistent attainable downward orientation prevents wrist spin during transfer. The archived 6 September evaluator and results are unchanged; its scores do not evaluate this revised model.

The RGB interface uses a calibrated table plane and known marker dimensions. Pointing is **touch/hover selection by projecting the fingertip pixel onto the table**. It does not implement arbitrary distant 3D pointing or recover metric depth from an uncalibrated monocular image. Red color detection is a constrained one-box baseline; physical lift/placement verification requires tag 70. The hand detector is an operating aid, not a safety-rated interlock.

The ROS hardware adapter is implemented but has not been exercised against this arm, a real PCA9685, or the Pi ROS deployment. Hardware remains disabled until measured-profile and camera gates pass. See the runbook for the remaining bench checks. No simulation score is evidence of physical readiness.
