# Humanoid gallery verification — 2026-09-09

This extends the [G1/camera verification](verification-2026-09-09.md) on local
branch `experiment/harness-vlm-wam`. Two agents handled the public model adapter
and gallery controls; the root agent generated the design, authored/exported the
original model, checked dynamics and inspected the visible browser.

## Delivered models and provenance

| Model | Geometry source | Supported simulation |
| --- | --- | --- |
| G1 | Existing pinned Menagerie assets | 29 joints, seven-joint arms |
| H1 | Menagerie `8161bba264d7fa7c99ca301e91e7fb44737676ad` | 19 joints, four-joint arms |
| Concept 01 | Original generated turnaround and authored mesh parts | 29 joints, seven-joint arms |

H1's 24 downloaded files were checked against upstream Git blob identities and
saved with SHA256 provenance. Its BSD-3-Clause license and attribution remain
with the assets; see [MODEL_NOTICE](MODEL_NOTICE.md). The adapter changes H1's
torque motors into position servos while preserving their torque ceilings.

Concept 01 was designed using the **built-in image_gen tool**. The exact
[prompt](models/concept01/prompt.txt) and unmodified
[four-view image](models/concept01/turnaround.png) are retained. The subsequent
3D model is an authored approximation guided by that image, not an automatic
neural reconstruction. Its scale and physical parameters are estimates.

The [model package](models/concept01/README.md) contains an editable Python
builder, MJCF, 41 closed positive-volume mesh assets, a portable Y-up GLB with
74 named visual parts, and four orthographic renders from the finished model.
The GLB has a named rigid-part hierarchy in its reference pose; actuation is in
MJCF, and no animation clip or skin deformation is claimed.

## Executed dynamics checks

| Model | Five-second reaches | Final hand-position errors | Joint/numerical faults |
| --- | --- | --- | --- |
| H1 | Right default, right offset, left default | 3.96 / 5.69 / 3.96 mm | None reported |
| Concept 01 | Default and offset for each arm | 9.74 / 14.29 mm per arm | None; per-step joint bounds checked |

Raw reports: [H1](results/h1-reaches-2026-09-09.json),
[Concept 01](models/concept01/verification.json). These are bounded arm-reaching
smoke checks. The pelvis is supported; the custom model disables self-collision
and has fixed fingers. No walking, balance, grasping, physical robot validation
or learned humanoid policy is established.

The custom OBJ assets were reloaded and checked for finite geometry, closed
surfaces and positive volume. The exported GLB was reloaded to check its named
hierarchy, Y-up convention and approximately 1.61 m height. All four final
renders were visually inspected. Build, model, GLB, reference and render hashes
are in the [build report](models/concept01/build-report.json) and verification.
The simulator's runtime model hash covers the decorated scene, including
absolute mesh paths and framebuffer dimensions; it differs from the standalone
robot XML hash in the build report.

## Browser and regression checks

The visible loopback viewer at `http://127.0.0.1:8846/` was exercised with all
three models. Switching replaces the robot scene, selects the new workspace,
resets to HOLD, and releases this demo's camera. H1 scripted reaching was also
observed live. Front, side, rear and three-quarter controls were inspected on
the original model; its reference folder displays the generated design sheet.
The viewport was the actual docked application pane, without a viewport override.
The final startup framing correction was verified after a server restart and
page reload, without clicking a view button: the full original robot remained
visible during reaching. Initial camera fitting is bounded and does not keep
overriding user orbit/zoom. View-button state is synchronized on model changes.

Camera/bridge tests use synthetic frames and packets. The camera remained off
throughout this gallery task; the earlier capture check is not evidence of
human-controlled motion. No packages were merged into the camera environment,
and no other application's device session was stopped.

Commands and environments are in the [run guide](README.md). **69 tests passed**
across the appropriate environments: simulation 14, adapter 3, camera/bridge 27,
app 13, model registry/adaptation 8 and original concept 4. The final app revision
was rerun after the startup fix; the other passing suites preceded that
app-only change. The delivered local commit is recorded in the project checkpoint.
Repository CI's Ruff selection (`ruff check --select E9,F`) and
`git diff --check` passed for this change.
