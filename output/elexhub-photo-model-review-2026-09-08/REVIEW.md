# Owner-photo robot model — visual revision v2

Requested and photo references received 8 September 2026; final acceptance
completed 9 September 2026 (Asia/Bangkok). Source baseline: `9c5f797` on
`experiment/elexhub-webcam`. This delivery is local working-tree work.

## Result

The robot-only model is reconstructed from the owner's five photographs using
the existing shared CAD/mesh pipeline. The distinguishing correction is the
silver fixed forearm connector and its flanged ends, replacing the earlier long
black forearm yoke. Short perforated cradles, upper-arm cheeks, compact turret
supports, asymmetric joint hardware, claw outlines, fine servo leads, and cable
ties were refined from the other views.

The package contains editable FreeCAD, STEP, GLB, URDF, the material OBJ meshes,
a robot-only MuJoCo companion, and a self-contained browser viewer. The original
room photographs are excluded. Standard exports retain the profile's home/open
assembly; inspection images use `[0, 0.9, -1.35, -1.18, 0]` radians in the joint
order recorded in their manifest. This is a valid inspection pose, not a fitted
or measured copy of the photograph's joint angles.

The photographed spacer is a fixed visual component. Five positioning joints,
one gripper drive, the existing simplified sliding-jaw motion, and the original
kinematic/dynamic parameters are retained. The robot-only companion has seven
coordinates and six controls, with a fixed base and no scene objects or camera.

## Verification

- **210 valid CAD solids**, saved and reopened in FreeCAD 1.1.3. STEP was reimported
  with matching solid count, volume, and global placements. Saved colors reopened
  correctly. The existing 0.15 mm CAD/mesh bound tolerance covers tessellation;
  it is not a claim of physical measurement accuracy.
- **7 focused tests passed, 43 deselected**, covering export frames, visual/physics
  separation, kinematic agreement, rendering/reach, robot-only content, review
  poses/bounds, and package integrity/stale-evidence handling.
- Ruff and formatting passed for all five changed Python files.
- Both six-view render sets passed visual review. Each manifest matches 54 model
  and mesh hashes plus six rendered-image hashes in the delivered asset folder.
- The visible local HTTP viewer passed side/front/top/reset views, keyboard zoom
  and rotation, and pointer dragging. No browser console errors were reported.
  FreeCAD, STEP, and GLB download responses returned HTTP 200 and matched local
  file hashes. Direct `file://` navigation was blocked by browser policy, so direct
  file opening was not tested; the viewer's code and geometry are self-contained.
- All 36 protected baseline files match their initial hashes: physical/simulation
  profiles, firmware header, and the frozen 6 September evaluation artifacts.

Detailed results and source hashes are in `VALIDATION.json`; the render-specific
poses, camera settings, and identities are in each `RENDER_MANIFEST.json`.

## Recovered export issues

The first CAD attempt rejected four capsule outlines containing duplicate arc
tangent points. Mesh triangulation had hidden the zero-length sketch edges.
Removing consecutive duplicate outline vertices allowed the editable sketches
to build. A later GUI shutdown fault followed the failed attempt; the corrected
build exited successfully using the same runtime and workflow.

Layered cradle bore rings and tangent claw discs also shared vertices. Default
OBJ reload welded those boundaries into nonmanifold edges even though individual
source solids were watertight. Explicit bore/disc clearances fixed the material
mesh roundtrip without relaxing the export tests. Claw screw heads were seated
and supported by visible shafts/spacers after inspecting the first side render.

The optional legacy combined-body OBJ files have a separate coincident seam
issue that was reproduced unchanged at the baseline. The model package includes
the valid material meshes referenced by URDF and MuJoCo and excludes those
optional compatibility files. The XML equivalence regression compares semantic
content, avoiding false differences caused only by indentation.

## Remaining estimates

Scale, joint-center distances, hidden construction, masses, actuator identities,
and joint calibration remain unmeasured. The claw is visually represented at its
open pose; complete linkage motion is deferred. Decorative wire runs have short
free endpoints near joints and do not simulate continuous flexible wiring.

The next update uses the owner's measurements to revise named visual dimensions
and the canonical link lengths/offsets together. See the included measurement
guide for endpoints and units. No physical commissioning or trained-policy
evaluation is established by this visual revision.
