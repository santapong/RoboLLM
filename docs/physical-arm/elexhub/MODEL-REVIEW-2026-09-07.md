# Elexhub photo-model review — 7 September 2026

The owner confirmed that their arm matches the black metal Elexhub kit with a rectangular two-plate base and linkage claw. This revision reconstructs its visible construction as original editable solids and shared simulation meshes. It is **a visual approximation with provisional dimensions**, not manufacturer CAD or a measured digital twin.

## Reference evidence

- [Elexhub complete kit](https://elexhub.com/shop/XvAAPZ) establishes the kit and supplies the assembly photographs.
- [Overall product photograph](https://elexhub.com/images/items/XvAAPZ0.jpg): black U brackets, two base plates, exposed motors and linkage claw.
- [Front detail](https://elexhub.com/images/items/XvAAPZ1.jpg) and [side assembly](https://elexhub.com/images/items/XvAAPZ2.jpg): claw plate, bearing discs, screws and cable routing.
- [Elexhub chassis listing](https://elexhub.com/shop/KMPxZc) offers the six-MG996 variant. The owner's actual servo labels remain unverified.
- [TowerPro MG996R](https://towerpro.com.tw/product/mg996R/) gives a nominal 40.7 × 19.7 × 42.9 mm envelope used as a size reference. This does not establish the provenance, dimensions or electrical ratings of the installed motors.

The original photographs were inspected in the browser. They are linked, not redistributed. No dimensioned Elexhub engineering drawing was found in the reviewed product material. Hole spacing, plate outlines, servo mounting details and cable paths are modeled estimates; perspective images were not treated as calibrated metrology.

## Delivered geometry

The editable assembly contains 283 solids: black perforated plate sketches/extrusions, six servo assemblies, bearing discs, bored fasteners, claw links and servo leads. MuJoCo and URDF share the same colored visual meshes and body frames. The standalone browser viewer supports orbit, zoom and fixed views without loading remote scripts. A GLB export uses Y-up; CAD, MJCF and URDF retain Z-up and metres in the shared descriptor.

| Quantity | Current value | Evidence level |
|---|---|---|
| Positioning joints / claw drive | 5 + 1 | Photographed kit topology; owner-confirmed variant |
| Upper arm / forearm joint spacing | 105 / 100 mm | Retained provisional values; unmeasured |
| Claw opening / finger length | 55 / 65 mm | Chosen photo-guided estimates; not scale-recovered |
| Typical bracket thickness / fork width | 2 / 51 mm | Visual construction estimates |
| Bearing disc diameter | 18 mm | Visual construction estimate |
| Base footprint | approximately 105 × 95 mm visual outline | Estimated; conservative 105 × 105 mm base proxy retained |
| Masses, motor limits, friction and delay | Existing provisional profile | No physical identification |

Visual detail is excluded from the collision solver and does not add mass to the simulation. Collision proxies and box inertias remain approximate. The claw's open-pose appearance follows the photographs, but the simulated grip remains a coupled sliding-jaw approximation; it does not yet reproduce the measured four-bar linkage through its full stroke. Cables are cosmetic rigid segments, not simulated flexible wiring.

## Simulation compatibility

The longer fingers exposed limitations of the previous tabletop setup. The provisional mat origin is now `(0.23, -0.09, 0)` m, placing the default box at base X = 150 mm. The transit TCP height is 52 mm. Grasp height preserves fingertip/table clearance, and jaw closure preserves the nominal compression of the configured 20 mm box despite the wider opening.

The previous downward target used a roll-flipped orientation outside the wrist's usable range and gave yaw zero weight. A diagnostic run observed wrist rotation during grasp/lift. The revised target is attainable by pitching down, with orientation constrained throughout transfer. Reach, grasp, lift, release, collision and success checks retain their existing tolerances; no object weld or success-threshold relaxation was introduced.

## Verification

- **56 tests passed** across the Elexhub and arm-driver foundation suites, including contact-based pick/place, orientation consistency, independent forward kinematics, visual/CAD/MuJoCo frame consistency, visual mass exclusion, source isolation and unavailable-feed recovery. Two dependency deprecation warnings remain.
- After the final cosmetic changes, the two visual-export/render checks passed again. They cover the final meshes and MuJoCo render; the component-reference text was also verified in the browser.
- FreeCAD 1.1.3 saved and reopened all **283 valid solids** with their colors. Plate sketches, extrusion thickness, box/cylinder dimensions and bores remain editable.
- STEP reimport checked solid count, volume and global bounds. CAD bounds also agree with the independently generated meshes to 0.15 mm, allowing tessellation error; this is export consistency, **not physical accuracy**.
- FreeCAD stores display colors with 8-bit channels; color-reopen validation allows half a channel step. Each CAD run carries a fresh build identity so a failed command cannot reuse stale validation.
- MuJoCo rendered angle, front, side and workspace views. Browser checks exercised the interactive model and its fixed views.
- Firmware compiled for Mega 2560: 9,710 bytes flash, 1,102 bytes RAM. Compilation did not open a device or upload firmware.

Evidence is under `output/elexhub-model-review-2026-09-07/`; CAD-specific validation and photo provenance are alongside the exported model. The 6 September evaluation archive is unchanged and does not constitute evaluation of this revision. Kaggle training, Pi services, physical PWM and the real webcam were not operated.

## Measurements still required

Measure pivot-to-pivot distances, base/shoulder height, fork spacing, claw pivot positions, opening at several servo angles, tool offset and individual link masses. Confirm the installed servo models and mounting orientation. Those observations should replace estimates in the hardware profile and visual construction before physical calibration or a faithful linkage/contact model is claimed.
