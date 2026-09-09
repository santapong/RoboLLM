# Measurements for the photo-based robot model

Prepared 8 September 2026 from the owner's five photographs. This first version
models the robot's visible construction. Its dimensions remain estimates; no
length measurements were supplied for this revision.

The silver forearm connector is represented as a fixed structural part. Its
appearance does not establish a telescoping joint. The model retains five arm
joints and one gripper actuator, with simplified sliding gripper contacts.

## Send these measurements when available

Use millimeters. For each value, identify both endpoints, ideally with a photo
showing where the ruler or calipers touch. Shaft-center distances and the outside
length of a bracket are different measurements.

| Measurement | Reference points | Model destination |
| --- | --- | --- |
| Base plate width, depth, thickness, and separation | Plate edges and facing surfaces; measure the upper and lower plates separately if different | Base visual dimensions; collision envelope reviewed separately |
| Shoulder height | Supporting surface to the shoulder shaft center | Base origin plus shoulder offset |
| Upper-arm joint spacing | Shoulder shaft center to elbow shaft center | Upper-arm length and following elbow offset |
| Forearm joint spacing | Elbow shaft center to wrist-pitch shaft center | Forearm length and following wrist offset |
| Silver connector | Exposed cylinder length and diameter; flange diameter and thickness | Forearm visual parameters; not a replacement for joint spacing |
| End brackets | Width, depth, plate thickness, and shaft position relative to mounting face | Cradle outlines, mounting offsets, and perforations |
| Wrist assembly | Wrist-pitch shaft center to wrist-roll mounting face; any sideways offset | Wrist length and following tool offset |
| Gripper mounting | Wrist-roll mounting face to claw root pivots and fingertip | Tool and gripper geometry |
| Gripper opening | Inner fingertip gap fully open and closed | Gripper opening parameter |
| Finger and linkage geometry | Finger length/width/thickness, pivot-center distances, and pivot spacing at the claw root | Gripper visual parameters; linkage motion requires a separate update |
| Servo housing | Case width/depth/height, mounting-ear spacing, and readable label | Actual component identity and visual envelope |

Provide whichever measurements you have; the remaining values stay provisional.
Masses, joint zeros, joint limits, and actuator characteristics are separate from
the visual length measurements.

## How dimensions are maintained

The canonical hardware profile stores structural geometry in meters. Measured
joint spacing must update both the link dimension and the next joint's offset.
The shared appearance source stores named visual dimensions and derives the
connector span from the structural forearm length. All CAD and mesh formats are
regenerated from this shared description.

For each measurement, record its value, units, endpoints, date, source photo or
note, and uncertainty if known. Do not mark the entire model measured after
updating only a few dimensions. Direct FreeCAD edits must be transferred back to
the shared source before regeneration.

## Current assembly and files

- FreeCAD and STEP use millimeters; the source, URDF, MuJoCo, and GLB use meters.
- CAD, URDF, and MuJoCo use Z-up; GLB uses Y-up.
- Standard CAD, GLB, and browser exports show the existing home/open assembly.
  Photo comparison renders use a separately recorded inspection pose.
- The gripper links are visual approximations at the open assembly pose. Static
  photographs do not establish their complete motion or contact geometry.
- Robot-only exports contain the base, arm, servos, mounting hardware, claw, and
  attached leads. The owner's room photographs are not included in the package.
