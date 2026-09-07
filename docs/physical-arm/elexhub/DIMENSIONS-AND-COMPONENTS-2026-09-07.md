# Elexhub dimensions and component research

Checked 7 September 2026. The owner confirmed the black Elexhub metal kit with rectangular base plates and a black linkage claw. Its current reconstruction is visually close according to the owner; physical dimensions have not been measured. Published component specifications below are references, not evidence that Elexhub uses the same supplier or part revision.

## What Elexhub establishes

The [complete-kit listing](https://elexhub.com/shop/XvAAPZ) lists a metal frame, six servo motors, a gripper, brackets, screws, wiring and an assembly guide. The [chassis listing](https://elexhub.com/shop/KMPxZc) explicitly offers a six-MG996 variant. Neither reviewed page provides pivot-to-pivot lengths, base dimensions or a dimensioned engineering drawing. The actual servo labels on this owner's arm remain unverified.

| Component | Quantity/status for this project |
|---|---|
| Positioning servos | 5: base yaw, shoulder, elbow, wrist pitch and wrist roll; topology inferred from product assembly photographs |
| Gripper servo | 1: opening/closing the linkage claw |
| Metal frame and claw | 1 assembly; base plates, U brackets, servo mounts and claw are visible in the Elexhub photos |
| Horns, bearings, bolts/nuts and leads | Present in the photographed assembly; exact sizes and counts require inventory |
| PCA9685 board | Owner-reported existing component; not established as an included Elexhub kit part |
| Webcam | Owner's planned observation device; model and calibration unknown |

Six motors here mean five positioning joints plus a gripper; they do not establish six independently controllable Cartesian pose coordinates plus a seventh gripper actuator.

## Published dimensional references

| Reference part | Published size in mm | Scope |
|---|---|---|
| TowerPro MG996R | 40.7 × 19.7 × 42.9 | Manufacturer's nominal envelope, not a measurement of the installed servos |
| Olimex long U-bracket | 56 × 25 × 65 | MS-BRACKET-U catalog part |
| Sinoning long U-bracket | 56 × 25 × 64 | Similar bracket; the 1 mm difference illustrates supplier variation |
| Sinoning short U-bracket | 56 × 25 × 44 | Candidate component reference |
| Sinoning multifunction servo mount | 58 × 37 × 25 | Candidate component reference |
| Sinoning oblique U-bracket | 56 × 25 × 41 | Candidate component reference |
| Sinoning L-bracket | 37 × 29 | Two catalog dimensions only |
| Sinoning U-beam | 90 × 29 × 42 | Individual beam, not assembled Elexhub base footprint |
| Sinoning cup bearing | 3 inside × 8 outside × 4 thick | Bearing size; not the visible horn/cover diameter |

Sources: [TowerPro MG996R](https://towerpro.com.tw/product/mg996R/), [Olimex MS-BRACKET-U](https://www.olimex.com/Products/Robot-CNC-Parts/ServoMotors/MS-BRACKET-U/), [Sinoning bracket catalog](https://www.sinoning.com/product/black-robot-arm-robotic-part-accessory-platfrom-u-beam-servo-bracket-for-standard-servo-steering-bearing-bracket-steam-diy-parts/). Sinoning specifies 2 mm aluminum for these bracket types. Its catalog also includes a different 138 × 100 × 38 mm base in iron; that is not automatically the Elexhub base. The catalog was read in the visible browser after the text fetch timed out. No dimensions were recovered from uncalibrated image pixels.

The TowerPro page's headline envelope and its separate lettered configuration table differ slightly; without an unambiguous dimension drawing they should not be combined into an exact mounting-hole model. Its nominal 55 g weight and 4.8–6.6 V range apply to that specified servo, not unidentified installed motors. They have not been copied into the hardware profile's measured fields.

## Overall arm length: related kit, not confirmed identity

[diymore's ROT3U](https://www.diymore.cc/products/diymore-rot3u-6dof-aluminium-robot-arm-mechanical-robotic-clamp-claw-kit-for-arduino-mega2560) lists **460 mm height**, a **355 mm “radius of gyration”**, **55 mm maximum claw opening**, and **98 mm widest holder distance**. The radius wording does not define a guaranteed collision-free working reach. Its [black assembly photograph](https://www.diymore.cc/cdn/shop/products/050580_3_287.jpg?v=1588643724) was inspected; its claw and base assembly differ from the selected Elexhub reference. These figures are comparison data, not verified dimensions for the owner's arm.

The related ROT3U package lists five multifunction brackets, four long U-brackets, three waist U-brackets, one L-bracket, four bearings, one claw and a screw set. These counts are **not** an Elexhub bill of materials. A [Robu assembly guide](https://robu.in/wp-content/uploads/2025/01/R192638-Engish-6-DOF-Special-Shaped-Robot-Arm-Installation-Manual.zh_.pdf) also identifies M3 × 6, M3 × 8 and M3 × 10 fasteners for its kit; this is another variant reference, not a verified fastener inventory for this arm.

## Comparison with the delivered model

| Quantity | Current model | Evidence/next measurement |
|---|---|---|
| Table to shoulder pivot | 80 mm | 25 mm base datum + 55 mm turret offset; provisional |
| Shoulder to elbow pivot | 105 mm | Measure shaft-center to shaft-center |
| Elbow to wrist-pitch pivot | 100 mm | Measure shaft-center to shaft-center |
| Wrist-pitch to wrist-roll offset | 45 mm | Measure the actual joint-frame displacement |
| Wrist-roll to jaw-root offset | 25 mm | Measure actual claw mounting offset |
| Finger length / maximum opening | 65 / 55 mm | Photo-guided choices; measure pivot layout and opening at several servo angles |
| Nominal upright chain to fingertip | 420 mm | Sum of the model's axial dimensions at zero joint angles, not measured total height |
| Base collision footprint | 105 × 105 mm | Visual plates approximately 105 × 95 mm; both estimated |

Research alone has not replaced any link length, mass, hardware limit or calibration flag. A 65 mm bracket dimension is an outer part size, not necessarily a 65 mm joint-center distance: overlap, horn placement and hole choice change the assembled kinematics. The useful next measurements are the shoulder–elbow distance, elbow–wrist distance, table–shoulder height, wrist-to-claw offsets and base footprint. Record shaft centers rather than plate endpoints.

The owner's PCA9685 is a PWM command component. [NXP's datasheet](https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf) specifies 16 channels and 12-bit PWM; it does not supply servo position feedback. Board footprint and power connectors depend on the particular breakout board. Webcam observation, servo calibration and link measurements remain separate work.

This research and the model changes are confined to the isolated Elexhub worktree. No Kaggle training, Pi service, camera capture or physical servo operation was performed.
