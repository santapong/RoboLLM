# RoboLLM · TALOS joint inventory

> **RoboLLM field guide** · Build → Observe → Measure → Learn<br>
> [Home](../../../README.md) · [Technical notes](../TECHNICAL.md) · [Runbook](mirror-run.md) · [Examples](../../README.md)

Measured from the expanded URDF (vendored in full, verified 2026-07-27).

| Joint Name | Axis | Lower (rad) | Upper (rad) | Lower (deg) | Upper (deg) | Group |
|---|---|---|---|---|---|---|
| `arm_left_1_joint` | z | -1.570796 | 0.785398 | -90.00 | 45.00 | arm_l |
| `arm_left_2_joint` | x | 0.008727 | 2.871067 | 0.50 | 164.50 | arm_l |
| `arm_left_3_joint` | z | -2.426008 | 2.426008 | -139.00 | 139.00 | arm_l |
| `arm_left_4_joint` | y | -2.234021 | -0.003491 | -128.00 | -0.20 | arm_l |
| `arm_left_5_joint` | z | -2.513274 | 2.513274 | -144.00 | 144.00 | arm_l |
| `arm_left_6_joint` | x | -1.370083 | 1.370083 | -78.50 | 78.50 | arm_l |
| `arm_left_7_joint` | y | -0.680678 | 0.680678 | -39.00 | 39.00 | arm_l |
| `arm_right_1_joint` | z | -0.785398 | 1.570796 | -45.00 | 90.00 | arm_r |
| `arm_right_2_joint` | x | -2.871067 | -0.008727 | -164.50 | -0.50 | arm_r |
| `arm_right_3_joint` | z | -2.426008 | 2.426008 | -139.00 | 139.00 | arm_r |
| `arm_right_4_joint` | y | -2.234021 | -0.003491 | -128.00 | -0.20 | arm_r |
| `arm_right_5_joint` | z | -2.513274 | 2.513274 | -144.00 | 144.00 | arm_r |
| `arm_right_6_joint` | x | -1.370083 | 1.370083 | -78.50 | 78.50 | arm_r |
| `arm_right_7_joint` | y | -0.680678 | 0.680678 | -39.00 | 39.00 | arm_r |
| `gripper_left_joint` | x | -0.959931 | 0.000000 | -55.00 | 0.00 | gripper_l |
| `gripper_right_joint` | x | -0.959931 | 0.000000 | -55.00 | 0.00 | gripper_r |
| `head_1_joint` | y | -0.209440 | 0.785398 | -12.00 | 45.00 | head |
| `head_2_joint` | z | -1.308997 | 1.308997 | -75.00 | 75.00 | head |
| `leg_left_1_joint` | z | -0.349066 | 1.570796 | -20.00 | 90.00 | leg_l |
| `leg_left_2_joint` | x | -0.523600 | 0.523600 | -30.00 | 30.00 | leg_l |
| `leg_left_3_joint` | y | -2.095000 | 0.700000 | -120.03 | 40.11 | leg_l |
| `leg_left_4_joint` | y | 0.000000 | 2.618000 | 0.00 | 150.00 | leg_l |
| `leg_left_5_joint` | y | -1.270000 | 0.680000 | -72.77 | 38.96 | leg_l |
| `leg_left_6_joint` | x | -0.523600 | 0.523600 | -30.00 | 30.00 | leg_l |
| `leg_right_1_joint` | z | -1.570796 | 0.349066 | -90.00 | 20.00 | leg_r |
| `leg_right_2_joint` | x | -0.523600 | 0.523600 | -30.00 | 30.00 | leg_r |
| `leg_right_3_joint` | y | -2.095000 | 0.700000 | -120.03 | 40.11 | leg_r |
| `leg_right_4_joint` | y | 0.000000 | 2.618000 | 0.00 | 150.00 | leg_r |
| `leg_right_5_joint` | y | -1.270000 | 0.680000 | -72.77 | 38.96 | leg_r |
| `leg_right_6_joint` | x | -0.523600 | 0.523600 | -30.00 | 30.00 | leg_r |
| `torso_1_joint` | z | -1.256637 | 1.256637 | -72.00 | 72.00 | torso |
| `torso_2_joint` | y | -0.226893 | 0.733038 | -13.00 | 42.00 | torso |

## Summary by Group

- **arm_l**: 7 joint(s) — `arm_left_1_joint`, `arm_left_2_joint`, `arm_left_3_joint`, `arm_left_4_joint`, `arm_left_5_joint`, `arm_left_6_joint`, `arm_left_7_joint`
- **arm_r**: 7 joint(s) — `arm_right_1_joint`, `arm_right_2_joint`, `arm_right_3_joint`, `arm_right_4_joint`, `arm_right_5_joint`, `arm_right_6_joint`, `arm_right_7_joint`
- **gripper_l**: 1 joint(s) — `gripper_left_joint`
- **gripper_r**: 1 joint(s) — `gripper_right_joint`
- **head**: 2 joint(s) — `head_1_joint`, `head_2_joint`
- **leg_l**: 6 joint(s) — `leg_left_1_joint`, `leg_left_2_joint`, `leg_left_3_joint`, `leg_left_4_joint`, `leg_left_5_joint`, `leg_left_6_joint`
- **leg_r**: 6 joint(s) — `leg_right_1_joint`, `leg_right_2_joint`, `leg_right_3_joint`, `leg_right_4_joint`, `leg_right_5_joint`, `leg_right_6_joint`
- **torso**: 2 joint(s) — `torso_1_joint`, `torso_2_joint`

**Total: 32 actuated joints**
