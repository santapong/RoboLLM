# RoboLLM · Hand-guided pick-and-place theory

> **RoboLLM field guide** · Build → Observe → Measure → Learn<br>
> [Home](../../../README.md) · [Example](../README.md) · [Technical notes](../TECHNICAL.md) · [Runbook](pickplace-run.md)

Target system: Kinova Gen3 lite (6-DOF + gen3_lite_2f gripper), ROS 2 Jazzy in
Docker (Fast DDS), mock hardware, MoveIt planning scene, MediaPipe
GestureRecognizer 0.10.35, LEFT-hand teleop, Closed_Fist = grip / Open_Palm =
release. This document is the theory companion to the binding build specs
(orientation, gesture stability, release fallback) returned by R3. Sections 6–8
are normative background for those specs; the specs themselves are the binding
text.

---

## 1. Scope and lineage

This build ports and extends `robot_arm_moveit_config/scripts/hand_follow.py`
(the U1–U5 weld-arm follower): threaded camera capture, One-Euro filtering,
LEFT-hand selection, scaled/clamped workspace-box retargeting, per-tick slew
limits, loss/hold/home logic, `/follow_enable`, preflight, synthetic mode, and
the latency probe. New in `gen3_pick_place`: hand-derived gripper
**orientation** (user decision: hand-derived over fixed top-down, with graceful
degradation), gesture-gated **grasping** against a MoveIt planning-scene box
(add / attach / detach semantics verified in Inception), and the robustness
machinery the Inception critic mandated (transition latching, release
fallbacks, proximity gating, IK fallback chain, box reset).

## 2. System architecture

```
camera 30 fps ──> ThreadedCapture ──> flip(mirror) ──> GestureRecognizer
                                                        (ONE recognize_for_video call returns
                                                         gestures + handedness + 21 image
                                                         landmarks + 21 world landmarks,
                                                         all PARALLEL ARRAYS by hand index)
   vision thread:  select LEFT hand index i ──┬─> position: image landmarks (u,v,scale) ─> workspace box map ─> OneEuro xyz
                                              ├─> orientation: world landmarks ─> palm frame ─> quaternion ─> SO(3) OneEuro
                                              └─> gesture: gestures[i] ─> debounced state machine (+ openness metric)
   control tick 20 Hz:  latch/freeze gate ─> slew clamp (pos + slerp rate clamp) ─> IK (oriented ─> fixed-down ─> hold)
                        ─> per-joint delta clamp ─> single-point JointTrajectory ─> /arm_controller
   grasp path:  proximity gate ─> /apply_planning_scene attach (absorb-from-world) ─> gripper_controller
                (second joint_trajectory_controller on right_finger_bottom_joint — the stock
                 GripperActionController silently ignores topic streams, verified in Inception)
```

The single-inference design matters on this 4-core CPU host: one
`recognize_for_video` (~27.5 ms measured) serves position, orientation, and
gesture. There is no second model call.

**Parallel-array invariant (binding, Inception critic gap 2):** the gesture,
handedness, image landmarks and world landmarks for one physical hand share one
result index `i`. All three consumers above MUST read index `i` chosen once by
the LEFT-hand selector. Reading `gestures[0]` while tracking
`hand_landmarks[1]` silently binds the gesture of a bystanding RIGHT hand to
the tracked LEFT hand.

## 3. Human-in-the-loop teleoperation theory

**The human closes the loop.** Monocular RGB cannot recover absolute depth
(Inception, Section "Fundamental limit"); the user watching RViz supplies the
outer feedback loop, as in the reference implementations
(mediapipe_dual_arm_control, FrankaTeleop, mernaahany/teleoperation-robot-arm)
and as framed by Sheridan's supervisory-control model of telerobotics
(*Telerobotics, Automation, and Human Supervisory Control*, Sheridan 1992).
Consequences:

- **Mirror mapping is mandatory.** The operator corrects errors by moving the
  hand toward where the tool should go *on screen*; any axis inversion breaks
  the correction loop. The validated convention (hand_follow.py header): in the
  flipped selfie view, hand right → base +Y, hand up → base +Z, hand toward
  camera (bigger) → base +X.
- **Latency budget.** Human visuomotor tracking degrades sharply past
  ~100–200 ms loop delay (classical teleoperation result; see the Telerobotics
  chapter by Niemeyer, Preusche, Hirzinger in the *Springer Handbook of
  Robotics*). Measured pipeline here: ~27.5 ms inference + capture + IK, under
  the 70 ms budget from U5 — headroom exists for the added orientation math
  (sub-millisecond vector algebra).
- **Jitter vs lag.** Filtering trades precision at rest against lag in motion.
  The One-Euro filter (Casiez, Roussel, Vogel, *1€ Filter*, CHI 2012) resolves
  this with a speed-adaptive cutoff: low cutoff (heavy smoothing) at rest, high
  cutoff (low lag) in motion. Already tuned in U5 (`min_cutoff=1.0`,
  `beta=0.5`); the orientation channel gets the same treatment on SO(3)
  (Section 5).
- **Workspace indexing / clamping as virtual fixtures.** The scaled, clamped
  workspace box is a *virtual fixture* (Rosenberg 1993): it guarantees
  reachability (validated 125-pt grid, worst IK error 1.45 mm on the old arm;
  MUST be re-tuned for gen3 lite reach) and bounds the damage of any perception
  fault. Clutching/indexing (temporarily decoupling master and slave, standard
  in telerobotics per Niemeyer et al.) appears here in three forms:
  `/follow_enable`, the loss-hold state, and — new — the gesture-transition
  latch (Section 7).
- **Never teleport.** Every path into a new target passes a per-tick slew clamp
  (`max_step_m`, per-joint `max_dq`); re-acquisition and resume re-seed from
  `/joint_states`. This is a poor-man's online trajectory generation; the
  principled version is jerk-limited OTG (Berscheid & Kröger, *Ruckig*, RA-L
  2021), overkill for a mock-hardware demo but the correct citation for why
  per-tick clamps are sound: they bound velocity discontinuities at the
  controller input.

## 4. Hand perception model

MediaPipe Hands (Zhang, Bazarevsky, Vakunov et al., *MediaPipe Hands: On-device
Real-time Hand Tracking*, CVPR Workshops 2020) predicts 21 landmarks per hand:

```
0 WRIST
1–4   THUMB  (CMC, MCP, IP, TIP)
5–8   INDEX  (MCP, PIP, DIP, TIP)
9–12  MIDDLE (MCP, PIP, DIP, TIP)
13–16 RING   (MCP, PIP, DIP, TIP)
17–20 PINKY  (MCP, PIP, DIP, TIP)
```

Two coordinate sets per hand (Google AI Edge Hand Landmarker /
GestureRecognizer docs):

- `hand_landmarks` — normalized image coordinates (x,y ∈ [0,1], heuristic
  relative z). These **translate with the hand** → used for position (wrist
  u,v + wrist→middle-MCP size as depth proxy, the U4 lesson: world landmarks
  are hand-centered and nearly stationary under translation, so they CANNOT
  drive position).
- `hand_world_landmarks` — metric meters, origin at the hand's geometric
  center, axes aligned with the (flipped) camera frame: x right, y down, z away
  from camera. These are translation-free but **orientation-preserving and
  scale-true** → the correct input for deriving the palm frame (Section 6).
  This split — image landmarks for position, world landmarks for orientation —
  is exactly the division of labor used by vision-teleop systems such as
  AnyTeleop (Qin et al., RSS 2023).

The GestureRecognizer adds a canned-gesture head over the same landmarks:
labels `Closed_Fist`, `Open_Palm`, `Pointing_Up`, `Thumb_Up`, `Thumb_Down`,
`Victory`, `ILoveYou`, and `None` (no confident class). Two properties drive
the design:

1. **Classification lags and jitters at transitions** — single-frame labels
   are untrustworthy near a fist↔palm change; a debounce (N consecutive
   frames ≥ score threshold) is the standard remedy, structurally a Schmitt
   trigger / hysteresis element.
2. **The canned `Open_Palm` head is viewpoint-sensitive** — trained
   predominantly on roughly camera-facing palms; a palm tilted toward edge-on
   degrades to `None` or low score even while the 21 landmarks remain usable.
   Hence the landmark-openness secondary release (Section 7.3), which is
   computed from metric world landmarks and is viewpoint-invariant up to
   landmark quality.

## 5. Filtering theory: position and orientation

Position: One-Euro per axis on the mapped target (Casiez et al. 2012),
unchanged from U5.

Orientation lives on SO(3); componentwise filtering of quaternions is invalid
(it leaves the unit sphere and misbehaves under the double cover q ≡ −q). The
correct primitives:

- **Slerp** (Shoemake, *Animating Rotation with Quaternion Curves*, SIGGRAPH
  1985) — constant-angular-velocity interpolation; the SO(3) analogue of a
  scalar lerp, hence the SO(3) analogue of an exponential low-pass is
  `q_f ← slerp(q_f, q_raw, α)`.
- **Hemisphere continuity** — before any blend, flip `q_raw ← −q_raw` if
  `q_f · q_raw < 0` (the antipodal-pair issue formalized in Markley, Cheng,
  Crassidis, Oshman, *Averaging Quaternions*, J. Guidance Control & Dynamics
  2007).
- **Speed-adaptive cutoff** — the One-Euro idea transfers verbatim: estimate
  angular speed ω from consecutive samples via
  `ω = 2·arccos(|q_f·q_raw|)/Δt`, low-pass ω, and set the blend cutoff
  `f_c = f_cmin + β_q·ω̂`. At rest the gripper orientation is glassy; during a
  deliberate wrist rotation it follows with low lag.

A separate **rate clamp at the control tick** (max slerp step per tick,
mirroring `max_step_m` for position) bounds what the IK and controller ever
see, independent of filter tuning — filter and safety clamp are deliberately
two mechanisms, as in the position path.

## 6. Orientation derivation from the hand (normative background)

### 6.1 Why a palm frame

Dexterous vision-teleop systems derive the end-effector frame from the hand
pose rather than commanding a fixed orientation: DexPilot (Handa et al., ICRA
2020) retargets the full human hand frame to a robot hand; Robotic Telekinesis
(Shaw, Bahl, Pathak, RSS 2022) and AnyTeleop (Qin et al., RSS 2023) both
construct a wrist/palm frame from monocular hand keypoints and map it to the
robot wrist; the mernaahany/teleoperation-robot-arm reference (named in
Inception) uses the wrist→middle-finger vector as an orientation cue. For a
2-finger gripper the full hand pose is unnecessary; what is needed is:

- an **approach axis** — where the gripper points → the *palm normal* (the
  direction the palm faces). Palm down = top-down grasp; palm toward camera =
  gripper toward user. This is the intuitive "show the robot the grasp
  direction" mapping.
- a **roll reference** — rotation about the approach axis → the projected
  wrist→middle-MCP direction. Rolling the flat hand about its own normal
  rolls the gripper.

### 6.2 The three spanning landmarks

The palm is the rigid part of the hand: WRIST (0), INDEX_MCP (5), MIDDLE_MCP
(9), PINKY_MCP (17) move as one near-rigid body through finger curls — which
is precisely why the palm frame remains valid *while the hand is fisted during
a carry* (the MCP knuckles stay visible and tracked in a fist). Fingertip
landmarks are excluded by construction: they move with grip state and would
corrupt orientation during the very gesture transitions that matter most.

Spanning vectors (world landmarks): `v1 = P9 − P0` (wrist → middle MCP, palm
"forward"), `v2 = P5 − P17` (pinky MCP → index MCP, across the palm). The palm
normal is their cross product.

### 6.3 Chirality under the mirror flip (sign derivation)

The pipeline flips every frame horizontally before inference; MediaPipe labels
the physical left hand `Left` in this selfie view (verified U4/U5 behavior).
In the flipped view, the physical LEFT hand palm-facing-camera appears exactly
as one sees one's own left palm: thumb on image-right, so INDEX_MCP(5) is
image-right of PINKY_MCP(17). With camera axes x right, y down, z into scene:
`v1 ≈ (0,−1,0)` (fingers up), `v2 ≈ (+1,0,0)`, and the palm faces the viewer,
i.e. the true normal is `−z`. Then

```
v2 × v1 = (1,0,0) × (0,−1,0) = (0,0,−1)  ✓ (toward camera = palm-facing)
```

**Binding sign: `n = v2 × v1` is the palm-facing direction for the
Left-labeled hand in this flipped pipeline.** Because the sign chain (physical
chirality → mirror flip → handedness label → world-landmark axis convention)
is long and any wrong link flips the gripper 180°, the build MUST include a
one-time visual verification: preview-overlay the projected normal and confirm
palm-toward-camera reports n·ẑ_cam < 0 (this is a stated acceptance item for
the orientation unit, not optional).

### 6.4 Camera→base rotation

Orientation must transform with the SAME fixed rotation that the position map
already implies, or translation and rotation would disagree in the mirror
loop. From the validated position convention (image right → +Y, image up
(−y_cam) → +Z, toward camera (−z_cam) → +X):

```
R_cb : x_cam ↦ (0,1,0),  y_cam ↦ (0,0,−1),  z_cam ↦ (−1,0,0)     det = +1
```

Sanity anchors: palm toward camera → approach = +X (gripper faces the user in
RViz's front view — mirror-consistent); palm down (n = +y_cam) → approach = −Z
(top-down grasp).

### 6.5 Conditioning, clamps, degradation

- **Conditioning gate:** the frame is well-defined only while v1 and v2 are
  non-degenerate and non-parallel — `sin∠(v1,v2) = ‖v1×v2‖/(‖v1‖‖v2‖)` acts
  as the condition metric. An edge-on hand shrinks and destabilizes both
  vectors; the gate demotes rather than emitting garbage.
- **Tilt clamp (60° cone about vertical-down):** the gen3 lite is a 6-DOF arm
  with a small wrist workspace near its reach envelope; unbounded hand
  orientations command upside-down or wrist-limit poses that IK cannot serve.
  Clamping the approach axis into a cone about −Z is another virtual fixture
  (Rosenberg 1993): the user keeps intuitive authority inside the cone; the
  system guarantees feasibility outside it.
- **Degradation chain (Inception critic gap 1):** oriented → fixed-downward
  (yaw preserved from the last valid command, so degradation does not visibly
  spin the tool) → position/orientation-hold, with rate-limited (slerp)
  transitions and hysteresis on promotion so the system does not flap between
  modes at the validity boundary. Holding the last command while continuing to
  stream is essential: JointTrajectoryController treats a silent publisher as
  "keep last trajectory", but the surrounding logic (reseed, latch) assumes an
  unbroken q_cmd stream; the hold state maintains that invariant. The chain is
  the graceful-degradation structure the user chose when selecting
  hand-derived orientation over fixed top-down.

Full binding parameters (indices, thresholds, filter constants, state machine)
are in the R3 `orientation_spec`.

## 7. Gesture-mediated grasping (normative background)

### 7.1 The transition-perturbation problem (Inception critic gap 3)

Closing the hand into a fist is not landmark-neutral:

- the wrist typically flexes/rotates as the fingers curl → the wrist landmark
  (the position anchor) translates in the image;
- wrist flexion tilts the palm → the projected wrist→middle-MCP length (the
  depth proxy `s`) shrinks → a phantom depth step (the arm surges);
- landmark noise spikes for a few frames while the tracker re-fits the
  changing silhouette; the palm frame wobbles;
- the classifier flips *after* the physical motion has already corrupted
  several frames.

An unprotected pipeline therefore grabs at a target displaced by several cm
from where the user intended. The remedy is a **latch**: as soon as a
transition *candidate* appears, the commanded target (position AND
orientation) is frozen at its value from ~150 ms *before* the candidate —
back-dating steps behind the perturbation onset, exploiting a ring buffer of
recent filtered targets. This is clutching (Niemeyer et al., Springer
Handbook) applied automatically at gesture boundaries instead of manually.

### 7.2 Debounce, commit, re-engage

The gesture state machine commits a transition only after N consecutive
confident frames (hysteresis; single-frame flickers of the classifier are
discarded). After commit + settle, control does not jump to the live target:
the live filters keep running warm in the background during the freeze, and
re-engagement waits until the live target has come back within a small radius
of the latched one (or a timeout), then slews. The freeze protects intent; the
proximity re-engage protects continuity; the slew protects the controller.

### 7.3 Release robustness (Inception critic gap 4)

Three concentric detectors, all requiring *positive* evidence of an open hand
(loss of tracking never releases — dropping a carried object on a tracking
glitch is the worst failure mode of the demo):

1. **Primary** — debounced `Open_Palm` from the canned classifier.
2. **Secondary — landmark openness:** per-finger extension ratio
   `‖TIP−WRIST‖ / ‖MCP−WRIST‖` on metric world landmarks, thresholded with a
   hysteresis band; ≥3 of 4 non-thumb fingers extended, sustained, releases.
   Viewpoint-robust: ratios of 3-D distances in the hand-centered metric frame
   do not degrade with palm tilt the way the appearance-trained classifier
   head does.
3. **Tertiary — None-score dwell:** a sustained window in which the classifier
   is confident about nothing (`None` / sub-threshold) *while* mean openness
   stays high resolves the "tilted, mostly-open hand" deadlock in bounded
   time.

A refractory period after release prevents grip/release chatter at the
boundary. Full parameters in `release_fallback_spec`.

### 7.4 Proximity gate and attach semantics (gaps 5, 9, 11)

Mock hardware has no contact physics; "grasping" is a planning-scene state
change. The verified sequence (Inception, 20/20 checks on the old arm; link
names to be re-verified for gen3 lite in U2):

- Box exists as a world `CollisionObject` (`PlanningScene is_diff:true`).
- **Proximity gate:** a committed FIST alone closes the gripper but attaches
  NOTHING unless the gripper frame is within `D_ATTACH` of the box. The gate
  keeps re-evaluating while the fist is held, so fisting far away and then
  carrying the fist to the box attaches on arrival — matching the user's
  natural "grab" mental model instead of punishing early fisting.
- **Attach** via the absorb-from-world form (`robot_state.is_diff:true`,
  `AttachedCollisionObject{link_name, touch_links, object{id, header.frame_id
  = <link>}}`); the attached box then follows the link in
  `/monitored_planning_scene` while trajectories stream.
- **Close to pinch, not to zero:** the gripper closes to a width that visually
  pinches the 4 cm box (attach FIRST, then close), because a full close would
  visually crush through the attached geometry; the attach is what "holds",
  the finger width is presentation.
- **Detach + `/box_reset`:** release detaches; a release in mid-air leaves the
  box floating in the world scene (no gravity in the planning scene), which is
  expected and recoverable by the `/box_reset` service respawning the box at
  its spawn pose — the spawn pose itself provably inside the tuned workspace
  box at grasp height, tied to `D_ATTACH` (non-circular acceptance, gaps 10
  and 16).

## 8. Failure-handling theory (gaps 6, 12, 15)

- **IK-failure ticks:** hold last `q_cmd`, keep publishing, warn throttled; N
  consecutive failures freeze target advance (otherwise the target keeps
  slewing into unreachable space and snaps the arm when IK recovers on a far
  branch). This is the discrete analogue of damped-least-squares behavior
  near singularities (Chiaverini, *Singularity-robust task-priority
  redundancy resolution*, IEEE Trans. Robotics & Automation 1997): degrade
  tracking accuracy, never emit a discontinuity.
- **FK source:** reseed/error metrics use `/compute_fk` if move_group serves
  it, else a TF lookup of the verified gripper frame — never a hand-rolled FK
  that can drift from the URDF.
- **Preflight subset-reconcile (gap 8):** gen3 lite `/joint_states` includes
  finger/mimic joints whose presence and propagation under mock hardware U2
  must verify and record; the joint check requires the 6 arm joints as a
  subset and tolerates extras — the old exact-set check would abort.
- **Camera-mode smoke test (gap 15):** 30 s with no hand in view, arm holds —
  this exercises the real perception path's *negative* behavior (no false
  acquisitions, loss logic stable), which synthetic mode cannot.

## 9. Sources (by name)

- Casiez, Roussel, Vogel — *1€ Filter: A Simple Speed-based Low-pass Filter
  for Noisy Input in Interactive Systems*, CHI 2012.
- Zhang, Bazarevsky, Vakunov, Tkachenka, Sung, Chang, Grundmann — *MediaPipe
  Hands: On-device Real-time Hand Tracking*, CVPR Workshops 2020; Google AI
  Edge Hand Landmarker & Gesture Recognizer documentation.
- Shoemake — *Animating Rotation with Quaternion Curves*, SIGGRAPH 1985
  (slerp).
- Markley, Cheng, Crassidis, Oshman — *Averaging Quaternions*, Journal of
  Guidance, Control, and Dynamics 2007 (hemisphere/antipodal handling).
- Handa, Van Wyk, Yang, Liang, Chao, Wan, Birchfield, Ratliff, Fox —
  *DexPilot: Vision-Based Teleoperation of Dexterous Robotic Hand-Arm
  System*, ICRA 2020.
- Qin, Yang, Huang, Shaw, Su, Pathak, Wang — *AnyTeleop: A General
  Vision-Based Dexterous Robot Arm-Hand Teleoperation System*, RSS 2023.
- Shaw, Bahl, Pathak — *Robotic Telekinesis: Learning a Robotic Hand Imitator
  by Watching Humans on YouTube*, RSS 2022.
- Niemeyer, Preusche, Hirzinger — *Telerobotics*, in Siciliano & Khatib
  (eds.), Springer Handbook of Robotics (clutching/indexing, latency limits).
- Sheridan — *Telerobotics, Automation, and Human Supervisory Control*, MIT
  Press 1992.
- Rosenberg — *Virtual Fixtures: Perceptual Tools for Telerobotic
  Manipulation*, IEEE VRAIS 1993.
- Berscheid, Kröger — *Jerk-limited Real-time Trajectory Generation with
  Arbitrary Target States* (Ruckig), RA-L/ICRA 2021.
- Chiaverini — *Singularity-robust task-priority redundancy resolution for
  real-time kinematic control of robot manipulators*, IEEE TRA 1997.
- ros2_control `joint_trajectory_controller` and `GripperActionController`
  Jazzy userdocs (topic-interface semantics; the silent-ignore behavior
  verified in Inception).
- MoveIt 2 planning-scene documentation (`PlanningScene`,
  `AttachedCollisionObject` absorb-from-world semantics; verified 20/20 in
  Inception).
- Reference implementations named in Inception: mediapipe_dual_arm_control,
  Multi-Arm-6-DOF-Robot-Tele-operation, FrankaTeleop,
  mernaahany/teleoperation-robot-arm, gesturebot.

*Prior project docs: `docs/handfollow-inception.md`, `docs/handfollow-run.md`,
`robot_arm_moveit_config/scripts/hand_follow.py` (U1–U5 heritage).*
