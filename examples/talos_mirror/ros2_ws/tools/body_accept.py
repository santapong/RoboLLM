#!/usr/bin/env python3
"""body_accept -- TALOS M3 acceptance: is full-body tracking correct?

Ported from examples/humanoid_mirror/ros2_ws/tools/body_accept.py and
extended for the leg chain + per-limb gating this task adds. Three tiers:

  (default) SYNTHETIC -- pure geometry, no camera, no mediapipe, no ROS.
            Known-answer tests on the torso frame, the axis mapping, the
            per-limb visibility gates (arms/legs/head, independently), the
            quaternion, and the pose_source full-body motion generator.
            Also a STATIC check that BodyTracker.read() never flips before
            inference (the raw-frame Pose law), WITH a negative control that
            proves the check would fail against a deliberately-flipped copy
            of the same source. Runs headless, no person needed.

  --live    CAMERA -- needs a person in frame. Runs the flip/label
            discriminator that guards the single nastiest trap in this
            example, plus inference rate and full-body landmark-quality
            reporting. NOT run by this task (gate 2) -- included for parity
            with humanoid_mirror and for later use.

  --ros     TOPICS -- needs `ros2-arm track` running. Checks /body/tracked,
            /body/observation and /body/markers actually publish, and that
            human/* TF frames for at least one arm, one leg, the head and
            the torso are broadcast.

Emits RESULT:{json}, matching the other accept tools in this repo.
"""

import argparse
import json
import math
import re
import sys

sys.path.insert(0, __file__.rsplit("/tools/", 1)[0] + "/src/talos_mirror")

from talos_mirror.body_track import (  # noqa: E402
    LEFT_ANKLE,
    LEFT_EAR,
    LEFT_ELBOW,
    LEFT_HIP,
    LEFT_KNEE,
    LEFT_SHOULDER,
    LEFT_WRIST,
    LIMB_GATE_LANDMARKS,
    LIMB_LANDMARKS,
    LIMBS,
    NOSE,
    RIGHT_ANKLE,
    RIGHT_EAR,
    RIGHT_ELBOW,
    RIGHT_HIP,
    RIGHT_KNEE,
    RIGHT_SHOULDER,
    RIGHT_WRIST,
    BodyObservation,
    basis_to_quaternion,
    cross,
    dot,
    gates,
    limb_gate,
    norm,
    observed_landmarks,
    sub,
    to_body,
    torso_frame,
)
from talos_mirror.pose_source import SyntheticBodyPoseSource, body_points  # noqa: E402

FAILURES = []
BODY_TRACK_PATH = __file__.rsplit("/tools/", 1)[0] + "/src/talos_mirror/talos_mirror/body_track.py"


def check(ok, label, detail=""):
    print(f"  [{'PASS' if ok else 'FAIL'}] {label}{(' — ' + detail) if detail else ''}")
    if not ok:
        FAILURES.append(label)
    return ok


def close(a, b, tol=1e-6):
    return abs(a - b) <= tol


# ---------------------------------------------------------------------------
# A synthetic upright person FACING the camera, expressed in MediaPipe WORLD
# axes (+x = subject's left, +y = down, -z = forward). Arms straight down,
# legs straight down. Every number is chosen so the expected torso frame is
# exactly the identity, which makes the assertions below unambiguous.
# ---------------------------------------------------------------------------
def synthetic_world(arm="down", leg="down"):
    p = {
        LEFT_SHOULDER: (0.20, -0.50, 0.0),
        RIGHT_SHOULDER: (-0.20, -0.50, 0.0),
        LEFT_HIP: (0.10, 0.0, 0.0),
        RIGHT_HIP: (-0.10, 0.0, 0.0),
        NOSE: (0.0, -0.75, -0.10),
        LEFT_EAR: (0.08, -0.72, 0.0),
        RIGHT_EAR: (-0.08, -0.72, 0.0),
    }
    if arm == "down":            # elbows below shoulders, wrists below elbows
        p[LEFT_ELBOW] = (0.20, -0.22, 0.0)
        p[LEFT_WRIST] = (0.20, 0.05, 0.0)
        p[RIGHT_ELBOW] = (-0.20, -0.22, 0.0)
        p[RIGHT_WRIST] = (-0.20, 0.05, 0.0)
    elif arm == "t_pose":        # arms straight out to the sides
        p[LEFT_ELBOW] = (0.48, -0.50, 0.0)
        p[LEFT_WRIST] = (0.75, -0.50, 0.0)
        p[RIGHT_ELBOW] = (-0.48, -0.50, 0.0)
        p[RIGHT_WRIST] = (-0.75, -0.50, 0.0)
    if leg == "down":             # knees below hips, ankles below knees
        p[LEFT_KNEE] = (0.10, 0.45, 0.0)
        p[LEFT_ANKLE] = (0.10, 0.85, 0.0)
        p[RIGHT_KNEE] = (-0.10, 0.45, 0.0)
        p[RIGHT_ANKLE] = (-0.10, 0.85, 0.0)
    elif leg == "lifted":          # left knee raised/bent forward (a step)
        p[LEFT_KNEE] = (0.10, 0.30, -0.25)
        p[LEFT_ANKLE] = (0.10, 0.55, -0.15)
        p[RIGHT_KNEE] = (-0.10, 0.45, 0.0)
        p[RIGHT_ANKLE] = (-0.10, 0.85, 0.0)
    return p


def as_body(world):
    return {i: to_body(v) for i, v in world.items()}


def make_obs(arm="down", leg="down", vis=None, hips=True):
    world = synthetic_world(arm, leg)
    if not hips:
        world.pop(LEFT_HIP, None)
        world.pop(RIGHT_HIP, None)
    points = as_body(world)
    visibility = {i: 1.0 for i in points}
    if vis:
        visibility.update(vis)
    basis, source = torso_frame(points, visibility)
    return BodyObservation(0.0, points, visibility, basis, source, basis is not None)


# ---------------------------------------------------------------- synthetic
def run_synthetic():
    print("\naxis mapping (measured convention: body_x=-wz, body_y=+wx, body_z=-wy)")
    bl = to_body((0.20, -0.50, 0.0))
    check(bl[1] > 0, "subject's LEFT maps to body +y", f"y={bl[1]:+.2f}")
    check(bl[2] > 0, "shoulders (world -y) map to body +z (up)", f"z={bl[2]:+.2f}")
    fwd = to_body((0.0, 0.0, -0.10))
    check(fwd[0] > 0, "world -z (toward camera) maps to body +x (forward)",
          f"x={fwd[0]:+.2f}")

    print("\ntorso frame — hips visible")
    obs = make_obs("down", "down")
    check(obs.frame_source == "hips", "uses the hip path when hips are visible",
          obs.frame_source)
    f, l, u = obs.basis
    check(close(norm(f), 1.0, 1e-9) and close(norm(l), 1.0, 1e-9)
          and close(norm(u), 1.0, 1e-9), "basis vectors are unit length")
    check(abs(dot(f, l)) < 1e-9 and abs(dot(l, u)) < 1e-9 and abs(dot(f, u)) < 1e-9,
          "basis is orthogonal")
    check(close(dot(cross(l, u), f), 1.0, 1e-9), "basis is RIGHT-handed (f = l x u)")
    check(u[2] > 0.99, "up points along body +z for an upright person",
          f"u={tuple(round(v, 3) for v in u)}")
    check(l[1] > 0.99, "left points along body +y", f"l={tuple(round(v, 3) for v in l)}")
    check(f[0] > 0.99, "forward points along body +x",
          f"f={tuple(round(v, 3) for v in f)}")

    print("\ntorso frame — hips MISSING (falls back to camera_up)")
    obs_nohip = make_obs("down", "down", hips=False)
    check(obs_nohip.basis is not None, "still produces a frame without hips")
    check(obs_nohip.frame_source == "camera_up", "falls back to camera_up",
          obs_nohip.frame_source)
    fa, la, ua = obs.basis
    fb, lb, ub = obs_nohip.basis
    agree = max(norm(sub(fa, fb)), norm(sub(la, lb)), norm(sub(ua, ub)))
    check(agree < 1e-6, "fallback frame == hip frame for an upright subject",
          f"max axis delta {agree:.2e}")

    print("\nvisibility gating — overall trackability needs only the shoulders")
    blind = make_obs("down", "down", vis={LEFT_SHOULDER: 0.1})
    check(blind.basis is None, "no frame when a shoulder is not visible")
    nose_blind = make_obs("down", "down", vis={NOSE: 0.0})
    check(nose_blind.basis is not None,
          "losing the NOSE alone still yields a torso frame "
          "(nose is the HEAD limb's own gate, not a whole-body one)")

    print("\nper-limb gates — independent, all five limbs")
    obs_full = make_obs("down", "down")
    g_full = obs_full.gates()
    check(all(g_full[limb] for limb in LIMBS), "all five limbs gate TRUE when fully visible",
          str(g_full))
    check(len(obs_full.observed_landmarks()) == sum(len(LIMB_LANDMARKS[l]) for l in LIMBS),
          "observed_landmarks is the full union when every limb is visible")

    for hidden in LIMBS:
        vis = {i: 1.0 for i in obs_full.visibility}
        for i in LIMB_GATE_LANDMARKS[hidden]:
            vis[i] = 0.0
        g = gates(vis)
        others_ok = all(g[l] for l in LIMBS if l != hidden)
        check(g[hidden] is False and others_ok,
              f"hiding {hidden} drops ONLY {hidden} (other four gates unaffected)",
              str(g))
        obs_hidden = BodyObservation(0.0, obs_full.points_body, vis, obs_full.basis,
                                     obs_full.frame_source, True)
        seen = obs_hidden.observed_landmarks()
        check(not any(i in seen for i in LIMB_LANDMARKS[hidden]),
              f"{hidden}'s landmarks are ABSENT from the observation, not marked invalid")
        for other in LIMBS:
            if other == hidden:
                continue
            check(all(i in seen for i in LIMB_LANDMARKS[other]),
                  f"{other}'s landmarks stay present while {hidden} is hidden")

    print("\nlandmark set completeness per gate state")
    for limb in LIMBS:
        vis_pass = {i: 1.0 for i in obs_full.visibility}
        check(limb_gate(vis_pass, limb) is True,
              f"{limb} gate TRUE -> full landmark set available",
              str(LIMB_LANDMARKS[limb]))
        vis_fail = dict(vis_pass)
        # Fail just ONE of the gate landmarks -- the whole limb must still
        # drop (all-or-nothing gating, not per-point).
        one = LIMB_GATE_LANDMARKS[limb][0]
        vis_fail[one] = 0.0
        check(limb_gate(vis_fail, limb) is False,
              f"{limb}: ONE low-visibility gate landmark ({one}) fails the WHOLE limb")
        seen_fail = observed_landmarks(vis_fail)
        check(not any(i in seen_fail for i in LIMB_LANDMARKS[limb]),
              f"{limb}: no partial chain leaks through when only one gate point is bad")

    print("\nlimb directions in the torso frame — arms")
    a_l, b_l = make_obs("down", "down").limb_dirs("left")
    check(a_l[2] < -0.99, "arms-down: left upper arm points DOWN (-z)",
          f"a={tuple(round(v, 3) for v in a_l)}")
    check(b_l[2] < -0.99, "arms-down: left forearm points DOWN (-z)")
    a_t, _ = make_obs("t_pose", "down").limb_dirs("left")
    check(a_t[1] > 0.99, "T-pose: left upper arm points LEFT (+y)",
          f"a={tuple(round(v, 3) for v in a_t)}")
    a_tr, _ = make_obs("t_pose", "down").limb_dirs("right")
    check(a_tr[1] < -0.99, "T-pose: RIGHT upper arm points RIGHT (-y)",
          f"a={tuple(round(v, 3) for v in a_tr)}")

    print("\nlimb directions in the torso frame — legs")
    thigh_l, shin_l = make_obs("down", "down").leg_dirs("left")
    check(thigh_l[2] < -0.99, "legs-down: left thigh points DOWN (-z)",
          f"a={tuple(round(v, 3) for v in thigh_l)}")
    check(shin_l[2] < -0.99, "legs-down: left shin points DOWN (-z)")
    thigh_lift, shin_lift = make_obs("down", "lifted").leg_dirs("left")
    check(thigh_lift[0] > 0.3, "lifted step: left thigh swings FORWARD (+x)",
          f"a={tuple(round(v, 3) for v in thigh_lift)}")

    print("\nelbow / knee angle (bend detection)")
    for arm_pose, expect, label in (("down", 0.0, "straight"), ("t_pose", 0.0, "straight")):
        a, b = make_obs(arm_pose, "down").limb_dirs("left")
        ang = math.acos(max(-1.0, min(1.0, dot(a, b))))
        check(close(ang, expect, 1e-6), f"elbow, {arm_pose}: angle is {label}",
              f"{math.degrees(ang):.2f} deg")
    bent = make_obs("down", "down")
    bent.points_body[LEFT_WRIST] = to_body((0.47, -0.22, 0.0))
    a, b = bent.limb_dirs("left")
    ang = math.degrees(math.acos(max(-1.0, min(1.0, dot(a, b)))))
    check(close(ang, 90.0, 1e-3), "bent elbow reads 90 deg", f"{ang:.2f} deg")

    straight_leg = make_obs("down", "down")
    a, b = straight_leg.leg_dirs("left")
    ang = math.degrees(math.acos(max(-1.0, min(1.0, dot(a, b)))))
    check(close(ang, 0.0, 1e-3), "straight leg: knee angle is 0 deg", f"{ang:.2f} deg")

    print("\nhead direction")
    h = make_obs("down", "down").head_dir()
    check(h[0] > 0.9, "head points FORWARD when facing the camera",
          f"h={tuple(round(v, 3) for v in h)}")

    print("\nquaternion")
    qx, qy, qz, qw = basis_to_quaternion(make_obs("down", "down").basis)
    check(close(math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw), 1.0, 1e-9), "unit norm")
    check(close(abs(qw), 1.0, 1e-6), "identity basis -> identity quaternion",
          f"({qx:.3f},{qy:.3f},{qz:.3f},{qw:.3f})")
    yaw = ((0.0, 1.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 0.0, 1.0))
    q = basis_to_quaternion(yaw)
    check(close(q[2], math.sin(math.pi / 4), 1e-6)
          and close(q[3], math.cos(math.pi / 4), 1e-6),
          "90 deg yaw -> (0,0,.7071,.7071)",
          f"({q[0]:.3f},{q[1]:.3f},{q[2]:.3f},{q[3]:.3f})")

    print("\nraw-frame Pose law — the node must not flip before inference")
    run_raw_frame_law()

    print("\npose_source — the deterministic full-body motion generator")
    run_pose_source()


def _extract_method(source, class_name, method_name):
    """Crude but sufficient: locate `class {class_name}`, then the named
    method's body up to the next same-indent `def` or class boundary."""
    cls_idx = source.index(f"class {class_name}")
    class_src = source[cls_idx:]
    m = re.search(rf"\n    def {method_name}\(.*?\n(?=    def |\nclass |\Z)",
                  class_src, re.S)
    if not m:
        raise AssertionError(f"could not locate {class_name}.{method_name} in source")
    return m.group(0)


def run_raw_frame_law():
    """Static compliance check: BodyTracker.read() (inference) must contain
    no cv2.flip; BodyTracker.annotate() (preview only) is documented to have
    one. Then a NEGATIVE CONTROL: tamper a copy of read()'s source to
    introduce a flip and confirm the SAME check fails against it -- proving
    this is a real guard, not a vacuous pass. No camera needed for any of
    this; it is pure text analysis of body_track.py."""
    source = open(BODY_TRACK_PATH).read()
    read_src = _extract_method(source, "BodyTracker", "read")
    annotate_src = _extract_method(source, "BodyTracker", "annotate")

    check("cv2.flip" not in read_src,
          "BodyTracker.read() contains no cv2.flip (inference sees the RAW frame)")
    check("cv2.flip" in annotate_src,
          "BodyTracker.annotate() DOES flip (preview-only, as documented)")

    print("  negative control: does this check actually catch a flip if one is introduced?")
    needle = "rgb = self._cv2.cvtColor(frame, self._cv2.COLOR_BGR2RGB)"
    if needle not in read_src:
        check(False, "negative-control tamper string not found in read() — update this test")
        return
    tampered = read_src.replace(
        needle, "frame = self._cv2.flip(frame, 1)\n        " + needle, 1
    )
    check(tampered != read_src, "tampering actually changed the text (sanity)")
    tampered_passes = "cv2.flip" not in tampered
    check(not tampered_passes,
          "the SAME check FAILS against a deliberately-flipped read() "
          "(proves it is a real guard, not a no-op)")


def run_pose_source():
    """The synthetic full-body generator: always tracked, exercises every
    limb, and genuinely captures torso lean (not just the camera_up
    fallback) -- these are the properties the track node's synthetic mode
    depends on to prove the publish path end-to-end."""
    src = SyntheticBodyPoseSource(period_s=16.0)
    n = 200
    tracked_count = 0
    frame_sources = set()
    lean_seen = False
    limb_gate_counts = {limb: 0 for limb in LIMBS}
    for i in range(n):
        t = i * src.period_s / n
        obs = src.read(t)
        if obs.tracked:
            tracked_count += 1
            frame_sources.add(obs.frame_source)
            g = obs.gates()
            for limb in LIMBS:
                if g[limb]:
                    limb_gate_counts[limb] += 1
            if abs(obs.basis[2][0]) > 0.01:
                lean_seen = True

    check(tracked_count == n, "synthetic source is ALWAYS tracked (no camera to drop out)",
          f"{tracked_count}/{n}")
    check(frame_sources == {"hips"},
          "synthetic source always has visible hips -> always uses the hip path",
          str(frame_sources))
    check(lean_seen, "torso lean is captured at some point in the cycle "
          "(up vector's forward component departs from 0)")
    check(all(c == n for c in limb_gate_counts.values()),
          "all five limbs stay gated TRUE across the whole synthetic cycle "
          "(the generator always emits full-visibility landmarks)",
          str(limb_gate_counts))

    p0 = body_points(0.0, 16.0)
    check(len(p0) == sum(len(LIMB_LANDMARKS[l]) for l in LIMBS),
          "body_points emits exactly the landmark set the five limbs claim",
          f"{len(p0)} landmarks")
    p0_again = body_points(0.0, 16.0)
    check(p0 == p0_again, "body_points is a pure function of time (deterministic)")


# --------------------------------------------------------------------- live
def run_live(device, model, seconds):
    """THE REGRESSION GUARD for the flip/label trap, plus rate/quality
    reporting. NOT part of this task's run (needs a person in frame) --
    kept for parity with humanoid_mirror and for later use."""
    import statistics
    import time

    import cv2
    import mediapipe as mp
    from mediapipe.tasks import python as mp_python
    from mediapipe.tasks.python import vision as mpv

    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        check(False, "camera opens", device)
        return {}
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    frames = []
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        ok, f = cap.read()
        if ok:
            frames.append(f)
    cap.release()
    check(len(frames) > 10, "captured frames", f"{len(frames)}")
    if not frames:
        return {}

    det = mpv.PoseLandmarker.create_from_options(
        mpv.PoseLandmarkerOptions(
            base_options=mp_python.BaseOptions(model_asset_path=model),
            running_mode=mpv.RunningMode.VIDEO, num_poses=1))

    ms, hits, vis_acc = [], 0, {}
    last = None
    for i, f in enumerate(frames):
        img = mp.Image(image_format=mp.ImageFormat.SRGB,
                       data=cv2.cvtColor(f, cv2.COLOR_BGR2RGB))
        t0 = time.perf_counter()
        res = det.detect_for_video(img, int(i * 33))
        ms.append((time.perf_counter() - t0) * 1e3)
        if res.pose_world_landmarks:
            hits += 1
            last = res
            for idx, lm in enumerate(res.pose_world_landmarks[0]):
                vis_acc.setdefault(idx, []).append(lm.visibility)

    median_ms = statistics.median(ms[3:]) if len(ms) > 4 else float("nan")
    print(f"\ninference: median {median_ms:.1f} ms -> {1000 / median_ms:.1f} Hz, "
          f"detections {hits}/{len(frames)}")
    check(hits > len(frames) * 0.5, "a body is detected in most frames",
          f"{hits}/{len(frames)} — stand in frame if this fails")
    check(median_ms < 70.0, "inference inside the 70 ms U5 budget", f"{median_ms:.1f} ms")

    if not last:
        return {"live_detections": hits, "live_frames": len(frames)}

    print("\nvisibility (medians) — MediaPipe hallucinates what it cannot see")
    named = {"L_shoulder": 11, "R_shoulder": 12, "L_hip": 23, "R_hip": 24,
             "L_knee": 25, "L_ankle": 27, "nose": 0, "L_elbow": 13, "L_wrist": 15}
    meds = {k: statistics.median(vis_acc.get(v, [0.0])) for k, v in named.items()}
    print("   " + "  ".join(f"{k}={v:.2f}" for k, v in meds.items()))
    check(meds["L_shoulder"] > 0.5 and meds["R_shoulder"] > 0.5,
          "both shoulders visible (the core requirement)")

    print("\nFLIP/LABEL REGRESSION GUARD (the nastiest trap in this example)")
    still = frames[len(frames) // 2]
    det_img = mpv.PoseLandmarker.create_from_options(
        mpv.PoseLandmarkerOptions(
            base_options=mp_python.BaseOptions(model_asset_path=model),
            running_mode=mpv.RunningMode.IMAGE, num_poses=1))

    def norm_lms(bgr):
        r = det_img.detect(mp.Image(image_format=mp.ImageFormat.SRGB,
                                    data=cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)))
        return r.pose_landmarks[0] if r.pose_landmarks else None

    raw, flp = norm_lms(still), norm_lms(cv2.flip(still, 1))
    if raw is None or flp is None:
        check(False, "flip guard needs a detection in both raw and flipped")
        return {"live_detections": hits, "live_frames": len(frames)}

    content = abs(flp[LEFT_SHOULDER].x - (1.0 - raw[RIGHT_SHOULDER].x))
    side = abs(flp[LEFT_SHOULDER].x - (1.0 - raw[LEFT_SHOULDER].x))
    print(f"   |flip.LEFT - (1-raw.RIGHT)| = {content:.4f}  <- content-following")
    print(f"   |flip.LEFT - (1-raw.LEFT)|  = {side:.4f}  <- side-following")
    check(content < side, "POSE labels follow ANATOMY, so cv2.flip swaps them",
          f"{side / max(content, 1e-6):.0f}x separation")
    check(content < 0.10, "the content-following match is tight", f"{content:.4f}")
    print("   => Pose MUST run on the RAW frame. Only the preview is flipped.")

    return {
        "live_frames": len(frames), "live_detections": hits,
        "infer_median_ms": round(median_ms, 1),
        "flip_content": round(content, 4), "flip_side": round(side, 4),
        "visibility": {k: round(v, 2) for k, v in meds.items()},
    }


# ---------------------------------------------------------------------- ros
def run_ros(seconds):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Bool, String
    from tf2_ros import Buffer, TransformListener
    from visualization_msgs.msg import MarkerArray

    rclpy.init()
    node = Node("body_accept")
    got = {"tracked": 0, "tracked_true": 0, "markers": 0, "observations": 0}
    parsed = {"gates_seen": set(), "landmark_counts": []}

    def on_tracked(m):
        got["tracked"] += 1
        got["tracked_true"] += int(m.data)

    def on_markers(m):
        got["markers"] += 1

    def on_observation(m):
        got["observations"] += 1
        try:
            d = json.loads(m.data)
        except json.JSONDecodeError:
            return
        if d.get("tracked"):
            parsed["gates_seen"].add(tuple(sorted(d.get("gates", {}).items())))
            parsed["landmark_counts"].append(len(d.get("landmarks", {})))

    node.create_subscription(Bool, "/body/tracked", on_tracked, 10)
    node.create_subscription(MarkerArray, "/body/markers", on_markers, 10)
    node.create_subscription(String, "/body/observation", on_observation, 10)
    buf = Buffer()
    TransformListener(buf, node)
    end = node.get_clock().now().nanoseconds + seconds * 1e9
    while node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    print()
    check(got["tracked"] > 0, "/body/tracked publishes", f"{got['tracked']} msgs")
    check(got["observations"] > 0, "/body/observation publishes",
          f"{got['observations']} msgs")
    check(got["markers"] > 0, "/body/markers publishes", f"{got['markers']} msgs")
    if parsed["landmark_counts"]:
        check(max(parsed["landmark_counts"]) > 0,
              "/body/observation carries a non-empty landmark set while tracked",
              f"max {max(parsed['landmark_counts'])} landmarks/msg")

    frames = ["human/torso", "human/l_shoulder", "human/r_shoulder",
              "human/l_hip", "human/l_knee", "human/l_ankle", "human/head"]
    found = [f for f in frames if buf.can_transform("camera_link", f, rclpy.time.Time())]
    check(len(found) >= 5, "human/* TF frames (arm+leg+head+torso) are broadcast",
          f"{len(found)}/{len(frames)}: {found}")
    node.destroy_node()
    rclpy.shutdown()
    return got


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--live", action="store_true", help="camera tier (needs a person)")
    ap.add_argument("--ros", action="store_true", help="topic tier (needs `ros2-arm track`)")
    ap.add_argument("--seconds", type=float, default=4.0)
    ap.add_argument("--camera", default="/dev/video0")
    ap.add_argument("--model", default="/opt/models/pose_landmarker_full.task")
    args = ap.parse_args()

    print("=" * 68)
    print("  talos_mirror M3 acceptance — full-body tracking")
    print("=" * 68)

    extra = {}
    run_synthetic()
    if args.live:
        extra.update(run_live(args.camera, args.model, args.seconds) or {})
    if args.ros:
        extra.update(run_ros(args.seconds) or {})

    print("\n" + "=" * 68)
    if FAILURES:
        print(f"  FAILED — {len(FAILURES)} check(s):")
        for f in FAILURES:
            print(f"    - {f}")
    else:
        tiers = "synthetic" + (" + live" if args.live else "") + (" + ros" if args.ros else "")
        print(f"  ALL CHECKS PASSED ({tiers})")
    print("=" * 68)
    print("RESULT:" + json.dumps({"passed": not FAILURES,
                                  "failures": FAILURES[:12], **extra}))
    sys.exit(1 if FAILURES else 0)


if __name__ == "__main__":
    main()
