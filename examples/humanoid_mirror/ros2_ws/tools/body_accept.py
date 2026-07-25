#!/usr/bin/env python3
"""M3 acceptance: is body tracking correct?

Two tiers, because the interesting failures live in different places:

  (default) SYNTHETIC — pure geometry, no camera, no mediapipe, no ROS.
            Known-answer tests on the torso frame, the axis mapping, the
            visibility gate and the quaternion. Runs in CI.

  --live    CAMERA — needs a person in frame. Runs the flip/label
            discriminator that guards the single nastiest trap in this
            example, plus inference rate and landmark-quality reporting.

  --ros     TOPICS — needs `ros2-arm track` running. Checks /body/tracked,
            /body/markers and the human/* TF frames actually publish.

Emits RESULT:{json}, matching the other accept tools.
"""

import argparse
import json
import math
import sys

sys.path.insert(0, __file__.rsplit("/tools/", 1)[0] + "/src/humanoid_mirror")

from humanoid_mirror.body_track import (  # noqa: E402
    LEFT_EAR,
    LEFT_ELBOW,
    LEFT_HIP,
    LEFT_SHOULDER,
    LEFT_WRIST,
    NOSE,
    RIGHT_EAR,
    RIGHT_ELBOW,
    RIGHT_HIP,
    RIGHT_SHOULDER,
    RIGHT_WRIST,
    BodyObservation,
    basis_to_quaternion,
    dot,
    norm,
    sub,
    to_body,
    torso_frame,
)

FAILURES = []


def check(ok, label, detail=""):
    print(f"  [{'PASS' if ok else 'FAIL'}] {label}{(' — ' + detail) if detail else ''}")
    if not ok:
        FAILURES.append(label)
    return ok


def close(a, b, tol=1e-6):
    return abs(a - b) <= tol


# ---------------------------------------------------------------------------
# A synthetic upright person FACING the camera, expressed in MediaPipe WORLD
# axes (+x = subject's left, +y = down, -z = forward). Arms straight down.
# Every number here is chosen so the expected torso frame is exactly the
# identity, which makes the assertions below unambiguous.
# ---------------------------------------------------------------------------
def synthetic_world(arm="down"):
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
    elif arm == "forward":       # arms straight forward (toward camera)
        p[LEFT_ELBOW] = (0.20, -0.50, -0.28)
        p[LEFT_WRIST] = (0.20, -0.50, -0.55)
        p[RIGHT_ELBOW] = (-0.20, -0.50, -0.28)
        p[RIGHT_WRIST] = (-0.20, -0.50, -0.55)
    return p


def as_body(world):
    return {i: to_body(v) for i, v in world.items()}


def make_obs(arm="down", vis=None, hips=True):
    world = synthetic_world(arm)
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
    # Subject's LEFT shoulder must land on +y (left) in body axes.
    bl = to_body((0.20, -0.50, 0.0))
    check(bl[1] > 0, "subject's LEFT maps to body +y", f"y={bl[1]:+.2f}")
    check(bl[2] > 0, "shoulders (world -y) map to body +z (up)", f"z={bl[2]:+.2f}")
    # Nose is forward of ears: world -z  ->  body +x
    fwd = to_body((0.0, 0.0, -0.10))
    check(fwd[0] > 0, "world -z (toward camera) maps to body +x (forward)",
          f"x={fwd[0]:+.2f}")

    print("\ntorso frame — hips visible")
    obs = make_obs("down")
    check(obs.frame_source == "hips", "uses the hip path when hips are visible",
          obs.frame_source)
    f, l, u = obs.basis
    check(close(norm(f), 1.0, 1e-9) and close(norm(l), 1.0, 1e-9)
          and close(norm(u), 1.0, 1e-9), "basis vectors are unit length")
    check(abs(dot(f, l)) < 1e-9 and abs(dot(l, u)) < 1e-9 and abs(dot(f, u)) < 1e-9,
          "basis is orthogonal")
    # Right-handed: f = l x u  =>  dot(cross(l,u), f) = +1
    from humanoid_mirror.body_track import cross
    check(close(dot(cross(l, u), f), 1.0, 1e-9), "basis is RIGHT-handed (f = l x u)")
    check(u[2] > 0.99, "up points along body +z for an upright person",
          f"u={tuple(round(v,3) for v in u)}")
    check(l[1] > 0.99, "left points along body +y", f"l={tuple(round(v,3) for v in l)}")
    check(f[0] > 0.99, "forward points along body +x",
          f"f={tuple(round(v,3) for v in f)}")

    print("\ntorso frame — hips MISSING (the real desk case: hip vis measured 0.01)")
    obs_nohip = make_obs("down", hips=False)
    check(obs_nohip.basis is not None, "still produces a frame without hips")
    check(obs_nohip.frame_source == "camera_up", "falls back to camera_up",
          obs_nohip.frame_source)
    obs_lowvis = make_obs("down", vis={LEFT_HIP: 0.01, RIGHT_HIP: 0.01})
    check(obs_lowvis.frame_source == "camera_up",
          "low-visibility hips take the fallback, not the hip path",
          obs_lowvis.frame_source)
    # The fallback must agree with the hip path for an upright subject.
    fa, la, ua = obs.basis
    fb, lb, ub = obs_nohip.basis
    agree = max(norm(sub(fa, fb)), norm(sub(la, lb)), norm(sub(ua, ub)))
    check(agree < 1e-6, "fallback frame == hip frame for an upright subject",
          f"max axis delta {agree:.2e}")

    print("\nvisibility gating")
    blind = make_obs("down", vis={LEFT_SHOULDER: 0.1})
    check(blind.basis is None, "no frame when a shoulder is not visible")
    check(torso_frame(as_body(synthetic_world()), {LEFT_SHOULDER: 0.1,
                                                   RIGHT_SHOULDER: 1.0})[1]
          == "shoulders_not_visible", "reports why it failed")

    print("\nlimb directions in the torso frame")
    a_l, b_l = make_obs("down").limb_dirs("left")
    check(a_l[2] < -0.99, "arms-down: left upper arm points DOWN (-z)",
          f"a={tuple(round(v,3) for v in a_l)}")
    check(b_l[2] < -0.99, "arms-down: left forearm points DOWN (-z)")
    a_t, _ = make_obs("t_pose").limb_dirs("left")
    check(a_t[1] > 0.99, "T-pose: left upper arm points LEFT (+y)",
          f"a={tuple(round(v,3) for v in a_t)}")
    a_tr, _ = make_obs("t_pose").limb_dirs("right")
    check(a_tr[1] < -0.99, "T-pose: RIGHT upper arm points RIGHT (-y)",
          f"a={tuple(round(v,3) for v in a_tr)}")
    a_f, _ = make_obs("forward").limb_dirs("left")
    check(a_f[0] > 0.99, "arms-forward: upper arm points FORWARD (+x)",
          f"a={tuple(round(v,3) for v in a_f)}")

    print("\nelbow angle (the M4 retargeting primitive)")
    for arm, expect, label in (("down", 0.0, "straight"), ("t_pose", 0.0, "straight")):
        a, b = make_obs(arm).limb_dirs("left")
        ang = math.acos(max(-1.0, min(1.0, dot(a, b))))
        check(close(ang, expect, 1e-6), f"{arm}: elbow angle is {label}",
              f"{math.degrees(ang):.2f} deg")
    # Bent elbow: forearm 90 deg from a downward upper arm.
    bent = make_obs("down")
    bent.points_body[LEFT_WRIST] = to_body((0.47, -0.22, 0.0))
    a, b = bent.limb_dirs("left")
    ang = math.degrees(math.acos(max(-1.0, min(1.0, dot(a, b)))))
    check(close(ang, 90.0, 1e-3), "bent elbow reads 90 deg", f"{ang:.2f} deg")

    print("\nhead direction")
    h = make_obs("down").head_dir()
    check(h[0] > 0.9, "head points FORWARD when facing the camera",
          f"h={tuple(round(v,3) for v in h)}")

    print("\nquaternion")
    qx, qy, qz, qw = basis_to_quaternion(make_obs("down").basis)
    check(close(math.sqrt(qx*qx+qy*qy+qz*qz+qw*qw), 1.0, 1e-9), "unit norm")
    check(close(abs(qw), 1.0, 1e-6), "identity basis -> identity quaternion",
          f"({qx:.3f},{qy:.3f},{qz:.3f},{qw:.3f})")
    # A 90 deg yaw basis must round-trip to a known quaternion.
    yaw = ((0.0, 1.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 0.0, 1.0))
    q = basis_to_quaternion(yaw)
    check(close(q[2], math.sin(math.pi / 4), 1e-6)
          and close(q[3], math.cos(math.pi / 4), 1e-6),
          "90 deg yaw -> (0,0,.7071,.7071)",
          f"({q[0]:.3f},{q[1]:.3f},{q[2]:.3f},{q[3]:.3f})")


# --------------------------------------------------------------------- live
def run_live(device, model, seconds):
    """THE REGRESSION GUARD for the flip trap, plus rate/quality reporting."""
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
    check(median_ms < 70.0, "inference inside the 70 ms U5 budget",
          f"{median_ms:.1f} ms")

    if not last:
        return {"live_detections": hits, "live_frames": len(frames)}

    print("\nvisibility (medians) — MediaPipe hallucinates what it cannot see")
    named = {"L_shoulder": 11, "R_shoulder": 12, "L_hip": 23, "R_hip": 24,
             "nose": 0, "L_elbow": 13, "L_wrist": 15}
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
    check(content < side,
          "POSE labels follow ANATOMY, so cv2.flip swaps them",
          f"{side / max(content, 1e-6):.0f}x separation")
    check(content < 0.10,
          "the content-following match is tight (labels really are anatomical)",
          f"{content:.4f}")
    print("   => Pose MUST run on the RAW frame. Only the preview is flipped.")
    print("   (hands are the OPPOSITE: handedness assumes a mirrored image,")
    print("    which is why hand_follow's flip is correct and mandatory.)")

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
    from std_msgs.msg import Bool
    from tf2_ros import Buffer, TransformListener
    from visualization_msgs.msg import MarkerArray

    rclpy.init()
    node = Node("body_accept")
    got = {"tracked": 0, "tracked_true": 0, "markers": 0}
    node.create_subscription(Bool, "/body/tracked", lambda m: (
        got.__setitem__("tracked", got["tracked"] + 1),
        got.__setitem__("tracked_true", got["tracked_true"] + int(m.data))), 10)
    node.create_subscription(MarkerArray, "/body/markers",
                             lambda m: got.__setitem__("markers", got["markers"] + 1), 10)
    buf = Buffer()
    TransformListener(buf, node)
    end = node.get_clock().now().nanoseconds + seconds * 1e9
    while node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    print()
    check(got["tracked"] > 0, "/body/tracked publishes", f"{got['tracked']} msgs")
    check(got["markers"] > 0, "/body/markers publishes", f"{got['markers']} msgs")
    frames = ["human/torso", "human/l_shoulder", "human/r_shoulder", "human/head"]
    found = [f for f in frames if buf.can_transform("camera_link", f,
                                                    rclpy.time.Time())]
    check(len(found) >= 3, "human/* TF frames are broadcast",
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
    print("  humanoid_mirror M3 acceptance — body tracking")
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
