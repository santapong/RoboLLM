#!/usr/bin/env python3
"""M4 verification: is the retargeting geometry actually right?

Three tiers, hardest question first:

  --fk    GROUND TRUTH. Compares this repo's fk_arm() against MoveIt's own
          /compute_fk over random joint configurations, for BOTH arms.
          If these disagree, every angle downstream is wrong and no amount
          of round-trip self-consistency would reveal it.

  (default) ROUND-TRIP + known answers, pure math, no ROS, no camera.
          solve_arm() must reproduce the direction it was asked for, over a
          dense sweep of reachable directions, and known human poses must
          produce sane joint angles.

  --ros   LIVE. With `ros2-arm mirror` running, checks the arms are actually
          commanded and stay inside limits.

Emits RESULT:{json}.
"""

import argparse
import json
import math
import random
import sys

sys.path.insert(0, __file__.rsplit("/tools/", 1)[0] + "/src/humanoid_mirror")

from humanoid_mirror.ffw_arm import (  # noqa: E402
    LIMITS,
    _dot,
    _sub,
    _unit,
    arm_dirs,
    fk_arm,
    solve_arm,
)

FAILURES = []


def check(ok, label, detail=""):
    print(f"  [{'PASS' if ok else 'FAIL'}] {label}{(' — ' + detail) if detail else ''}")
    if not ok:
        FAILURES.append(label)
    return ok


def ang(u, v):
    return math.degrees(math.acos(max(-1.0, min(1.0, _dot(_unit(u), _unit(v))))))


# --------------------------------------------------------------- FK vs MoveIt
def run_fk(samples):
    """The check that matters most: our FK vs MoveIt's, on the real model."""
    import rclpy
    from moveit_msgs.srv import GetPositionFK
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Header

    rclpy.init()
    node = Node("retarget_bench_fk")
    cli = node.create_client(GetPositionFK, "/compute_fk")
    if not cli.wait_for_service(timeout_sec=20.0):
        check(False, "/compute_fk available", "is `ros2-arm humanoid` running?")
        node.destroy_node(); rclpy.shutdown()
        return {}

    rng = random.Random(20260725)
    worst = {"left": 0.0, "right": 0.0}
    n_ok = 0
    for side in ("left", "right"):
        pre = "arm_l" if side == "left" else "arm_r"
        # MoveIt reports link poses in base_link; our FK is in arm_base_link.
        # Compare DIFFERENCES between two links, which cancels the unknown
        # base offset entirely and tests exactly what retargeting uses.
        for _ in range(samples):
            q = [rng.uniform(lo + 0.1, hi - 0.1) for lo, hi in LIMITS[side][:7]]
            req = GetPositionFK.Request()
            req.header = Header(frame_id="base_link")
            req.fk_link_names = [f"{pre}_link1", f"{pre}_link4", f"{pre}_link7"]
            req.robot_state.joint_state = JointState(
                name=[f"{pre}_joint{i}" for i in range(1, 8)], position=q)
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
            res = fut.result()
            if res is None or res.error_code.val != 1 or len(res.pose_stamped) != 3:
                continue
            P = [(p.pose.position.x, p.pose.position.y, p.pose.position.z)
                 for p in res.pose_stamped]
            mine = fk_arm(q, side)
            # link_k's frame origin IS joint_k's origin (URDF semantics), so:
            #   link1 <-> "j1",  link4 <-> "elbow",  link7 <-> "wrist".
            # Compare DIFFERENCES so the unknown base_link -> arm_base_link
            # offset cancels. Check LENGTHS too: a direction-only test would
            # pass even if our link offsets were scaled wrong.
            for m_a, m_b, k_a, k_b in ((0, 1, "j1", "elbow"),
                                       (1, 2, "elbow", "wrist")):
                moveit_vec = _sub(P[m_b], P[m_a])
                ours = _sub(mine[k_b], mine[k_a])
                worst[side] = max(worst[side], ang(moveit_vec, ours))
                dl = abs(math.dist(moveit_vec, (0, 0, 0))
                         - math.dist(ours, (0, 0, 0)))
                if dl > 1e-4:
                    worst[side] = max(worst[side], 99.0)  # length mismatch
            n_ok += 1

    node.destroy_node(); rclpy.shutdown()
    print()
    check(n_ok >= samples, "compute_fk answered", f"{n_ok}/{samples * 2} samples")
    for side in ("left", "right"):
        check(worst[side] < 0.5, f"{side}: our fk_arm matches MoveIt /compute_fk",
              f"worst direction error {worst[side]:.4f} deg")
    return {"fk_worst_deg": {k: round(v, 4) for k, v in worst.items()},
            "fk_samples": n_ok}


# ------------------------------------------------------------------ pure math
def run_math(samples):
    rng = random.Random(7)

    print("\nFK sanity (zero pose)")
    for side in ("left", "right"):
        p = fk_arm([0] * 7, side)
        net = _sub(p["wrist"], p["shoulder"])
        check(abs(net[0]) < 1e-9 and abs(net[1]) < 1e-9 and net[2] < -0.6,
              f"{side}: at q=0 the arm hangs straight down",
              f"shoulder->wrist = ({net[0]:+.4f}, {net[1]:+.4f}, {net[2]:+.4f})")
    ua, fa = arm_dirs([0] * 7, "left")
    zig = ang(ua, fa)
    check(14.0 < zig < 15.5,
          "the +-0.041 m elbow offset makes the joint centres zigzag at q=0",
          f"{zig:.2f} deg — this is why the naive 'arm points at (0,0,-1)' "
          f"model is biased")

    print("\nROUND-TRIP: solve_arm() reproduces the direction it was asked for")
    worst, clamped_n, tested = {"left": 0.0, "right": 0.0}, 0, 0
    for side in ("left", "right"):
        for _ in range(samples):
            # Sample a REACHABLE pose by running FK forward, then ask the
            # solver to recover it. Guarantees the target is achievable, so a
            # failure is a solver bug and never an impossible request.
            q = [rng.uniform(lo + 0.15, hi - 0.15) for lo, hi in LIMITS[side][:4]]
            a, b = arm_dirs(q, side)
            qs, info = solve_arm(a, b, side)
            worst[side] = max(worst[side], info["err_deg"])
            clamped_n += int(info["clamped"])
            tested += 1
        check(worst[side] < 1.0, f"{side}: round-trip error under 1 deg",
              f"worst {worst[side]:.4f} deg over {samples} poses")

    print("\nKNOWN ANSWERS vs an INDEPENDENT brute-force optimum")
    # The solver is compared against random search + hill climbing, which
    # shares none of its closed-form reasoning. This tests THE SOLVER rather
    # than my assumptions about what the geometry ought to allow — the
    # distinction that matters, because some natural human poses are simply
    # not reachable on this arm and asserting "error < 1 deg" against them
    # would be asserting a falsehood.
    def brute(a, b, side, iters=24000):
        rng2 = random.Random(11)
        lim = LIMITS[side][:4]
        best_q, best_e = None, 9e9
        for _ in range(iters):
            q = [rng2.uniform(lo + 0.05, hi - 0.05) for lo, hi in lim]
            ua, fa = arm_dirs(q, side)
            e = max(ang(ua, a), ang(fa, b))
            if e < best_e:
                best_q, best_e = q, e
        step = 0.2
        while step > 1e-4:                       # coordinate hill climb
            improved = False
            for k in range(4):
                for s_ in (+step, -step):
                    q = list(best_q)
                    q[k] = min(max(q[k] + s_, lim[k][0] + 0.05), lim[k][1] - 0.05)
                    ua, fa = arm_dirs(q, side)
                    e = max(ang(ua, a), ang(fa, b))
                    if e < best_e - 1e-9:
                        best_q, best_e, improved = q, e, True
            if not improved:
                step *= 0.5
        return best_e

    POSES = [
        ("arms down", (0.0, 0.0, -1.0), (0.0, 0.0, -1.0)),
        ("T-pose out to the side", (0.0, 1.0, 0.0), (0.0, 1.0, 0.0)),
        ("arms forward", (1.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
        ("90 deg elbow bend", (0.0, 0.0, -1.0), (1.0, 0.0, 0.0)),
        ("hand raised to shoulder", (0.0, 0.7, -0.71), (0.0, 0.3, 0.95)),
    ]
    reach_notes = {}
    for label, a, b in POSES:
        for side in ("left", "right"):
            aa = a if side == "left" else (a[0], -a[1], a[2])
            bb = b if side == "left" else (b[0], -b[1], b[2])
            q, info = solve_arm(_unit(aa), _unit(bb), side)
            opt = brute(_unit(aa), _unit(bb), side)
            gap = info["err_deg"] - opt
            check(gap < 0.75,
                  f"{side}: '{label}' is as good as brute force",
                  f"solver {info['err_deg']:.3f} deg vs optimum "
                  f"{opt:.3f} deg (gap {gap:+.3f})")
            if side == "left":
                reach_notes[label] = round(opt, 2)

    print("\n  geometric reality — best ACHIEVABLE error per pose (left arm):")
    for label, deg in reach_notes.items():
        note = "reachable" if deg < 1.0 else f"UNREACHABLE by {deg:.1f} deg"
        print(f"    {label:26s} {deg:6.2f} deg   {note}")
    check(reach_notes.get("T-pose out to the side", 9) < 1.0,
          "a T-pose IS reachable — humeral yaw rotates the elbow offset away",
          f"{reach_notes.get('T-pose out to the side')} deg. The shoulder cone "
          f"|a_y| <= sqrt(1 - w_x^2) looks like a limit at q3=0, but w_x -> 0 "
          f"at q3 = +-pi/2, so the bound is q3-dependent, not fixed.")

    print("\nsign convention: one formula serves BOTH arms")
    # The researched design guessed the right arm needed asin(-a_y). If that
    # were applied, right-arm roll would go positive, outside [-3.14, 0], clamp
    # to ~0, and the arm would never lift.
    ql, _ = solve_arm(_unit((0.0, 0.9, -0.44)), _unit((0.0, 0.9, -0.44)), "left")
    qr, _ = solve_arm(_unit((0.0, -0.9, -0.44)), _unit((0.0, -0.9, -0.44)), "right")
    check(ql[1] > 0.5 and qr[1] < -0.5,
          "left and right rolls are OPPOSITE-SIGNED for a symmetric raise",
          f"q2_left={ql[1]:+.3f}  q2_right={qr[1]:+.3f}")
    check(all(LIMITS["left"][k][0] <= ql[k] <= LIMITS["left"][k][1] for k in range(4))
          and all(LIMITS["right"][k][0] <= qr[k] <= LIMITS["right"][k][1] for k in range(4)),
          "both solutions are inside their own joint limits")

    print("\nelbow flexes NEGATIVE on FFW")
    for side in ("left", "right"):
        q, _i = solve_arm(_unit((0.0, 0.0, -1.0)), _unit((1.0, 0.0, 0.0)), side)
        check(q[3] < 0, f"{side}: bent elbow gives q4 < 0", f"q4={q[3]:+.4f}")

    print("\nMIRROR SEMANTICS (the sign errors nobody sees until they wave)")
    sys.path.insert(0, __file__.rsplit("/tools/", 1)[0] + "/tools")
    import body_accept as BA
    from humanoid_mirror.body_track import BodyObservation, torso_frame
    from humanoid_mirror.retarget import retarget

    def body(raise_left=False, raise_right=False, vis=None):
        w = BA.synthetic_world("down")
        if raise_left:      # human's own LEFT arm out to their left (+x world)
            w[BA.LEFT_ELBOW] = (0.48, -0.50, 0.0)
            w[BA.LEFT_WRIST] = (0.75, -0.50, 0.0)
        if raise_right:
            w[BA.RIGHT_ELBOW] = (-0.48, -0.50, 0.0)
            w[BA.RIGHT_WRIST] = (-0.75, -0.50, 0.0)
        pts = BA.as_body(w)
        v = {i: 1.0 for i in pts}
        if vis:
            v.update(vis)
        basis, src = torso_frame(pts, v)
        return BodyObservation(0.0, pts, v, basis, src, basis is not None)

    obs = body(raise_left=True)
    a_l, _ = obs.limb_dirs("left")
    check(a_l[1] > 0.99, "synthetic human LEFT arm points to their own left (+y)",
          f"{tuple(round(v, 3) for v in a_l)}")

    tm, im = retarget(obs, mirror=True)
    td, idr = retarget(obs, mirror=False)
    ua_r_mirror, _ = arm_dirs([tm[f"arm_r_joint{k}"] for k in range(1, 5)], "right")
    ua_l_direct, _ = arm_dirs([td[f"arm_l_joint{k}"] for k in range(1, 5)], "left")
    # Mirror: the robot faces you, so your left arm drives its RIGHT arm, and
    # the limb ends up on the robot's right (-y) — the SAME side of the room.
    check(ua_r_mirror[1] < -0.9,
          "mirror: your LEFT arm raises the robot's RIGHT arm, same side of the room",
          f"robot right arm -> {tuple(round(v, 3) for v in ua_r_mirror)} (-y = robot's right)")
    # Direct: the robot is you seen from behind, so left drives left.
    check(ua_l_direct[1] > 0.9,
          "direct: your LEFT arm raises the robot's LEFT arm",
          f"robot left arm -> {tuple(round(v, 3) for v in ua_l_direct)}")
    check(len(im["arms"]) == 2 and len(idr["arms"]) == 2,
          "both arms retarget when both are visible", f"{im['arms']}")

    # PER-ARM GATING — the behaviour the desk measurements demand.
    one = body(raise_left=True, vis={BA.RIGHT_ELBOW: 0.02, BA.RIGHT_WRIST: 0.01})
    t1, i1 = retarget(one, mirror=True)
    check("left->right" in i1["arms"] and len(i1["arms"]) == 1,
          "an arm with invisible landmarks is SKIPPED, not guessed",
          f"arms={i1['arms']} skipped={list(i1['skipped'])}")
    check(not any(k.startswith("arm_l_") for k in t1),
          "no targets emitted for the skipped arm (the node holds it)",
          f"{sorted(k for k in t1 if k.startswith('arm_'))[:3]}...")
    check(any(k.startswith("head_") for k in t1),
          "the head still tracks while an arm is skipped")

    # HEAD sign: mirror must flip yaw, never pitch.
    tilt = body()
    tilt.points_body[BA.NOSE] = BA.to_body((0.25, -0.75, -0.10))  # look to own left
    hm, _ = retarget(tilt, mirror=True), None
    hd = retarget(tilt, mirror=False)[0]
    hm = hm[0]
    check(hm["head_joint2"] * hd["head_joint2"] < 0,
          "mirror flips head YAW", f"mirror {hm['head_joint2']:+.3f} vs direct {hd['head_joint2']:+.3f}")
    check(abs(hm["head_joint1"] - hd["head_joint1"]) < 1e-9,
          "mirror leaves head PITCH alone", f"{hm['head_joint1']:+.4f}")
    check(-0.35 <= hm["head_joint2"] <= 0.35 and -0.2317 <= hm["head_joint1"] <= 0.6951,
          "head targets stay inside the (tiny) neck limits")

    print("\ncontinuity (no branch flips frame to frame)")
    # Sweep an arm slowly through a reachable arc; consecutive solutions must
    # not jump. A jump here would show up as the robot snapping in the demo.
    prev, biggest = None, 0.0
    for i in range(120):
        t = i / 119.0
        a = _unit((0.3 * math.sin(t * math.pi), 0.9 * t, -1.0 + 0.5 * t))
        q, _ = solve_arm(a, a, "left", q_seed=prev)
        if prev is not None:
            biggest = max(biggest, max(abs(q[k] - prev[k]) for k in range(4)))
        prev = q
    check(biggest < 0.25, "warm-started solutions stay continuous",
          f"largest single-step joint jump {biggest:.4f} rad")

    print("\nspeed")
    import time
    a = _unit((0.2, 0.6, -0.8))
    t0 = time.perf_counter()
    for _ in range(400):
        solve_arm(a, a, "left", q_seed=prev)
    per = (time.perf_counter() - t0) / 400 * 1e3
    check(per < 5.0, "solve cost fits a 50 Hz budget (20 ms/tick, 2 arms)",
          f"{per:.3f} ms per arm")

    return {"roundtrip_worst_deg": {k: round(v, 4) for k, v in worst.items()},
            "roundtrip_samples": tested, "solve_ms": round(per, 3),
            "continuity_max_rad": round(biggest, 4)}


# ----------------------------------------------------------------------- ros
def run_ros(seconds):
    import time

    import rclpy
    from rclpy.node import Node
    from trajectory_msgs.msg import JointTrajectory

    from humanoid_mirror.joint_limits import MEASURED

    rclpy.init()
    node = Node("retarget_bench_ros")
    got = {"l": 0, "r": 0}
    bad = []

    def cb(key, msg):
        got[key] += 1
        for n, p in zip(msg.joint_names, msg.points[0].positions):
            lo, hi = MEASURED[n][0], MEASURED[n][1]
            if not (lo - 1e-6 <= p <= hi + 1e-6):
                bad.append(f"{n}={p:.4f} outside [{lo}, {hi}]")

    node.create_subscription(JointTrajectory, "/arm_l_controller/joint_trajectory",
                             lambda m: cb("l", m), 20)
    node.create_subscription(JointTrajectory, "/arm_r_controller/joint_trajectory",
                             lambda m: cb("r", m), 20)
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node(); rclpy.shutdown()

    print()
    check(got["l"] > 20 and got["r"] > 20, "both arms are being commanded",
          f"left {got['l']}, right {got['r']} msgs")
    check(not bad, "every commanded angle is inside its URDF limit",
          f"{len(bad)} violations" + (f" e.g. {bad[0]}" if bad else ""))
    return {"ros_left_msgs": got["l"], "ros_right_msgs": got["r"],
            "ros_limit_violations": len(bad)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fk", action="store_true", help="compare against MoveIt /compute_fk")
    ap.add_argument("--ros", action="store_true", help="check live arm commands")
    ap.add_argument("--samples", type=int, default=150)
    ap.add_argument("--seconds", type=float, default=6.0)
    args = ap.parse_args()

    print("=" * 70)
    print("  humanoid_mirror M4 verification — arm retargeting")
    print("=" * 70)

    extra = run_math(args.samples)
    if args.fk:
        extra.update(run_fk(min(args.samples, 40)) or {})
    if args.ros:
        extra.update(run_ros(args.seconds) or {})

    print("\n" + "=" * 70)
    if FAILURES:
        print(f"  FAILED — {len(FAILURES)} check(s):")
        for f in FAILURES:
            print(f"    - {f}")
    else:
        print("  ALL CHECKS PASSED")
    print("=" * 70)
    print("RESULT:" + json.dumps({"passed": not FAILURES,
                                  "failures": FAILURES[:12], **extra}))
    sys.exit(1 if FAILURES else 0)


if __name__ == "__main__":
    main()
