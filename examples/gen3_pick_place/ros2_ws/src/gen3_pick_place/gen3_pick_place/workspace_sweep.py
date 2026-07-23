"""workspace_sweep.py — NON-circular workspace-box tuning/validation (U3).

Runs against the live U2 bring-up (move_group /compute_ik + /compute_fk) and
proves, rather than assumes, that the tuned WORKSPACE_BOX in ik_client.py is
usable (gap 16):

  A. GRID       5x5x5 fixed-downward IK grid over the box — every point must
                solve (the box is tuned until it does; a clamped/tightened
                box would show up here as failures, not be silently accepted).
  B. SWEEP      200-step continuous ORIENTED sweep (Lissajous path inside the
                box + synthetic palm world-landmarks with tilt up to 25 deg
                and yaw sweep, run through palm_orientation -> the full
                degradation chain, warm-seeded, joint-jump guard 0.5 rad).
                Acceptance: >= 99 % solved, median /compute_ik round-trip
                < 15 ms warm, no accepted step jumps > 0.5 rad.
  C. SPAWN      the 4 cm cube spawn pose + grasp height (gap 10) are checked
                GEOMETRICALLY inside the box with >= 3 cm margin on all six
                faces AND PROVEN reachable by explicit IK solves (fixed-down
                grasp, 15/25-deg tilted oriented grasps, pre-grasp hover at
                +10 cm) — non-circular: reachability comes from /compute_ik,
                not from the box definition itself.
  D. DEGRADE    the chain demonstrably fires: an adversarial orientation at a
                box-edge position must fall through oriented -> fixed_down,
                and an out-of-reach position must return MODE_FAILED (caller
                would hold, gap 6).
  E. FK         /compute_fk round-trip check (gap 12): FK(IK(pose)) matches
                the requested pose within 1 mm / 0.5 deg for a sample of
                sweep points.

Run inside the ros2-arm container with the pickplace bring-up running:

    ros2 run gen3_pick_place workspace_sweep          # or
    python3 -m gen3_pick_place.workspace_sweep [--steps 200]

Exit code 0 = all phases pass.
"""
from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from typing import List, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from gen3_pick_place.ik_client import (
    ARM_JOINTS, BOX_SIZE, BOX_SPAWN, D_ATTACH, DOWN_QUAT, GRASP_HEIGHT,
    IK_LINK, MODE_FAILED, MODE_FIXED_DOWN, MODE_ORIENTED, WORKSPACE_BOX,
    IKClient, box_margins, cam_to_base, palm_orientation,
)

MARGIN_REQ = 0.03          # m, gap 16
JUMP_GUARD = 0.5           # rad/step acceptance guard
SWEEP_MIN_SOLVED = 0.99
RT_MEDIAN_MAX_MS = 15.0


# ---------------------------------------------------------------------------
# Synthetic palm landmarks: build a 21-point world-landmark list whose palm
# points (0, 5, 9, 17) encode a chosen approach axis + forward axis in BASE
# frame.  This exercises palm_orientation() itself (cross-product order,
# gates, R_cb) instead of feeding quaternions straight to IK — non-circular.

def _base_to_cam(v: Sequence[float]) -> Tuple[float, float, float]:
    # inverse of cam_to_base: base = (-zc, xc, -yc)  =>  cam = (by, -bz, -bx)
    bx, by, bz = v
    return (by, -bz, -bx)


def _unit(v):
    n = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
    return (v[0] / n, v[1] / n, v[2] / n)


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def synth_palm_landmarks(z_t_base: Sequence[float],
                         x_t_base: Sequence[float]) -> List[Tuple[float, float, float]]:
    """21 world landmarks for a palm whose derived tool frame is
    (x_t_base, ., z_t_base). z_t must be unit; x_t is orthogonalized."""
    z = _unit(z_t_base)
    d = x_t_base[0] * z[0] + x_t_base[1] * z[1] + x_t_base[2] * z[2]
    xr = (x_t_base[0] - d * z[0], x_t_base[1] - d * z[1], x_t_base[2] - d * z[2])
    if math.sqrt(xr[0] ** 2 + xr[1] ** 2 + xr[2] ** 2) < 1e-6:
        # requested forward parallel to approach: use any perpendicular
        helper = (0.0, 0.0, 1.0) if abs(z[2]) < 0.9 else (0.0, 1.0, 0.0)
        xr = _cross(helper, z)
    x = _unit(xr)
    n_cam = _base_to_cam(z)          # palm normal (camera frame)
    f_cam = _base_to_cam(x)          # palm forward (camera frame)
    a_cam = _cross(f_cam, n_cam)     # across palm so that v2 x v1 == n
    lms = [(0.0, 0.0, 0.0)] * 21
    p0 = (0.0, 0.0, 0.0)
    lms[0] = p0
    lms[9] = (p0[0] + 0.09 * f_cam[0], p0[1] + 0.09 * f_cam[1],
              p0[2] + 0.09 * f_cam[2])                       # |v1| = 0.09
    lms[5] = (p0[0] + 0.04 * f_cam[0] + 0.03 * a_cam[0],
              p0[1] + 0.04 * f_cam[1] + 0.03 * a_cam[1],
              p0[2] + 0.04 * f_cam[2] + 0.03 * a_cam[2])
    lms[17] = (p0[0] + 0.04 * f_cam[0] - 0.03 * a_cam[0],
               p0[1] + 0.04 * f_cam[1] - 0.03 * a_cam[1],
               p0[2] + 0.04 * f_cam[2] - 0.03 * a_cam[2])    # |v2| = 0.06
    return lms


def tilted_down(tilt_deg: float, azim_deg: float) -> Tuple[float, float, float]:
    """Approach axis: straight-down tilted by tilt_deg toward azimuth."""
    t, a = math.radians(tilt_deg), math.radians(azim_deg)
    return (math.sin(t) * math.cos(a), math.sin(t) * math.sin(a), -math.cos(t))


def yawed_forward(yaw_deg: float) -> Tuple[float, float, float]:
    y = math.radians(yaw_deg)
    return (math.cos(y), math.sin(y), 0.0)


# ---------------------------------------------------------------------------

class SweepNode(Node):
    def __init__(self):
        super().__init__("workspace_sweep")
        self.js: Optional[JointState] = None
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)

    def _on_js(self, msg):
        self.js = msg


def run(steps: int) -> int:
    rclpy.init()
    node = SweepNode()
    client = IKClient(node)
    report = {"box": WORKSPACE_BOX, "box_spawn": BOX_SPAWN,
              "grasp_height": GRASP_HEIGHT, "d_attach": D_ATTACH}
    failures: List[str] = []

    # ---- 0. cross-module reconciliation (scene_box <-> ik_client) --------
    try:
        from gen3_pick_place import scene_box as sb
        recon = {
            "ws_box_equal": sb.WS_BOX == WORKSPACE_BOX,
            "spawn_equal": tuple(sb.SPAWN_XYZ) == tuple(BOX_SPAWN),
            "d_attach_equal": abs(sb.D_ATTACH - D_ATTACH) < 1e-9,
        }
        report["scene_box_reconciled"] = recon
        if not all(recon.values()):
            failures.append(f"RECONCILE: scene_box constants diverged: {recon}")
        # spawn margin must also cover the attach gate (scene_box invariant)
        if min(box_margins(BOX_SPAWN)) + 1e-9 < D_ATTACH:
            failures.append("RECONCILE: spawn margin < D_ATTACH")
    except ImportError:
        report["scene_box_reconciled"] = "scene_box not importable (skipped)"

    if not client.wait_for_services(timeout=30.0):
        print("FATAL: /compute_ik or /compute_fk unavailable — is the "
              "pickplace bring-up running?")
        rclpy.shutdown()
        return 2

    # seed from /joint_states (subset reconcile, gap 8)
    t0 = time.monotonic()
    while node.js is None and time.monotonic() - t0 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.js is not None and client.seed_from_joint_state(node.js):
        report["seeded_from_joint_states"] = True
    else:
        report["seeded_from_joint_states"] = False
        print("WARN: no /joint_states within 5 s — using default seed")

    (x0, x1), (y0, y1), (z0, z1) = (WORKSPACE_BOX["x"], WORKSPACE_BOX["y"],
                                    WORKSPACE_BOX["z"])

    # ---- A. fixed-down 5x5x5 grid ----------------------------------------
    n_grid, n_grid_ok = 0, 0
    grid_fail_pts = []
    for i in range(5):
        for j in range(5):
            for k in range(5):
                p = (x0 + (x1 - x0) * i / 4, y0 + (y1 - y0) * j / 4,
                     z0 + (z1 - z0) * k / 4)
                res = client.solve(p, quat=DOWN_QUAT)   # full chain + ladder
                n_grid += 1
                if res.mode != MODE_FAILED:
                    n_grid_ok += 1
                else:
                    grid_fail_pts.append([round(v, 3) for v in p])
    report["grid"] = {"points": n_grid, "solved": n_grid_ok,
                      "failed_at": grid_fail_pts[:10]}
    if n_grid_ok != n_grid:
        failures.append(f"GRID: {n_grid - n_grid_ok}/{n_grid} fixed-down "
                        f"grid points unsolved: {grid_fail_pts[:10]}")

    # ---- B. 200-step oriented sweep --------------------------------------
    cx, cy, cz = ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)
    ax, ay, az = ((x1 - x0) / 2 - 0.005, (y1 - y0) / 2 - 0.005,
                  (z1 - z0) / 2 - 0.005)

    def path(u: float) -> Tuple[float, float, float]:
        return (cx + ax * math.sin(2 * math.pi * 1 * u),
                cy + ay * math.sin(2 * math.pi * 2 * u + 0.5),
                cz + az * math.sin(2 * math.pi * 3 * u + 1.0))

    # warm-up: bring the seed to the path start (excluded from stats; the
    # real system slews there over many ticks)
    for _ in range(5):
        client.solve(path(0.0), quat=DOWN_QUAT)

    rts: List[float] = []           # per-/compute_ik-call round-trips
    cont_jumps: List[float] = []    # warm-tracked consecutive steps, max |dq|
    rec_jumps: List[float] = []     # recovery/re-acquisition steps (caller
                                    # slews these — hand_follow branch guard)
    modes = {MODE_ORIENTED: 0, MODE_FIXED_DOWN: 0, MODE_FAILED: 0}
    prev_q: Optional[Tuple[float, ...]] = None
    sweep_fail_steps = []
    n_recovered_steps = 0
    for s in range(steps):
        u = s / steps
        tilt = 25.0 * math.sin(2 * math.pi * 2 * u)          # -25..25 deg
        azim = 360.0 * u
        yaw = 40.0 * math.sin(2 * math.pi * 1 * u)           # -40..40 deg
        lms = synth_palm_landmarks(tilted_down(abs(tilt), azim),
                                   yawed_forward(yaw))
        out = client.solve(path(u), world_landmarks=lms,
                           max_jump_rad=JUMP_GUARD)
        for _, att in out.attempts:
            rts.append(att.rt_ms)
        modes[out.mode] += 1
        if out.mode == MODE_FAILED:
            sweep_fail_steps.append(s)
        else:
            if prev_q is not None:
                j = max(abs(a - b) for a, b in zip(out.joints, prev_q))
                # A recovered step has no continuity claim: the caller
                # freezes target advance and slews (per-tick joint clamp)
                # until the gap closes.  Warm-tracked steps must be smooth.
                (rec_jumps if out.recovered else cont_jumps).append(j)
            n_recovered_steps += bool(out.recovered)
            prev_q = out.joints

    solved = (modes[MODE_ORIENTED] + modes[MODE_FIXED_DOWN]) / steps
    med_rt = statistics.median(rts)
    p95_rt = sorted(rts)[int(0.95 * len(rts)) - 1]
    max_cont_jump = max(cont_jumps) if cont_jumps else 0.0
    report["sweep"] = {
        "steps": steps, "oriented": modes[MODE_ORIENTED],
        "fixed_down": modes[MODE_FIXED_DOWN], "failed": modes[MODE_FAILED],
        "solved_frac": round(solved, 4),
        "rt_ms": {"median": round(med_rt, 2), "p95": round(p95_rt, 2),
                  "n_calls": len(rts)},
        "max_warm_step_jump_rad": round(max_cont_jump, 4),
        "recovered_steps": n_recovered_steps,
        "max_recovery_jump_rad": round(max(rec_jumps), 4) if rec_jumps else 0.0,
    }
    if solved < SWEEP_MIN_SOLVED:
        failures.append(f"SWEEP: solved {solved:.1%} < {SWEEP_MIN_SOLVED:.0%} "
                        f"(failed steps {sweep_fail_steps[:10]})")
    if med_rt >= RT_MEDIAN_MAX_MS:
        failures.append(f"SWEEP: median rt {med_rt:.2f} ms >= "
                        f"{RT_MEDIAN_MAX_MS} ms")
    if max_cont_jump > JUMP_GUARD:
        failures.append(f"SWEEP: warm-tracked step jump {max_cont_jump:.3f} "
                        f"rad > {JUMP_GUARD}")

    # ---- C. spawn/grasp non-circular validation --------------------------
    margins = box_margins(BOX_SPAWN)
    report["spawn_margins_m"] = [round(m, 3) for m in margins]
    if min(margins) < MARGIN_REQ:
        failures.append(f"SPAWN: box-spawn margin {min(margins):.3f} m < "
                        f"{MARGIN_REQ} m to workspace-box face")
    gm = box_margins((BOX_SPAWN[0], BOX_SPAWN[1], GRASP_HEIGHT))
    if min(gm) < MARGIN_REQ:
        failures.append(f"SPAWN: grasp-height margin {min(gm):.3f} m < "
                        f"{MARGIN_REQ} m")

    grasp = (BOX_SPAWN[0], BOX_SPAWN[1], GRASP_HEIGHT)
    hover = (BOX_SPAWN[0], BOX_SPAWN[1], GRASP_HEIGHT + 0.10)
    grasp_checks = {}
    for name, pos, lms in (
        ("grasp_fixed_down", grasp, None),
        ("grasp_tilt15", grasp,
         synth_palm_landmarks(tilted_down(15, 90), yawed_forward(0))),
        ("grasp_tilt25", grasp,
         synth_palm_landmarks(tilted_down(25, 180), yawed_forward(20))),
        ("pregrasp_hover", hover, None),
    ):
        if lms is None:
            out = client.solve(pos, quat=DOWN_QUAT)
            ok = out.mode != MODE_FAILED
        else:
            out = client.solve(pos, world_landmarks=lms)
            ok = out.mode == MODE_ORIENTED   # tilted grasp must solve ORIENTED
        grasp_checks[name] = out.mode
        if not ok:
            failures.append(f"SPAWN: {name} unsolved (mode={out.mode})")
    report["grasp_checks"] = grasp_checks

    # ---- D. degradation chain demonstrably fires -------------------------
    # D1: adversarial orientations at box-edge positions until oriented fails
    # while fixed_down still solves.
    adversarial = [
        ((x1, 0.0, z1), (-1.0, 0.0, 0.0)),    # far-high corner, approach -X
        ((x1, y1, z1), (0.0, -1.0, 0.0)),     # corner, approach -Y
        ((cx, cy, z0), (0.0, 0.0, 1.0)),      # low center, approach UP
        ((x1, y0, z1), (0.0, 0.0, 1.0)),      # corner, approach UP
        ((x0, 0.0, z1), (1.0, 0.0, 0.0)),     # near-high edge, approach +X
    ]
    d1 = None
    for pos, zt in adversarial:
        out = client.solve(pos, world_landmarks=synth_palm_landmarks(
            zt, yawed_forward(0)))
        if (out.mode == MODE_FIXED_DOWN and out.orientation_valid
                and out.attempts[0][0] == MODE_ORIENTED
                and not out.attempts[0][1].ok):
            d1 = {"pos": [round(v, 3) for v in pos], "approach": list(zt),
                  "chain": [(m, a.ok) for m, a in out.attempts]}
            break
    report["degradation_oriented_to_fixed"] = d1
    if d1 is None:
        failures.append("DEGRADE: no adversarial orientation made the chain "
                        "fall oriented->fixed_down")

    # D2: out-of-reach position -> MODE_FAILED (caller holds, gap 6)
    before = client.consecutive_failures
    out = client.solve((0.90, 0.0, 0.50), world_landmarks=synth_palm_landmarks(
        (0.0, 0.0, -1.0), (1.0, 0.0, 0.0)))
    report["degradation_failed"] = {
        "mode": out.mode,
        "attempts": [(m, a.ok) for m, a in out.attempts],
        "consecutive_failures": client.consecutive_failures,
    }
    if out.mode != MODE_FAILED or client.consecutive_failures != before + 1:
        failures.append("DEGRADE: out-of-reach pose did not yield MODE_FAILED "
                        "+ failure-counter increment")

    # D3: orientation-INVALID frame (degenerate landmarks) degrades to
    # fixed_down without touching the oriented stage.
    flat = [(0.0, 0.0, 0.0)] * 21    # all-zero palm -> gates reject
    out = client.solve(grasp, world_landmarks=flat)
    report["degradation_invalid_frame"] = {
        "mode": out.mode, "orientation_valid": out.orientation_valid,
        "reason": out.invalid_reason}
    if out.orientation_valid or out.mode != MODE_FIXED_DOWN:
        failures.append("DEGRADE: invalid-orientation frame did not degrade "
                        "to fixed_down")

    # ---- E. FK round-trip (gap 12) ---------------------------------------
    fk_errs = []
    for u in (0.1, 0.35, 0.6, 0.85):
        pos = path(u)
        out = client.solve(pos, quat=DOWN_QUAT)
        if out.mode == MODE_FAILED:
            continue
        fk = client.fk(out.joints)
        if fk is None or IK_LINK not in fk:
            failures.append(f"FK: /compute_fk failed at u={u}")
            continue
        (fx, fy, fz), _ = fk[IK_LINK]
        fk_errs.append(math.dist((fx, fy, fz), pos))
    report["fk_roundtrip_err_mm"] = [round(e * 1e3, 3) for e in fk_errs]
    if not fk_errs or max(fk_errs) > 1e-3:
        failures.append(f"FK: round-trip error {fk_errs} exceeds 1 mm")

    # ---- report -----------------------------------------------------------
    report["client_stats"] = client.stats
    print(json.dumps(report, indent=2))
    if failures:
        print("\nRESULT: FAIL")
        for f in failures:
            print("  -", f)
    else:
        print("\nRESULT: PASS — box tuned & validated, sweep + degradation + "
              "FK all green")
    node.destroy_node()
    rclpy.shutdown()
    return 1 if failures else 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--steps", type=int, default=200,
                    help="oriented sweep steps (default 200)")
    args = ap.parse_args(argv)
    sys.exit(run(args.steps))


if __name__ == "__main__":
    main()
