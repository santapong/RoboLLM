#!/usr/bin/env python3
"""c2-qp (M5) A/B replay harness.

    qp_ab_harness.py record  SESSION.jsonl [--duration 15] [--rate 30]
    qp_ab_harness.py replay  SESSION.jsonl --mode direct|qp [--control-hz 50]
    qp_ab_harness.py ab      SESSION.jsonl [--duration 15] [--rate 30] [--control-hz 50]

`record` drives talos_mirror.pose_source.SyntheticBodyPoseSource (the SAME
deterministic whole-body generator `ros2-arm track synthetic:=true` uses)
for `--duration` seconds at `--rate` Hz and writes one JSON line per sample
-- the exact wire-format dict track_node publishes on /body/observation
(talos_mirror.body_track.observation_dict). Pure function of t, so the file
is byte-identical across runs: this IS "one deterministic synthetic
body-observation session", recorded once, replayed as many times as needed.

`replay` drives ONE retarget mode across the whole recorded session at a
fixed `--control-hz` (default 50, matching mirror_node's real rate), with
HOLD-LATEST semantics for which recorded sample is "current" -- exactly
mirror_node's own rate-decoupling: the control tick always re-solves
whatever /body/observation is newest, never blocking for a fresh one (see
mirror_node.py's module docstring, "RATE DECOUPLING"). It runs the SAME
talos_mirror.retarget.retarget() and talos_mirror.qp_retarget.WristQPSolver
code mirror_node.py itself calls -- not a re-implementation -- so what this
measures is the actual retarget layer's behaviour, not a model of it.

`ab` does both replays back to back and prints ONE comparison table so a
run always reports on the SAME session for both modes.

METRICS, and how each is kept from lying to itself:
  - wrist Cartesian tracking error (mean/max, position + orientation): the
    TARGET is computed by pin_model.TalosPinModel.build_wrist_target, a
    pure geometry function of (this tick's M4 q, this tick's mirrored
    forearm direction) that is NOT part of WristQPSolver's own per-tick
    warm-start/QP-limits machinery under test -- reusing it here is reusing
    a shared, already-independent DEFINITION of the target, not reusing the
    solver's live state. direct mode's ACHIEVED pose trivially equals that
    target in POSITION (M4's own FK, by construction) but not orientation
    (wrist parked at 0) -- so direct mode's position error should read ~0
    and its orientation error should read the actual angular gap the QP
    layer exists to close; qp mode's errors show what the QP actually
    achieves against limits/warm-start/dt, both numbers from the SAME
    ground-truth definition.
  - mean/max jerk: 3rd difference of joint positions BY TICK INDEX (never
    from wall-clock arrival timestamps -- house law), scaled by the KNOWN
    constant control-tick dt (1/control_hz), not measured timing.
  - joint-limit margin violations: joint_limits.excursion() against
    joint_limits.MEASURED (position limits match docs/joints.json; velocity
    additionally carried, joints.json has no velocity field -- see that
    module's header) at the SAME margin (0.05 rad) mirror_node's own
    clamp() uses, applied to the RAW retarget/QP output BEFORE any
    downstream clamp -- this measures how often the safety clamp would have
    had to intervene, which is identical machinery in both modes and would
    not itself produce an A/B difference if measured post-clamp.
  - solve time per tick (mean/max/p95): wall-clock around retarget() alone
    (direct) or retarget()+WristQPSolver.solve() (qp), using time.perf_counter
    -- a genuine walltime measurement (not a proxy), reported so a reader can
    check it against the 50 Hz (20 ms) budget this whole example targets.

Needs /opt/mpvenv (pinocchio for the metric FK in BOTH modes, pink for qp
mode) -- `ros2-arm ab-bench` runs this under /opt/mpvenv/bin/python.
"""

import argparse
import json
import math
import statistics
import sys
import time

import numpy as np

_ROOT = __file__.rsplit("/tools/", 1)[0]
sys.path.insert(0, _ROOT + "/src/talos_mirror")

from talos_mirror.joint_limits import MEASURED, excursion  # noqa: E402
from talos_mirror.pin_model import TalosPinModel, rotation_angle  # noqa: E402
from talos_mirror.pose_source import SyntheticBodyPoseSource  # noqa: E402
from talos_mirror.body_track import observation_dict  # noqa: E402
from talos_mirror.retarget import retarget  # noqa: E402
from talos_mirror.talos_config import ALL_JOINTS, home_joint_state  # noqa: E402


# ----------------------------------------------------------------- record
def record(path, duration_s=15.0, rate_hz=30.0, period_s=16.0):
    src = SyntheticBodyPoseSource(period_s=period_s)
    src.start()
    n = int(round(duration_s * rate_hz)) + 1
    with open(path, "w") as f:
        for i in range(n):
            t = i / rate_hz
            obs = src.read(t)
            f.write(json.dumps(observation_dict(obs)) + "\n")
    src.stop()
    return n


def load_session(path):
    with open(path) as f:
        return [json.loads(line) for line in f if line.strip()]


# ----------------------------------------------------------------- replay
def replay(
    session,
    mode,
    control_hz=50.0,
    urdf_xml=None,
    position_cost=1.0,
    orientation_cost=0.5,
    posture_cost=1e-2,
    landmark_min_cutoff=1.0,
    landmark_beta=0.5,
    mirror=True,
    margin=0.05,
    head_gain_yaw=0.8,
    head_gain_pitch=0.6,
    torso_gain_yaw=0.5,
    torso_gain_pitch=0.5,
):
    """One full offline pass of `session` through `mode`. Returns a list of
    per-tick dicts: {"t", "q" (all 30 joints), "solve_time_s",
    "pos_err_m"/"rot_err_deg" per gated side (missing if that side was not
    gated this tick)}.
    """
    assert mode in ("direct", "qp")
    pm = TalosPinModel(urdf_xml)

    solver = None
    lfilter = None
    if mode == "qp":
        from talos_mirror.qp_retarget import WristQPSolver
        from talos_mirror.qp_pose_source import LandmarkFilter, _ARM_LANDMARK_IDS

        solver = WristQPSolver(
            urdf_xml=urdf_xml, position_cost=position_cost,
            orientation_cost=orientation_cost, posture_cost=posture_cost,
        )
        lfilter = LandmarkFilter(_ARM_LANDMARK_IDS, min_cutoff=landmark_min_cutoff, beta=landmark_beta)

    total_t = session[-1]["t"]
    dt = 1.0 / control_hz
    n_ticks = int(total_t * control_hz) + 1

    q_cmd = dict(home_joint_state())
    records = []
    j = 0
    for k in range(n_ticks):
        t = k * dt
        while j + 1 < len(session) and session[j + 1]["t"] <= t:
            j += 1
        obs = session[j]

        t0 = time.perf_counter()
        if mode == "direct":
            feed = obs
        else:
            feed = dict(obs)
            feed["landmarks"] = lfilter(obs["landmarks"], t)

        m4_targets, info = retarget(
            feed, mirror=mirror, prev=q_cmd, margin=margin,
            head_gain_yaw=head_gain_yaw, head_gain_pitch=head_gain_pitch,
            torso_gain_yaw=torso_gain_yaw, torso_gain_pitch=torso_gain_pitch,
        )

        if not m4_targets:
            solve_time = time.perf_counter() - t0
            records.append({"t": t, "q": dict(q_cmd), "solve_time_s": solve_time})
            continue

        if mode == "direct":
            out = m4_targets
        else:
            out, _qp_info = solver.solve(m4_targets, info["forearm_dir"], dt)
        solve_time = time.perf_counter() - t0

        rec = {"t": t, "q": dict(out), "solve_time_s": solve_time}

        # Independent wrist Cartesian metric -- see module docstring.
        # build_wrist_target() runs its OWN fk(m4_q) internally and returns
        # a standalone SE3 (translation/rotation both copied out of
        # pm.data), so it is safe to call for both sides BEFORE the single
        # fk(q_out) pass below overwrites pm.data for the ACHIEVED reads --
        # doing it in the other order silently reads m4_q's own FK back as
        # "achieved" (both target and achieved pos = FK(m4_q), forcing
        # pos_err to a fixed 0.0 in every mode and rot_err to a solver-
        # independent constant), a bug caught by exactly this book-keeping.
        m4_q = pm.q_from_dict(m4_targets)
        q_out = pm.q_from_dict(out)
        targets_by_side = {}
        for side in ("left", "right"):
            fdir = info["forearm_dir"].get(side)
            if fdir is None:
                continue
            targets_by_side[side] = pm.build_wrist_target(m4_q, fdir, side)
        pm.fk(q_out)
        for side, target in targets_by_side.items():
            achieved = pm.data.oMf[pm.wrist_frame_id[side]]
            rec[f"pos_err_{side}_m"] = float(
                np.linalg.norm(achieved.translation - target.translation)
            )
            rec[f"rot_err_{side}_deg"] = math.degrees(
                rotation_angle(achieved.rotation.T @ target.rotation)
            )

        records.append(rec)
        q_cmd = dict(out)

    return records


# ----------------------------------------------------------------- metrics
def _stats(values):
    if not values:
        return {"mean": None, "max": None, "p95": None, "n": 0}
    s = sorted(values)
    p95 = s[min(len(s) - 1, int(math.ceil(0.95 * len(s))) - 1)]
    return {
        "mean": statistics.fmean(values), "max": max(values), "p95": p95, "n": len(values),
    }


def compute_jerk(records, control_hz):
    """3rd difference of each joint's position sequence, BY TICK INDEX
    (never wall-clock arrival time -- house law), scaled by the KNOWN
    constant control-tick dt so the result is in rad/s^3."""
    dt = 1.0 / control_hz
    all_abs_jerk = []
    per_joint_max = {}
    for name in ALL_JOINTS:
        q = [r["q"][name] for r in records if name in r["q"]]
        if len(q) < 4:
            continue
        d1 = [q[i] - q[i - 1] for i in range(1, len(q))]
        d2 = [d1[i] - d1[i - 1] for i in range(1, len(d1))]
        d3 = [d2[i] - d2[i - 1] for i in range(1, len(d2))]
        jerk = [abs(v) / (dt ** 3) for v in d3]
        all_abs_jerk.extend(jerk)
        per_joint_max[name] = max(jerk) if jerk else 0.0
    if not all_abs_jerk:
        return {"mean_abs_rad_s3": None, "max_abs_rad_s3": None, "max_joint": None}
    worst = max(per_joint_max, key=per_joint_max.get)
    return {
        "mean_abs_rad_s3": statistics.fmean(all_abs_jerk),
        "max_abs_rad_s3": max(all_abs_jerk),
        "max_joint": worst,
        "max_joint_value": per_joint_max[worst],
    }


def compute_limit_violations(records, margin=0.05):
    violated_ticks = 0
    max_excursion = 0.0
    per_joint_hits = {}
    for r in records:
        hit = False
        for name, value in r["q"].items():
            if name not in MEASURED:
                continue
            exc = excursion(value, name, MEASURED, margin)
            if exc > 0.0:
                hit = True
                max_excursion = max(max_excursion, exc)
                per_joint_hits[name] = per_joint_hits.get(name, 0) + 1
        if hit:
            violated_ticks += 1
    return {
        "violated_ticks": violated_ticks,
        "total_ticks": len(records),
        "max_excursion_rad": max_excursion,
        "per_joint_hit_count": per_joint_hits,
    }


def summarize(records, control_hz, margin=0.05):
    solve_times = [r["solve_time_s"] for r in records]
    out = {
        "n_ticks": len(records),
        "control_hz": control_hz,
        "solve_time_s": _stats(solve_times),
        "jerk": compute_jerk(records, control_hz),
        "limit_violations": compute_limit_violations(records, margin),
        "wrist_err": {},
    }
    for side in ("left", "right"):
        pos = [r[f"pos_err_{side}_m"] for r in records if f"pos_err_{side}_m" in r]
        rot = [r[f"rot_err_{side}_deg"] for r in records if f"rot_err_{side}_deg" in r]
        out["wrist_err"][side] = {"pos_m": _stats(pos), "rot_deg": _stats(rot)}
    return out


# ----------------------------------------------------------------- CLI
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p_rec = sub.add_parser("record")
    p_rec.add_argument("path")
    p_rec.add_argument("--duration", type=float, default=15.0)
    p_rec.add_argument("--rate", type=float, default=30.0)
    p_rec.add_argument("--period", type=float, default=16.0)

    p_rep = sub.add_parser("replay")
    p_rep.add_argument("path")
    p_rep.add_argument("--mode", choices=("direct", "qp"), required=True)
    p_rep.add_argument("--control-hz", type=float, default=50.0)
    p_rep.add_argument("--out", default=None)

    p_ab = sub.add_parser("ab")
    p_ab.add_argument("path")
    p_ab.add_argument("--duration", type=float, default=15.0)
    p_ab.add_argument("--rate", type=float, default=30.0)
    p_ab.add_argument("--control-hz", type=float, default=50.0)
    p_ab.add_argument("--skip-record", action="store_true", help="reuse an existing session file")

    args = ap.parse_args(argv)

    if args.cmd == "record":
        n = record(args.path, duration_s=args.duration, rate_hz=args.rate, period_s=args.period)
        print(json.dumps({"recorded_samples": n, "path": args.path}))
        return 0

    if args.cmd == "replay":
        session = load_session(args.path)
        records = replay(session, args.mode, control_hz=args.control_hz)
        summary = summarize(records, args.control_hz)
        print(json.dumps({"mode": args.mode, "summary": summary}, indent=2))
        if args.out:
            with open(args.out, "w") as f:
                json.dump({"mode": args.mode, "records": records, "summary": summary}, f)
        return 0

    if args.cmd == "ab":
        if not args.skip_record:
            n = record(args.path, duration_s=args.duration, rate_hz=args.rate)
            print(f"# recorded {n} samples -> {args.path}", file=sys.stderr)
        session = load_session(args.path)

        results = {}
        for mode in ("direct", "qp"):
            t0 = time.perf_counter()
            records = replay(session, mode, control_hz=args.control_hz)
            wall = time.perf_counter() - t0
            summary = summarize(records, args.control_hz)
            summary["wall_replay_s"] = wall
            results[mode] = summary
            print(f"# replayed mode={mode} in {wall:.2f}s wall", file=sys.stderr)

        print(json.dumps({"session": args.path, "results": results}, indent=2))
        return 0

    return 1


if __name__ == "__main__":
    raise SystemExit(main())
