#!/usr/bin/env python3
"""wallweld_accept.py — U1 acceptance monitor for wall_weld.py.

Run INSIDE a ros2-arm container (plain python3, no mediapipe needed) while a
launcher-started wallweld stack is up:

  python3 /ros2_ws/tools/wallweld_accept.py full     synthetic full weld (a)
  python3 /ros2_ws/tools/wallweld_accept.py abort    synthetic palm abort (b)
  python3 /ros2_ws/tools/wallweld_accept.py idle 35  camera no-hand idle (c)

Subscribes /joint_states /wall_weld_markers /weld_progress /weld_state
/weld_event, and (full mode) spot-checks recorded states along the executed
path against /check_state_validity WITH the wall in the scene. Prints a JSON
verdict on stdout (single line prefixed RESULT:).
"""
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String
from visualization_msgs.msg import Marker, MarkerArray
from moveit_msgs.srv import GetStateValidity

sys.path.insert(0, "/ros2_ws/tools")
import arm_ik

JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


class Monitor(Node):
    def __init__(self):
        super().__init__("wallweld_accept")
        self.js = []          # (t, [q1..q6])
        self.bead = []        # (t, count) on change
        self.states = []      # (t, state) on change
        self.events = []      # (t, text)
        self.progress = 0.0
        self.create_subscription(JointState, "/joint_states", self._js, 50)
        self.create_subscription(MarkerArray, "/wall_weld_markers",
                                 self._mk, 10)
        self.create_subscription(String, "/weld_state", self._st, 10)
        self.create_subscription(String, "/weld_event", self._ev, 10)
        self.create_subscription(Float32, "/weld_progress", self._pr, 10)
        self.validity = self.create_client(GetStateValidity,
                                           "/check_state_validity")

    def _js(self, m):
        idx = {n: i for i, n in enumerate(m.name)}
        if all(j in idx for j in JOINTS):
            self.js.append((time.monotonic(),
                            [m.position[idx[j]] for j in JOINTS]))

    def _mk(self, m):
        for mk in m.markers:
            if mk.ns == "bead":
                n = len(mk.points) if mk.action == Marker.ADD else 0
                if not self.bead or self.bead[-1][1] != n:
                    self.bead.append((time.monotonic(), n))

    def _st(self, m):
        if not self.states or self.states[-1][1] != m.data:
            self.states.append((time.monotonic(), m.data))
        else:                  # heartbeat: refresh last-seen time
            self.last_hb = time.monotonic()

    def _ev(self, m):
        self.events.append((time.monotonic(), m.data))
        print("  event: {}".format(m.data), flush=True)

    def _pr(self, m):
        self.progress = m.data

    # ---------------------------------------------------------------- utils
    def wait_for(self, pred, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if pred():
                return True
        return False

    def ev_time(self, prefix):
        for t, e in self.events:
            if e.startswith(prefix):
                return t, e
        return None, None

    def max_joint_step(self, t0=None, t1=None):
        worst = 0.0
        prev = None
        for t, q in self.js:
            if t0 is not None and t < t0:
                continue
            if t1 is not None and t > t1:
                break
            if prev is not None:
                worst = max(worst, max(abs(a - b) for a, b in zip(q, prev)))
            prev = q
        return worst

    def check_validity(self, samples):
        if not self.validity.wait_for_service(timeout_sec=10.0):
            return None, "service unavailable"
        ok = 0
        for q in samples:
            req = GetStateValidity.Request()
            req.group_name = "arm"
            req.robot_state.joint_state.name = list(JOINTS)
            req.robot_state.joint_state.position = [float(v) for v in q]
            fut = self.validity.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
            res = fut.result()
            if res is not None and res.valid:
                ok += 1
        return ok, None


def run_full(mon):
    print("waiting for weld DONE (timeout 280 s)...", flush=True)
    got = mon.wait_for(
        lambda: mon.ev_time("DONE")[0] is not None
        and mon.states and mon.states[-1][1] == "IDLE"
        and mon.states[-1][0] > mon.ev_time("DONE")[0], 280)
    t_start, _ = mon.ev_time("START")
    t_done, done_ev = mon.ev_time("DONE")
    _, plan_ev = mon.ev_time("PLAN")
    planned = None
    if plan_ev:
        planned = int(plan_ev.split("waypoints=")[1].split()[0])
    bead_final = max((n for _, n in mon.bead), default=0)
    step = mon.max_joint_step()
    # sample recorded states across START..(state back IDLE) for validity.
    # If the monitor attached after START was published (missed event), fall
    # back to the first recorded joint state so validity still gets sampled.
    if t_start is None and mon.js:
        t_start = mon.js[0][0]
    t_end = mon.states[-1][0] if mon.states else None
    span = [q for t, q in mon.js
            if t_start is not None and t_start <= t <= (t_end or t)]
    k = max(len(span) // 100, 1)
    samples = span[::k][:120]
    n_ok, verr = mon.check_validity(samples)
    out = {
        "mode": "full", "completed": bool(got),
        "planned_waypoints": planned,
        "final_progress": round(mon.progress, 4),
        "bead_final_pts": bead_final,
        "bead_growth_samples": len(mon.bead),
        "max_joint_step_rad": round(step, 4),
        "validity_sampled": len(samples),
        "validity_ok": n_ok, "validity_err": verr,
        "validity_pct": round(100.0 * n_ok / len(samples), 1)
        if samples and n_ok is not None else None,
        "done_event": done_ev,
        "n_js": len(mon.js),
        "pass": bool(got) and planned is not None
        and bead_final == planned and mon.progress >= 0.999
        and step <= 0.15 and n_ok is not None
        and n_ok >= 0.95 * len(samples),
    }
    print("RESULT:" + json.dumps(out), flush=True)


def run_abort(mon):
    print("waiting for palm-abort + re-home (timeout 220 s)...", flush=True)
    got = mon.wait_for(
        lambda: mon.ev_time("ABORT")[0] is not None
        and mon.states and mon.states[-1][1] == "IDLE"
        and mon.states[-1][0] > mon.ev_time("ABORT")[0], 220)
    t_inj, _ = mon.ev_time("SYNTH_PALM_INJECT")
    t_abort, abort_ev = mon.ev_time("ABORT")
    bead_after = [(t, n) for t, n in mon.bead
                  if t_inj is not None and t > t_inj]
    t_last_growth = bead_after[-1][0] if bead_after else t_inj
    stop_latency = (t_last_growth - t_inj) if t_inj is not None else None
    bead_final = mon.bead[-1][1] if mon.bead else 0
    bead_max = max((n for _, n in mon.bead), default=0)
    # slewed return: joint steps stay clamped through the whole run
    step = mon.max_joint_step()
    q_final = mon.js[-1][1] if mon.js else None
    home_err = None
    if q_final:
        home_err = math.dist(arm_ik.fk_tool0(q_final), (0.20, 0.0, 0.30))
    seq = [s for _, s in mon.states]
    out = {
        "mode": "abort", "aborted_and_homed": bool(got),
        "abort_event": abort_ev,
        "stop_latency_s": round(stop_latency, 3)
        if stop_latency is not None else None,
        "bead_final_pts": bead_final, "bead_max_pts": bead_max,
        "bead_persists": bead_final == bead_max and bead_final > 0,
        "max_joint_step_rad": round(step, 4),
        "home_err_m": round(home_err, 4) if home_err is not None else None,
        "state_seq": seq,
        "pass": bool(got) and stop_latency is not None
        and stop_latency <= 0.5 and bead_final == bead_max > 0
        and step <= 0.15 and home_err is not None and home_err < 0.02
        and "ABORT_FREEZE" in seq and "HOMING" in seq,
    }
    print("RESULT:" + json.dumps(out), flush=True)


def run_idle(mon, dur):
    print("watching idle hold for {:.0f} s...".format(dur), flush=True)
    t0 = time.monotonic()
    mon.last_hb = t0
    mon.wait_for(lambda: False, dur)          # just collect
    t_end = time.monotonic()
    # drift over the last 20 s (arm must simply hold home)
    drift = mon.max_joint_step(t0=t_end - 20.0)
    span = [q for t, q in mon.js if t >= t_end - 20.0]
    rng = 0.0
    if span:
        for j in range(6):
            vals = [q[j] for q in span]
            rng = max(rng, max(vals) - min(vals))
    states = set(s for _, s in mon.states)
    bad_ev = [e for _, e in mon.events
              if e.startswith(("START", "ABORT", "DONE"))]
    hb_age = t_end - max(mon.last_hb,
                         mon.states[-1][0] if mon.states else 0)
    out = {
        "mode": "idle", "duration_s": round(t_end - t0, 1),
        "n_js": len(mon.js),
        "joint_range_last20s_rad": round(rng, 5),
        "max_step_last20s_rad": round(drift, 5),
        "states_seen": sorted(states),
        "weld_events": bad_ev,
        "heartbeat_age_s": round(hb_age, 1),
        "pass": len(mon.js) > 100 and rng < 0.01 and not bad_ev
        and "IDLE" in states and hb_age < 3.0,
    }
    print("RESULT:" + json.dumps(out), flush=True)


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "full"
    rclpy.init()
    mon = Monitor()
    try:
        if mode == "full":
            run_full(mon)
        elif mode == "abort":
            run_abort(mon)
        elif mode == "idle":
            run_idle(mon, float(sys.argv[2]) if len(sys.argv) > 2 else 35.0)
        else:
            print("unknown mode " + mode)
            sys.exit(2)
    finally:
        mon.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
