"""U4 acceptance meter — run inside ros2-arm:jazzy while hand_follow streams.
Records /joint_states + /arm_controller/joint_trajectory for DURATION s and
prints: per-joint amplitude, max consecutive-sample position delta (smoothness)
and both message rates.  Also emits a final machine-readable RESULT:{json} line
(fields mirror wallweld_accept: pass, rates, max deltas) for CI parsing.
Usage: python3 hand_accept.py [duration_s]"""
import json
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
JOINTS = [f"joint{i}" for i in range(1, 7)]


class Meter(Node):
    def __init__(self):
        super().__init__("hand_accept_meter")
        self.js = []          # (t, [pos joint1..6])
        self.traj_t = []
        self.create_subscription(JointState, "/joint_states", self.on_js, 100)
        self.create_subscription(JointTrajectory,
                                 "/arm_controller/joint_trajectory",
                                 self.on_traj, 100)

    def on_js(self, m):
        idx = {n: i for i, n in enumerate(m.name)}
        if all(j in idx for j in JOINTS):
            self.js.append((time.monotonic(),
                            [m.position[idx[j]] for j in JOINTS]))

    def on_traj(self, m):
        self.traj_t.append(time.monotonic())


def main():
    rclpy.init()
    n = Meter()
    t_end = time.monotonic() + DUR
    while time.monotonic() < t_end:
        rclpy.spin_once(n, timeout_sec=0.1)
    js, tt = n.js, n.traj_t
    print(f"samples: joint_states={len(js)}  traj_msgs={len(tt)}  dur={DUR}s")
    amps = [0.0] * 6
    max_d, max_dj = 0.0, -1
    js_rate = 0.0
    traj_rate = 0.0
    traj_gap_ms = None
    enough = len(js) >= 10
    if not enough:
        print("FAIL: too few joint_states samples")
    else:
        for j in range(6):
            vals = [p[1][j] for p in js]
            amps[j] = max(vals) - min(vals)
            print(f"  joint{j+1}: min={min(vals):+.3f} max={max(vals):+.3f} "
                  f"amplitude={amps[j]:.3f} rad")
        for a, b in zip(js, js[1:]):
            for j in range(6):
                d = abs(b[1][j] - a[1][j])
                if d > max_d:
                    max_d, max_dj = d, j
        span_js = js[-1][0] - js[0][0]
        js_rate = (len(js) - 1) / span_js if span_js > 0 else 0.0
        print(f"max consecutive /joint_states delta: {max_d:.4f} rad "
              f"(joint{max_dj+1})  [accept: <= 0.15]")
        print(f"/joint_states rate: {js_rate:.1f} Hz")
    if len(tt) > 1:
        span = tt[-1] - tt[0]
        gaps = [b - a for a, b in zip(tt, tt[1:])]
        traj_rate = (len(tt) - 1) / span if span > 0 else 0.0
        traj_gap_ms = max(gaps) * 1e3
        print(f"traj cmd rate: {traj_rate:.2f} Hz  "
              f"(max gap {traj_gap_ms:.0f} ms)  [accept: >= 15 Hz]")
    else:
        print("traj cmd rate: NO MESSAGES  [accept: >= 15 Hz] FAIL")
    amp1 = amps[0]
    ok = enough and amp1 > 0.2 and max_d <= 0.15 and traj_rate >= 15.0
    print("ACCEPT(a):", "PASS" if ok else "check-individual-lines")
    out = {
        "mode": "handfollow",
        "duration_s": DUR,
        "n_js": len(js),
        "n_traj": len(tt),
        "js_rate_hz": round(js_rate, 2),
        "traj_rate_hz": round(traj_rate, 2),
        "traj_max_gap_ms": round(traj_gap_ms, 1)
        if traj_gap_ms is not None else None,
        "amplitudes_rad": [round(a, 4) for a in amps],
        "amp_joint1_rad": round(amp1, 4),
        "max_js_step_rad": round(max_d, 4),
        "max_step_joint": (max_dj + 1) if max_dj >= 0 else None,
        "pass": bool(ok),
    }
    print("RESULT:" + json.dumps(out), flush=True)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
