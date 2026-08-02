"""Deterministic CPU-time bench of the mirror hot path (retarget + dedup),
driven by diff_replay's own recorded session at the real 30 Hz obs /
50 Hz control decoupling. Measures process CPU time (time.process_time),
not wall clock, so host contention does not distort it.

Runs whatever build is installed in /ros2_ws/install (the container rebuilds
talos_mirror unconditionally), so BEFORE/AFTER are selected by WS=.
"""
import sys, time, threading, json

sys.path.insert(0, "/ros2_ws/tools")
import diff_replay as DR
from talos_mirror.retarget import MirrorPoseSource, retarget
from talos_mirror import retarget as RT
from talos_mirror.talos_config import GROUP_JOINTS, home_joint_state
HOME = home_joint_state()


class FakeNode:
    def __init__(self):
        self._lock = threading.Lock()
        self.observation = None
        self.mirror_mode = True
        self.margin = 0.05
        self.head_gain_yaw = 0.8
        self.head_gain_pitch = 0.6
        self.torso_gain_yaw = 0.5
        self.torso_gain_pitch = 0.5
        self.retarget_info = {}
        self.q_cmd = dict(HOME)


def run(reps):
    session = DR.record_session()          # 30 Hz observation dicts
    n_obs = len(session)
    rate_hz = 50.0
    obs_hz = DR.RATE_HZ
    node = FakeNode()
    src = MirrorPoseSource(node)
    n_ticks = int(DR.DURATION_S * rate_hz) + 1
    solves = {"n": 0}
    orig = RT.retarget

    t0 = time.process_time()
    for _ in range(reps):
        for k in range(n_ticks):
            t = k / rate_hz
            idx = min(int(t * obs_hz), n_obs - 1)
            with node._lock:
                node.observation = session[idx]
            targets = src.read(t)
            # imitate _tick's q_cmd walk so `prev` drifts exactly as live
            if targets:
                for j, v in targets.items():
                    if j in node.q_cmd:
                        prev = node.q_cmd[j]
                        d = float(v) - prev
                        if d > 0.03:
                            d = 0.03
                        elif d < -0.03:
                            d = -0.03
                        node.q_cmd[j] = prev + d
    t1 = time.process_time()
    total = reps * n_ticks
    print(json.dumps({
        "reps": reps, "ticks_total": total,
        "cpu_s": round(t1 - t0, 4),
        "cpu_us_per_tick": round(1e6 * (t1 - t0) / total, 2),
        "obs_rate_hz": obs_hz, "control_rate_hz": rate_hz,
        "duration_s": DR.DURATION_S,
        "halvings_24": open(RT.__file__).read().count("for _ in range(24)"),
        "halvings_12": open(RT.__file__).read().count("for _ in range(12)"),
        "has_dedup": "_cache_obs" in open(RT.__file__).read(),
    }))


if __name__ == "__main__":
    run(int(sys.argv[1]) if len(sys.argv) > 1 else 5)
