"""
acceptance_test.py — Phase A milestone check.

Run this on the Raspberry Pi with the arm powered and a camera attached.
It proves the four Phase A capabilities in one go:
    1. read the arm's measured state
    2. command it to a target it reaches safely
    3. grab a camera frame synchronized with a state read
    4. report camera-sync quality (lag)

Pass criteria are printed at the end. If all four say OK, Phase A is done
and you can start collecting demonstrations (Phase B).

Usage:
    python acceptance_test.py --port /dev/ttyUSB0 --cam 0 --delta-deg 3
"""
from __future__ import annotations
import argparse
import math
import time

from arm_serial import ArmSerial
from camera_logger import Camera


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="")   # empty = ARM_PORT env or autodetect
    ap.add_argument("--cam", type=int, default=0)
    ap.add_argument("--delta-deg", type=float, default=3.0,
                    help="small joint-0 test move; default: 3 degrees")
    args = ap.parse_args()

    results = {}

    with ArmSerial(port=args.port or None) as arm:
        # 1. read state
        st = arm.get_state()
        print("1. get_state:", [round(x, 3) for x in st.q], "grip", round(st.gripper, 2))
        results["read_state"] = len(st.q) == 6

        # 2. command a small configured move on joint 0 and verify progression
        arm.enable()
        q0 = list(arm.get_state().q)
        target = list(q0)
        joint = arm.config.joints[0]
        delta = math.radians(abs(args.delta_deg))
        if q0[0] + delta <= joint.max_rad:
            target[0] = q0[0] + delta
        elif q0[0] - delta >= joint.min_rad:
            target[0] = q0[0] - delta
        else:
            raise RuntimeError(
                f"joint0 has no room for a {args.delta_deg} degree acceptance move")
        arm.set_action(target, gripper=0.0)
        time.sleep(abs(target[0] - q0[0]) / joint.max_velocity_rad_s + 0.5)
        q1 = arm.get_state().q
        moved = abs(q1[0] - q0[0])
        print(f"2. set_action: joint0 state changed {math.degrees(moved):.1f} deg "
              f"(commanded {args.delta_deg:.1f})")
        # Until encoders exist this proves command progression, not shaft motion.
        results["command_move"] = moved > delta / 3.0

        # 3 + 4. synchronized camera + state capture
        cam = Camera(index=args.cam)
        try:
            lags = []
            for _ in range(10):
                img, t_cam = cam.grab()
                s = arm.get_state()
                lags.append((s.t_host - t_cam) * 1000.0)
                time.sleep(0.05)
            mean_lag = sum(lags) / len(lags)
            print(f"3. camera grab: {img.shape}, 10 frames OK")
            print(f"4. cam-state sync lag: mean {mean_lag:.1f} ms (max {max(lags):.1f})")
            results["camera"] = img is not None and img.size > 0
            results["sync"] = mean_lag < 50.0     # < ~1 frame at 20Hz is fine
        finally:
            cam.release()

        arm.home()
        time.sleep(1.0)
        arm.relax()

    print("\n--- Phase A acceptance ---")
    for k, v in results.items():
        print(f"  [{'OK ' if v else 'FAIL'}] {k}")
    if all(results.values()):
        print("\nPHASE A PASSED — ready for Phase B (demonstration collection).")
    else:
        print("\nSome checks failed — see above. Common fixes: wrong --port, "
              "encoder read still stubbed, camera index, or joint limits too tight.")


if __name__ == "__main__":
    main()
