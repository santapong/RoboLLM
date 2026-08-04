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
    python acceptance_test.py --port /dev/ttyUSB0 --cam 0
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
    args = ap.parse_args()

    results = {}

    with ArmSerial(port=args.port or None) as arm:
        # 1. read state
        st = arm.get_state()
        print("1. get_state:", [round(x, 3) for x in st.q], "grip", round(st.gripper, 2))
        results["read_state"] = len(st.q) == 6

        # 2. command a small, safe move on joint 0 and verify it moves toward target
        arm.enable()
        q0 = list(arm.get_state().q)
        target = list(q0)
        target[0] = q0[0] + math.radians(15)      # nudge joint 0 by 15 deg
        arm.set_action(target, gripper=0.0)
        time.sleep(1.5)                            # let it get there
        q1 = arm.get_state().q
        moved = abs(q1[0] - q0[0])
        print(f"2. set_action: joint0 moved {math.degrees(moved):.1f} deg (commanded 15)")
        # accept if it moved at least a third of the way (servos + encoder noise)
        results["command_move"] = moved > math.radians(5)

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
