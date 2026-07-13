#!/usr/bin/env python3
"""verify_arm_pybullet.py — load the arm URDF and confirm it's a working robot.

Loads two_link_arm.urdf into PyBullet (headless), prints its joints, then drives
the shoulder + elbow and checks the end-effector (tip of link2) actually moves.
This closes the loop: FreeCAD CAD -> URDF -> simulated, articulated robot.

    .venv/bin/python cad/verify_arm_pybullet.py            # headless
    .venv/bin/python cad/verify_arm_pybullet.py --gui      # watch it move
"""
import argparse
import os

import pybullet as p

HERE = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.join(HERE, "..", "assets", "urdf", "two_link_arm", "two_link_arm.urdf")


def main(gui):
    p.connect(p.GUI if gui else p.DIRECT)
    p.setGravity(0, 0, -9.81)
    arm = p.loadURDF(URDF, basePosition=[0, 0, 0], useFixedBase=True)

    n = p.getNumJoints(arm)
    revolute = []
    print(f"loaded {os.path.basename(URDF)}: {n} joints")
    for j in range(n):
        info = p.getJointInfo(arm, j)
        jtype = {p.JOINT_REVOLUTE: "revolute", p.JOINT_FIXED: "fixed"}.get(info[2], info[2])
        print(f"  joint {j}: {info[1].decode():10s} type={jtype}")
        if info[2] == p.JOINT_REVOLUTE:
            revolute.append(j)

    tip = n - 1  # last link = link2
    start = p.getLinkState(arm, tip)[0]
    # command both joints to bend the arm
    for j in revolute:
        p.setJointMotorControl2(arm, j, p.POSITION_CONTROL, targetPosition=0.8, force=20)
    for _ in range(480):
        p.stepSimulation()
    end = p.getLinkState(arm, tip)[0]

    moved = sum((a - b) ** 2 for a, b in zip(start, end)) ** 0.5
    print(f"\nrevolute joints: {len(revolute)} (expected 2)")
    print(f"end-effector moved {moved*1000:.0f} mm after bending the joints")
    ok = len(revolute) == 2 and moved > 0.02
    print("RESULT:", "PASS — CAD->URDF->sim arm is articulated and moves" if ok else "FAIL")
    p.disconnect()
    return ok


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--gui", action="store_true")
    raise SystemExit(0 if main(ap.parse_args().gui) else 1)
