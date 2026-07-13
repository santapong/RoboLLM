#!/usr/bin/env python3
"""Inverse kinematics on YOUR OWN CAD arm — reach (x, z) targets with math.

Forward kinematics (FK): joint angles → where is the tip?
Inverse kinematics (IK): where I want the tip → what joint angles?
This is the first real manipulation math every arm roboticist learns, done here
analytically for the 2-link planar arm you built in FreeCAD (cad/).

For a 2-link arm the IK has a closed-form solution (law of cosines):
    cos(elbow) = (r² − L1² − L2²) / (2·L1·L2)
Then we command those angles in PyBullet and measure the REAL tip position from
the physics engine to prove the math is right (they should agree within ~1 cm).

    .venv/bin/python examples/pybullet/arm_ik_control.py            # headless check
    .venv/bin/python examples/pybullet/arm_ik_control.py --gui      # watch it reach

If the URDF is missing, generate it first:
    freecadcmd cad/build_two_link_arm.py && .venv/bin/python cad/make_arm_urdf.py
"""
import argparse
import math
import os

import pybullet as p

HERE = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.join(HERE, "..", "..", "assets", "urdf", "two_link_arm", "two_link_arm.urdf")

# arm geometry (must match cad/build_two_link_arm.py, in meters)
SHOULDER_Z = 0.02   # shoulder joint height (top of the base plate)
L1 = 0.15           # shoulder → elbow
L2 = 0.12           # elbow → tip


def ik_2link(x, z):
    """Analytic IK: tip target (x, z) in world → (shoulder, elbow) angles.
    Angles are measured from vertical (+z), rotating about +y (toward +x).
    Returns None if the target is out of reach or beyond the ±90° joint limits."""
    px, pz = x, z - SHOULDER_Z            # target relative to the shoulder
    r = math.hypot(px, pz)
    if not (abs(L1 - L2) <= r <= L1 + L2):
        return None                        # outside the reachable annulus
    cos_elbow = (r * r - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    elbow = math.acos(max(-1.0, min(1.0, cos_elbow)))          # elbow-down branch
    shoulder = math.atan2(px, pz) - math.atan2(L2 * math.sin(elbow),
                                               L1 + L2 * math.cos(elbow))
    if abs(shoulder) > 1.57 or abs(elbow) > 1.57:              # URDF joint limits
        return None
    return shoulder, elbow


def fk_2link(shoulder, elbow):
    """Forward kinematics — where the math says the tip is."""
    x = L1 * math.sin(shoulder) + L2 * math.sin(shoulder + elbow)
    z = SHOULDER_Z + L1 * math.cos(shoulder) + L2 * math.cos(shoulder + elbow)
    return x, z


def measured_tip(arm):
    """Tip position from the PHYSICS engine: link2 frame origin + rotated [0,0,L2]."""
    pos, orn = p.getLinkState(arm, 1)[4:6]         # link2 URDF frame (world)
    offset = p.rotateVector(orn, [0, 0, L2])
    return pos[0] + offset[0], pos[2] + offset[2]


def main(gui):
    p.connect(p.GUI if gui else p.DIRECT)
    p.setGravity(0, 0, -9.81)
    arm = p.loadURDF(URDF, useFixedBase=True)

    targets = [(0.10, 0.25), (0.20, 0.15), (-0.12, 0.22), (0.05, 0.28)]
    print(f"{'target (x,z)':>16} | {'IK angles (deg)':>18} | {'reached (x,z)':>16} | err")
    worst = 0.0
    for tx, tz in targets:
        sol = ik_2link(tx, tz)
        if sol is None:
            print(f"({tx:+.2f}, {tz:+.2f})  → unreachable (correctly rejected)")
            continue
        sh, el = sol
        for j, q in ((0, sh), (1, el)):
            p.setJointMotorControl2(arm, j, p.POSITION_CONTROL, targetPosition=q, force=20)
        for _ in range(480):                        # let the motors settle (~2 s)
            p.stepSimulation()
        mx, mz = measured_tip(arm)
        err = math.hypot(mx - tx, mz - tz)
        worst = max(worst, err)
        print(f"({tx:+.2f}, {tz:+.2f})   | ({math.degrees(sh):+6.1f}, {math.degrees(el):+6.1f}) "
              f"| ({mx:+.3f}, {mz:+.3f}) | {err*1000:.1f} mm")

    assert ik_2link(0.5, 0.5) is None, "far target should be unreachable"
    ok = worst < 0.01
    print(f"\nworst error {worst*1000:.1f} mm — "
          + ("PASS: analytic IK matches the simulated arm" if ok else "FAIL"))
    p.disconnect()
    return ok


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--gui", action="store_true")
    raise SystemExit(0 if main(ap.parse_args().gui) else 1)
