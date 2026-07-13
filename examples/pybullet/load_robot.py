#!/usr/bin/env python3
"""PyBullet quickstart — a physics sim that runs on your CPU (no GPU, no ROS).

Great for learning robot dynamics, URDFs, and RL environments locally, then
scaling to GPU sim (Isaac Lab / MuJoCo MJX) on your GCP/AWS later.

Run (needs a display for the GUI window):
    python3 load_robot.py
Headless (no window, just prints):
    python3 load_robot.py --headless

Loads the built-in R2D2 URDF onto a ground plane, drops it, and steps physics.
Try next: swap the URDF for one you exported from FreeCAD into ../../assets/urdf.
"""
import argparse
import time

import pybullet as p
import pybullet_data


def main(headless: bool):
    cid = p.connect(p.DIRECT if headless else p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # built-in models
    p.setGravity(0, 0, -9.81)

    p.loadURDF("plane.urdf")
    robot = p.loadURDF("r2d2.urdf", basePosition=[0, 0, 1.0])
    print(f"loaded r2d2: body id={robot}, joints={p.getNumJoints(robot)}")
    for j in range(p.getNumJoints(robot)):
        info = p.getJointInfo(robot, j)
        print(f"  joint {j}: {info[1].decode()}  type={info[2]}")

    # step the simulation ~5 seconds (240 Hz default)
    for step in range(240 * 5):
        p.stepSimulation()
        if step % 240 == 0:
            pos, _ = p.getBasePositionAndOrientation(robot)
            print(f"t={step/240:.0f}s  base height={pos[2]:.3f} m")
        if not headless:
            time.sleep(1 / 240)

    p.disconnect()
    print("done.")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--headless", action="store_true", help="no GUI window")
    main(ap.parse_args().headless)
