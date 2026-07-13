#!/usr/bin/env python3
"""MuJoCo quickstart — Google DeepMind's fast physics engine, CPU-friendly.

MuJoCo is the standard for modern robot learning (locomotion, manipulation,
humanoids). This defines a simple pendulum in MJCF (MuJoCo's XML), simulates it,
and prints the swing. The same engine (via MuJoCo MJX) scales to thousands of
GPU-parallel envs on your cloud for RL — this is the local, learn-the-API version.

Run headless (prints the pendulum angle over time):
    python3 hello_mujoco.py
With the interactive 3D viewer (needs a display):
    python3 hello_mujoco.py --view
"""
import argparse

import mujoco

MODEL_XML = """
<mujoco>
  <option gravity="0 0 -9.81"/>
  <worldbody>
    <light pos="0 0 3"/>
    <geom type="plane" size="2 2 0.1" rgba="0.3 0.3 0.3 1"/>
    <body name="arm" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 1 0"/>
      <geom type="capsule" fromto="0 0 0 0 0 -0.5" size="0.04" rgba="0.2 0.6 0.9 1"/>
    </body>
  </worldbody>
</mujoco>
"""


def main(view: bool):
    model = mujoco.MjModel.from_xml_string(MODEL_XML)
    data = mujoco.MjData(model)
    data.qpos[0] = 1.2  # start the pendulum raised

    if view:
        from mujoco import viewer as mj_viewer
        with mj_viewer.launch_passive(model, data) as viewer:
            while viewer.is_running():
                mujoco.mj_step(model, data)
                viewer.sync()
        return

    for step in range(1000):
        mujoco.mj_step(model, data)
        if step % 100 == 0:
            print(f"t={data.time:5.2f}s  hinge angle={data.qpos[0]:+.3f} rad")
    print("done.")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--view", action="store_true", help="open the 3D viewer")
    main(ap.parse_args().view)
