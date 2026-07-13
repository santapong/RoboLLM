#!/usr/bin/env python3
"""make_arm_urdf.py — assemble the FreeCAD-exported link meshes into a URDF.

Reads the three STL meshes built by build_two_link_arm.py, computes each link's
mass + inertia (via trimesh at a chosen density), and writes a 2-revolute-joint
arm to assets/urdf/two_link_arm/two_link_arm.urdf.

    .venv/bin/python cad/make_arm_urdf.py

Kinematics (meters; meshes are mm so referenced with scale 0.001):
  base_link --[shoulder: revolute, axis y, @ z=0.02]--> link1
  link1     --[elbow:    revolute, axis y, @ z=0.15]--> link2
"""
import os

import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
ARM = os.path.join(HERE, "..", "assets", "urdf", "two_link_arm")
MESHES = os.path.join(ARM, "meshes")
MM_TO_M = 0.001
DENSITY = 700.0  # kg/m^3 (light aluminium-ish)

# (link, parent, joint_name, joint_origin_z_m) — root has parent None
LINKS = [
    ("base_link", None, None, None),
    ("link1", "base_link", "shoulder", 0.02),
    ("link2", "link1", "elbow", 0.15),
]


def prepare_mesh(name):
    """Load FreeCAD's mm STL, scale to meters, and re-export as OBJ (PyBullet's
    STL loader is unreliable; OBJ via tinyobjloader is solid). Returns inertia."""
    m = trimesh.load(os.path.join(MESHES, f"{name}.stl"), force="mesh")
    m.apply_scale(MM_TO_M)          # mm -> m
    m.export(os.path.join(MESHES, f"{name}.obj"))   # meters, sim-ready
    m.density = DENSITY
    src = m if m.is_watertight else m.convex_hull
    src.density = DENSITY
    return src.mass, src.moment_inertia, src.center_mass


def link_xml(name):
    mass, it, com = prepare_mesh(name)
    mesh = f'<mesh filename="meshes/{name}.obj" scale="1 1 1"/>'
    return f"""  <link name="{name}">
    <visual>
      <geometry>{mesh}</geometry>
      <material name="alu"><color rgba="0.7 0.7 0.75 1"/></material>
    </visual>
    <collision><geometry>{mesh}</geometry></collision>
    <inertial>
      <origin xyz="{com[0]:.5f} {com[1]:.5f} {com[2]:.5f}"/>
      <mass value="{mass:.5f}"/>
      <inertia ixx="{it[0,0]:.6e}" ixy="{it[0,1]:.6e}" ixz="{it[0,2]:.6e}"
               iyy="{it[1,1]:.6e}" iyz="{it[1,2]:.6e}" izz="{it[2,2]:.6e}"/>
    </inertial>
  </link>""", mass


def joint_xml(name, parent, child, z):
    return f"""  <joint name="{name}" type="revolute">
    <parent link="{parent}"/>
    <child link="{child}"/>
    <origin xyz="0 0 {z}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="2.0"/>
  </joint>"""


def main():
    parts = ['<?xml version="1.0"?>', '<robot name="two_link_arm">']
    total = 0.0
    for name, parent, jname, z in LINKS:
        lx, mass = link_xml(name)
        parts.append(lx)
        total += mass
        if parent is not None:
            parts.append(joint_xml(jname, parent, name, z))
    parts.append("</robot>")
    out = os.path.join(ARM, "two_link_arm.urdf")
    with open(out, "w") as f:
        f.write("\n".join(parts) + "\n")
    print(f"wrote {out}")
    print(f"3 links, 2 revolute joints, total mass {total:.3f} kg @ {DENSITY:.0f} kg/m^3")
    print(f"Verify:  .venv/bin/python cad/verify_arm_pybullet.py")


if __name__ == "__main__":
    main()
