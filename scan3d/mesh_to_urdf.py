#!/usr/bin/env python3
"""mesh_to_urdf.py — wrap a scanned/CAD mesh into a ready-to-use URDF link.

Turns any mesh (.obj/.ply/.stl) into ../assets/urdf/<name>/<name>.urdf with:
  - a VISUAL mesh (the detailed surface)
  - a COLLISION mesh (the convex hull — simple & fast for physics engines)
  - an <inertial> block computed from the geometry at a chosen density

Drop the result into PyBullet (examples/pybullet) or a ROS 2 / Gazebo world.

    python3 mesh_to_urdf.py ../assets/scan/mug/mug_hull.obj --name mug --density 300
"""
import argparse
import os
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mesh", help="input .obj/.ply/.stl")
    ap.add_argument("--name", default=None, help="link/file name (default: from mesh)")
    ap.add_argument("--density", type=float, default=400.0,
                    help="kg/m^3 for the inertia estimate (water=1000, light plastic~400)")
    args = ap.parse_args()

    name = args.name or os.path.splitext(os.path.basename(args.mesh))[0]
    out_dir = os.path.join(HERE, "..", "assets", "urdf", name)
    mesh_dir = os.path.join(out_dir, "meshes")
    os.makedirs(mesh_dir, exist_ok=True)

    mesh = trimesh.load(args.mesh, force="mesh")
    mesh.density = args.density

    # visual = original surface; collision = convex hull (robust for sim)
    visual_path = os.path.join(mesh_dir, f"{name}_visual.stl")
    collision_path = os.path.join(mesh_dir, f"{name}_collision.stl")
    mesh.export(visual_path)
    hull = mesh.convex_hull
    hull.density = args.density          # convex_hull() does NOT inherit density
    hull.export(collision_path)

    m = mesh.mass if mesh.is_watertight else max(mesh.convex_hull.mass, 1e-3)
    it = mesh.moment_inertia if mesh.is_watertight else hull.moment_inertia
    com = mesh.center_mass if mesh.is_watertight else hull.center_mass

    urdf = f"""<?xml version="1.0"?>
<robot name="{name}">
  <link name="{name}">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="meshes/{name}_visual.stl"/></geometry>
      <material name="scan"><color rgba="0.8 0.8 0.85 1"/></material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="meshes/{name}_collision.stl"/></geometry>
    </collision>
    <inertial>
      <origin xyz="{com[0]:.5f} {com[1]:.5f} {com[2]:.5f}" rpy="0 0 0"/>
      <mass value="{m:.5f}"/>
      <inertia ixx="{it[0,0]:.6e}" ixy="{it[0,1]:.6e}" ixz="{it[0,2]:.6e}"
               iyy="{it[1,1]:.6e}" iyz="{it[1,2]:.6e}" izz="{it[2,2]:.6e}"/>
    </inertial>
  </link>
</robot>
"""
    urdf_path = os.path.join(out_dir, f"{name}.urdf")
    with open(urdf_path, "w") as f:
        f.write(urdf)

    dims = mesh.extents * 1000.0
    print(f"link '{name}': {dims[0]:.0f}×{dims[1]:.0f}×{dims[2]:.0f} mm, "
          f"mass {m:.3f} kg @ {args.density:.0f} kg/m³, watertight={mesh.is_watertight}")
    print(f"saved → {urdf_path}")
    print(f"Try it:  python3 ../examples/pybullet/load_robot.py   "
          f"(edit it to loadURDF('{os.path.relpath(urdf_path)}'))")


if __name__ == "__main__":
    main()
