"""build_two_link_arm.py — build a 2-link robot arm in FreeCAD and export meshes.

Runs headless via FreeCAD's CLI (no display needed):
    freecadcmd cad/build_two_link_arm.py
or paste into the FreeCAD GUI Python console, or send to the `freecad` MCP with
execute_code once its RPC server is running.

The KEY discipline for CAD -> URDF: model each link in ITS OWN frame, with the
joint that attaches it to its parent at the link's origin. Then the exported mesh
and the URDF joint origins line up with no guesswork.

Units: FreeCAD works in mm; the URDF (meters) references these with scale 0.001.
Frames (all boxes centered in x,y, growing +z from z=0):
  base_link : 60x60x20  base plate, sits on the ground
  link1     : 20x20x150 beam, origin at the SHOULDER joint (top of base)
  link2     : 20x20x120 beam, origin at the ELBOW joint (end of link1)
"""
import os

import FreeCAD as App
import Part

HERE = os.path.dirname(os.path.abspath(__file__))
MESHES = os.path.join(HERE, "..", "assets", "urdf", "two_link_arm", "meshes")
CAD = os.path.join(HERE, "..", "assets", "cad")
os.makedirs(MESHES, exist_ok=True)
os.makedirs(CAD, exist_ok=True)


def centered_box(length, width, height):
    """Box centered in x,y, growing from z=0 upward."""
    return Part.makeBox(length, width, height, App.Vector(-length / 2, -width / 2, 0))


doc = App.newDocument("two_link_arm")
links = {
    "base_link": centered_box(60, 60, 20),
    "link1": centered_box(20, 20, 150),
    "link2": centered_box(20, 20, 120),
}
for name, shape in links.items():
    Part.show(shape, name)
    stl = os.path.join(MESHES, name + ".stl")
    shape.exportStl(stl)
    print(f"exported {name} -> {stl}")

doc.recompute()
fcstd = os.path.join(CAD, "two_link_arm.FCStd")
doc.saveAs(fcstd)
print(f"saved {fcstd}")
