# RoboLLM · CAD → URDF → simulated robot

> **RoboLLM field guide** · Build → Observe → Measure → Learn<br>
> [Home](../README.md) · [Documentation](../docs/README.md) · [Technical notes](TECHNICAL.md) · [Architecture diagram](docs/cad-architecture.svg)

**Status:** verified reference pipeline on the environment named below.

Proves the CAD-to-robot pipeline end to end with a **2-link arm**, and it runs
**headless** (no display) so you can regenerate it anytime — or drive the same
build interactively through the `freecad` MCP.

```
 build_two_link_arm.py ──(freecadcmd)──▶ STL meshes + .FCStd
          │                                     │
          │                          make_arm_urdf.py ──▶ OBJ meshes + two_link_arm.urdf
          │                                     │
          └───────────────────────── verify_arm_pybullet.py ──▶ it moves ✅
```

## Run it
```bash
cd /path/to/RoboLLM
freecadcmd cad/build_two_link_arm.py        # FreeCAD builds + exports meshes
.venv/bin/python cad/make_arm_urdf.py       # -> assets/urdf/two_link_arm/two_link_arm.urdf
.venv/bin/python cad/verify_arm_pybullet.py # PASS: 2 revolute joints, tip moves ~200 mm
.venv/bin/python cad/verify_arm_pybullet.py --gui   # watch it (needs a display)
```
Verified headless: FreeCAD 0.21 built the meshes, the URDF loads in PyBullet with
2 working revolute joints, and the end-effector moves when commanded.

## Files
| File | Runs with | Does |
|------|-----------|------|
| `build_two_link_arm.py` | `freecadcmd` (or the FreeCAD GUI / MCP) | Build 3 link solids, export STL + `.FCStd` |
| `make_arm_urdf.py` | project `.venv` | STL→OBJ (meters), compute inertia, write URDF + 2 joints |
| `verify_arm_pybullet.py` | project `.venv` | Load URDF, check joints, drive it, confirm motion |

## The two things that matter (why this is the "right" way)
1. **Model each link in its own joint frame.** Each link's origin sits at the
   joint that attaches it to its parent, so the mesh and the URDF joint origins
   line up with no fudging. `build_two_link_arm.py` does this deliberately.
2. **Convert meshes to OBJ for sim.** FreeCAD exports ASCII STL that PyBullet's
   loader can't parse ("cannot extract mesh"). `make_arm_urdf.py` re-exports each
   mesh as OBJ in meters via trimesh — PyBullet loads OBJ reliably.

## Doing it through the FreeCAD MCP instead (interactive)
With FreeCAD open → **MCP Addon** workbench → **Start RPC Server**, Claude can run
the same build live: it sends `build_two_link_arm.py`'s body via the freecad MCP
`execute_code` tool, then you run `make_arm_urdf.py` + `verify_arm_pybullet.py`.
(Headless `freecadcmd` needs no RPC server and is what we verified here.)

## Extend
Change link sizes / add a 3rd joint / a gripper in `build_two_link_arm.py`, then
rerun the three steps. Scanned real objects (`../scan3d`) can become links too.
Outputs: `assets/urdf/two_link_arm/` (URDF + OBJ, tracked), `assets/cad/*.FCStd`.
