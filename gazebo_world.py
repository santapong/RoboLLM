#!/usr/bin/env python3
"""gazebo_world.py — control the Gazebo world itself: spawn/delete/reset objects.

Talks to Gazebo (Harmonic) through its `gz service` CLI, so it needs no rclpy and
works with whatever world is running (TurtleBot3 world, empty.sdf, your own).
Used by the MCP server to give Claude a hand on the simulation: "put a red box
0.5 m in front of the robot" becomes a spawn_object call.

The world name is auto-detected from the running simulator.
"""
from __future__ import annotations

import re
import subprocess

COLORS = {
    "red": (1, 0, 0), "green": (0, 1, 0), "blue": (0, 0, 1),
    "yellow": (1, 1, 0), "orange": (1, 0.5, 0), "white": (1, 1, 1),
    "black": (0.05, 0.05, 0.05), "grey": (0.5, 0.5, 0.5), "gray": (0.5, 0.5, 0.5),
}


def _run(args: list[str], timeout: float = 6.0) -> tuple[bool, str]:
    try:
        out = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return False, "timed out talking to Gazebo — is the sim running?"
    except FileNotFoundError:
        return False, "`gz` CLI not found — is Gazebo (Harmonic) installed?"
    text = (out.stdout or "") + (out.stderr or "")
    return out.returncode == 0, text.strip()


def world_name() -> str | None:
    """Detect the running world from Gazebo's service list."""
    ok, out = _run(["gz", "service", "-l"])
    if not ok:
        return None
    m = re.search(r"^/world/([^/]+)/create$", out, re.MULTILINE)
    return m.group(1) if m else None


def _service(world: str, name: str, reqtype: str, reptype: str, req: str) -> tuple[bool, str]:
    return _run(["gz", "service", "-s", f"/world/{world}/{name}",
                 "--reqtype", f"gz.msgs.{reqtype}", "--reptype", f"gz.msgs.{reptype}",
                 "--timeout", "3000", "--req", req])


def _shape_sdf(name: str, shape: str, size: float, color: str, static: bool) -> str | None:
    rgb = COLORS.get(color.lower())
    if rgb is None:
        return None
    if shape == "box":
        geom = f"<box><size>{size} {size} {size}</size></box>"
    elif shape == "sphere":
        geom = f"<sphere><radius>{size / 2}</radius></sphere>"
    elif shape == "cylinder":
        geom = f"<cylinder><radius>{size / 2}</radius><length>{size}</length></cylinder>"
    else:
        return None
    c = f"{rgb[0]} {rgb[1]} {rgb[2]} 1"
    return f"""<?xml version='1.0'?>
<sdf version='1.9'><model name='{name}'>
  <static>{str(static).lower()}</static>
  <link name='body'>
    <inertial><mass>0.5</mass></inertial>
    <collision name='col'><geometry>{geom}</geometry></collision>
    <visual name='vis'><geometry>{geom}</geometry>
      <material><ambient>{c}</ambient><diffuse>{c}</diffuse></material>
    </visual>
  </link>
</model></sdf>"""


def spawn(name: str, shape: str = "box", x: float = 0.5, y: float = 0.0,
          z: float = 0.25, size: float = 0.2, color: str = "red",
          static: bool = False) -> str:
    w = world_name()
    if w is None:
        return "error: no Gazebo world found — is the sim running?"
    sdf = _shape_sdf(name, shape, size, color, static)
    if sdf is None:
        return (f"error: shape must be box/sphere/cylinder and color one of "
                f"{sorted(COLORS)}")
    escaped = sdf.replace('"', '\\"').replace("\n", " ")
    req = (f'sdf: "{escaped}" '
           f'pose: {{position: {{x: {x}, y: {y}, z: {z}}}}} '
           f'allow_renaming: false')
    ok, out = _service(w, "create", "EntityFactory", "Boolean", req)
    if ok and "true" in out.lower():
        return f"spawned {color} {shape} '{name}' ({size} m) at ({x}, {y}, {z}) in world '{w}'"
    return f"error: spawn failed — {out or 'no reply'}"


def delete(name: str) -> str:
    w = world_name()
    if w is None:
        return "error: no Gazebo world found — is the sim running?"
    ok, out = _service(w, "remove", "Entity", "Boolean", f'name: "{name}" type: MODEL')
    if ok and "true" in out.lower():
        return f"deleted '{name}' from world '{w}'"
    return f"error: delete failed — {out or 'no reply'} (does '{name}' exist?)"


def reset() -> str:
    w = world_name()
    if w is None:
        return "error: no Gazebo world found — is the sim running?"
    ok, out = _service(w, "control", "WorldControl", "Boolean", "reset: {all: true}")
    if ok and "true" in out.lower():
        return f"world '{w}' reset (robot + objects back to start)"
    return f"error: reset failed — {out}"


def list_models() -> list[str] | str:
    ok, out = _run(["gz", "model", "--list"], timeout=8.0)
    if not ok:
        return "error: could not list models — is the sim running?"
    return re.findall(r"^\s*-\s*(\S+)", out, re.MULTILINE)
