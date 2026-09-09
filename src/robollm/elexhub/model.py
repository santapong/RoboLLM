"""One geometry description feeds MuJoCo, URDF, meshes and editable FreeCAD.

The shipped dimensions are an illustrative six-servo arm, explicitly provisional.
They must never be presented as a scan or measurement of the user's arm.
"""

from __future__ import annotations

import json
import math
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

import numpy as np
import trimesh
from robo_arm_driver.config import ArmConfig, ConfigError

from . import appearance


def vec(value) -> str:
    return " ".join(f"{float(x):.9g}" for x in value)


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * cross + (1 - math.cos(angle)) * cross @ cross


def transform(pos=(0, 0, 0), rot=None):
    t = np.eye(4)
    t[:3, 3] = pos
    if rot is not None:
        t[:3, :3] = rot
    return t


def geometry(config: ArmConfig) -> dict:
    g = config.profile_data.get("geometry", {})
    if len(g.get("links", [])) != config.njoints:
        raise ConfigError("geometry link count does not match joint count")
    if g.get("status") not in {"provisional", "measured"}:
        raise ConfigError("geometry status must be provisional or measured")
    for link, joint in zip(g["links"], config.joints):
        if link.get("joint") != joint.name:
            raise ConfigError("geometry joint order does not match profile")
        for key in ["length_m", "width_m", "depth_m", "mass_kg"]:
            v = link.get(key)
            if (
                isinstance(v, bool)
                or not isinstance(v, (float, int))
                or not math.isfinite(v)
                or not 0 < v < 2
            ):
                raise ConfigError(f"invalid {link['name']}.{key}")
        for key in ["offset_m", "axis"]:
            v = np.asarray(link.get(key), dtype=float)
            if v.shape != (3,) or not np.isfinite(v).all():
                raise ConfigError(f"invalid {key}")
        if not np.isclose(np.linalg.norm(link["axis"]), 1):
            raise ConfigError("joint axes must be unit vectors")
    for key in [
        "opening_m",
        "finger_length_m",
        "finger_width_m",
        "finger_depth_m",
        "finger_mass_kg",
    ]:
        if not math.isfinite(g["gripper"][key]) or not 0 < g["gripper"][key] < 0.2:
            raise ConfigError(f"invalid gripper {key}")
    return g


def forward(config: ArmConfig, q):
    g = geometry(config)
    q = np.asarray(q, float)
    if q.shape != (config.njoints,) or not np.isfinite(q).all():
        raise ConfigError("invalid forward-kinematics joint vector")
    t = transform(g["base_origin_m"])
    frames = {}
    for link, angle in zip(g["links"], q):
        t = t @ transform(link["offset_m"], rotation(link["axis"], angle))
        frames[link["name"]] = t.copy()
    tcp = t @ transform(
        [0, 0, g["links"][-1]["length_m"] + 0.75 * g["gripper"]["finger_length_m"]]
    )
    return frames, tcp


def mat_to_base(config, xy, height=0.0):
    g = geometry(config)["table"]
    xy = np.asarray(xy, float)
    if xy.shape != (2,) or not np.isfinite(xy).all() or not math.isfinite(height):
        raise ConfigError("invalid target")
    if (
        np.any(xy < 0)
        or np.any(xy > np.array(g["workspace_m"]))
        or not 0 <= height <= 0.25
    ):
        raise ConfigError("target outside measured tabletop workspace")
    return (
        np.array(g["mat_origin_m"])
        + rotation([0, 0, 1], g["mat_yaw_rad"]) @ np.r_[xy, height]
    )


def build_assets(config: ArmConfig, out: Path) -> dict:
    g = geometry(config)
    out.mkdir(parents=True, exist_ok=True)
    meshes = out / "meshes"
    meshes.mkdir(exist_ok=True)
    home = [j.raw_to_rad(j.home_deg) for j in config.joints]
    frames, _ = forward(config, home)
    solids_by_body = appearance.parts(g)
    visual_meshes = appearance.export_meshes(solids_by_body, meshes)
    descriptor = {
        "profile_hash": config.profile_hash,
        "status": g["status"],
        "units": "m",
        "notice": "Photo-referenced appearance; dimensions and dynamics remain provisional. "
        + g.get("notice", ""),
        "appearance": appearance.REFERENCE,
        "visual_meshes": visual_meshes,
        "parts": [],
    }
    frames["base"] = np.eye(4)
    grip = g["gripper"]
    for sign, name in [(-1, "finger_left"), (1, "finger_right")]:
        frames[name] = frames[g["links"][-1]["name"]] @ transform(
            [
                0,
                sign * (grip["opening_m"] + grip["finger_width_m"]) / 2,
                g["links"][-1]["length_m"],
            ]
        )
    links = {link["name"]: link for link in g["links"]}
    scene = trimesh.Scene()
    for name, solids in solids_by_body.items():
        part = {"name": name, "frame": frames[name].tolist(), "solids": solids}
        if name in links:
            part.update(joint=links[name]["joint"], axis=links[name]["axis"])
        descriptor["parts"].append(part)
        groups = defaultdict(list)
        for solid in solids:
            mesh = appearance.mesh_for(solid)
            world_mesh = mesh.copy()
            world_mesh.apply_transform(frames[name])
            solid["world_mesh_bounds_m"] = world_mesh.bounds.tolist()
            groups[solid["material"]].append(mesh)
        for material, items in groups.items():
            mesh = trimesh.util.concatenate(items)
            mesh.visual = trimesh.visual.TextureVisuals(
                material=trimesh.visual.material.PBRMaterial(
                    name=material,
                    baseColorFactor=np.array(appearance.MATERIALS[material]) * 255,
                    metallicFactor=0.6 if material in {"steel", "anodized"} else 0.0,
                    roughnessFactor=0.36 if material == "steel" else 0.65,
                )
            )
            scene.add_geometry(
                mesh, node_name=name + "_" + material, transform=frames[name]
            )
    viewer = {
        "meshes": [],
        "center": scene.bounds.mean(axis=0).tolist(),
        "radius": float(np.linalg.norm(scene.extents) / 2),
        "dimensions": {
            "Base collision footprint": f"{g['base_size_m'][0] * 1000:g} × {g['base_size_m'][1] * 1000:g} mm (estimate; visual plates have a narrower outline)",
            "Upper arm / forearm": f"{g['links'][1]['length_m'] * 1000:g} / {g['links'][2]['length_m'] * 1000:g} mm (joint spacing unverified)",
            "Claw opening / length": f"{grip['opening_m'] * 1000:g} / {grip['finger_length_m'] * 1000:g} mm (photo estimates)",
            "Servo reference": "40.7 × 19.7 × 42.9 mm; actual motor unverified",
        },
    }
    for node in scene.graph.nodes_geometry:
        pose, mesh_name = scene.graph[node]
        mesh = scene.geometry[mesh_name].copy()
        material = mesh.visual.material.name
        mesh.apply_transform(pose)
        viewer["meshes"].append(
            {
                "positions": mesh.vertices.round(7).ravel().tolist(),
                "normals": mesh.vertex_normals.round(5).ravel().tolist(),
                "indices": mesh.faces.ravel().tolist(),
                "color": appearance.MATERIALS[material],
            }
        )
    template = Path(__file__).with_name("model_viewer.html").read_text()
    (out / "model.html").write_text(
        template.replace("__MODEL_DATA__", json.dumps(viewer, separators=(",", ":")))
    )
    # glTF viewers conventionally use +Y up; CAD/MuJoCo/URDF use +Z up.
    gltf = scene.copy()
    gltf.apply_transform(transform(rot=rotation([1, 0, 0], -math.pi / 2)))
    gltf.export(out / "elexhub.glb")
    (out / "cad-description.json").write_text(json.dumps(descriptor, indent=2) + "\n")
    (out / "elexhub.xml").write_text(mjcf(config, visual_meshes))
    (out / "elexhub-robot.xml").write_text(
        mjcf(config, visual_meshes, include_scene=False)
    )
    (out / "elexhub.urdf").write_text(urdf(config, visual_meshes))
    (out / "APPEARANCE_PROVENANCE.json").write_text(
        json.dumps(appearance.REFERENCE, indent=2) + "\n"
    )
    (out / "MODEL_STATUS.json").write_text(
        json.dumps(
            {
                "profile_hash": config.profile_hash,
                "geometry_status": g["status"],
                "hardware_ready": False
                if g["status"] == "provisional"
                else config.calibrated,
                "notice": g.get("notice", ""),
                "inertias": "box approximations; not system-identified",
                "joint_names": config.joint_names,
                "appearance_revision": appearance.REVISION,
                "appearance_source": "Owner's five robot photographs; catalog references for component context only",
                "dimensions_verified": False,
                "collision_geometry": "provisional proxies; decorative parts are not collision geometry",
            },
            indent=2,
        )
        + "\n"
    )
    return descriptor


def _xml(root):
    ET.indent(root, "  ")
    return ET.tostring(root, encoding="unicode") + "\n"


def _visuals(parent, records, *, urdf=False):
    for record in records:
        if urdf:
            visual = ET.SubElement(parent, "visual", name=record["name"])
            ET.SubElement(
                ET.SubElement(visual, "geometry"),
                "mesh",
                filename=f"meshes/{record['name']}.obj",
            )
            material = ET.SubElement(visual, "material", name=record["material"])
            ET.SubElement(
                material, "color", rgba=vec(appearance.MATERIALS[record["material"]])
            )
        else:
            ET.SubElement(
                parent,
                "geom",
                name=record["name"] + "_visual",
                type="mesh",
                mesh=record["name"],
                material=record["material"],
                mass="0",
                contype="0",
                conaffinity="0",
                group="1",
            )


def mjcf(config, visual_meshes, *, include_scene=True):
    """Export the shared robot, optionally with the existing tabletop scene."""
    g = geometry(config)
    root = ET.Element("mujoco", model="elexhub_" + g["status"])
    ET.SubElement(root, "compiler", angle="radian", autolimits="true")
    ET.SubElement(
        root, "option", timestep="0.002", integrator="implicitfast", gravity="0 0 -9.81"
    )
    ET.SubElement(
        ET.SubElement(root, "visual"), "global", offwidth="640", offheight="480"
    )
    default = ET.SubElement(root, "default")
    ET.SubElement(default, "joint", damping="0.04", armature="0.002")
    ET.SubElement(default, "geom", friction="1.2 0.01 0.001", condim="4")
    asset = ET.SubElement(root, "asset")
    ET.SubElement(
        asset,
        "texture",
        name="studio_sky",
        type="skybox",
        builtin="flat",
        width="256",
        height="1536",
        rgb1=".93 .95 .96",
        rgb2=".93 .95 .96",
    )
    for records in visual_meshes.values():
        for record in records:
            ET.SubElement(
                asset, "mesh", name=record["name"], file=f"meshes/{record['name']}.obj"
            )
    for name, color in appearance.MATERIALS.items():
        ET.SubElement(
            asset,
            "material",
            name=name,
            rgba=vec(color),
            specular="0.6" if name == "steel" else "0.25",
            shininess="0.4",
        )
    if include_scene:
        ET.SubElement(
            asset,
            "texture",
            name="floor_tex",
            type="2d",
            builtin="checker",
            width="256",
            height="256",
            rgb1=".86 .87 .87",
            rgb2=".79 .81 .82",
        )
        ET.SubElement(
            asset, "material", name="floor_mat", texture="floor_tex", texrepeat="8 8"
        )
    world = ET.SubElement(root, "worldbody")
    ET.SubElement(
        world, "light", pos="0 -.3 1", diffuse=".8 .8 .8", ambient=".28 .28 .28"
    )
    ET.SubElement(
        world, "light", pos="-.3 .4 .7", diffuse=".55 .55 .55", castshadow="false"
    )
    if include_scene:
        ET.SubElement(
            world,
            "geom",
            name="table",
            type="plane",
            size=".6 .6 .02",
            material="floor_mat",
        )
        # Mat has no extra collision surface: its printed surface is table z=0.
        center = mat_to_base(config, np.array(g["table"]["workspace_m"]) / 2)
        ET.SubElement(
            world,
            "geom",
            name="mat",
            type="box",
            pos=vec(center + [0, 0, 0.0001]),
            size=vec([0.06, 0.09, 0.0001]),
            rgba=".95 .95 .92 1",
            contype="0",
            conaffinity="0",
        )
    base = ET.SubElement(world, "body", name="base")
    _visuals(base, visual_meshes["base"])
    ET.SubElement(
        base,
        "geom",
        name="base_collision",
        type="box",
        size=vec(np.array(g["base_size_m"]) / 2),
        pos=vec([0, 0, g["base_size_m"][2] / 2]),
        rgba="0 0 0 0",
        group="3",
    )
    parent = base
    contact = ET.SubElement(root, "contact")
    actuator = ET.SubElement(root, "actuator")
    for i, (link, j) in enumerate(zip(g["links"], config.joints)):
        offset = np.array(link["offset_m"]) + (
            np.array(g["base_origin_m"]) if i == 0 else 0
        )
        body = ET.SubElement(parent, "body", name=link["name"], pos=vec(offset))
        ET.SubElement(
            body,
            "joint",
            name=j.name,
            type="hinge",
            axis=vec(link["axis"]),
            range=vec([j.min_rad, j.max_rad]),
        )
        dims = np.array([link["width_m"], link["depth_m"], link["length_m"]])
        inertia = (
            link["mass_kg"]
            / 12
            * np.array(
                [
                    dims[1] ** 2 + dims[2] ** 2,
                    dims[0] ** 2 + dims[2] ** 2,
                    dims[0] ** 2 + dims[1] ** 2,
                ]
            )
        )
        ET.SubElement(
            body,
            "inertial",
            pos=vec([0, 0, dims[2] / 2]),
            mass=str(link["mass_kg"]),
            diaginertia=vec(inertia),
        )
        _visuals(body, visual_meshes[link["name"]])
        ET.SubElement(
            body,
            "geom",
            name=link["name"] + "_collision",
            type="capsule",
            fromto=vec([0, 0, 0.008, 0, 0, max(0.009, dims[2] - 0.008)]),
            size=str(min(dims[:2]) * 0.43),
            rgba="0 0 0 0",
            group="3",
        )
        ET.SubElement(contact, "exclude", body1=parent.get("name"), body2=link["name"])
        a = g["actuator_reference"]
        ET.SubElement(
            actuator,
            "position",
            name=j.name + "_servo",
            joint=j.name,
            kp=str(a["kp"]),
            kv=str(a["kv"]),
            ctrlrange=vec([j.min_rad, j.max_rad]),
            forcerange=vec([-a["force_limit"], a["force_limit"]]),
        )
        parent = body
    grip = g["gripper"]
    ET.SubElement(
        parent,
        "site",
        name="tcp",
        pos=vec([0, 0, g["links"][-1]["length_m"] + 0.75 * grip["finger_length_m"]]),
        size=".003",
        rgba=".1 .9 .6 1",
    )
    for sign, name in [(-1, "finger_left"), (1, "finger_right")]:
        body = ET.SubElement(
            parent,
            "body",
            name=name,
            pos=vec(
                [
                    0,
                    sign * (grip["opening_m"] + grip["finger_width_m"]) / 2,
                    g["links"][-1]["length_m"],
                ]
            ),
        )
        ET.SubElement(
            body,
            "joint",
            name=name,
            type="slide",
            axis=vec([0, -sign, 0]),
            range=vec([0, grip["opening_m"] / 2]),
            damping="1",
        )
        _visuals(body, visual_meshes[name])
        ET.SubElement(
            body,
            "geom",
            name=name + "_pad",
            type="box",
            size=vec(
                [
                    grip["finger_depth_m"] / 2,
                    grip["finger_width_m"] / 2,
                    grip["finger_length_m"] / 2,
                ]
            ),
            pos=vec([0, 0, grip["finger_length_m"] / 2]),
            mass=str(grip["finger_mass_kg"]),
            rgba="0 0 0 0",
            group="3",
            friction="2 0.01 0.001",
        )
        ET.SubElement(contact, "exclude", body1=parent.get("name"), body2=name)
    ET.SubElement(
        ET.SubElement(root, "equality"),
        "joint",
        joint1="finger_left",
        joint2="finger_right",
        polycoef="0 1 0 0 0",
        solref=".005 1",
    )
    ET.SubElement(
        actuator,
        "position",
        name="gripper_servo",
        joint="finger_left",
        kp="250",
        kv="3",
        ctrlrange=vec([0, grip["opening_m"] / 2]),
        forcerange="-3 3",
    )
    if include_scene:
        cube = g["object"]
        object_pos = mat_to_base(config, [0.07, 0.05], cube["edge_m"] / 2 + 0.0002)
        obj = ET.SubElement(world, "body", name="red_box", pos=vec(object_pos))
        ET.SubElement(obj, "freejoint", name="object_free")
        ET.SubElement(
            obj,
            "geom",
            name="red_box_geom",
            type="box",
            size=vec([cube["edge_m"] / 2] * 3),
            mass=str(cube["mass_kg"]),
            rgba=".85 .08 .06 1",
        )
        camera = g["camera"]
        backward = np.array(camera["position_m"]) - np.array(camera["look_at_m"])
        backward /= np.linalg.norm(backward)
        right = np.cross([0, 0, 1], backward)
        right /= np.linalg.norm(right)
        up = np.cross(backward, right)
        ET.SubElement(
            world,
            "camera",
            name="webcam",
            pos=vec(camera["position_m"]),
            xyaxes=vec(np.r_[right, up]),
            fovy=str(camera["fovy_deg"]),
        )
    return _xml(root)


def urdf(config, visual_meshes):
    g = geometry(config)
    root = ET.Element("robot", name="elexhub_" + g["status"])
    base = ET.SubElement(root, "link", name="base_link")
    _visuals(base, visual_meshes["base"], urdf=True)
    parent = "base_link"
    for i, (link, j) in enumerate(zip(g["links"], config.joints)):
        node = ET.SubElement(root, "link", name=link["name"])
        _visuals(node, visual_meshes[link["name"]], urdf=True)
        dims = np.array([link["width_m"], link["depth_m"], link["length_m"]])
        c = [0, 0, dims[2] / 2]
        collision = ET.SubElement(node, "collision")
        ET.SubElement(collision, "origin", xyz=vec(c))
        ET.SubElement(ET.SubElement(collision, "geometry"), "box", size=vec(dims))
        inertial = ET.SubElement(node, "inertial")
        ET.SubElement(inertial, "origin", xyz=vec(c))
        ET.SubElement(inertial, "mass", value=str(link["mass_kg"]))
        iv = (
            link["mass_kg"]
            / 12
            * np.array(
                [
                    dims[1] ** 2 + dims[2] ** 2,
                    dims[0] ** 2 + dims[2] ** 2,
                    dims[0] ** 2 + dims[1] ** 2,
                ]
            )
        )
        ET.SubElement(
            inertial,
            "inertia",
            ixx=str(iv[0]),
            iyy=str(iv[1]),
            izz=str(iv[2]),
            ixy="0",
            ixz="0",
            iyz="0",
        )
        joint = ET.SubElement(root, "joint", name=j.name, type="revolute")
        ET.SubElement(joint, "parent", link=parent)
        ET.SubElement(joint, "child", link=link["name"])
        ET.SubElement(
            joint,
            "origin",
            xyz=vec(
                np.array(link["offset_m"])
                + (np.array(g["base_origin_m"]) if i == 0 else 0)
            ),
        )
        ET.SubElement(joint, "axis", xyz=vec(link["axis"]))
        ET.SubElement(
            joint,
            "limit",
            lower=str(j.min_rad),
            upper=str(j.max_rad),
            effort="1.5",
            velocity=str(j.max_velocity_rad_s),
        )
        parent = link["name"]
    grip = g["gripper"]
    for sign, name in [(-1, "finger_left"), (1, "finger_right")]:
        node = ET.SubElement(root, "link", name=name)
        _visuals(node, visual_meshes[name], urdf=True)
        for kind in ["collision"]:
            v = ET.SubElement(node, kind)
            ET.SubElement(v, "origin", xyz=vec([0, 0, grip["finger_length_m"] / 2]))
            ET.SubElement(
                ET.SubElement(v, "geometry"),
                "box",
                size=vec(
                    [
                        grip["finger_depth_m"],
                        grip["finger_width_m"],
                        grip["finger_length_m"],
                    ]
                ),
            )
        j = ET.SubElement(root, "joint", name=name, type="prismatic")
        ET.SubElement(j, "parent", link=parent)
        ET.SubElement(j, "child", link=name)
        ET.SubElement(
            j,
            "origin",
            xyz=vec(
                [
                    0,
                    sign * (grip["opening_m"] + grip["finger_width_m"]) / 2,
                    g["links"][-1]["length_m"],
                ]
            ),
        )
        ET.SubElement(j, "axis", xyz=vec([0, -sign, 0]))
        ET.SubElement(
            j,
            "limit",
            lower="0",
            upper=str(grip["opening_m"] / 2),
            effort="3",
            velocity=".03",
        )
        if sign == 1:
            ET.SubElement(j, "mimic", joint="finger_left", multiplier="1", offset="0")
    ET.SubElement(root, "link", name="tool_center")
    tcp = ET.SubElement(root, "joint", name="tcp_fixed", type="fixed")
    ET.SubElement(tcp, "parent", link=parent)
    ET.SubElement(tcp, "child", link="tool_center")
    ET.SubElement(
        tcp,
        "origin",
        xyz=vec([0, 0, g["links"][-1]["length_m"] + 0.75 * grip["finger_length_m"]]),
    )
    return _xml(root)
