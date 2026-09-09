"""Build an original articulated humanoid from the generated turnaround reference.

This is authored geometry guided by the image, not a learned image-to-3D or
photogrammetric reconstruction. All dimensions, masses and actuators are design
estimates. No G1 or H1 mesh, inertia or kinematic file is copied.
"""

from __future__ import annotations

import hashlib
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import trimesh

HERE = Path(__file__).resolve().parent
WHITE = (0.86, 0.88, 0.87, 1)
DARK = (0.10, 0.13, 0.15, 1)
METAL = (0.23, 0.28, 0.30, 1)
CYAN = (0.12, 0.78, 0.88, 1)
VISOR = (0.025, 0.048, 0.060, 1)


def numbers(values):
    return " ".join(f"{float(v):.8g}" for v in values)


def rounded_shell(extents, exponent=0.42, taper=0.0):
    """Closed superellipsoid with broad faces and rounded corners."""
    vertices = []
    rings, sides = 20, 40

    def signed_power(value):
        return math.copysign(abs(value) ** exponent, value)

    for i in range(rings + 1):
        latitude = -math.pi / 2 + math.pi * i / rings
        cv, sv = signed_power(math.cos(latitude)), signed_power(math.sin(latitude))
        if i in (0, rings):
            cv = 0.0  # Exact poles prevent tiny non-manifold rings after welding.
        for j in range(sides):
            longitude = 2 * math.pi * j / sides
            vertices.append([extents[0] / 2 * cv * signed_power(math.cos(longitude)),
                             extents[1] / 2 * cv * signed_power(math.sin(longitude)) * (1 + taper * sv),
                             extents[2] / 2 * sv])
    faces = []
    for i in range(rings):
        for j in range(sides):
            a, b = i * sides + j, i * sides + (j + 1) % sides
            c, d = a + sides, b + sides
            faces.extend(((a, b, c), (b, d, c)))
    mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=True)
    mesh.update_faces(mesh.nondegenerate_faces())
    mesh.remove_unreferenced_vertices()
    mesh.fix_normals()
    if not mesh.is_watertight or mesh.volume <= 0:
        raise ValueError("Generated shell must be a closed positive-volume mesh")
    return mesh


class Builder:
    def __init__(self):
        self.root = ET.Element("mujoco", model="RoboLLM original concept 01")
        ET.SubElement(self.root, "compiler", angle="radian", meshdir="meshes", autolimits="true")
        ET.SubElement(self.root, "option", timestep="0.002", gravity="0 0 -9.81", integrator="implicitfast")
        default = ET.SubElement(self.root, "default")
        ET.SubElement(default, "joint", damping="1.0", armature="0.025")
        ET.SubElement(default, "geom", contype="0", conaffinity="0", group="1")
        self.assets = ET.SubElement(self.root, "asset")
        self.world = ET.SubElement(self.root, "worldbody")
        self.actuators = ET.SubElement(self.root, "actuator")
        self.meshes = {}
        self.instances = []
        self.parents = {}
        self.joints = []

    def body(self, parent, name, pos=(0, 0, 0), mass=0.1, com=(0, 0, 0), inertia=0.001):
        body = ET.SubElement(parent, "body", name=name, pos=numbers(pos))
        ET.SubElement(body, "inertial", pos=numbers(com), mass=str(mass), diaginertia=numbers([inertia] * 3))
        self.parents[name] = parent.get("name", "world")
        return body

    def joint(self, body, name, axis, limits, kp=130, kv=10):
        ET.SubElement(body, "joint", name=name, type="hinge", axis=numbers(axis), range=numbers(limits))
        ET.SubElement(self.actuators, "position", name=name.removesuffix("_joint") + "_servo", joint=name,
                      kp=str(kp), kv=str(kv), ctrlrange=numbers(limits), forcerange="-90 90")
        self.joints.append(name)

    def shape(self, body, name, shape, rgba=WHITE, pos=(0, 0, 0), quat=(1, 0, 0, 0)):
        if name not in self.meshes:
            self.meshes[name] = shape
            ET.SubElement(self.assets, "mesh", name=name, file=name + ".obj")
        geom_name = body.get("name") + "_" + name + "_" + str(len(self.instances))
        ET.SubElement(body, "geom", name=geom_name, type="mesh", mesh=name,
                      rgba=numbers(rgba), pos=numbers(pos), quat=numbers(quat))
        self.instances.append({"name": geom_name, "body": body.get("name"), "mesh": name,
                               "pos": list(pos), "quat": list(quat), "rgba": list(rgba)})

    def shell(self, body, name, extents, rgba=WHITE, pos=(0, 0, 0), exponent=.42, taper=0.0):
        self.shape(body, name, rounded_shell(extents, exponent, taper), rgba, pos)

    def axle(self, body, name, radius, length, rgba=DARK, pos=(0, 0, 0), axis="y"):
        quat = {"z": (1, 0, 0, 0), "y": (math.sqrt(.5), math.sqrt(.5), 0, 0),
                "x": (math.sqrt(.5), 0, math.sqrt(.5), 0)}[axis]
        self.shape(body, name, trimesh.creation.cylinder(radius=radius, height=length, sections=40), rgba, pos, quat)

    def collider(self, body, length, radius=.025, pos=(0, 0, 0)):
        # Robot colliders see the floor; self-collision is disabled for this
        # supported concept prototype. Visual shells contribute no contact.
        ET.SubElement(body, "geom", type="capsule", size=numbers([radius]),
                      fromto=numbers([*pos, pos[0], pos[1], pos[2] - length]),
                      contype="2", conaffinity="1", group="3")


def build():
    b = Builder()
    pelvis = b.body(b.world, "pelvis", (0, 0, .82), mass=2.4, inertia=.025)
    b.shell(pelvis, "pelvis_shell", (.19, .29, .18), pos=(.005, 0, -.015), taper=.25)
    b.shell(pelvis, "pelvis_inset", (.016, .115, .13), DARK, pos=(.105, 0, -.013), taper=.15)

    waist = pelvis
    for name, axis, limits in (("yaw", (0, 0, 1), (-.65, .65)),
                               ("roll", (1, 0, 0), (-.40, .40)),
                               ("pitch", (0, 1, 0), (-.45, .45))):
        waist = b.body(waist, f"waist_{name}_link", (0, 0, .04) if name == "yaw" else (0, 0, 0), mass=.25)
        b.joint(waist, f"waist_{name}_joint", axis, limits, kp=250, kv=20)
    for i in range(3):
        b.shell(waist, f"waist_ring_{i}", (.15, .20, .048), DARK, pos=(0, 0, .045 + i * .05))
    torso = b.body(waist, "torso", mass=4.4, com=(0, 0, .315), inertia=.07)
    b.shell(torso, "torso_shell", (.22, .335, .34), pos=(0, 0, .315), taper=.14)
    b.shell(torso, "chest_bezel", (.013, .028, .070), DARK, pos=(.115, 0, .338))
    b.shell(torso, "chest_light", (.016, .014, .055), CYAN, pos=(.122, 0, .338))
    b.shell(torso, "back_service_panel", (.014, .130, .145), DARK, pos=(-.116, 0, .355))
    for i in range(3):
        b.shell(torso, f"back_vent_{i}", (.012, .065, .006), VISOR, pos=(-.117, 0, .225 - i * .012))
    b.axle(torso, "neck", .044, .09, METAL, (0, 0, .525), "z")
    for h in (.499, .525, .551):
        b.axle(torso, "neck_ring", .047, .008, DARK, (0, 0, h), "z")
    head = b.body(torso, "head", (0, 0, .65), mass=1.1, inertia=.004)
    b.shell(head, "head_shell", (.190, .205, .205), exponent=.53)
    b.shell(head, "visor", (.015, .176, .070), VISOR, pos=(.096, 0, .015), exponent=.5)
    for side in (-1, 1):
        b.axle(head, "eye_light", .013, .008, CYAN, (.106, side * .043, .015), "x")
        b.axle(head, "ear_outer", .045, .017, WHITE, (0, side * .109, 0))
        b.axle(head, "ear_dark", .034, .019, DARK, (0, side * .116, 0))
        b.axle(head, "ear_cyan", .027, .021, CYAN, (0, side * .121, 0))
        b.axle(head, "ear_center", .023, .023, METAL, (0, side * .125, 0))

    for side, sign in (("left", 1), ("right", -1)):
        arm = b.body(torso, f"{side}_shoulder_pitch_link", (0, sign * .226, .432), mass=.25)
        b.joint(arm, f"{side}_shoulder_pitch_joint", (0, 1, 0), (-2.6, 1.2))
        b.axle(arm, "shoulder_mechanism", .053, .096, DARK)
        arm = b.body(arm, f"{side}_shoulder_roll_link", mass=.18)
        b.joint(arm, f"{side}_shoulder_roll_joint", (1, 0, 0), (-1.6, 1.6))
        arm = b.body(arm, f"{side}_shoulder_yaw_link", mass=.8, com=(0, 0, -.12), inertia=.006)
        b.joint(arm, f"{side}_shoulder_yaw_joint", (0, 0, 1), (-1.8, 1.8))
        b.shell(arm, "shoulder_cap", (.115, .09, .11), pos=(0, sign * .027, -.015), exponent=.55)
        b.shell(arm, "upper_arm_shell", (.085, .088, .180), pos=(0, 0, -.145), taper=.10, exponent=.5)
        b.collider(arm, .21, radius=.030)
        arm = b.body(arm, f"{side}_elbow_link", (0, 0, -.252), mass=.75, com=(0, 0, -.11), inertia=.005)
        b.joint(arm, f"{side}_elbow_joint", (0, -1, 0), (-.15, 2.4))
        b.axle(arm, "elbow", .039, .095, DARK)
        b.shell(arm, "forearm_shell", (.090, .085, .181), pos=(0, 0, -.128), taper=.12, exponent=.5)
        b.collider(arm, .20, radius=.027)
        for name, axis in (("roll", (1, 0, 0)), ("pitch", (0, 1, 0)), ("yaw", (0, 0, 1))):
            arm = b.body(arm, f"{side}_wrist_{name}_link", (0, 0, -.23) if name == "roll" else (0, 0, 0), mass=.06, inertia=.00015)
            b.joint(arm, f"{side}_wrist_{name}_joint", axis, (-1.4, 1.4), kp=45, kv=3)
        b.axle(arm, "wrist", .027, .035, METAL, (0, 0, -.003), "z")
        b.shell(arm, "palm", (.032, .061, .070), pos=(.003, 0, -.050), exponent=.55)
        for finger in range(4):
            yy = (finger - 1.5) * .016
            length = .046 if finger in (1, 2) else .037
            b.shell(arm, f"finger_{finger}", (.014, .012, length), METAL, pos=(.010, yy, -.090 - length / 2), exponent=.65)
            b.axle(arm, "finger_knuckle", .007, .012, DARK, (.010, yy, -.091))
        b.shell(arm, "thumb", (.023, .015, .045), METAL, pos=(.021, -sign * .044, -.070), exponent=.75)
        ET.SubElement(arm, "site", name=f"{side}_hand_tip", pos="0.008 0 -0.12", size="0.009", rgba="0.12 0.8 0.7 1")

        leg = pelvis
        for name, axis, limits in (("pitch", (0, 1, 0), (-1.8, 1.5)),
                                   ("roll", (1, 0, 0), (-.65, .65)),
                                   ("yaw", (0, 0, 1), (-.9, .9))):
            leg = b.body(leg, f"{side}_hip_{name}_link", (0, sign * .102, -.035) if name == "pitch" else (0, 0, 0), mass=.3)
            b.joint(leg, f"{side}_hip_{name}_joint", axis, limits, kp=240, kv=18)
        b.axle(leg, "hip", .062, .09, DARK)
        thigh = b.body(leg, f"{side}_thigh", mass=2.5, com=(0, 0, -.15), inertia=.025)
        b.shell(thigh, "thigh_shell", (.135, .118, .273), pos=(0, 0, -.17), taper=.17, exponent=.46)
        b.collider(thigh, .30, radius=.04)
        knee = b.body(thigh, f"{side}_knee_link", (0, 0, -.33), mass=1.9, com=(0, 0, -.15), inertia=.025)
        b.joint(knee, f"{side}_knee_joint", (0, 1, 0), (-.12, 2.3), kp=240, kv=18)
        b.axle(knee, "knee", .048, .13, DARK)
        b.shell(knee, "shin_shell", (.12, .106, .280), pos=(.007, 0, -.171), taper=.09, exponent=.48)
        b.collider(knee, .30, radius=.038)
        ankle = b.body(knee, f"{side}_ankle_pitch_link", (0, 0, -.345), mass=.2)
        b.joint(ankle, f"{side}_ankle_pitch_joint", (0, 1, 0), (-.75, .75), kp=200, kv=16)
        ankle = b.body(ankle, f"{side}_ankle_roll_link", mass=.65, com=(.025, 0, -.055), inertia=.003)
        b.joint(ankle, f"{side}_ankle_roll_joint", (1, 0, 0), (-.45, .45), kp=200, kv=16)
        b.axle(ankle, "ankle", .034, .11, METAL)
        b.shell(ankle, "sole", (.228, .118, .041), DARK, pos=(.035, 0, -.085), exponent=.35)
        b.shell(ankle, "foot_top", (.191, .109, .060), WHITE, pos=(.045, 0, -.060), exponent=.5)
        ET.SubElement(ankle, "geom", type="box", size="0.11 0.055 0.019", pos=".035 0 -.087", contype="2", conaffinity="1", group="3")

    mesh_dir = HERE / "meshes"
    mesh_dir.mkdir(exist_ok=True)
    for name, mesh in b.meshes.items():
        (mesh_dir / f"{name}.obj").write_text(mesh.export(file_type="obj").rstrip() + "\n")
    ET.indent(b.root)
    ET.ElementTree(b.root).write(HERE / "model.xml", encoding="unicode")
    model = mujoco.MjModel.from_xml_path(str(HERE / "model.xml"))
    data = mujoco.MjData(model)
    home = {}
    for side, sign in (("left", 1), ("right", -1)):
        home[f"{side}_shoulder_roll_joint"] = sign * .25
        home[f"{side}_shoulder_pitch_joint"] = -.02
        home[f"{side}_elbow_joint"] = .10
    for name, value in home.items():
        data.qpos[model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)]] = value
    mujoco.mj_forward(model, data)
    scene = trimesh.Scene(base_frame="world")
    # MJCF is Z-up; portable glTF uses Y-up with a single root transform.
    scene.graph.update(frame_from="world", frame_to="robot_z_up", matrix=trimesh.transformations.rotation_matrix(-math.pi / 2, [1, 0, 0]))
    for body_id in range(1, model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
        parent_id = model.body_parentid[body_id]
        parent_name = "robot_z_up" if parent_id == 0 else mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, parent_id)
        matrix = np.eye(4)
        matrix[:3, :3] = data.xmat[body_id].reshape(3, 3)
        matrix[:3, 3] = data.xpos[body_id]
        if parent_id:
            parent = np.eye(4)
            parent[:3, :3] = data.xmat[parent_id].reshape(3, 3)
            parent[:3, 3] = data.xpos[parent_id]
            matrix = np.linalg.inv(parent) @ matrix
        scene.graph.update(frame_from=parent_name, frame_to=name, matrix=matrix)
    for item in b.instances:
        mesh = b.meshes[item["mesh"]].copy()
        mesh.visual.face_colors = np.array(item["rgba"]) * 255
        transform = trimesh.transformations.quaternion_matrix(item["quat"])
        transform[:3, 3] = item["pos"]
        scene.add_geometry(mesh, node_name=item["name"], geom_name=item["name"],
                           parent_node_name=item["body"], transform=transform)
    scene.export(HERE / "concept01.glb", file_type="glb")
    workspace = {"right": {"min": [.16, -.54, .88], "max": [.47, -.14, 1.36], "default": [.32, -.31, 1.13]},
                 "left": {"min": [.16, .14, .88], "max": [.47, .54, 1.36], "default": [.32, .31, 1.13]}}
    metadata = {"id": "concept01", "label": "Original · Concept 01", "source_kind": "original-procedural",
                "source_revision": "design-v1", "description": "Original image-guided 3D concept; estimated dimensions and physics; supported pelvis; self-collision disabled.",
                "reference_image": "turnaround.png", "view_center": [0, 0, .86], "view_distance": 2.4,
                "workspace": workspace, "home": home}
    (HERE / "model.json").write_text(json.dumps(metadata, indent=2) + "\n")
    report = {"method": "authored superellipsoid/cylinder geometry guided by generated multi-view reference; not learned image reconstruction",
              "image_generator": "built-in image_gen", "reference_sha256": hashlib.sha256((HERE / "turnaround.png").read_bytes()).hexdigest(),
              "model_sha256": hashlib.sha256((HERE / "model.xml").read_bytes()).hexdigest(),
              "glb_sha256": hashlib.sha256((HERE / "concept01.glb").read_bytes()).hexdigest(),
              "mujoCo_version": mujoco.__version__, "trimesh_version": trimesh.__version__,
              "joints": model.njnt, "actuators": model.nu, "estimated_mass_kg": float(model.body_mass.sum()),
              "mesh_assets": len(b.meshes), "visual_parts": len(b.instances), "triangles": sum(len(mesh.faces) for mesh in scene.geometry.values()),
              "gltf_y_up_bounds_m": scene.bounds.tolist(), "assumptions": ["Scale and all physical parameters are estimates", "Fingers are fixed geometric pieces, not actuated", "No walking, balance or self-collision validation", "GLB preserves named body hierarchy in reference pose; MJCF contains actuation"]}
    (HERE / "build-report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    build()
