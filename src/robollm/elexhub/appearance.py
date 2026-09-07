"""Original CAD reconstruction from the Elexhub black-metal kit photographs.

Visual dimensions are estimates, not measured replacements for the kinematic
profile. The same solids drive FreeCAD, URDF and MuJoCo visual meshes. Motor
limits, collision proxies and inertias remain explicitly provisional.
"""

from __future__ import annotations

import math
from collections import defaultdict
from itertools import pairwise

import mapbox_earcut
import numpy as np
import trimesh

REVISION = "elexhub-black-metal-photo-v1"
MATERIALS = {
    "anodized": [0.10, 0.115, 0.13, 1.0],
    "servo": [0.065, 0.072, 0.080, 1.0],
    "cap": [0.13, 0.145, 0.16, 1.0],
    "steel": [0.63, 0.67, 0.71, 1.0],
    "rubber": [0.035, 0.04, 0.045, 1.0],
    "red": [0.65, 0.055, 0.035, 1.0],
    "orange": [0.96, 0.49, 0.055, 1.0],
    "brown": [0.21, 0.12, 0.08, 1.0],
}
REFERENCE = {
    "revision": REVISION,
    "checked_date": "2026-09-07",
    "variant": "black metal kit, rectangular two-plate base, linkage claw",
    "variant_confirmation": "user confirmed matching variant on 2026-09-07",
    "sources": [
        {
            "url": "https://elexhub.com/shop/XvAAPZ",
            "use": "kit identity and photographs",
        },
        {
            "url": "https://elexhub.com/shop/KMPxZc",
            "use": "six MG996 motor variant listing",
        },
        {
            "url": "https://elexhub.com/images/items/XvAAPZ0.jpg",
            "use": "base, U brackets, exposed servos, fasteners and claw",
        },
        {
            "url": "https://elexhub.com/images/items/XvAAPZ1.jpg",
            "use": "front claw and servo arrangement",
        },
        {
            "url": "https://elexhub.com/images/items/XvAAPZ2.jpg",
            "use": "side structure, bearing discs and cable routing",
        },
        {
            "url": "https://towerpro.com.tw/product/mg996R/",
            "use": "nominal servo envelope reference only; user's servo model unverified",
        },
    ],
    "servo_reference_envelope_mm": [40.7, 19.7, 42.9],
    "estimated_visual_parameters_mm": {
        "plate_thickness": 2.0,
        "yoke_outer_width": 51.0,
        "bearing_disc_diameter": 18.0,
        "mounting_hole_diameter": 3.2,
        "base_plate_separation": 20.0,
        "claw_opening": 55.0,
        "claw_length": 65.0,
    },
    "metric_accuracy": "unverified; photos are perspective references, not metrology",
    "physics": "provisional joint centres and dynamics; claw opening/length revised to photo estimates; collision proxies remain simplified",
    "gripper": "linkage appearance at open pose; coupled sliding contact jaws remain an approximation",
    "cables": "illustrative local routing, not a deformable cable simulation",
    "hardware_ready": False,
    "remote_media_vendored": False,
}


def _basis(normal=(0, 0, 1), u=(1, 0, 0)):
    n = np.asarray(normal, float)
    n /= np.linalg.norm(n)
    x = np.asarray(u, float)
    x -= np.dot(x, n) * n
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(n, x), n])


def _solid(name, kind, center, material="anodized", basis=None, **values):
    return {
        "name": name,
        "kind": kind,
        "center": np.asarray(center, float).tolist(),
        "basis": (np.eye(3) if basis is None else np.asarray(basis)).tolist(),
        "material": material,
        "color": MATERIALS[material],
        **values,
    }


def _box(name, size, center, material="anodized", basis=None):
    return _solid(name, "box", center, material, basis, size=list(size))


def _cylinder(
    name, radius, length, center, normal=(0, 0, 1), material="steel", inner=0
):
    u = (1, 0, 0) if abs(normal[0]) < 0.9 else (0, 1, 0)
    return _solid(
        name,
        "cylinder",
        center,
        material,
        _basis(normal, u),
        radius=radius,
        length=length,
        inner_radius=inner,
    )


def _round_rect(width, height, radius, center=(0, 0)):
    result = []
    for x, y, angle in [(1, 1, 0), (-1, 1, 90), (-1, -1, 180), (1, -1, 270)]:
        c = np.array([x * (width / 2 - radius), y * (height / 2 - radius)]) + center
        for theta in np.linspace(angle, angle + 90, 7):
            result.append(
                (
                    c
                    + radius
                    * np.array(
                        [math.cos(math.radians(theta)), math.sin(math.radians(theta))]
                    )
                ).tolist()
            )
    return result


def _hole(x, y, radius=0.0016):
    return {"center": [x, y], "radius": radius}


def _plate(
    name, outline, center, holes=(), normal=(0, 0, 1), u=(1, 0, 0), thickness=0.002
):
    return _solid(
        name,
        "plate",
        center,
        basis=_basis(normal, u),
        outline=outline,
        holes=list(holes),
        thickness=thickness,
    )


def _bolts(prefix, points, normal):
    return [
        _cylinder(f"{prefix}_{i}", 0.0027, 0.0016, point, normal, inner=0.0011)
        for i, point in enumerate(points)
    ]


def _servo(name, output, normal=(0, 0, 1), u=(1, 0, 0)):
    """Standard-size case; output axis defines the visible horn centre."""
    basis = _basis(normal, u)
    output = np.asarray(output, float)

    def point(value):
        return output + basis @ value

    solids = [
        _box(
            name + "_case",
            [0.0407, 0.0197, 0.034],
            point([-0.010, 0, -0.0225]),
            "servo",
            basis,
        ),
        _box(
            name + "_gear_cap",
            [0.0407, 0.0197, 0.004],
            point([-0.010, 0, -0.0035]),
            "cap",
            basis,
        ),
        _box(
            name + "_back_cap",
            [0.0407, 0.0197, 0.0024],
            point([-0.010, 0, -0.0407]),
            "cap",
            basis,
        ),
        _box(
            name + "_mount_ears",
            [0.054, 0.0197, 0.0022],
            point([-0.010, 0, -0.008]),
            "servo",
            basis,
        ),
        _cylinder(name + "_shaft", 0.003, 0.005, point([0, 0, 0]), normal),
    ]
    for i, x in enumerate([-0.033, 0.013]):
        for j, y in enumerate([-0.006, 0.006]):
            solids += _bolts(f"{name}_mount_{i}_{j}", [point([x, y, -0.006])], normal)
    return solids


def _loom(name, points, radius=0.0028, material="rubber"):
    solids = []
    for i, (a, b) in enumerate(pairwise(points)):
        a, b = np.asarray(a, float), np.asarray(b, float)
        delta = b - a
        length = np.linalg.norm(delta)
        solids.append(
            _cylinder(
                f"{name}_{i}",
                radius,
                length + 0.0005,
                (a + b) / 2,
                delta / length,
                material,
            )
        )
    return solids


def _yoke(name, length, width=0.049):
    solids = []
    # Long U-shaped sheet-metal cheeks with pivot holes and a mounting pattern.
    outline = _round_rect(0.030, length + 0.026, 0.013, (0, length / 2))
    holes = [_hole(0, 0, 0.0032), _hole(0, length, 0.0032)]
    for z in [0.013, length - 0.013]:
        holes.extend([_hole(-0.007, z), _hole(0.007, z)])
    if length > 0.075:
        holes.extend(
            [_hole(0, z, 0.0022) for z in np.linspace(0.031, length - 0.031, 3)]
        )
    for side in [-1, 1]:
        normal = [0, -1, 0]
        solids.append(
            _plate(
                f"{name}_cheek_{side}", outline, [0, side * width / 2, 0], holes, normal
            )
        )
        for end in [0, length]:
            solids.append(
                _cylinder(
                    f"{name}_bearing_{side}_{end}",
                    0.009,
                    0.002,
                    [0, side * (width / 2 + 0.002), end],
                    [0, 1, 0],
                    inner=0.0031,
                )
            )
            solids += _bolts(
                f"{name}_pivot_{side}_{end}",
                [[0, side * (width / 2 + 0.0038), end]],
                [0, 1, 0],
            )
    solids.append(
        _plate(
            name + "_return_flange",
            _round_rect(0.028, width, 0.002),
            [0, 0, length * 0.48],
            [_hole(-0.007, -0.010), _hole(0.007, 0.010)],
        )
    )
    # A local loop leaves slack at the pivots; it is cosmetic, not simulated wire.
    z = np.linspace(0.008, length - 0.008, 10)
    path = [
        [-0.023 - 0.009 * math.sin(i * math.pi / 9), -0.030, h] for i, h in enumerate(z)
    ]
    solids += _loom(name + "_loom", path)
    for index, material in enumerate(["brown", "red", "orange"]):
        wire = [[p[0] - 0.002 - index * 0.0011, p[1] - 0.001, p[2]] for p in path]
        solids += _loom(name + "_" + material, wire, 0.00048, material)
    return solids


def parts(g):
    """Return solids in each MuJoCo body's local coordinates, at open gripper."""
    result = {
        name: []
        for name in [
            "base",
            *[link["name"] for link in g["links"]],
            "finger_left",
            "finger_right",
        ]
    }
    base = result["base"]
    width, depth, height = g["base_size_m"]
    foot = _round_rect(width, depth * 0.9, 0.007)
    holes = [
        _hole(x, y, 0.0025)
        for x in [-width * 0.40, width * 0.40]
        for y in [-depth * 0.34, 0, depth * 0.34]
    ]
    holes += [
        _hole(x, y) for x in [-0.018, 0, 0.018] for y in [-depth * 0.32, depth * 0.32]
    ]
    for i, z in enumerate([0.004, height - 0.001]):
        base.append(_plate(f"base_plate_{i}", foot, [0, 0, z], holes))
    for x in [-width * 0.32, width * 0.32]:
        for y in [-depth * 0.28, depth * 0.28]:
            base.append(
                _cylinder(
                    f"standoff_{x}_{y}",
                    0.0032,
                    height - 0.006,
                    [x, y, height / 2 + 0.001],
                    material="anodized",
                )
            )
            base.append(
                _cylinder(
                    f"foot_{x}_{y}", 0.006, 0.003, [x, y, 0.0015], material="rubber"
                )
            )
            base += _bolts(f"base_{x}_{y}", [[x, y, height + 0.0008]], [0, 0, 1])
    turret = g["links"][0]
    yaw_height = g["base_origin_m"][2] + turret["length_m"] * 0.60
    base += _servo("base_yaw_servo", [0, 0, yaw_height])
    base += _loom(
        "base_cable",
        [
            [-0.025, 0.012, yaw_height - 0.035],
            [-0.035, 0.021, 0.023],
            [-0.047, 0.026, 0.008],
            [-0.050, 0.012, 0.006],
        ],
    )

    for index, link in enumerate(g["links"]):
        name, length = link["name"], link["length_m"]
        solids = result[name]
        if index == 0:
            # Base yaw is a vertical axis; short fork carries the shoulder servo.
            solids += _yoke("turret", length)
            solids.append(_cylinder("yaw_horn", 0.012, 0.003, [0, 0, length * 0.60]))
            solids += _servo("shoulder_servo", [0, -0.022, length], [0, -1, 0])
        elif index in [1, 2]:
            solids += _yoke(name, length)
            next_name = "elbow" if index == 1 else "wrist_pitch"
            solids += _servo(next_name + "_servo", [0, -0.022, length], [0, -1, 0])
        elif index == 3:
            solids += _yoke("wrist", length, width=0.053)
            solids += _servo("wrist_roll_servo", [0, 0, length])
            solids.append(_cylinder("roll_horn", 0.011, 0.003, [0, 0, length + 0.001]))
        else:
            # Front plate and one transverse motor drive the photographed claw.
            solids.append(
                _plate(
                    "claw_backplate",
                    _round_rect(0.052, 0.032, 0.008),
                    [0.012, 0, length - 0.005],
                    [
                        _hole(-0.020, -0.010),
                        _hole(0.020, -0.010),
                        _hole(-0.020, 0.010),
                        _hole(0.020, 0.010),
                    ],
                    normal=(1, 0, 0),
                    u=(0, 1, 0),
                )
            )
            solids += _servo(
                "gripper_servo", [0.010, 0.010, length - 0.006], [1, 0, 0], [0, 1, 0]
            )
            solids.append(_cylinder("tool_coupler", 0.008, 0.012, [0, 0, 0.006]))
            for y in [-0.010, 0.010]:
                solids.append(
                    _cylinder(
                        f"claw_drive_disc_{y}",
                        0.009,
                        0.0025,
                        [0.014, y, length],
                        [1, 0, 0],
                        "anodized",
                        inner=0.002,
                    )
                )
                solids += _bolts(f"claw_drive_{y}", [[0.016, y, length]], [1, 0, 0])

    grip = g["gripper"]
    for side, name in [(-1, "finger_left"), (1, "finger_right")]:
        solids = result[name]
        length = grip["finger_length_m"]
        # Outline of the two-bar metal claw, based on the open pose in photos.
        delta = (grip["opening_m"] + grip["finger_width_m"]) / 2 - 0.010
        jaw_holes = [length * 0.43, length * 0.43 + 0.013, length * 0.94]
        for i, z in enumerate(jaw_holes[:2]):
            start, end = np.array([-side * delta, i * 0.013]), np.array([0, z])
            direction = (end - start) / np.linalg.norm(end - start)
            basis = np.column_stack([[direction[1], -direction[0]], direction])
            outline = (
                np.asarray(
                    _round_rect(
                        0.006,
                        np.linalg.norm(end - start) + 0.006,
                        0.0029,
                        (0, np.linalg.norm(end - start) / 2),
                    )
                )
                @ basis.T
                + start
            ).tolist()
            solids.append(
                _plate(
                    f"claw_link_{i}",
                    outline,
                    [0.013 + i * 0.003, 0, 0],
                    [_hole(*start), _hole(*end)],
                    normal=(1, 0, 0),
                    u=(0, 1, 0),
                )
            )
            solids += _bolts(
                f"claw_upper_pivot_{i}", [[0.018 + i * 0.003, *start]], [1, 0, 0]
            )
        jaw = _round_rect(
            grip["finger_width_m"] + 0.003, length * 0.66, 0.003, (0, length * 0.69)
        )
        solids.append(
            _plate(
                "claw_finger",
                jaw,
                [0.009, 0, 0],
                [_hole(0, z) for z in jaw_holes],
                normal=(1, 0, 0),
                u=(0, 1, 0),
                thickness=0.003,
            )
        )
        for z in jaw_holes:
            solids += _bolts(f"jaw_pivot_{z}", [[0.012, 0, z]], [1, 0, 0])
    return result


def mesh_for(solid):
    kind = solid["kind"]
    if kind == "box":
        mesh = trimesh.creation.box(extents=solid["size"])
    elif kind == "cylinder":
        if solid["inner_radius"]:
            mesh = trimesh.creation.annulus(
                solid["inner_radius"], solid["radius"], solid["length"], sections=24
            )
        else:
            mesh = trimesh.creation.cylinder(
                solid["radius"], solid["length"], sections=24
            )
    elif kind == "plate":
        rings = [np.asarray(solid["outline"], float)]
        for hole in solid["holes"]:
            angles = np.linspace(0, 2 * math.pi, 20, endpoint=False)
            rings.append(
                np.column_stack([np.cos(angles), np.sin(angles)]) * hole["radius"]
                + hole["center"]
            )
        vertices = np.concatenate(rings)
        ends = np.cumsum([len(ring) for ring in rings], dtype=np.uint32)
        faces = mapbox_earcut.triangulate_float64(vertices, ends).reshape(-1, 3)
        mesh = trimesh.creation.extrude_triangulation(
            vertices, faces, solid["thickness"]
        )
        mesh.apply_translation([0, 0, -solid["thickness"] / 2])
    else:
        raise ValueError("unknown appearance solid " + kind)
    pose = np.eye(4)
    pose[:3, :3] = solid["basis"]
    pose[:3, 3] = solid["center"]
    mesh.apply_transform(pose)
    return mesh


def export_meshes(all_parts, directory):
    records = {}
    for name, solids in all_parts.items():
        groups = defaultdict(list)
        for solid in solids:
            mesh = mesh_for(solid)
            if not mesh.is_watertight or mesh.volume <= 0:
                raise ValueError(
                    "invalid appearance mesh " + name + "/" + solid["name"]
                )
            groups[solid["material"]].append(mesh)
        records[name] = []
        for material, meshes in groups.items():
            mesh_name = f"{name}__{material}"
            trimesh.util.concatenate(meshes).export(directory / f"{mesh_name}.obj")
            records[name].append({"name": mesh_name, "material": material})
        # Keep the single mesh useful to other existing robot description tools.
        trimesh.util.concatenate(
            [m for meshes in groups.values() for m in meshes]
        ).export(directory / f"{name}.obj")
    return records
