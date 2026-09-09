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

REVISION = "elexhub-owner-photo-v2"
# Cosmetic dimensions only. Kinematic lengths, offsets and the jaw travel remain
# owned by the hardware profile. Replace these estimates with owner measurements
# independently; matching a photo does not establish the scale of the assembly.
ESTIMATED_VISUAL_MM = {
    "plate_thickness": 2.0,
    "upper_arm_outer_width": 51.0,
    "upper_arm_cheek_width": 28.0,
    "forearm_cradle_outer_width": 46.0,
    "forearm_cradle_depth": 18.0,
    "forearm_cradle_cheek_width": 30.0,
    "forearm_spacer_diameter": 12.0,
    "forearm_flange_diameter": 23.0,
    "forearm_flange_thickness": 3.0,
    "wrist_outer_width": 53.0,
    "turret_outer_width": 49.0,
    "turret_panel_width": 42.0,
    "bearing_disc_diameter": 18.0,
    "bearing_disc_thickness": 2.0,
    "servo_horn_diameter": 14.0,
    "mounting_hole_diameter": 3.2,
    "pivot_bore_diameter": 6.4,
    "bracket_access_hole_diameter": 10.0,
    "bracket_tongue_access_hole_diameter": 10.4,
    "bracket_tongue_width": 15.0,
    "bracket_tongue_length": 27.0,
    "bolt_head_diameter": 5.4,
    "bolt_head_height": 1.6,
    "bolt_socket_diameter": 2.2,
    "servo_case_body_height": 34.0,
    "servo_gear_cap_height": 4.0,
    "servo_back_cap_height": 2.4,
    "servo_mount_ear_span": 54.0,
    "servo_mount_ear_thickness": 2.2,
    "claw_palm_width": 52.0,
    "claw_palm_length": 44.0,
    "claw_plate_thickness": 3.0,
    "claw_knuckle_width": 16.0,
    "claw_tip_width": 8.0,
    "claw_link_width": 7.0,
    "claw_drive_disc_diameter": 16.0,
    "claw_idler_disc_diameter": 9.0,
    "claw_link_layer_pitch": 3.0,
    "claw_pivot_shaft_diameter": 2.6,
    "claw_pivot_spacer_diameter": 4.8,
    "wire_diameter": 0.96,
    "wire_pitch": 1.1,
    "cable_tie_width": 2.0,
    "cable_tie_outer_diameter": 4.5,
}
SERVO_REFERENCE_ENVELOPE_MM = [40.7, 19.7, 42.9]
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
    "checked_date": "2026-09-08",
    "variant": "owner's black metal kit, two-plate base, silver forearm spacer, linkage claw",
    "variant_confirmation": "user confirmed matching variant on 2026-09-07",
    "owner_photo_reference": {
        "source": "five owner-supplied robot-arm photographs",
        "received_date": "2026-09-08",
        "checked_date": "2026-09-08",
        "capture_date": None,
        "images_vendored": False,
        "photos": [
            {
                "id": "photo-1",
                "file": "1-Photo-1.jpg",
                "sha256": "b9c6d801a3251a4949eef3943550270525326bc0dfe132d0065e989dfccb936e",
                "use": "side silhouette, spacer flanges, wrist and attached leads",
            },
            {
                "id": "photo-2",
                "file": "2-Photo-2.jpg",
                "sha256": "1cedaf0320de20f9e61c84434858df372c76341a2b80a9a3de82a9666e5c29d9",
                "use": "perforated cradle faces, tube alignment and bearing hardware",
            },
            {
                "id": "photo-3",
                "file": "3-Photo-3.jpg",
                "sha256": "ad538abf3542867ac94c9668824f2f48a974d4340693db5edf1e20a37fdc2b63",
                "use": "overall side construction, upper-arm cheeks and compact base",
            },
            {
                "id": "photo-4",
                "file": "4-Photo-4.jpg",
                "sha256": "bac9f620e3636f8b17cfa09fc5a00f16b378e3e0777e5f975a264037fca5e2e8",
                "use": "top view, bracket widths, spacer ends and servo placement",
            },
            {
                "id": "photo-5",
                "file": "5-Photo-5.jpg",
                "sha256": "0061002cd1541067811a180a29ed6a3aad27256a3562d4509ec1ea00fadd5e0a",
                "use": "claw silhouette, layered linkage plates and front servo housing",
            },
        ],
    },
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
    "servo_reference_envelope_mm": SERVO_REFERENCE_ENVELOPE_MM,
    "installed_servo_model": "unverified; labels are unreadable in the owner photographs",
    "estimated_visual_parameters_mm": ESTIMATED_VISUAL_MM,
    "profile_derived_dimensions": {
        "base_footprint": "geometry.base_size_m; visual depth is 0.9 times profile depth",
        "joint_spans": "geometry.links[*].length_m; all joint frames remain unchanged",
        "forearm_spacer_length": "forearm length minus twice the cradle depth (64 mm at the unchanged 100 mm profile length)",
        "claw_opening_and_length": "geometry.gripper; unchanged 55 mm opening and 65 mm finger length at this revision",
    },
    "metric_accuracy": "unverified; photos are perspective references, not metrology",
    "physics": "unchanged provisional joint centres, dynamics and simplified collision proxies; silver spacer is a fixed visual component",
    "gripper": "refined linkage silhouette at canonical open pose; rigid visuals follow coupled sliding contact jaws, so closed-pose pivot alignment remains approximate",
    "cables": "fine attached servo leads and short ties; cosmetic rigid segments without loose tails or deformable cable simulation",
    "unresolved": "metric scale, hidden supports, servo identity, joint limits, masses and full claw linkage motion",
    "robot_only": True,
    "hardware_ready": False,
    "remote_media_vendored": False,
}


def _mm(name):
    return ESTIMATED_VISUAL_MM[name] * 0.001


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
            point = c + radius * np.array(
                [math.cos(math.radians(theta)), math.sin(math.radians(theta))]
            )
            # At the capsule limit (radius == width / 2), adjacent quarter-arcs
            # share a tangent point. A repeated point is harmless to earcut but
            # creates a zero-length LineSegment in the editable FreeCAD sketch.
            if not result or np.linalg.norm(point - result[-1]) > 1e-12:
                result.append(point.tolist())
    if len(result) > 1 and np.linalg.norm(np.asarray(result[0]) - result[-1]) < 1e-12:
        result.pop()
    return result


def _hole(x, y, radius=None):
    if radius is None:
        radius = _mm("mounting_hole_diameter") / 2
    return {"center": [x, y], "radius": radius}


def _plate(
    name, outline, center, holes=(), normal=(0, 0, 1), u=(1, 0, 0), thickness=None
):
    return _solid(
        name,
        "plate",
        center,
        basis=_basis(normal, u),
        outline=outline,
        holes=list(holes),
        thickness=_mm("plate_thickness") if thickness is None else thickness,
    )


def _bolts(prefix, points, normal):
    return [
        _cylinder(
            f"{prefix}_{i}",
            _mm("bolt_head_diameter") / 2,
            _mm("bolt_head_height"),
            point,
            normal,
            inner=_mm("bolt_socket_diameter") / 2,
        )
        for i, point in enumerate(points)
    ]


def _servo(name, output, normal=(0, 0, 1), u=(1, 0, 0)):
    """Standard-size case; output axis defines the visible horn centre."""
    basis = _basis(normal, u)
    output = np.asarray(output, float)

    def point(value):
        return output + basis @ value

    case_length, case_width, _ = np.asarray(SERVO_REFERENCE_ENVELOPE_MM) * 0.001
    solids = [
        _box(
            name + "_case",
            [case_length, case_width, _mm("servo_case_body_height")],
            point([-0.010, 0, -0.0225]),
            "servo",
            basis,
        ),
        _box(
            name + "_gear_cap",
            [case_length, case_width, _mm("servo_gear_cap_height")],
            point([-0.010, 0, -0.0035]),
            "cap",
            basis,
        ),
        _box(
            name + "_back_cap",
            [case_length, case_width, _mm("servo_back_cap_height")],
            point([-0.010, 0, -0.0407]),
            "cap",
            basis,
        ),
        _box(
            name + "_mount_ears",
            [
                _mm("servo_mount_ear_span"),
                case_width,
                _mm("servo_mount_ear_thickness"),
            ],
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


def _wire_segments(name, points, material):
    solids = []
    for i, (a, b) in enumerate(pairwise(points)):
        a, b = np.asarray(a, float), np.asarray(b, float)
        delta = b - a
        length = np.linalg.norm(delta)
        solids.append(
            _cylinder(
                f"{name}_{i}",
                _mm("wire_diameter") / 2,
                length + 0.0002,
                (a + b) / 2,
                delta / length,
                material,
            )
        )
    return solids


def _leads(name, length, width):
    """Short attached three-wire runs; no black sleeve or hanging cable tails."""
    points = np.array(
        [
            [-0.016, -width / 2 - 0.003, length * 0.08],
            [-0.022, -width / 2 - 0.004, length * 0.28],
            [-0.021, -width / 2 - 0.004, length * 0.64],
            [-0.017, -width / 2 - 0.003, length * 0.91],
        ]
    )
    solids = []
    for i, material in enumerate(["brown", "red", "orange"]):
        wire = points + [(i - 1) * _mm("wire_pitch"), 0, 0]
        solids += _wire_segments(name + "_" + material, wire, material)
    for i in [1, 2]:
        normal = points[i + 1] - points[i - 1]
        normal /= np.linalg.norm(normal)
        solids.append(
            _cylinder(
                f"{name}_tie_{i}",
                _mm("cable_tie_outer_diameter") / 2,
                _mm("cable_tie_width"),
                points[i],
                normal,
                "rubber",
                inner=0.00175,
            )
        )
    return solids


def _pivot_hardware(name, z, width):
    """A small drive horn opposite a larger bearing washer, as in the photos."""
    solids = []
    for side in [-1, 1]:
        radius = _mm("servo_horn_diameter" if side < 0 else "bearing_disc_diameter") / 2
        solids.append(
            _cylinder(
                f"{name}_{'horn' if side < 0 else 'bearing'}",
                radius,
                _mm("bearing_disc_thickness"),
                [0, side * (width / 2 + 0.0018), z],
                [0, 1, 0],
                inner=_mm("pivot_bore_diameter") / 2,
            )
        )
        solids += _bolts(
            f"{name}_pivot_{side}",
            [[0, side * (width / 2 + 0.0036), z]],
            [0, 1, 0],
        )
    return solids


def _yoke(name, length, width, cheek_width=None):
    """Paired upper-arm/wrist cheeks; the forearm has its own tube assembly."""
    cheek_width = cheek_width or _mm("upper_arm_cheek_width")
    outline = _round_rect(cheek_width, length + 0.024, 0.012, (0, length / 2))
    holes = [_hole(0, z, _mm("pivot_bore_diameter") / 2) for z in [0, length]]
    for z in [0.012, length - 0.012]:
        holes += [_hole(-0.007, z), _hole(0.007, z)]
    if length > 0.075:
        holes += [_hole(0, z, 0.0019) for z in np.linspace(0.032, length - 0.032, 3)]
    solids = [
        _plate(
            f"{name}_cheek_{side}",
            outline,
            [0, side * width / 2, 0],
            holes,
            normal=(0, -1, 0),
        )
        for side in [-1, 1]
    ]
    for i, z in enumerate([0, length]):
        solids += _pivot_hardware(f"{name}_{i}", z, width)
    solids.append(
        _plate(
            name + "_return_flange",
            _round_rect(0.024, width, 0.002),
            [0, 0, length * 0.45],
            [_hole(-0.007, -0.011), _hole(0.007, 0.011)],
        )
    )
    return solids + _leads(name + "_leads", length, width)


def _cradle_face(name, z, width):
    """Perforated cross plate and short rounded mounting tongue."""
    access = _mm("bracket_access_hole_diameter") / 2
    holes = [
        _hole(0, 0, access),
        _hole(0, -0.012, 0.0036),
        _hole(0, 0.012, 0.0036),
    ]
    holes += [_hole(x, y) for x in [-0.010, 0.010] for y in [-0.016, 0.016]]
    solids = [
        _plate(
            name + "_face",
            _round_rect(_mm("forearm_cradle_cheek_width"), width, 0.003),
            [0, 0, z],
            holes,
        ),
        _plate(
            name + "_tongue",
            _round_rect(
                _mm("bracket_tongue_width"),
                _mm("bracket_tongue_length"),
                0.006,
                (0, 0.006),
            ),
            [0, 0, z + 0.002],
            [
                # The overlay has clearance around the underlying access bore.
                # Equal rings at their contact face become nonmanifold when an
                # OBJ loader welds coincident vertices from these separate parts.
                _hole(0, 0, _mm("bracket_tongue_access_hole_diameter") / 2),
                _hole(-0.004, 0.014),
                _hole(0.004, 0.014),
            ],
        ),
    ]
    solids += _bolts(
        name + "_tongue_mount",
        [[x, 0.014, z + 0.0038] for x in [-0.004, 0.004]],
        [0, 0, 1],
    )
    return solids


def _forearm(length):
    width = _mm("forearm_cradle_outer_width")
    depth = min(_mm("forearm_cradle_depth"), length * 0.24)
    solids = []
    # The spacer is fixed cosmetic geometry. Its length tracks the existing
    # elbow-to-wrist span; this adds no joint or inferred telescoping motion.
    for i, (pivot, sign) in enumerate([(0, 1), (length, -1)]):
        outline = _round_rect(
            _mm("forearm_cradle_cheek_width"),
            depth + 0.022,
            0.010,
            (0, sign * (depth / 2 - 0.011)),
        )
        holes = [
            _hole(0, 0, _mm("pivot_bore_diameter") / 2),
            _hole(-0.008, sign * (depth - 0.005)),
            _hole(0.008, sign * (depth - 0.005)),
        ]
        for side in [-1, 1]:
            solids.append(
                _plate(
                    f"forearm_cradle_{i}_cheek_{side}",
                    outline,
                    [0, side * width / 2, pivot],
                    holes,
                    normal=(0, -1, 0),
                )
            )
        face_z = pivot + sign * depth
        solids += _cradle_face(f"forearm_cradle_{i}", face_z, width)
        solids += _pivot_hardware(f"forearm_cradle_{i}", pivot, width)
        solids.append(
            _cylinder(
                f"forearm_spacer_flange_{i}",
                _mm("forearm_flange_diameter") / 2,
                _mm("forearm_flange_thickness"),
                [0, 0, face_z + sign * 0.0025],
                inner=0.004,
            )
        )
        solids += _bolts(
            f"forearm_flange_{i}_mount",
            [
                [x, y, face_z + sign * 0.0047]
                for x, y in [(0.0085, 0), (-0.0085, 0), (0, 0.0085), (0, -0.0085)]
            ],
            [0, 0, 1],
        )
    solids.append(
        _cylinder(
            "forearm_silver_spacer",
            _mm("forearm_spacer_diameter") / 2,
            length - 2 * depth,
            [0, 0, length / 2],
        )
    )
    solids += _servo("wrist_pitch_servo", [0, -0.022, length], [0, -1, 0])
    return solids + _leads("forearm_leads", length, width)


def _turret(length):
    width = _mm("turret_outer_width")
    half = _mm("turret_panel_width") / 2
    crown = 0.014
    outline = [
        [half, length * 0.18],
        [half, length * 0.65],
        [crown, length],
    ]
    outline += [
        [crown * math.cos(t), length + crown * math.sin(t)]
        for t in np.linspace(0, math.pi, 13)[1:]
    ]
    outline += [[-half, length * 0.65], [-half, length * 0.18]]
    holes = [_hole(0, length, _mm("pivot_bore_diameter") / 2)]
    holes += [_hole(x, length * 0.38) for x in [-0.012, 0.012]]
    solids = [
        _plate(
            f"turret_saddle_panel_{side}",
            outline,
            [0, side * width / 2, 0],
            holes,
            normal=(0, -1, 0),
        )
        for side in [-1, 1]
    ]
    solids.append(
        _plate(
            "turret_yaw_mount",
            _round_rect(_mm("turret_panel_width"), width, 0.003),
            [0, 0, length * 0.60],
            [_hole(0, 0, 0.006)]
            + [_hole(x, y) for x in [-0.014, 0.014] for y in [-0.015, 0.015]],
        )
    )
    solids += _pivot_hardware("turret_shoulder", length, width)
    solids.append(_cylinder("yaw_horn", 0.012, 0.003, [0, 0, length * 0.60]))
    solids += _servo("shoulder_servo", [0, -0.022, length], [0, -1, 0])
    return solids + _leads("turret_leads", length, width)


def _base(g):
    width, depth, height = g["base_size_m"]
    foot = _round_rect(width, depth * 0.9, 0.007)
    holes = []
    for x in [-width * 0.37, width * 0.37]:
        for y in [-depth * 0.31, depth * 0.31]:
            holes += [_hole(x, y, 0.0027), _hole(x - 0.006, y), _hole(x + 0.006, y)]
    holes += [
        _hole(x, y) for x in [-0.017, 0.017] for y in [-depth * 0.31, depth * 0.31]
    ]
    solids = [
        _plate(f"base_plate_{i}", foot, [0, 0, z], holes)
        for i, z in enumerate([0.001, height - 0.001])
    ]
    # Folded supports replace the generic exposed corner posts. Their hidden
    # underside arrangement remains an estimate, recorded in REFERENCE.
    for i, y in enumerate([-depth * 0.19, depth * 0.19]):
        solids.append(
            _plate(
                f"base_folded_support_{i}",
                _round_rect(width * 0.64, height - 0.004, 0.002),
                [0, y, height / 2],
                normal=(0, -1, 0),
            )
        )
        solids += _bolts(
            f"base_support_mount_{i}",
            [[x, y, height + 0.0008] for x in [-width * 0.25, width * 0.25]],
            [0, 0, 1],
        )
    turret = g["links"][0]
    yaw_height = g["base_origin_m"][2] + turret["length_m"] * 0.60
    solids += _servo("base_yaw_servo", [0, 0, yaw_height])
    for i, material in enumerate(["brown", "red", "orange"]):
        solids += _wire_segments(
            "base_lead_" + material,
            [
                [-0.029 + i * _mm("wire_pitch"), 0.012, yaw_height - 0.033],
                [-0.036 + i * _mm("wire_pitch"), 0.020, height + 0.007],
                [-0.039 + i * _mm("wire_pitch"), 0.024, height + 0.002],
            ],
            material,
        )
    return solids


def _claw_palm(length):
    palm_width = _mm("claw_palm_width")
    palm_length = _mm("claw_palm_length")
    solids = [
        _plate(
            "claw_backplate",
            _round_rect(palm_width, palm_length, 0.008),
            [0.012, 0, length + 0.004],
            [_hole(y, z) for y in [-0.020, 0.020] for z in [-0.012, 0.014]],
            normal=(1, 0, 0),
            u=(0, 1, 0),
        ),
        _cylinder("tool_coupler", 0.008, 0.012, [0, 0, 0.006]),
    ]
    solids += _servo("gripper_servo", [0.010, 0.010, length], [1, 0, 0], [0, 1, 0])
    for y in [-0.010, 0.010]:
        for i, z in enumerate([length, length + 0.013]):
            solids.append(
                _cylinder(
                    f"claw_drive_disc_{y}_{i}",
                    _mm(
                        "claw_drive_disc_diameter"
                        if i == 0
                        else "claw_idler_disc_diameter"
                    )
                    / 2,
                    0.0025,
                    [0.01425 + i * _mm("claw_link_layer_pitch"), y, z],
                    [1, 0, 0],
                    "anodized",
                    inner=0.002,
                )
            )
    return solids


def _claw_fastener(name, yz, front, back, spacer_front=None):
    """Seat the head on its plate and bridge the visible stack with a shaft."""
    y, z = yz
    solids = _bolts(
        name,
        [[front + _mm("bolt_head_height") / 2, y, z]],
        [1, 0, 0],
    )
    solids.append(
        _cylinder(
            name + "_shaft",
            _mm("claw_pivot_shaft_diameter") / 2,
            front - back + 0.0002,
            [(front + back) / 2, y, z],
            [1, 0, 0],
        )
    )
    if spacer_front is not None:
        solids.append(
            _cylinder(
                name + "_spacer",
                _mm("claw_pivot_spacer_diameter") / 2,
                spacer_front - back,
                [(spacer_front + back) / 2, y, z],
                [1, 0, 0],
                inner=0.00155,
            )
        )
    return solids


def _claw_finger(grip, side):
    length = grip["finger_length_m"]
    delta = (grip["opening_m"] + grip["finger_width_m"]) / 2 - 0.010
    jaw_holes = [length * 0.43, length * 0.43 + 0.013, length * 0.94]
    solids = []
    for i, z in enumerate(jaw_holes[:2]):
        link_x = 0.0165 + i * _mm("claw_link_layer_pitch")
        start, end = np.array([-side * delta, i * 0.013]), np.array([0, z])
        direction = (end - start) / np.linalg.norm(end - start)
        basis = np.column_stack([[direction[1], -direction[0]], direction])
        span = np.linalg.norm(end - start)
        link_width = _mm("claw_link_width") * (1.28 if i == 0 else 1)
        outline = (
            np.asarray(
                _round_rect(
                    link_width, span + link_width, link_width / 2, (0, span / 2)
                )
            )
            @ basis.T
            + start
        ).tolist()
        solids.append(
            _plate(
                f"claw_link_{i}",
                outline,
                [link_x, 0, 0],
                [_hole(*start), _hole(*end)],
                normal=(1, 0, 0),
                u=(0, 1, 0),
            )
        )
        solids += _claw_fastener(
            f"claw_upper_pivot_{i}",
            start,
            front=link_x + _mm("plate_thickness") / 2,
            back=0.013,
        )
    # A shaped knuckle tapers into each narrow metal fingertip. These outlines
    # describe the photographed plates; the existing sliding jaw body remains.
    knuckle = _mm("claw_knuckle_width") / 2
    tip = _mm("claw_tip_width") / 2
    jaw = [
        [-knuckle * 0.60, length * 0.32],
        [knuckle * 0.55, length * 0.32],
        [knuckle, length * 0.39],
        [knuckle * 0.90, length * 0.50],
        [tip, length * 0.69],
        [tip, length * 0.97],
        [tip * 0.50, length],
        [-tip * 0.50, length],
        [-tip, length * 0.97],
        [-tip, length * 0.75],
        [-knuckle * 0.70, length * 0.58],
        [-knuckle, length * 0.49],
        [-knuckle * 0.95, length * 0.39],
    ]
    jaw = [[side * y, z] for y, z in jaw]
    if side < 0:
        jaw.reverse()
    solids.append(
        _plate(
            "claw_finger",
            jaw,
            [0.009, 0, 0],
            [_hole(0, z) for z in jaw_holes],
            normal=(1, 0, 0),
            u=(0, 1, 0),
            thickness=_mm("claw_plate_thickness"),
        )
    )
    for i, z in enumerate(jaw_holes):
        jaw_front = 0.009 + _mm("claw_plate_thickness") / 2
        if i < 2:
            link_x = 0.0165 + i * _mm("claw_link_layer_pitch")
            solids += _claw_fastener(
                f"jaw_pivot_{i}",
                [0, z],
                front=link_x + _mm("plate_thickness") / 2,
                back=jaw_front,
                spacer_front=link_x - _mm("plate_thickness") / 2,
            )
        else:
            solids += _claw_fastener(
                f"jaw_pivot_{i}", [0, z], front=jaw_front, back=0.0075
            )
    return solids


def parts(g):
    """Return robot-only visual solids in unchanged local body frames."""
    result = {"base": _base(g)}
    for index, link in enumerate(g["links"]):
        name, length = link["name"], link["length_m"]
        if index == 0:
            solids = _turret(length)
        elif index == 1:
            solids = _yoke(name, length, _mm("upper_arm_outer_width"))
            solids += _servo("elbow_servo", [0, -0.022, length], [0, -1, 0])
        elif index == 2:
            solids = _forearm(length)
        elif index == 3:
            solids = _yoke("wrist", length, _mm("wrist_outer_width"))
            solids += _servo("wrist_roll_servo", [0, 0, length])
            solids.append(_cylinder("roll_horn", 0.011, 0.003, [0, 0, length + 0.001]))
        else:
            solids = _claw_palm(length)
        result[name] = solids
    for side, name in [(-1, "finger_left"), (1, "finger_right")]:
        result[name] = _claw_finger(g["gripper"], side)
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
