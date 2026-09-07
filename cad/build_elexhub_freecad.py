"""Run in FreeCAD's Python: JSON descriptor -> editable FCStd + STEP assembly.

Usage: ELEXHUB_CAD_DESCRIPTOR=/absolute/cad-description.json freecadcmd this.py
Boxes/cylinders have editable dimensions; perforated plates are editable sketches
and extrusions. No remote code or external CAD is imported.
"""

import hashlib
import json
import os
from pathlib import Path

import FreeCAD as App
import Import
import Part
import Sketcher  # noqa: F401 - registers editable sketch object types


def placement(center, basis):
    matrix = App.Matrix()
    for i in range(3):
        for j in range(3):
            setattr(matrix, f"A{i + 1}{j + 1}", basis[i][j])
        setattr(matrix, f"A{i + 1}4", center[i] * 1000)
    return App.Placement(matrix)


def make_solid(doc, group, solid, sheet, row):
    name = group.Name + "_" + solid["name"]
    kind = solid["kind"]
    basis = solid.get("basis", [[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    center = solid["center"]

    def dimension(item, prop, value, col):
        sheet.set(f"{col}{row}", f"{value * 1000:.6f} mm")
        item.setExpression(prop, f"Parameters.{col}{row}")

    if kind == "box":
        item = doc.addObject("Part::Box", name)
        local = [-v / 2 for v in solid["size"]]
        for col, prop, value in zip(
            ["B", "C", "D"], ["Length", "Width", "Height"], solid["size"]
        ):
            dimension(item, prop, value, col)
    elif kind == "cylinder":
        outer = doc.addObject("Part::Cylinder", name + "_outer")
        group.addObject(outer)
        dimension(outer, "Radius", solid["radius"], "B")
        dimension(outer, "Height", solid["length"], "C")
        local = [0, 0, -solid["length"] / 2]
        item = outer
        if solid["inner_radius"]:
            inner = doc.addObject("Part::Cylinder", name + "_bore")
            group.addObject(inner)
            dimension(inner, "Radius", solid["inner_radius"], "D")
            dimension(inner, "Height", solid["length"], "C")
            item = doc.addObject("Part::Cut", name)
            item.Base = outer
            item.Tool = inner
    elif kind == "plate":
        sketch = doc.addObject("Sketcher::SketchObject", name + "_outline")
        group.addObject(sketch)
        points = [App.Vector(x * 1000, y * 1000, 0) for x, y in solid["outline"]]
        for a, b in zip(points, points[1:] + points[:1]):
            sketch.addGeometry(Part.LineSegment(a, b), False)
        for hole in solid["holes"]:
            x, y = hole["center"]
            sketch.addGeometry(
                Part.Circle(
                    App.Vector(x * 1000, y * 1000, 0),
                    App.Vector(0, 0, 1),
                    hole["radius"] * 1000,
                ),
                False,
            )
        item = doc.addObject("Part::Extrusion", name)
        item.Base = sketch
        item.DirMode = "Normal"
        item.Solid = True
        dimension(item, "LengthFwd", solid["thickness"], "B")
        local = [0, 0, -solid["thickness"] / 2]
    else:
        raise ValueError("unsupported CAD solid " + kind)
    group.addObject(item)
    origin = [
        center[i] + sum(basis[i][j] * local[j] for j in range(3)) for i in range(3)
    ]
    item.Placement = placement(origin, basis)
    item.addProperty("App::PropertyBool", "AppearanceSolid").AppearanceSolid = True
    item.addProperty(
        "App::PropertyString", "ReferenceMaterial"
    ).ReferenceMaterial = solid.get("material", "unspecified")
    item.addProperty(
        "App::PropertyString", "DimensionStatus"
    ).DimensionStatus = "photo estimate; not measured"
    sheet.set(f"A{row}", item.Name)
    if getattr(App, "GuiUp", False):
        item.ViewObject.ShapeColor = tuple(solid["color"][:3])
        item.ViewObject.LineColor = (0.12, 0.12, 0.12)
        for dependency in item.OutList:
            if hasattr(dependency, "ViewObject"):
                dependency.ViewObject.Visibility = False
    return item


def build(descriptor_path):
    path = Path(descriptor_path).resolve()
    desc = json.loads(path.read_text())
    doc = App.newDocument("Elexhub")
    doc.Comment = f"{desc['status'].upper()} geometry. {desc['notice']} Profile {desc['profile_hash']}"
    sheet = doc.addObject("Spreadsheet::Sheet", "Parameters")
    sheet.set("A1", "Elexhub - " + desc["status"].upper())
    sheet.set(
        "A2", "Dimensions in mm; joint axes and servo ratings need bench verification."
    )
    row = 4
    export_objects = []
    originals = []
    for part in desc["parts"]:
        group = doc.addObject("App::Part", part["name"])
        group.Label = part["name"].replace("_", " ").title()
        matrix = App.Matrix()
        frame = part["frame"]
        for i in range(4):
            for j in range(4):
                setattr(
                    matrix,
                    f"A{i + 1}{j + 1}",
                    frame[i][j] * (1000 if j == 3 and i < 3 else 1),
                )
        group.Placement = App.Placement(matrix)
        if "joint" in part:
            group.addProperty("App::PropertyString", "JointName").JointName = part[
                "joint"
            ]
            group.addProperty(
                "App::PropertyVector", "JointAxis"
            ).JointAxis = App.Vector(*part["axis"])
        for solid in part["solids"]:
            originals.append(make_solid(doc, group, solid, sheet, row))
            row += 1
    sheet.setColumnWidth("A", 240)
    doc.recompute()
    expected_solids = [solid for part in desc["parts"] for solid in part["solids"]]
    for item, expected in zip(originals, expected_solids):
        shape = item.Shape.copy()
        shape.Placement = item.getGlobalPlacement()
        if not shape.isValid() or shape.Volume <= 0 or len(shape.Solids) != 1:
            raise RuntimeError("invalid CAD solid " + item.Name)
        if "world_mesh_bounds_m" in expected:
            for index, axis in enumerate("XYZ"):
                for end, suffix in enumerate(["Min", "Max"]):
                    mesh_bound = expected["world_mesh_bounds_m"][end][index] * 1000
                    if abs(getattr(shape.BoundBox, axis + suffix) - mesh_bound) > 0.15:
                        raise RuntimeError(
                            "CAD and mesh placements differ: " + item.Name
                        )
        tmp = doc.addObject("Part::Feature", "Export_" + item.Name)
        tmp.Shape = shape
        if getattr(App, "GuiUp", False):
            tmp.ViewObject.ShapeColor = item.ViewObject.ShapeColor
        export_objects.append(tmp)
    Import.export(export_objects, str(path.parent / "elexhub.step"))
    expected_volume = sum(obj.Shape.Volume for obj in export_objects)
    expected_bounds = Part.makeCompound([obj.Shape for obj in export_objects]).BoundBox
    for obj in export_objects:
        doc.removeObject(obj.Name)
    doc.recompute()
    doc.saveAs(str(path.parent / "elexhub.FCStd"))
    App.closeDocument(doc.Name)
    reopened = App.openDocument(str(path.parent / "elexhub.FCStd"))
    solids = [obj for obj in reopened.Objects if getattr(obj, "AppearanceSolid", False)]
    if len(solids) != len(export_objects) or any(
        not obj.Shape.isValid() for obj in solids
    ):
        raise RuntimeError("editable CAD failed reopen validation")
    if getattr(App, "GuiUp", False):
        palette = {
            solid.get("material", "unspecified"): solid["color"][:3]
            for part in desc["parts"]
            for solid in part["solids"]
        }
        for item in solids:
            if any(
                # FCStd stores display colours at 8-bit channel precision.
                abs(a - b) > 0.5 / 255 + 1e-6
                for a, b in zip(
                    item.ViewObject.ShapeColor, palette[item.ReferenceMaterial]
                )
            ):
                raise RuntimeError("CAD colors did not survive reopening")
    App.closeDocument(reopened.Name)
    imported = Part.read(str(path.parent / "elexhub.step"))
    if not imported.isValid() or len(imported.Solids) != len(solids):
        raise RuntimeError("STEP failed reopen/solid-count validation")
    if abs(imported.Volume - expected_volume) > max(0.01, expected_volume * 1e-6):
        raise RuntimeError("STEP volume differs from the editable CAD")
    for key in ["XMin", "YMin", "ZMin", "XMax", "YMax", "ZMax"]:
        if abs(getattr(imported.BoundBox, key) - getattr(expected_bounds, key)) > 1e-4:
            raise RuntimeError("STEP placements differ from CAD")
    (path.parent / "CAD_VALIDATION.json").write_text(
        json.dumps(
            {
                "status": desc["status"],
                "build_id": os.environ.get("ELEXHUB_CAD_BUILD_ID"),
                "descriptor_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                "freecad_version": App.Version(),
                "valid_solids": len(solids),
                "appearance_revision": desc.get("appearance", {}).get("revision"),
                "editable_features": "boxes, cylinders, bores, sketch outlines and plate extrusions",
                "profile_hash": desc["profile_hash"],
                "step_exported": True,
                "fcstd_saved": True,
                "fcstd_reopened": True,
                "step_reopened": True,
                "step_volume_mm3": imported.Volume,
                "global_placements_verified": True,
                "cad_mesh_bounds_tolerance_mm": 0.15,
                "colors_saved_and_reopened": bool(getattr(App, "GuiUp", False)),
            },
            indent=2,
        )
        + "\n"
    )


if "ELEXHUB_CAD_DESCRIPTOR" in os.environ:
    try:
        build(os.environ["ELEXHUB_CAD_DESCRIPTOR"])
        print("Elexhub editable CAD and STEP exported successfully")
    finally:
        if os.environ.get("ELEXHUB_CAD_QUIT") and getattr(App, "GuiUp", False):
            from PySide import QtCore, QtWidgets

            QtCore.QTimer.singleShot(0, QtWidgets.QApplication.instance().quit)
