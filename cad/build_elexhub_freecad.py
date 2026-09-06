"""Run in FreeCAD's Python: JSON descriptor -> editable FCStd + STEP assembly.

Usage: ELEXHUB_CAD_DESCRIPTOR=/absolute/cad-description.json freecadcmd this.py
All boxes have editable length/width/height properties; the canonical profile
remains the regeneration source. No remote code or external CAD is imported.
"""

import json
import os
from pathlib import Path

import FreeCAD as App
import Part


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
            item = doc.addObject("Part::Box", part["name"] + "_" + solid["name"])
            group.addObject(item)
            size = [v * 1000 for v in solid["size"]]
            corner = [
                (c - s / 2) * 1000 for c, s in zip(solid["center"], solid["size"])
            ]
            for col, prop, val in zip(
                ["B", "C", "D"], ["Length", "Width", "Height"], size
            ):
                sheet.set(f"{col}{row}", f"{val:.6f} mm")
                item.setExpression(prop, f"Parameters.{col}{row}")
            sheet.set(f"A{row}", item.Name)
            item.Placement.Base = App.Vector(*corner)
            if getattr(App, "GuiUp", False):
                item.ViewObject.ShapeColor = tuple(solid["color"][:3])
            row += 1
    sheet.setColumnWidth("A", 240)
    doc.recompute()
    for group in [o for o in doc.Objects if o.TypeId == "App::Part"]:
        for item in group.Group:
            if hasattr(item, "Shape"):
                shape = item.Shape.copy()
                shape.Placement = item.getGlobalPlacement()
                if not shape.isValid() or shape.Volume <= 0:
                    raise RuntimeError("invalid CAD solid " + item.Name)
                tmp = doc.addObject("Part::Feature", "Export_" + item.Name)
                tmp.Shape = shape
                export_objects.append(tmp)
    Part.export(export_objects, str(path.parent / "elexhub.step"))
    expected_volume = sum(obj.Shape.Volume for obj in export_objects)
    expected_bounds = Part.makeCompound([obj.Shape for obj in export_objects]).BoundBox
    for obj in export_objects:
        doc.removeObject(obj.Name)
    doc.recompute()
    doc.saveAs(str(path.parent / "elexhub.FCStd"))
    App.closeDocument(doc.Name)
    reopened = App.openDocument(str(path.parent / "elexhub.FCStd"))
    boxes = [obj for obj in reopened.Objects if obj.TypeId == "Part::Box"]
    if len(boxes) != len(export_objects) or any(
        not obj.Shape.isValid() for obj in boxes
    ):
        raise RuntimeError("editable CAD failed reopen validation")
    App.closeDocument(reopened.Name)
    imported = Part.read(str(path.parent / "elexhub.step"))
    if not imported.isValid() or len(imported.Solids) != len(boxes):
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
                "freecad_version": App.Version(),
                "valid_solids": len(boxes),
                "profile_hash": desc["profile_hash"],
                "step_exported": True,
                "fcstd_saved": True,
                "fcstd_reopened": True,
                "step_reopened": True,
                "step_volume_mm3": imported.Volume,
                "global_placements_verified": True,
            },
            indent=2,
        )
        + "\n"
    )


if "ELEXHUB_CAD_DESCRIPTOR" in os.environ:
    build(os.environ["ELEXHUB_CAD_DESCRIPTOR"])
    print("Elexhub editable CAD and STEP exported successfully")
