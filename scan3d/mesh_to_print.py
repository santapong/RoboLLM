#!/usr/bin/env python3
"""mesh_to_print.py — scanned mesh -> watertight, true-scale STL for 3D printing / CAD.

The common tail for BOTH cameras: phone (Route C photogrammetry) and webcam
(Route A visual hull). Slicers and CAD refuse what a raw scan produces, so:

  1. keep the largest connected component (drops floaters / background scraps)
  2. drop degenerate + duplicate faces, fix winding/normals
  3. fill holes; if still leaky, voxel-remesh (marching cubes) — GUARANTEES
     a watertight solid at the cost of some detail (tune --voxel-mm)
  4. scale to real millimetres — photogrammetry has NO absolute scale, so you
     MUST pass a measured dimension: --height-mm (calliper the real object)
  5. report dims / volume / watertightness, export binary STL

CAD/CAM note: STL is a mesh, not BREP. For parametric CAD, import the STL in
FreeCAD (Part -> Shape from mesh -> refine) or use it as a reference body to
remodel over. A scan never becomes clean STEP automatically.

Usage:
  python3 mesh_to_print.py ../assets/scan/mug/mug_photo.ply --height-mm 95
  python3 mesh_to_print.py ../assets/scan/mug/mug_hull.obj  --height-mm 95 --smooth 10

Deps: pip install trimesh numpy scipy networkx scikit-image  (repo venv has all;
      elsewhere use -c constraints.txt to keep the numpy 1.26.4 pin)
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import trimesh


def largest_component(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    parts = mesh.split(only_watertight=False)
    if len(parts) <= 1:
        return mesh
    best = max(parts, key=lambda m: len(m.faces))
    print(f"  components: {len(parts)} -> kept largest ({len(best.faces)} faces)")
    return best


def basic_repair(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    mesh.update_faces(mesh.nondegenerate_faces())
    mesh.update_faces(mesh.unique_faces())
    mesh.remove_unreferenced_vertices()
    trimesh.repair.fix_winding(mesh)
    trimesh.repair.fix_normals(mesh)
    trimesh.repair.fill_holes(mesh)
    return mesh


def voxel_remesh(mesh: trimesh.Trimesh, pitch: float) -> trimesh.Trimesh:
    print(f"  not watertight after repair -> voxel remesh at {pitch:.3g} (mesh units)")
    vox = mesh.voxelized(pitch).fill()
    out = vox.marching_cubes
    out.apply_scale(pitch)                  # marching cubes is in voxel-index space
    out.apply_translation(vox.translation)
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("mesh", help=".ply/.obj/.stl from Route A or C")
    ap.add_argument("--height-mm", type=float, default=None,
                    help="real measured height (Z extent) in mm; not needed if the "
                         "session has scale.json from scale_mat.py")
    ap.add_argument("--scale-json", default=None,
                    help="path to scale.json (default: auto-detect next to the mesh)")
    ap.add_argument("--smooth", type=int, default=0, metavar="N",
                    help="N Laplacian smoothing iterations (photogrammetry noise; try 5-15)")
    ap.add_argument("--voxel-mm", type=float, default=0.4,
                    help="voxel size in OUTPUT mm for the watertight fallback remesh (default 0.4)")
    ap.add_argument("-o", "--out", default=None, help="output STL path (default <input>_print.stl)")
    args = ap.parse_args()

    src = Path(args.mesh)
    mesh = trimesh.load(src, force="mesh")
    print(f"loaded {src.name}: {len(mesh.vertices)} verts, {len(mesh.faces)} faces")

    mesh = largest_component(mesh)
    mesh = basic_repair(mesh)

    if args.smooth > 0:
        trimesh.smoothing.filter_laplacian(mesh, iterations=args.smooth)
        print(f"  smoothed: {args.smooth} Laplacian iterations")

    # scale to real millimetres: prefer the solved ChArUco scale, else --height-mm
    scale_path = Path(args.scale_json) if args.scale_json else src.parent / "scale.json"
    if scale_path.exists():
        mm_per_unit = json.loads(scale_path.read_text())["mm_per_unit"]
        print(f"  metric scale from {scale_path.name}: {mm_per_unit:.4f} mm/unit")
        mesh.apply_scale(mm_per_unit)
    elif args.height_mm is not None:
        z_extent = mesh.extents[2]
        if z_extent <= 0:
            print("error: mesh has zero Z extent", file=sys.stderr)
            return 1
        mesh.apply_scale(args.height_mm / z_extent)
    else:
        print("error: no scale.json found and no --height-mm given — the mesh has "
              "no absolute scale. Shoot with the ChArUco mat (scale_mat.py make) "
              "or measure the object.", file=sys.stderr)
        return 1

    if not mesh.is_watertight:
        target_z = mesh.extents[2]  # already in true mm at this point
        mesh = voxel_remesh(mesh, pitch=args.voxel_mm)
        mesh = basic_repair(mesh)
        mesh.apply_scale(target_z / mesh.extents[2])  # remesh pads by a voxel

    # sit flat on the print bed: min corner at Z=0, centred in XY
    minc, _ = mesh.bounds
    center = mesh.bounds.mean(axis=0)
    mesh.apply_translation([-center[0], -center[1], -minc[2]])

    out = Path(args.out) if args.out else src.with_name(src.stem + "_print.stl")
    mesh.export(out)

    dims = mesh.extents
    print(f"\nwatertight : {mesh.is_watertight}")
    print(f"dims (mm)  : {dims[0]:.1f} x {dims[1]:.1f} x {dims[2]:.1f}")
    if mesh.is_watertight:
        vol_cm3 = mesh.volume / 1000.0
        print(f"volume     : {vol_cm3:.1f} cm^3  (~{vol_cm3 * 1.24:.0f} g solid PLA)")
    print(f"faces      : {len(mesh.faces)}")
    print(f"wrote      : {out}")
    if not mesh.is_watertight:
        print("WARNING: still not watertight — lower --voxel-mm or clean in MeshLab/Blender",
              file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
