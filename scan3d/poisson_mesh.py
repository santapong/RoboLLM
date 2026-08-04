#!/usr/bin/env python3
"""poisson_mesh.py — dense point cloud -> smooth watertight mesh (Screened Poisson).

Alternative to OpenMVS ReconstructMesh when its output is noisy or the voxel
fallback in mesh_to_print.py eats detail. Runs Open3D's Screened Poisson with
density-based trimming: Poisson always closes the surface, but it hallucinates
low-density "bubbles" far from real points — trimming the lowest-density
vertices removes them, then we keep the largest component.

Runs inside the scan3d/poisson Docker image (see poisson.Dockerfile) because
Open3D has no wheel for every host Python. Direct use:

  docker build -t scan3d/poisson -f poisson.Dockerfile .
  docker run --rm -v "$SESS:/work" scan3d/poisson \
      /work/scene_dense.ply -o /work/scene_mesh_poisson.ply

Deps (in-image): numpy, open3d (MIT).
"""
import argparse
import sys

import numpy as np
import open3d as o3d


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("cloud", help="dense point cloud .ply (from DensifyPointCloud)")
    ap.add_argument("-o", "--out", required=True, help="output mesh .ply")
    ap.add_argument("--depth", type=int, default=9,
                    help="Poisson octree depth: 8 fast/coarse, 9 default, 10+ fine/slow")
    ap.add_argument("--trim-quantile", type=float, default=0.0,
                    help="drop this fraction of lowest-density vertices. OFF by default: "
                         "Poisson output is watertight and trimming opens holes; the "
                         "largest-component filter already removes detached bubbles. "
                         "Use ~0.02 only if hallucinated surface stays ATTACHED to the object")
    args = ap.parse_args()

    pcd = o3d.io.read_point_cloud(args.cloud)
    n = len(pcd.points)
    if n < 1000:
        print(f"error: only {n} points — not enough for Poisson", file=sys.stderr)
        return 1
    print(f"{n} points loaded")

    if not pcd.has_normals():
        print("estimating normals (cloud had none)")
        pcd.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(
                radius=np.linalg.norm(pcd.get_max_bound() - pcd.get_min_bound()) / 100,
                max_nn=30))
        pcd.orient_normals_consistent_tangent_plane(30)

    print(f"Screened Poisson, depth={args.depth}")
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=args.depth)

    if args.trim_quantile > 0:
        dens = np.asarray(densities)
        keep = dens >= np.quantile(dens, args.trim_quantile)
        mesh.remove_vertices_by_mask(~keep)
        print(f"trimmed {int((~keep).sum())} low-density vertices "
              f"(bottom {args.trim_quantile:.0%}) — may open holes; "
              f"mesh_to_print.py will close them")

    # largest connected component only
    cluster, counts, _ = mesh.cluster_connected_triangles()
    cluster = np.asarray(cluster)
    counts = np.asarray(counts)
    if len(counts) > 1:
        mesh.remove_triangles_by_mask(cluster != counts.argmax())
        mesh.remove_unreferenced_vertices()
        print(f"components: {len(counts)} -> kept largest")

    mesh.compute_vertex_normals()
    o3d.io.write_triangle_mesh(args.out, mesh)
    print(f"wrote {args.out}: {len(mesh.vertices)} verts, "
          f"{len(mesh.triangles)} tris, watertight={mesh.is_watertight()}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
