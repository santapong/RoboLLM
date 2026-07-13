# scan3d — turn your webcam into a 3D scanner

Two ways to get a 3D mesh from your (bad, that's fine!) laptop webcam. Both feed
`mesh_to_urdf.py`, so a scanned real object becomes a robot/sim part.

```
 webcam ──capture.py──▶ images ──┬─ visual_hull.py  (CPU, laptop)  ─┐
                                 │                                   ├─▶ mesh ──mesh_to_urdf.py──▶ URDF ──▶ PyBullet / Gazebo / FreeCAD
                                 └─ reconstruct.sh  (COLMAP, +GPU)  ─┘
```

## A. CPU visual hull — works on this laptop today
Best for a quick, watertight solid. Cannot capture concavities (a cup looks full).

```bash
cd ~/Desktop/robot-llm-loop/scan3d
../.venv/bin/python capture.py --background                 # 1) empty scene
../.venv/bin/python capture.py --turntable 36 --session mug # 2) rotate object 360°
../.venv/bin/python visual_hull.py --session mug --height-mm 95
../.venv/bin/python mesh_to_urdf.py ../assets/scan/mug/mug_hull.obj --name mug
```
How it works: each frame is a silhouette (shadow); the 3D shape is the
intersection of all shadows. Only needs the object's OUTLINE — perfect for a
low-res camera.

## B. Photogrammetry — higher fidelity (needs a GPU for the dense step)
COLMAP recovers camera poses + geometry from overlapping photos. Sparse runs on
CPU here; the dense surface needs CUDA → run it on your GCP/AWS GPU instance.

```bash
sudo apt install colmap          # one-time
../.venv/bin/python capture.py --session mug        # ~30–60 overlapping snaps
./reconstruct.sh mug             # sparse here; script prints the GPU dense steps
```

## Getting a good scan from a cheap webcam
Resolution matters far less than these:
- **Light**: bright, even, diffuse. Kill glare and hard shadows.
- **Background**: plain and contrasting. Always shoot a `--background` frame first.
- **Object**: matte and textured scans best. Shiny/transparent/plain-white is hard.
- **Turntable**: a lazy-Susan or a plate you turn in even steps; keep the object
  centered and fully in frame the whole way around.
- **Overlap** (photogrammetry): each photo should share ~70% with the last.

## Files
| File | Role |
|------|------|
| `capture.py` | Webcam capture: snapshots or turntable; `--background` for cutout |
| `visual_hull.py` | CPU silhouette carving → watertight mesh (`.obj`/`.ply`) |
| `reconstruct.sh` | COLMAP photogrammetry (sparse on CPU, dense on GPU) |
| `mesh_to_urdf.py` | Any mesh → URDF link (visual + convex collision + inertia) |

Scans are written under `../assets/scan/<session>/` and are git-ignored (they can
be large / are personal). URDF links land in `../assets/urdf/<name>/`.
