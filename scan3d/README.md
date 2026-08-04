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

## C. Photogrammetry, fully local — Docker COLMAP + OpenMVS (CPU dense!)
Same fidelity goal as Route B, but the dense step uses OpenMVS instead of
COLMAP's CUDA-only stereo — so the WHOLE pipeline runs on this GPU-free
laptop, in Docker (no installs). Also takes phone photos or a phone video,
the same technique as the Orbiter rig (github.com/santapong/Orbiter) minus
the motorized turntable — you are the rig.

```bash
./reconstruct_cpu.sh mug                        # frames from capture.py
./reconstruct_cpu.sh mug ~/Pictures/mug/        # phone photos
./reconstruct_cpu.sh mug ~/Videos/mug.mp4       # phone video → 2 fps frames
../.venv/bin/python mesh_to_urdf.py ../assets/scan/mug/mug_photo.ply --name mug
```

Phone shooting: orbit a STATIC object in 2–3 height rings, ~10° between
shots, ≥60% overlap — do NOT rotate the object on a plate for this route
(a static background breaks feature matching). 40–80 photos; expect
10–40 min on CPU.

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
| `reconstruct_cpu.sh` | Full photogrammetry in Docker: COLMAP sparse + OpenMVS dense, CPU-only |
| `mesh_to_urdf.py` | Any mesh → URDF link (visual + convex collision + inertia) |

Scans are written under `../assets/scan/<session>/` and are git-ignored (they can
be large / are personal). URDF links land in `../assets/urdf/<name>/`.
