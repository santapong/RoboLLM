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

**Metric scale for free — put the ChArUco mat under the object.** One-time:
`python3 scale_mat.py make -o scale_mat.png`, print at 100% ("actual size"),
measure a square with a ruler (should be 30 mm). Shoot with the mat visible
in most frames; `reconstruct_cpu.sh` then detects it, solves the true
mm-per-unit against the recovered camera poses, and writes
`scale.json` — after which `mesh_to_print.py` needs no `--height-mm`.
If your printout measured e.g. 29.5 mm, re-solve with
`python3 scale_mat.py solve --session ../assets/scan/mug --square-mm 29.5`.

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

## D. 3D printing / CAD-CAM — `mesh_to_print.py`
Any scanned mesh (Route A hull or Route C photogrammetry) → watertight,
true-scale, bed-oriented **STL** ready for a slicer or FreeCAD:

```bash
python3 mesh_to_print.py ../assets/scan/mug/mug_photo.ply --height-mm 95 --smooth 10
# -> mug_photo_print.stl  (watertight, Z=95mm, sitting on the bed at Z=0)
```

- **Scale**: uses the session's `scale.json` (ChArUco mat, automatic) when
  present; otherwise pass `--height-mm` (calliper the real object) — without
  either, the print would be a random size, so the tool refuses to guess.
- Repairs automatically: drops floating debris, fills holes, fixes normals; if
  still leaky it voxel-remeshes (guaranteed watertight, detail set by
  `--voxel-mm`). Prints dims, volume, and estimated solid-PLA grams.
- Best route per goal: **phone + Route C** for fidelity/organic shapes
  (concavities captured); **webcam + Route A** for fast watertight solids of
  convex objects (a cup scans as filled — fine for CAD reference, wrong for
  printing the cavity).
- CAD/CAM: STL is a mesh, not BREP — in FreeCAD use Part → Shape from mesh →
  refine, or remodel over the scan as a reference body. A scan never becomes
  clean STEP automatically.
- A scan missing whole regions (unscanned underside) comes back as a thin
  shell, not a solid — rescan with more coverage rather than fighting it.

## Files
| File | Role |
|------|------|
| `capture.py` | Webcam capture: snapshots or turntable; `--background` for cutout |
| `visual_hull.py` | CPU silhouette carving → watertight mesh (`.obj`/`.ply`) |
| `reconstruct.sh` | COLMAP photogrammetry (sparse on CPU, dense on GPU) |
| `reconstruct_cpu.sh` | Full photogrammetry in Docker: COLMAP sparse + OpenMVS dense, CPU-only |
| `mesh_to_urdf.py` | Any mesh → URDF link (visual + convex collision + inertia) |
| `mesh_to_print.py` | Any mesh → watertight true-scale STL for slicer / FreeCAD |
| `scale_mat.py` | ChArUco scale mat: `make` printable mat, `solve` metric scale → `scale.json` |

Scans are written under `../assets/scan/<session>/` and are git-ignored (they can
be large / are personal). URDF links land in `../assets/urdf/<name>/`.

License note: OpenMVS is **AGPL-3.0**. Route C only invokes it as an unmodified
Docker binary (its output data is unaffected) — never vendor or link OpenMVS
code into this Apache-2.0 repo.
