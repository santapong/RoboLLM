# Original humanoid concept 01

An original humanoid created from the [generated front/side/rear/3/4 sheet](turnaround.png).
The reference was made with the **built-in image_gen tool**, using the exact
[saved prompt](prompt.txt). The model was then authored in Python as separate
rounded shells, joint housings, fingers and feet following that reference.
No mesh, inertia or kinematic file was copied from G1 or H1.

This is an **image-guided procedural reconstruction**, not an automatic neural
image-to-3D conversion or a metrically recovered robot. The reference does not
contain enough information to establish true joint axes, mass or inertia.

## Deliverables

- [concept01.glb](concept01.glb): portable 3D model, Y-up, metres, named body
  hierarchy, 74 visual parts/66,240 triangles. Geometry is grouped into separate
  parts in a reference A-pose; GLB has no animation clip or skin deformation.
- [model.xml](model.xml) and [meshes/](meshes/): MuJoCo robot with 29 actuated
  hinge joints. The surrounding viewer adds its stand, floor and reach target.
- [model.json](model.json): model identity, per-arm workspace, home pose and
  view framing for the gallery.
- [Front](renders/front.png), [side](renders/side.png), [rear](renders/rear.png),
  [3/4](renders/three-quarter.png): actual orthographic renders from the model,
  distinct from the generated design sheet.
- [Build report](build-report.json) and [verification](verification.json):
  hashes, mesh counts, geometry checks and four five-second simulated reaches.

The model is about 1.61 m high in its reference pose. Its 25.27 kg assigned mass,
link inertias, actuator gains, force limits and joint positions are **design
estimates**. Fingers are fixed geometry. Robot self-collision is disabled in
this supported prototype; there is no walking, balancing, grasping or physical
robot validation. Joint limits and floor contact geometry are explicit.

## Rebuild and inspect

From the repository root, with the existing simulation environment:

```bash
OPENBLAS_NUM_THREADS=1 /home/santapong/RoboLLM/.venv-vla-bed/bin/python \
  examples/humanoid_harness/models/concept01/build_model.py

MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 \
  /home/santapong/RoboLLM/.venv-vla-bed/bin/python \
  examples/humanoid_harness/models/concept01/verify_model.py

examples/humanoid_harness/run.sh --model concept01
```

The build uses MuJoCo 3.10.0 and trimesh 4.12.2. `build_model.py` contains the
editable dimensions and part construction, exports MJCF and a GLB hierarchy,
and records image/model hashes. `verify_model.py` reloads exported OBJ/GLB
geometry, checks closed positive-volume parts, runs both arms, and renders
matching views. The measured final reach errors were 9.74 and 14.29 mm per arm
for the two target locations, with no reported joint-limit or numerical fault.

Rendering lesson: with MuJoCo 3.10's free camera, set
`model.vis.global_.orthographic = 1` and `fovy` in metres. Setting only
`MjvCamera.orthographic` does not change the projection derived by
[the renderer](https://github.com/google-deepmind/mujoco/blob/3.10.0/src/engine/engine_vis_visualize.c).
The verifier also requests a framebuffer large enough for its 960×960 output.
