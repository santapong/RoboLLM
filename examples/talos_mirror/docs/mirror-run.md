# talos_mirror — live mirroring runbook

Operational notes for running TALOS M4 live mirroring: your webcam-tracked
body drives the full-body mock robot (torso, both 7-DOF arms, 2-DOF head,
both 6-DOF legs). For the joint inventory and SRDF groups see
[joint-inventory.md](joint-inventory.md); for the retarget math (direct vs
qp) see the top-of-file docstrings in
`ros2_ws/src/talos_mirror/talos_mirror/retarget.py` and `qp_retarget.py`.

## Quick start

```bash
cd examples/talos_mirror
./docker/ros2-arm mirror synthetic       # deterministic full-body sweep, no camera
./docker/ros2-arm mirror                 # LIVE — your body drives the robot
./docker/ros2-arm stop                   # kill every arm container, always safe
```

`mirror` (live) opens RViz on `rviz/mirror.rviz`: the robot AND your tracked
skeleton (`/body/markers` + `human/*` TF), not the robot-only config the
mock bring-up ships on its own. Headless and override options:

```bash
./docker/ros2-arm mirror use_rviz:=false           # fully headless
./docker/ros2-arm mirror rviz_config:=/path/to.rviz  # a different RViz config
```

`rviz_config:=` is a launcher-level flag read by `docker/ros2-arm` itself
(there is no such launch argument in `mirror.launch.py`) — it picks which
config the container's own backgrounded `rviz2` opens; everything else
after `mirror` still forwards straight through to
`mirror.launch.py` (`preview:=true`, `mode:=qp` to opt into the
experimental QP wrist layer — direct copy is the default, etc. — see the
help text in `docker/ros2-arm` for the full list).

**Startup takes ~25 s**: RViz shows the robot immediately, but tracking
starts only after all six controllers come up (`start_delay` in
`mirror.launch.py`). Your skeleton appears and mirroring begins once that
delay elapses — the quiet first half-minute is normal, not a hang.

**move_group is OFF by default** (`use_moveit:=false`) on `mirror`/`sweep`
now — measured (see [TECHNICAL.md](../TECHNICAL.md)'s CPU profile section)
at a flat ~3-3.5% CPU tax that mirroring never uses (neither `track_node`
nor `mirror_node` calls `/compute_fk` or `/compute_ik`). Pass
`use_moveit:=true` if you want `retarget-bench --fk` to run against a
`mirror`/`sweep` container instead of `talos` (which still defaults
`use_moveit:=true`):

```bash
./docker/ros2-arm mirror synthetic use_moveit:=true use_rviz:=false
```

## Before you start: how to actually get mirrored

**Stand back** far enough that your torso and both arms are inside the
camera frame. `mirror.launch.py` statically places `camera_link` 1.2 m out
in front of `base_link`, facing you — the RViz view opens wide enough to
show both the robot and a standing person at that range without orbiting.

**Raise your arms.** At rest, elbows measure **0.02–0.09 visibility** —
comfortably under the tracker's gate. Below the gate, that arm is HELD at
its last commanded pose, not guessed from a low-confidence landmark. If an
arm will not move, this is almost always why: raise it until it visibly
tracks (check with `preview:=true`, below), then the gate opens and mirror
motion resumes on that arm.

**Legs only mirror once your feet are in frame too.** The leg gate is
independent of the arm gate — arms can be tracking while legs sit idle
because your feet are cropped out, or vice versa. **Sitting down is
expected to give you arms-only mirroring**: seated, your legs and feet are
usually not both fully visible, so `leg_l`/`leg_r` stay gated off and the
robot's legs simply hold. This is not a bug to chase.

**`preview:=true`** opens the annotated webcam window (camera mode only,
mirrored for you) — the same landmark overlay the tracker itself is using,
so you can see exactly what is and is not passing each per-limb visibility
gate before wondering why a limb will not move.

## Deadman: freeze / resume without killing the stack

```bash
ros2 service call /mirror_enable std_srvs/srv/SetBool "{data: false}"  # freeze
ros2 service call /mirror_enable std_srvs/srv/SetBool "{data: true}"   # resume
```

Frozen means the node publishes **nothing** — not "publishes the same
value" — so nothing keeps commanding the controllers while you step away
or reset. Resume re-seeds from `/joint_states` and slews back in, so there
is no jump.

## Stopping everything

```bash
./docker/ros2-arm stop
```

Kills every named arm container (`armmirror`, `armtrack`, `armtalos`, …) in
one shot — the right thing to reach for over `docker rm -f` guesswork or
`Ctrl-C`-and-hope, especially after a frozen/aborted run.

## One ROS stack per container

Two launches sharing one container collide — the second `move_group` and
`controller_manager` fight the first over node names and the DDS graph,
and any acceptance/harness tool run afterward reports failures that have
nothing to do with the code under test. Each `ros2-arm <verb>` already gets
its own named container, so the launcher is safe by construction; only
hand-rolled `docker run ... bash -lc '<two launches>'` invocations hit
this. `./docker/ros2-arm stop` clears everything if you are not sure what
is still running.
