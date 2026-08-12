# RoboLLM · Wall-weld simulation runbook

> **RoboLLM field guide** · Build → Observe → Measure → Learn<br>
> [Home](../../../README.md) · [Example](../README.md) · [Technical notes](../TECHNICAL.md) · [Examples](../../README.md)

**Environment:** RViz simulation · **Success:** synthetic full/abort/idle
acceptance scenarios finish with the expected state and no collision failure.

Show the webcam a wall (via an ArUco marker), make a **fist**, and the weld
arm rasters the entire wall face autonomously — serpentine passes, growing
bead line, sparks at the torch. **Open palm aborts** mid-weld. This is
automation triggered by gesture, not hand-following: while idle the arm
holds home.

## Bring-up

```bash
ros2-arm wallweld                      # camera mode, RViz with the weld view
ros2-arm wallweld preview:=true        # + annotated webcam window
ros2-arm wallweld wall_mode:=marker    # ArUco marker places the wall
ros2-arm wallweld synthetic            # no camera: scripted wall + auto weld
ros2-arm wallweld synthetic synthetic_abort_frac:=0.4   # + scripted abort
ros2-arm stop                          # tear down (armwallweld is in the list)
```

RViz opens with the wall-weld view (`rviz/wallweld.rviz`): RobotModel +
PlanningScene (the wall) + the bead/spark markers. `rviz:=false` for headless.

## The gesture flow

| You | The robot |
|---|---|
| *(idle)* | holds home; wall shown in the scene |
| show ArUco marker (`wall_mode:=marker`) | wall moves to where the marker maps (position/size/tilt → clamped reachable window; a *mapped* placement, not metric SLAM) |
| **fist, held ~5 frames** | plans the raster (reachability pre-check shrinks the wall to fit if needed), approaches, welds every pass — bead grows, sparks fly, progress on `/weld_progress` |
| **open palm** (or hide hand >3 s) | aborts: freeze → slewed return home; the partial bead stays |
| fist again | welds again (fresh bead) |
| `ros2 service call /wall_reset std_srvs/srv/Trigger` | respawn wall, clear bead |

**Marker**: display `assets/wall_marker.png` (DICT_4X4_50, id 0) on your phone
screen and hold it up to the webcam — no printing needed. Capture only happens
while idle, after 10 stable detections; re-show it to re-place the wall.

## Verified numbers (this machine, i3-9100 CPU)

Synthetic full weld: **352/352 waypoints, bead 352 pts, 20.00 Hz stream,
max joint step 0.032 rad** (limit 0.15), **101/101 sampled states
collision-valid (100%)** vs the wall with the default 15 mm standoff
(5 mm failed validity at 88% — the torch collision body is thicker than its
tip; 15 mm is the verified default). Raster: 11 rows, 3.61 m path, ~72 s.

## Tuning knobs (all launch args — `ros2-arm wallweld key:=value`)

Wall pose/size: `wall_x/y/z`, `wall_yaw`, `wall_width/height`. Raster:
`row_step` (pass spacing), `pass_step`, `margin`, `standoff`, `weld_speed`,
`travel_speed`. Gesture: `n_fist`, `n_palm`, `gesture_score`, `hand`
(Any/Left/Right). Marker mapping windows: `wall_{x,y,z}_{min,max}`,
`yaw_clamp_deg`, `marker_s_near/far`. See `wallweld.launch.py` for all.

## Troubleshooting

- **Wall rejected / shrunk**: the pose failed the corner+grid IK pre-check;
  the log says why. Bring `wall_x` toward 0.30 or reduce size.
- **No fist response**: check the preview window (`preview:=true`) — the
  gesture label must read `Closed_Fist` and handedness must match `hand`.
- **Marker not captured**: needs 10 consecutive stable detections while
  IDLE — hold the phone steady, avoid glare, fill ~1/4 of the frame.
- **move_group segfault at Ctrl-C**: known upstream MoveIt teardown race,
  harmless.
- Acceptance meters: `tools/wallweld_selftest.py` (offline, 30 checks) and
  `tools/wallweld_accept.py full` (live monitor, run inside the container).

## Realtime wall tracking (`wall_track:=true`)

```bash
ros2-arm wallweld wall_mode:=marker wall_track:=true preview:=true
```

The wall **follows the marker live** while the arm is idle: move your phone
and the wall moves with it in RViz (EMA-smoothed, up to `track_hz` re-poses/s);
hold it where you want and **fist** — the raster is planned right there
(lazy plan + reachability precheck at fist time; a rejected pose logs
`WALL_CAPTURE_REJECTED at-fist`, just move the marker closer to center and
fist again). Marker lost = wall freezes in place. During a weld, tracking is
suspended; it resumes when the arm is back home. Knobs: `track_alpha`
(smoothing), `track_hz`, `track_eps_m/track_eps_yaw` (dead-band).
