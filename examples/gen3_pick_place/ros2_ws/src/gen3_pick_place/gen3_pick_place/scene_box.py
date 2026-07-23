#!/usr/bin/env python3
"""Planning-scene box manager for the Gen3 lite pick-and-place demo (U4).

Owns the 4 cm demo box in the MoveIt planning scene:

  * spawn   -- ADD as a world CollisionObject (PlanningScene is_diff:true)
  * attach  -- VERIFIED absorb-from-world form: robot_state.is_diff:true with
               AttachedCollisionObject{link_name, touch_links,
               object{id, header.frame_id=<link>, operation=ADD}} and NO
               geometry -- move_group absorbs the existing world object of the
               same id, so world-pose bookkeeping stays consistent.
  * detach  -- REMOVE the attached object; MoveIt re-inserts it into the world
               at its current pose.  We verify that, and if a MoveIt build
               ever drops it instead, we explicitly re-ADD at the gripper's
               current pose (FK via /compute_fk, TF fallback -- gap 12).
  * /box_reset (std_srvs/Trigger) -- detach if needed and respawn the box at
               its home spawn pose (gap 11; also recovers the floating box
               after a mid-air release).
  * proximity helper -- Euclidean gate hand-target <-> box pose vs D_ATTACH
               (gap 5: the caller keeps re-evaluating this while fisted; a far
               fist closes the gripper but must NOT attach).

Frames & links (VERIFIED on the processed gen3_lite_gen3_lite_2f URDF, U2):
  base_link -> ... -> end_effector_link -> dummy_link -> {tool_frame,
  gripper_base_link -> right/left_finger_prox_link -> right/left_finger_dist_link}

Workspace numbers (gap 10): U3's tuned workspace box was NOT yet published
when this was written, so WS_BOX below is the gen3-lite dexterous zone
(reach 760 mm, numbers chosen conservatively).  SPAWN_POSE sits inside
WS_BOX with >= D_ATTACH margin on every axis so the hand target can approach
the box from any direction without leaving the workspace (gaps 10/16).
U3 must reconcile WS_BOX when it lands; SPAWN_POSE/D_ATTACH then re-check:
    min margin = min over axes of distance(spawn, ws bound) >= D_ATTACH.
"""
import math
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetPositionFK
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger

# --- Geometry / tuning constants (record-of-truth for the other units) -------
BOX_ID = "demo_box"
BOX_SIZE = 0.04                      # 4 cm cube (spec)

# Attach target: the gripper base (rigid child of end_effector_link).
ATTACH_LINK = "gripper_base_link"
TOOL_FRAME = "tool_frame"            # TCP; use for hand-target <-> box distance
BASE_FRAME = "base_link"
# Every gripper link that may legitimately contact the box while pinching:
TOUCH_LINKS = [
    "gripper_base_link",
    "right_finger_prox_link",
    "right_finger_dist_link",
    "left_finger_prox_link",
    "left_finger_dist_link",
    "end_effector_link",
    "dummy_link",
]

# Proximity gate (gap 5): attach only when the hand target (tool_frame goal)
# is within D_ATTACH of the box centre.  Must not exceed the smallest
# spawn-pose-to-workspace-boundary margin (see WS_BOX check below).
D_ATTACH = 0.06                      # m

# U3's tuned workspace box (gap 10), RECONCILED with
# gen3_pick_place.ik_client.WORKSPACE_BOX (single tuned set; U3's
# workspace_sweep cross-checks the two modules stay equal).  Tuned
# empirically against /compute_ik: 5x5x5 fixed-down grid solves 125/125;
# the pre-tune stand-in (x<=0.45, z<=0.40) failed 25/125 at the z=0.40
# plane and far corners; x<=0.24 is marginal near the base.
WS_BOX = {"x": (0.26, 0.40), "y": (-0.16, 0.16), "z": (0.10, 0.30)}

# Home spawn pose: box centre.  x margin 0.08/0.06, y centred (margin 0.16),
# z low-but-reachable grasp height (margin 0.06 to z-min, = D_ATTACH, so an
# approach from below still stays inside the box).  IK-proven reachable
# (fixed-down + tilted grasps) by U3's workspace_sweep.
SPAWN_XYZ = (0.34, 0.0, 0.16)

# Gripper close target reconciled to the 4 cm box (gap 9): attach FIRST, then
# command right_finger_bottom_joint here -- visually pinches a 4 cm cube
# without the fingers passing through it (full close is 0.96 rad).
GRIP_PINCH = 0.50                    # rad on right_finger_bottom_joint
GRIP_OPEN = 0.0                      # rad


def _assert_spawn_inside_workspace():
    """Non-circular gap 10/16 guarantee, evaluated at import time."""
    margins = []
    for axis, (lo, hi) in zip("xyz", (WS_BOX["x"], WS_BOX["y"], WS_BOX["z"])):
        v = SPAWN_XYZ["xyz".index(axis)]
        if not (lo < v < hi):
            raise ValueError(
                "SPAWN_XYZ %s=%.3f outside WS_BOX %s" % (axis, v, (lo, hi)))
        margins.append(min(v - lo, hi - v))
    if min(margins) + 1e-9 < D_ATTACH:
        raise ValueError(
            "spawn margin %.3f < D_ATTACH %.3f -- hand target cannot reach "
            "the box from every side inside the workspace" %
            (min(margins), D_ATTACH))
    return min(margins)


SPAWN_MARGIN = _assert_spawn_inside_workspace()


def _remaining(deadline):
    """Seconds left until `deadline` (monotonic), floored at 0.05 so nested
    waits still make one quick attempt instead of raising on <= 0."""
    return max(0.05, deadline - time.monotonic())


def box_distance(target_xyz, box_xyz):
    """Euclidean distance helper for the proximity gate."""
    return math.dist(tuple(target_xyz), tuple(box_xyz))


def within_attach(target_xyz, box_xyz, d_attach=D_ATTACH):
    """True when the hand target is close enough to the box to attach.

    Gap 5 contract: the gesture unit calls this EVERY tick while the fist is
    held; a fist far from the box closes the gripper but does not attach, and
    carrying the fist to the box attaches on arrival.
    """
    return box_distance(target_xyz, box_xyz) <= d_attach


def make_pose(xyz, quat=(0.0, 0.0, 0.0, 1.0)):
    p = Pose()
    p.position.x, p.position.y, p.position.z = (float(v) for v in xyz)
    (p.orientation.x, p.orientation.y,
     p.orientation.z, p.orientation.w) = (float(v) for v in quat)
    return p


class SceneBoxManager(Node):
    """Node owning the demo box's planning-scene lifecycle."""

    def __init__(self, node_name="scene_box_manager", spawn_on_start=True):
        super().__init__(node_name)
        # Reentrant group: the /box_reset callback makes nested service calls
        # whose responses must be processed while the callback is running.
        self._cbg = ReentrantCallbackGroup()
        self._aps = self.create_client(ApplyPlanningScene,
                                       "/apply_planning_scene",
                                       callback_group=self._cbg)
        self._gps = self.create_client(GetPlanningScene, "/get_planning_scene",
                                       callback_group=self._cbg)
        self._fk = self.create_client(GetPositionFK, "/compute_fk",
                                      callback_group=self._cbg)
        self._js = None
        self._js_lock = threading.Lock()
        self.create_subscription(JointState, "/joint_states",
                                 self._js_cb, 10,
                                 callback_group=self._cbg)
        self.attached = False
        self._spawn_on_start = spawn_on_start
        self.create_service(Trigger, "/box_reset", self._on_box_reset,
                            callback_group=self._cbg)
        self._started = False
        self.create_timer(0.5, self._deferred_start,
                          callback_group=self._cbg)

    # -- plumbing -------------------------------------------------------------
    def destroy_node(self):
        tf_node = getattr(self, "_tf_node", None)
        if tf_node is not None:            # lazy TF-fallback helper (F1)
            self._tf_node = None
            try:
                self._tf_listener.unregister()
            except Exception:  # noqa: BLE001 -- teardown is best-effort
                pass
            try:
                tf_node.destroy_node()
            except Exception:  # noqa: BLE001
                pass
        super().destroy_node()

    def _js_cb(self, msg):
        with self._js_lock:
            self._js = msg

    def _deferred_start(self):
        if self._started:
            return
        if self._spawn_on_start and self._aps.service_is_ready():
            self._started = True
            ok = self.spawn_box()
            self.get_logger().info("initial box spawn: %s" % ok)
        elif not self._spawn_on_start:
            self._started = True

    def _call(self, cli, req, timeout=10.0):
        """Bounded service call.  `timeout` is a TOTAL deadline covering BOTH
        service discovery and the response wait (F2 -- previously each phase
        got the full timeout, doubling the worst case)."""
        deadline = time.monotonic() + timeout
        if not cli.wait_for_service(timeout_sec=timeout):
            self.get_logger().error("service %s unavailable" % cli.srv_name)
            return None
        fut = cli.call_async(req)
        while not fut.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if fut.done():
            return fut.result()
        try:                      # drop the stale pending request so a late
            cli.remove_pending_request(fut)   # response can't confuse a retry
        except Exception:  # noqa: BLE001 -- best-effort cleanup
            pass
        self.get_logger().warning("service %s timed out after %.1f s"
                                  % (cli.srv_name, timeout))
        return None

    # -- gripper pose (gap 12: /compute_fk first, TF fallback) ---------------
    def gripper_pose(self, link=ATTACH_LINK, timeout=5.0):
        """Current pose of `link` in base_link, via /compute_fk if served,
        else a TF lookup.  `timeout` is a TOTAL budget across the FK wait,
        the FK call, and the TF fallback (F2).  Returns Pose or None."""
        deadline = time.monotonic() + timeout
        with self._js_lock:
            js = self._js
        if js is not None and self._fk.wait_for_service(
                timeout_sec=_remaining(deadline)):
            req = GetPositionFK.Request()
            req.header.frame_id = BASE_FRAME
            req.fk_link_names = [link]
            req.robot_state.joint_state = js
            res = self._call(self._fk, req, _remaining(deadline))
            if res is not None and res.error_code.val == 1:
                return res.pose_stamped[0].pose
            self.get_logger().warn("/compute_fk failed (code %s), trying TF"
                                   % (res.error_code.val if res else "none"))
        # TF fallback on a DEDICATED helper node (F1): SceneBoxManager is
        # already spun by the caller's executor, so TransformListener(...,
        # self, spin_thread=True) would add THIS node to a second executor
        # and double-dispatch /box_reset + /joint_states callbacks.  The
        # helper node exists only to feed the TF buffer on its own spin
        # thread; lookup_transform's polling timeout replaces the old fixed
        # time.sleep(1.0) so the 20 Hz control thread never hard-sleeps.
        try:
            from rclpy.duration import Duration
            from tf2_ros import Buffer, TransformListener
            buf = getattr(self, "_tf_buf", None)
            if buf is None:
                self._tf_node = rclpy.create_node(
                    self.get_name() + "_tf", context=self.context)
                self._tf_buf = Buffer()
                self._tf_listener = TransformListener(
                    self._tf_buf, self._tf_node, spin_thread=True)
                buf = self._tf_buf
            tr = buf.lookup_transform(
                BASE_FRAME, link, rclpy.time.Time(),
                timeout=Duration(seconds=_remaining(deadline)))
            p = Pose()
            p.position.x = tr.transform.translation.x
            p.position.y = tr.transform.translation.y
            p.position.z = tr.transform.translation.z
            p.orientation = tr.transform.rotation
            return p
        except Exception as exc:  # noqa: BLE001 - report and degrade
            self.get_logger().error("TF fallback failed: %s" % exc)
            return None

    # -- scene queries --------------------------------------------------------
    def scene_state(self, timeout=10.0):
        """(world CollisionObject or None, AttachedCollisionObject or None)."""
        req = GetPlanningScene.Request()
        req.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_NAMES
            | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
        res = self._call(self._gps, req, timeout)
        if res is None:
            return None, None
        world = next((o for o in res.scene.world.collision_objects
                      if o.id == BOX_ID), None)
        att = next((a for a in
                    res.scene.robot_state.attached_collision_objects
                    if a.object.id == BOX_ID), None)
        return world, att

    @staticmethod
    def world_xyz(co):
        """Effective world position of a CollisionObject we manage.

        We always publish geometry in primitive_poses with an identity
        object pose, but read back pose+primitive_pose composed (MoveIt
        Jazzy reports the object pose in `pose`; orientations stay identity
        for this axis-aligned box)."""
        px = co.pose.position
        if co.primitive_poses:
            pp = co.primitive_poses[0].position
            return (px.x + pp.x, px.y + pp.y, px.z + pp.z)
        return (px.x, px.y, px.z)

    # -- lifecycle ops --------------------------------------------------------
    def _apply(self, scene, timeout=10.0):
        req = ApplyPlanningScene.Request()
        req.scene = scene
        res = self._call(self._aps, req, timeout)
        return res is not None and res.success

    def spawn_box(self, xyz=SPAWN_XYZ, timeout=10.0):
        """ADD (or move) the box as a world CollisionObject at `xyz`."""
        scene = PlanningScene()
        scene.is_diff = True
        co = CollisionObject()
        co.header.frame_id = BASE_FRAME
        co.id = BOX_ID
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [BOX_SIZE] * 3
        co.primitives = [prim]
        co.primitive_poses = [make_pose(xyz)]
        co.pose = make_pose((0.0, 0.0, 0.0))
        co.operation = CollisionObject.ADD
        scene.world.collision_objects = [co]
        ok = self._apply(scene, timeout)
        if ok:
            self.attached = False
        return ok

    def attach_box(self, timeout=10.0):
        """Absorb the world box onto ATTACH_LINK (verified form).

        Returns (ok, detail).  The caller MUST have passed the proximity gate
        (within_attach) first -- this method does not re-check it, so keep
        gate policy in one place (the gesture state machine).

        `timeout` is a TOTAL budget shared by the 2-3 sequential service
        calls (F2): the 20 Hz control loop passes a small value so a wedged
        move_group can never stall arm streaming for the old 10 s-per-call
        worst case; a False return is retried by the gate on a later tick."""
        deadline = time.monotonic() + timeout
        world, att = self.scene_state(timeout=_remaining(deadline))
        if att is not None:
            return True, "already attached"
        if world is None:
            return False, "box not in world; spawn first (or scene query timed out)"
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        aco = AttachedCollisionObject()
        aco.link_name = ATTACH_LINK
        aco.touch_links = TOUCH_LINKS
        aco.object.id = BOX_ID
        aco.object.header.frame_id = ATTACH_LINK
        aco.object.operation = CollisionObject.ADD
        scene.robot_state.attached_collision_objects = [aco]
        ok = self._apply(scene, timeout=_remaining(deadline))
        if ok:
            world2, att2 = self.scene_state(timeout=_remaining(deadline))
            ok = att2 is not None and world2 is None
            self.attached = ok
            return ok, ("attached to %s touch=%s; world entry gone=%s"
                        % (ATTACH_LINK, TOUCH_LINKS, world2 is None))
        return False, "apply_planning_scene failed"

    def detach_box(self, timeout=10.0):
        """Detach; ensure the box lands back in the world at the gripper's
        current pose.  Returns (ok, world_xyz or None, detail).

        `timeout` is a TOTAL budget across all nested service calls
        (scene queries, FK/TF gripper pose, apply, optional re-ADD) -- F2:
        the control loop passes a small value; on a timeout-induced False
        the slow scene poll resyncs `attached` and the tick retries."""
        deadline = time.monotonic() + timeout
        world, att = self.scene_state(timeout=_remaining(deadline))
        if att is None:
            return False, None, "not attached (or scene query timed out)"
        grip = self.gripper_pose(ATTACH_LINK, timeout=_remaining(deadline))
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        aco = AttachedCollisionObject()
        aco.link_name = ATTACH_LINK
        aco.object.id = BOX_ID
        aco.object.operation = CollisionObject.REMOVE
        scene.robot_state.attached_collision_objects = [aco]
        if not self._apply(scene, timeout=_remaining(deadline)):
            return False, None, "detach apply failed"
        world2, att2 = self.scene_state(timeout=_remaining(deadline))
        if att2 is not None:
            return False, None, "still attached after REMOVE"
        if world2 is None:
            # This MoveIt build dropped the object on detach: explicitly
            # re-ADD at the gripper's current pose (FK/TF, gap 12).
            if grip is None:
                return False, None, "dropped on detach and no gripper pose"
            if not self.spawn_box((grip.position.x, grip.position.y,
                                   grip.position.z),
                                  timeout=_remaining(deadline)):
                return False, None, "re-ADD at gripper pose failed"
            world2, _ = self.scene_state(timeout=_remaining(deadline))
        self.attached = False
        xyz = self.world_xyz(world2) if world2 is not None else None
        return xyz is not None, xyz, "world pose %s" % (xyz,)

    def reset_box(self):
        """Detach if needed, then respawn at SPAWN_XYZ.  (ok, detail)."""
        _, att = self.scene_state()
        if att is not None:
            scene = PlanningScene()
            scene.is_diff = True
            scene.robot_state.is_diff = True
            aco = AttachedCollisionObject()
            aco.link_name = att.link_name or ATTACH_LINK
            aco.object.id = BOX_ID
            aco.object.operation = CollisionObject.REMOVE
            scene.robot_state.attached_collision_objects = [aco]
            if not self._apply(scene):
                return False, "detach during reset failed"
        if not self.spawn_box(SPAWN_XYZ):
            return False, "respawn failed"
        world, att2 = self.scene_state()
        ok = (world is not None and att2 is None
              and box_distance(self.world_xyz(world), SPAWN_XYZ) < 1e-3)
        return ok, "box at %s attached=%s" % (
            self.world_xyz(world) if world else None, att2 is not None)

    def _on_box_reset(self, _req, res):
        ok, detail = self.reset_box()
        res.success = ok
        res.message = detail
        self.get_logger().info("/box_reset -> %s (%s)" % (ok, detail))
        return res


def main(args=None):
    rclpy.init(args=args)
    node = SceneBoxManager()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
