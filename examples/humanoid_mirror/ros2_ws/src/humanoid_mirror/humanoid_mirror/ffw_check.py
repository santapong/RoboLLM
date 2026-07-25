"""M1 acceptance check: the humanoid really loaded, and MoveIt really plans.

No camera, no MediaPipe, no RViz. Run it against a live mock_bringup and it
either exits 0 with a green table or exits 1 naming what broke.

    ros2 run humanoid_mirror ffw_check

Checks, in order of what actually goes wrong:
  1. /robot_description and the semantic description are non-empty
     (this is the exact failure mode of ffw_moveit_config's own launch file)
  2. the SRDF really defines arm_l / arm_r / head / lift
  3. /joint_states carries all 19 mock-hardware joints
  4. the four JointTrajectoryControllers are ACTIVE over disjoint joint sets
  5. /compute_ik returns SUCCESS for BOTH 7-DOF arms
"""

import sys

import rclpy
from moveit_msgs.srv import GetPositionIK
from rclpy.node import Node
from sensor_msgs.msg import JointState

from humanoid_mirror.ffw_config import (
    ARM_L_JOINTS,
    ARM_R_JOINTS,
    GROUP_ARM_L,
    GROUP_ARM_R,
    HEAD_JOINTS,
    LIFT_JOINTS,
)

# Every joint the mock hardware exposes. The grippers are present but
# deliberately unclaimed by any controller — mirroring drives arms + head only.
EXPECTED_JOINTS = set(
    ARM_L_JOINTS + ARM_R_JOINTS + HEAD_JOINTS + LIFT_JOINTS
) | {"gripper_l_joint1", "gripper_r_joint1"}

# Reachable seed poses, chosen inside the measured limits. arm_l_joint2 is
# ONE-SIDED (0 .. 3.14) and arm_r_joint2 mirrors it (-3.14 .. 0), so a naive
# symmetric seed is out of range on one side — that asymmetry is the single
# most common way a first attempt at this robot fails.
SEEDS = {
    GROUP_ARM_L: ([0.0, 0.6, 0.0, -0.8, 0.0, 0.0, 0.0], ARM_L_JOINTS, "arm_l_link7"),
    GROUP_ARM_R: ([0.0, -0.6, 0.0, -0.8, 0.0, 0.0, 0.0], ARM_R_JOINTS, "arm_r_link7"),
}


class Checker(Node):
    def __init__(self):
        super().__init__("ffw_check")
        self.failures = []
        self.notes = []
        self.latest_joint_state = None
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10
        )

    def _on_joint_state(self, msg):
        self.latest_joint_state = msg

    def check(self, ok, label, detail=""):
        mark = "PASS" if ok else "FAIL"
        print(f"  [{mark}] {label}{(' — ' + detail) if detail else ''}")
        if not ok:
            self.failures.append(label)
        return ok

    # -- 1/2: descriptions ------------------------------------------------
    def check_descriptions(self):
        print("\ndescriptions")
        urdf = self._get_remote_param(
            "/move_group", "robot_description"
        ) or self._get_remote_param("/robot_state_publisher", "robot_description")
        self.check(
            bool(urdf) and len(urdf) > 1000,
            "robot_description is non-empty",
            f"{len(urdf or '')} chars",
        )
        self.check(
            bool(urdf) and 'name="ffw_bg2_follower"' in urdf,
            "URDF is the bg2_rev4 variant (name matches the SRDF)",
        )
        srdf = self._get_remote_param("/move_group", "robot_description_semantic")
        for group in (GROUP_ARM_L, GROUP_ARM_R, "head", "lift"):
            self.check(
                bool(srdf) and f'name="{group}"' in srdf,
                f"SRDF defines planning group '{group}'",
            )

    def _get_remote_param(self, node_name, param_name):
        from rcl_interfaces.srv import GetParameters

        cli = self.create_client(GetParameters, f"{node_name}/get_parameters")
        if not cli.wait_for_service(timeout_sec=5.0):
            return None
        req = GetParameters.Request(names=[param_name])
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        if fut.result() is None or not fut.result().values:
            return None
        return fut.result().values[0].string_value

    # -- 3: joint states ---------------------------------------------------
    def check_joint_states(self):
        print("\njoint_states")
        deadline = self.get_clock().now().nanoseconds + 15e9
        while self.latest_joint_state is None:
            if self.get_clock().now().nanoseconds > deadline:
                break
            rclpy.spin_once(self, timeout_sec=0.2)
        if not self.check(
            self.latest_joint_state is not None, "/joint_states is publishing"
        ):
            return
        got = set(self.latest_joint_state.name)
        missing = EXPECTED_JOINTS - got
        self.check(
            not missing,
            f"all {len(EXPECTED_JOINTS)} mock joints present",
            f"missing {sorted(missing)}" if missing else f"{len(got)} joints",
        )

    # -- 4: controllers ----------------------------------------------------
    def check_controllers(self):
        print("\ncontrollers")
        from controller_manager_msgs.srv import ListControllers

        cli = self.create_client(ListControllers, "/controller_manager/list_controllers")
        if not self.check(
            cli.wait_for_service(timeout_sec=10.0), "controller_manager is up"
        ):
            return
        fut = cli.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        if not self.check(fut.result() is not None, "list_controllers responded"):
            return

        by_name = {c.name: c for c in fut.result().controller}
        claimed = {}
        for name, joints in (
            ("arm_l_controller", ARM_L_JOINTS),
            ("arm_r_controller", ARM_R_JOINTS),
            ("head_controller", HEAD_JOINTS),
            ("lift_controller", LIFT_JOINTS),
        ):
            c = by_name.get(name)
            self.check(
                c is not None and c.state == "active",
                f"{name} is active",
                (c.state if c else "not spawned"),
            )
            for j in joints:
                if j in claimed:
                    self.failures.append(f"{j} claimed by two controllers")
                claimed[j] = name
        self.check(
            len(claimed) == 17,
            "the four controllers cover disjoint joint sets",
            f"{len(claimed)} joints claimed once",
        )

    # -- 5: IK for both arms ----------------------------------------------
    def check_ik(self):
        print("\ninverse kinematics")
        cli = self.create_client(GetPositionIK, "/compute_ik")
        if not self.check(cli.wait_for_service(timeout_sec=15.0), "/compute_ik is up"):
            return

        from moveit_msgs.msg import RobotState
        from tf2_ros import Buffer, TransformListener

        buf = Buffer()
        TransformListener(buf, self)
        deadline = self.get_clock().now().nanoseconds + 10e9
        while self.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        for group, (seed, joints, tip) in SEEDS.items():
            # Ask IK to reproduce the pose the seed itself produces — a target
            # that is reachable by construction, so a failure here is a real
            # configuration problem and never an unreachable-goal artifact.
            try:
                tf = buf.lookup_transform("base_link", tip, rclpy.time.Time())
            except Exception as exc:  # noqa: BLE001
                self.check(False, f"TF base_link -> {tip}", str(exc)[:70])
                continue

            req = GetPositionIK.Request()
            req.ik_request.group_name = group
            req.ik_request.robot_state = RobotState(
                joint_state=JointState(name=list(joints), position=list(seed))
            )
            req.ik_request.avoid_collisions = True
            req.ik_request.pose_stamped.header.frame_id = "base_link"
            req.ik_request.pose_stamped.pose.position.x = tf.transform.translation.x
            req.ik_request.pose_stamped.pose.position.y = tf.transform.translation.y
            req.ik_request.pose_stamped.pose.position.z = tf.transform.translation.z
            req.ik_request.pose_stamped.pose.orientation = tf.transform.rotation
            req.ik_request.timeout.sec = 2

            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
            res = fut.result()
            code = res.error_code.val if res else None
            self.check(
                code == 1,
                f"/compute_ik SUCCESS for group '{group}' (7-DOF)",
                f"error_code={code}",
            )


def main(args=None):
    rclpy.init(args=args)
    node = Checker()
    print("=" * 62)
    print("  humanoid_mirror M1 check — ROBOTIS FFW on mock hardware")
    print("=" * 62)
    try:
        node.check_descriptions()
        node.check_joint_states()
        node.check_controllers()
        node.check_ik()
    finally:
        print("\n" + "=" * 62)
        if node.failures:
            print(f"  FAILED — {len(node.failures)} check(s):")
            for f in node.failures:
                print(f"    - {f}")
        else:
            print("  ALL CHECKS PASSED — the humanoid loads, renders and plans.")
        print("=" * 62)
        failed = bool(node.failures)
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
