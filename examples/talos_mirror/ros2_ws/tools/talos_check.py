#!/usr/bin/env python3
"""talos_check — acceptance test for the TALOS mock bring-up
(talos_moveit_config's mock_bringup.launch.py, i.e. `ros2-arm talos`).

No camera, no sweep. Run it against a live bring-up and it either exits 0
with a green table or exits 1 naming what broke:

    python3 tools/talos_check.py

Checks, in order of what actually goes wrong:
  1. all 6 SRDF planning groups are defined (torso, arm_l, arm_r, head,
     leg_l, leg_r)
  2. every one of the 30 group joints in talos_mirror.joint_limits.MEASURED
     (a copy of docs/joint-inventory.md / docs/joints.json, minus the two
     out-of-scope gripper joints) exists in the LIVE /robot_description with
     matching position limits
  3. /compute_fk succeeds at the home/standing pose for all 6 groups' tip
     links, and the two arms' y-positions mirror
  4. /compute_ik reaches a conservative, in-workspace target for BOTH wrists
     (arm_l, arm_r) — the target is the home pose's own FK result, reachable
     by construction, so a failure here is a real configuration problem and
     never an unreachable-goal artifact (same trick humanoid_mirror's
     ffw_check.py uses for FFW's two 7-DOF arms)
  5. all 6 JointTrajectoryControllers are ACTIVE over disjoint joint sets

Ported from examples/humanoid_mirror/ros2_ws/src/humanoid_mirror/humanoid_mirror/
ffw_check.py — see that package's TECHNICAL.md for the traps this file
avoids: service calls made from a helper that itself spins the node can
drain queued messages out from under a subscription callback (mitigated here
by using dedicated clients + spin_until_future_complete, never spin_once
loops racing a subscription), and asin/acos results are only correct modulo
2*pi (not applicable to this file directly — no arm solving happens here —
but see talos_mirror's own retargeting notes if that code is ever added).
"""

import sys

import rclpy
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.node import Node
from sensor_msgs.msg import JointState

sys.path.insert(0, __file__.rsplit("/tools/", 1)[0] + "/src/talos_mirror")

from talos_mirror.joint_limits import MEASURED, parse_urdf_limits  # noqa: E402
from talos_mirror.talos_config import (  # noqa: E402
    ALL_JOINTS,
    BASE_LINKS,
    CONTROLLER_NAMES,
    GROUP_JOINTS,
    GROUPS,
    TIP_LINKS,
    home_joint_state,
)

HOME_FLAT = home_joint_state()
LIMIT_TOL = 1e-3  # rad — cross-check tolerance between MEASURED and live URDF


class Checker(Node):
    def __init__(self):
        super().__init__("talos_check")
        self.failures = []
        self.latest_joint_state = None
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)

    def _on_joint_state(self, msg):
        self.latest_joint_state = msg

    def check(self, ok, label, detail=""):
        mark = "PASS" if ok else "FAIL"
        print(f"  [{mark}] {label}{(' — ' + detail) if detail else ''}")
        if not ok:
            self.failures.append(label)
        return ok

    def _get_remote_param(self, node_name, param_name):
        from rcl_interfaces.srv import GetParameters

        cli = self.create_client(GetParameters, f"{node_name}/get_parameters")
        if not cli.wait_for_service(timeout_sec=10.0):
            return None
        req = GetParameters.Request(names=[param_name])
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        if fut.result() is None or not fut.result().values:
            return None
        return fut.result().values[0].string_value

    # -- 1: SRDF groups -----------------------------------------------------
    def check_groups(self):
        print("\nSRDF groups")
        srdf = self._get_remote_param("/move_group", "robot_description_semantic")
        if not self.check(bool(srdf) and len(srdf) > 200, "robot_description_semantic is non-empty",
                           f"{len(srdf or '')} chars"):
            return
        for group in GROUPS:
            self.check(f'name="{group}"' in srdf, f"SRDF defines planning group '{group}'")

    # -- 2: joints.json cross-check vs live URDF -----------------------------
    def check_joint_limits(self):
        print("\njoints.json cross-check (30 group joints vs live URDF)")
        urdf = self._get_remote_param(
            "/move_group", "robot_description"
        ) or self._get_remote_param("/robot_state_publisher", "robot_description")
        if not self.check(bool(urdf) and len(urdf) > 1000, "robot_description is non-empty",
                           f"{len(urdf or '')} chars"):
            return

        parsed = parse_urdf_limits(urdf)
        missing = [j for j in MEASURED if j not in parsed]
        self.check(not missing, f"all {len(MEASURED)} joints from docs/joints.json present in URDF",
                    f"missing {missing}" if missing else "")

        mismatched = []
        vel_mismatched = []
        for name, (lo, up, vel) in MEASURED.items():
            if name not in parsed:
                continue
            p_lo, p_up, p_vel = parsed[name]
            if abs(p_lo - lo) > LIMIT_TOL or abs(p_up - up) > LIMIT_TOL:
                mismatched.append(f"{name}: urdf=({p_lo:.4f},{p_up:.4f}) json=({lo:.4f},{up:.4f})")
            if p_vel is not None and vel is not None and abs(p_vel - vel) > LIMIT_TOL:
                vel_mismatched.append(f"{name}: urdf={p_vel:.4f} json={vel:.4f}")
        self.check(not mismatched, "limits match docs/joints.json for every present joint",
                    "; ".join(mismatched[:4]) if mismatched else "")
        self.check(not vel_mismatched,
                    "velocity limits match docs/joints.json for every present joint",
                    "; ".join(vel_mismatched[:4]) if vel_mismatched else "")

    # -- 3: FK sanity at home -------------------------------------------------
    def check_fk(self):
        print("\nforward kinematics (home / standing pose)")
        cli = self.create_client(GetPositionFK, "/compute_fk")
        if not self.check(cli.wait_for_service(timeout_sec=15.0), "/compute_fk is up"):
            return

        joint_state = JointState(
            name=list(HOME_FLAT.keys()), position=list(HOME_FLAT.values())
        )

        results = {}
        for group in GROUPS:
            tip = TIP_LINKS[group]
            req = GetPositionFK.Request()
            req.header.frame_id = BASE_LINKS[group]
            req.fk_link_names = [tip]
            req.robot_state.joint_state = joint_state

            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
            res = fut.result()
            code = res.error_code.val if res else None
            ok = self.check(
                code == 1, f"/compute_fk SUCCESS for group '{group}' (tip {tip})",
                f"error_code={code}",
            )
            if ok and res.pose_stamped:
                p = res.pose_stamped[0].pose.position
                results[group] = (p.x, p.y, p.z)

        if "arm_l" in results and "arm_r" in results:
            yl = results.get("arm_l", (0, 0, 0))[1]
            yr = results.get("arm_r", (0, 0, 0))[1]
            self.check(
                abs(yl + yr) < 1e-2,
                "arm_l / arm_r home-pose Y positions mirror",
                f"y_l={yl:.4f} y_r={yr:.4f}",
            )

    # -- 4: IK reaches for both wrists ---------------------------------------
    def check_ik(self):
        print("\ninverse kinematics — both wrists, conservative in-workspace targets")
        fk_cli = self.create_client(GetPositionFK, "/compute_fk")
        ik_cli = self.create_client(GetPositionIK, "/compute_ik")
        if not self.check(fk_cli.wait_for_service(timeout_sec=15.0), "/compute_fk is up"):
            return
        if not self.check(ik_cli.wait_for_service(timeout_sec=15.0), "/compute_ik is up"):
            return

        joint_state = JointState(
            name=list(HOME_FLAT.keys()), position=list(HOME_FLAT.values())
        )

        for group in ("arm_l", "arm_r"):
            tip = TIP_LINKS[group]
            base = BASE_LINKS[group]

            # Target = the home pose's own FK result. Reachable by
            # construction, so a failure here is a real configuration
            # problem and never an unreachable-goal artifact — same trick
            # humanoid_mirror's ffw_check.py uses.
            fk_req = GetPositionFK.Request()
            fk_req.header.frame_id = base
            fk_req.fk_link_names = [tip]
            fk_req.robot_state.joint_state = joint_state
            fut = fk_cli.call_async(fk_req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
            fk_res = fut.result()
            if not self.check(
                fk_res is not None and fk_res.error_code.val == 1 and fk_res.pose_stamped,
                f"FK seed pose available for '{group}'",
            ):
                continue
            target_pose = fk_res.pose_stamped[0].pose

            from moveit_msgs.msg import RobotState

            ik_req = GetPositionIK.Request()
            ik_req.ik_request.group_name = group
            ik_req.ik_request.robot_state = RobotState(joint_state=joint_state)
            ik_req.ik_request.avoid_collisions = True
            ik_req.ik_request.pose_stamped.header.frame_id = base
            ik_req.ik_request.pose_stamped.pose = target_pose
            ik_req.ik_request.timeout.sec = 3

            fut = ik_cli.call_async(ik_req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
            ik_res = fut.result()
            code = ik_res.error_code.val if ik_res else None
            self.check(
                code == 1, f"/compute_ik SUCCESS for group '{group}' (7-DOF wrist)",
                f"error_code={code}",
            )

    # -- 5: controllers -------------------------------------------------------
    def check_controllers(self):
        print("\ncontrollers")
        from controller_manager_msgs.srv import ListControllers

        cli = self.create_client(ListControllers, "/controller_manager/list_controllers")
        if not self.check(cli.wait_for_service(timeout_sec=15.0), "controller_manager is up"):
            return
        fut = cli.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        if not self.check(fut.result() is not None, "list_controllers responded"):
            return

        by_name = {c.name: c for c in fut.result().controller}
        # Coverage/mapping is asserted from the controllers' LIVE
        # claimed_interfaces, not our own GROUP_JOINTS table — a controllers
        # YAML that mapped arm_l_controller to the right-arm joints must FAIL
        # here, not pass because we compared our table against itself.
        claimed = {}
        for group in GROUPS:
            name = CONTROLLER_NAMES[group]
            c = by_name.get(name)
            self.check(
                c is not None and c.state == "active",
                f"{name} is active",
                (c.state if c else "not spawned"),
            )
            if c is None:
                continue
            live = sorted({i.split("/")[0] for i in c.claimed_interfaces})
            for j in live:
                if j in claimed:
                    self.failures.append(f"{j} claimed by two controllers")
                claimed[j] = name
            expected = sorted(GROUP_JOINTS[group])
            self.check(
                live == expected,
                f"{name} claims exactly the {group} joints (live claimed_interfaces)",
                f"live={live} expected={expected}" if live != expected else "",
            )
        self.check(
            len(claimed) == len(ALL_JOINTS),
            "the six controllers cover disjoint joint sets (live)",
            f"{len(claimed)}/{len(ALL_JOINTS)} joints claimed once",
        )


def main(args=None):
    rclpy.init(args=args)
    node = Checker()
    print("=" * 66)
    print("  talos_mirror acceptance check — PAL TALOS on mock hardware")
    print("=" * 66)
    try:
        node.check_groups()
        node.check_joint_limits()
        node.check_fk()
        node.check_ik()
        node.check_controllers()
    finally:
        print("\n" + "=" * 66)
        if node.failures:
            print(f"  FAILED — {len(node.failures)} check(s):")
            for f in node.failures:
                print(f"    - {f}")
        else:
            print("  ALL CHECKS PASSED — TALOS loads, renders and plans.")
        print("=" * 66)
        failed = bool(node.failures)
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
