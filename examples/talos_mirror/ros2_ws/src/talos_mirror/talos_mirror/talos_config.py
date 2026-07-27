"""Group / joint / controller constants for TALOS, shared by the sweep node,
the limit monitor and the acceptance tool.

Matches talos_moveit_config/config/talos.srdf (groups + base/tip links) and
config/ros2_controllers.yaml (controller names, which — unlike FFW's
ffw_moveit_config — are ours to pick, and were picked as f"{group}_controller"
uniformly, so CONTROLLER_NAMES below needs no exception table). Kept in the
python package, not in launch/, so the launch files and the tools/ scripts can
both import it after `source install/setup.bash` — installed launch files in
share/ cannot import each other.

30 actuated joints across 6 chain groups; gripper_l / gripper_r (2 more
joints in docs/joints.json) are OUT OF SCOPE — talos_moveit_config's URDF
does not even build the gripper macros (see that package's
config/talos.urdf.xacro header), so no controller, no SRDF group, and no
check here claims them.
"""

GROUP_TORSO = "torso"
GROUP_ARM_L = "arm_l"
GROUP_ARM_R = "arm_r"
GROUP_HEAD = "head"
GROUP_LEG_L = "leg_l"
GROUP_LEG_R = "leg_r"

GROUPS = [GROUP_TORSO, GROUP_ARM_L, GROUP_ARM_R, GROUP_HEAD, GROUP_LEG_L, GROUP_LEG_R]

TORSO_JOINTS = ["torso_1_joint", "torso_2_joint"]
ARM_L_JOINTS = [f"arm_left_{i}_joint" for i in range(1, 8)]
ARM_R_JOINTS = [f"arm_right_{i}_joint" for i in range(1, 8)]
HEAD_JOINTS = ["head_1_joint", "head_2_joint"]
LEG_L_JOINTS = [f"leg_left_{i}_joint" for i in range(1, 7)]
LEG_R_JOINTS = [f"leg_right_{i}_joint" for i in range(1, 7)]

GROUP_JOINTS = {
    GROUP_TORSO: TORSO_JOINTS,
    GROUP_ARM_L: ARM_L_JOINTS,
    GROUP_ARM_R: ARM_R_JOINTS,
    GROUP_HEAD: HEAD_JOINTS,
    GROUP_LEG_L: LEG_L_JOINTS,
    GROUP_LEG_R: LEG_R_JOINTS,
}

ALL_JOINTS = (
    TORSO_JOINTS + ARM_L_JOINTS + ARM_R_JOINTS + HEAD_JOINTS + LEG_L_JOINTS + LEG_R_JOINTS
)
assert len(ALL_JOINTS) == 30

# Controller names, matching talos_moveit_config/config/ros2_controllers.yaml
# and moveit_controllers.yaml exactly (MoveIt's simple controller manager
# looks a controller up by name, it does not discover it).
CONTROLLER_NAMES = {g: f"{g}_controller" for g in GROUPS}
CONTROLLER_TOPICS = {
    g: f"/{name}/joint_trajectory" for g, name in CONTROLLER_NAMES.items()
}

# Base/tip links per group, from talos_moveit_config/config/talos.srdf.
BASE_LINKS = {
    GROUP_TORSO: "base_link",
    GROUP_ARM_L: "torso_2_link",
    GROUP_ARM_R: "torso_2_link",
    GROUP_HEAD: "torso_2_link",
    GROUP_LEG_L: "base_link",
    GROUP_LEG_R: "base_link",
}
TIP_LINKS = {
    GROUP_TORSO: "torso_2_link",
    GROUP_ARM_L: "wrist_left_ft_tool_link",
    GROUP_ARM_R: "wrist_right_ft_tool_link",
    GROUP_HEAD: "head_2_link",
    GROUP_LEG_L: "left_sole_link",
    GROUP_LEG_R: "right_sole_link",
}

# PAL's own "zeros" default standing pose (talos_description/config/
# default_configuration_zeros.yaml), also the SRDF's "home" group_state and
# the pose config/mock_hardware.ros2_control.xacro boots the mock hardware
# into. NOT all-zero: arm_left_2_joint's range is [0.0087, 2.871] and
# arm_right_2_joint's is [-2.871, -0.0087], so literal 0.0 is out of range on
# both shoulders by about half a degree.
HOME_POSE = {
    GROUP_TORSO: [0.0, 0.0],
    GROUP_ARM_L: [0.0, 0.2, 0.0, -0.02, 0.0, 0.0, 0.0],
    GROUP_ARM_R: [0.0, -0.2, 0.0, -0.02, 0.0, 0.0, 0.0],
    GROUP_HEAD: [0.0, 0.0],
    GROUP_LEG_L: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    GROUP_LEG_R: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
}


def home_joint_state():
    """Flat {joint_name: value} for the whole-body home pose."""
    out = {}
    for group, joints in GROUP_JOINTS.items():
        for name, value in zip(joints, HOME_POSE[group]):
            out[name] = value
    return out
