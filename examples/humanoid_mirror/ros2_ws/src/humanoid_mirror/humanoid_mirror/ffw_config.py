"""FFW robot description + MoveIt config plumbing, shared by every launch file.

Lives in the python package (not in launch/) so the launch files can import it
after `source install/setup.bash` — launch files installed side by side in
share/ cannot import each other.

WHY THIS EXISTS AT ALL: ffw_moveit_config ships its own moveit.launch.py and it
CRASHES. It calls MoveItConfigsBuilder(...).robot_description_semantic(...) but
never .robot_description(...), and the package neither ships a config URDF nor
depends on ffw_description. The failure chain is:

    Could not find parameter robot_description
      -> XML_ERROR_EMPTY_DOCUMENT
      -> [FATAL] Unable to configure planning scene monitor
      -> exit code -6 (SIGABRT)

The bug is present in jazzy-branch HEAD too, so it will not be fixed out from
under us. build_moveit_configs() below is the corrected chain: the same builder
plus the missing .robot_description() call pointing at ffw_description.
"""

import os

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

# VARIANT CHOICE — ffw_bg2_rev4_follower, not ffw_sg2_rev1_follower:
#   * bg2_rev4's <robot name> is "ffw_bg2_follower", which MATCHES the SRDF.
#     sg2_rev1 is "ffw_sg2_follower", which does not, and MoveIt then logs
#     "Semantic description is not specified for the same robot as the URDF".
#   * All 26 of bg2_rev4's mesh references resolve; sg2_rev1 has 3 broken
#     ${swerve_meshes_dir} wheel meshes.
#   * bg2_rev4 is the stationary variant — no swerve base to render on a
#     GPU-free laptop, and nothing body-tracking could drive anyway.
FFW_MODEL = "ffw_bg2_rev4_follower"
FFW_URDF = "ffw_bg2_follower.urdf.xacro"

# Planning groups defined by the stock ffw_moveit_config SRDF (418 real
# disable_collisions pairs — genuine Setup Assistant output, not a stub).
GROUP_ARM_L = "arm_l"
GROUP_ARM_R = "arm_r"
GROUP_HEAD = "head"
GROUP_LIFT = "lift"

# Joint names per controller, matching ffw_moveit_config/moveit_controllers.yaml.
ARM_L_JOINTS = [f"arm_l_joint{i}" for i in range(1, 8)]
ARM_R_JOINTS = [f"arm_r_joint{i}" for i in range(1, 8)]
HEAD_JOINTS = ["head_joint1", "head_joint2"]
LIFT_JOINTS = ["lift_joint"]


def ffw_paths():
    """(urdf_xacro_path, srdf_path) for the variant we standardise on."""
    desc = get_package_share_directory("ffw_description")
    cfg = get_package_share_directory("ffw_moveit_config")
    return (
        os.path.join(desc, "urdf", FFW_MODEL, FFW_URDF),
        os.path.join(cfg, "config", "ffw.srdf"),
    )


def build_moveit_configs(use_mock_hardware=True):
    """The corrected MoveItConfigsBuilder chain.

    .robot_description() is the line ffw_moveit_config forgets.

    use_mock_hardware=True swaps the URDF's hardware plugin from
    dynamixel_hardware_interface/DynamixelHardware (which opens /dev/follower
    and talks to real servos) to mock_components/GenericSystem. Without the
    mapping, bring-up hangs looking for hardware that isn't there.
    """
    urdf, srdf = ffw_paths()
    return (
        MoveItConfigsBuilder(robot_name="ffw", package_name="ffw_moveit_config")
        .robot_description(
            file_path=urdf,
            mappings={"use_mock_hardware": "true" if use_mock_hardware else "false"},
        )
        .robot_description_semantic(file_path=srdf)
        .robot_description_kinematics()
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .to_moveit_configs()
    )
