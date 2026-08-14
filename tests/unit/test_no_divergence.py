"""The two vendored arm_ik.py copies must stay byte-identical.

examples/wall_weld/ros2_ws/tools/arm_ik.py and
examples/hand_follow/ros2_ws/tools/arm_ik.py are a files-identical canonical
pair (pure math, no ROS/numpy). If either drifts, the property suite only
guards one of them — this test is the divergence tripwire.
"""
import hashlib
import os

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_A = os.path.join(
    _ROOT, "examples", "wall_weld", "ros2_ws", "tools", "arm_ik.py")
_B = os.path.join(
    _ROOT, "examples", "hand_follow", "ros2_ws", "tools", "arm_ik.py")


def _sha256(path):
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def test_arm_ik_copies_byte_identical():
    assert os.path.exists(_A), "missing wall_weld arm_ik.py"
    assert os.path.exists(_B), "missing hand_follow arm_ik.py"
    with open(_A, "rb") as f:
        a = f.read()
    with open(_B, "rb") as f:
        b = f.read()
    assert a == b, "arm_ik.py copies diverged (wall_weld vs hand_follow)"
    assert _sha256(_A) == _sha256(_B)
