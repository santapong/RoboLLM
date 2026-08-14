"""Guard the repository-wide directory ownership contract."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def test_canonical_project_surfaces_exist():
    expected = (
        "src/robollm/bridge.py",
        "src/robollm/gazebo_world.py",
        "apps/mcp/server.py",
        "apps/mcp/entrypoint.py",
        "apps/dashboard/server.py",
        "configs/training/b1_smolvla.json",
        "requirements/ros-constraints.txt",
        "scripts/learning/b1/gpu/preflight.py",
        "tests/integration/ros/test_robot_bridge.py",
    )
    missing = [path for path in expected if not (ROOT / path).is_file()]
    assert not missing, f"canonical project files missing: {missing}"


def test_legacy_public_launchers_are_small_forwarders():
    for relative in (
        "run-server.sh",
        "launch_all.sh",
        "web/run-web.sh",
        "sim/launch_turtlebot.sh",
    ):
        text = (ROOT / relative).read_text(encoding="utf-8")
        assert "Compatibility launcher" in text
        assert len(text.splitlines()) <= 8
