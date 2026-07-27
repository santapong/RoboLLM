from glob import glob

from setuptools import setup

package_name = "talos_mirror"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="santapong",
    maintainer_email="santapong@users.noreply.github.com",
    description=(
        "M2 synthetic sweep (arms + head + torso + legs) for the vendored "
        "PAL TALOS humanoid, over talos_moveit_config's mock controllers. "
        "M3 adds full-body webcam tracking (talos_track: synthetic or "
        "camera) publishing human/* TF, /body/observation and /body/markers "
        "with the robot parked. M4 adds retarget.py (pure-math direct "
        "joint retargeting: shoulder+elbow arms, pan/tilt head, lean "
        "torso, hip+knee+ankle-pitch legs with the pelvis pinned) and "
        "mirror_node, which subscribes /body/observation and drives the "
        "six controllers in mirror mode by default."
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "sweep_node = talos_mirror.sweep_node:main",
            "limit_monitor = talos_mirror.limit_monitor:main",
            "track_node = talos_mirror.track_node:main",
            "mirror_node = talos_mirror.mirror_node:main",
        ],
    },
)
