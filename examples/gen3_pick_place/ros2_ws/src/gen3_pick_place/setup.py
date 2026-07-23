from glob import glob

from setuptools import setup

package_name = "gen3_pick_place"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="santapong",
    maintainer_email="santapong@users.noreply.github.com",
    description="Hand-guided pick-and-place demo for the Kinova Gen3 lite.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "scene_box = gen3_pick_place.scene_box:main",
            "workspace_sweep = gen3_pick_place.workspace_sweep:main",
            "hand_pick_place = gen3_pick_place.hand_pick_place:main",
        ],
    },
)
