from glob import glob

from setuptools import setup

package_name = "humanoid_mirror"

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
    description="Webcam body mirroring (both arms + head) for the ROBOTIS FFW semi-humanoid.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            # mirror_node lands in M2 — no entry point until the file exists,
            # so a broken build can never masquerade as a missing camera.
            "ffw_check = humanoid_mirror.ffw_check:main",
        ],
    },
)
