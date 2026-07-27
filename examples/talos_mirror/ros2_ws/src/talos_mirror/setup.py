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
        "PAL TALOS humanoid, over talos_moveit_config's mock controllers."
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "sweep_node = talos_mirror.sweep_node:main",
            "limit_monitor = talos_mirror.limit_monitor:main",
        ],
    },
)
