from glob import glob

from setuptools import find_packages, setup

package_name = "patrol_bot"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="santapong",
    maintainer_email="santapongsondhi@gmail.com",
    description="First real ROS 2 package: a parameterized square-patrol node.",
    license="MIT",
    entry_points={
        "console_scripts": [
            # `ros2 run patrol_bot patrol` runs patrol_bot/patrol.py:main
            "patrol = patrol_bot.patrol:main",
        ],
    },
)
