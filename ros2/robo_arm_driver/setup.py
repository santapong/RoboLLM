from glob import glob

from setuptools import find_packages, setup

package_name = "robo_arm_driver"

setup(
    name=package_name,
    version="0.2.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "pyserial", "PyYAML"],
    zip_safe=True,
    maintainer="santapong",
    maintainer_email="santapong@users.noreply.github.com",
    description="Validated JointTrajectory-to-Arduino bridge for the RoboLLM arm.",
    license="MIT",
    entry_points={"console_scripts": ["arm_bridge = robo_arm_driver.node:main"]},
)
