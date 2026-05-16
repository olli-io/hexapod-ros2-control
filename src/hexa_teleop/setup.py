import os
from glob import glob

from setuptools import find_packages, setup

package_name = "hexa_teleop"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Olli Moisio",
    maintainer_email="olli.moisio@protonmail.com",
    description="Human-input nodes for the hexapod: joystick and keyboard teleop.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "teleop_joy = hexa_teleop.teleop_joy:main",
        ],
    },
)
