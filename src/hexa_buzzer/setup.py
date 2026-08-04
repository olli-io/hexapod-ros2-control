import os
from glob import glob

from setuptools import find_packages, setup

package_name = "hexa_buzzer"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Olli Moisio",
    maintainer_email="olli.moisio@protonmail.com",
    description=(
        "Passive buzzer on the Pi's hardware PWM: the robot's only audible "
        "channel, played from a tune name on /buzzer/play."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "buzzer_node = hexa_buzzer.buzzer_node:main",
        ],
    },
)
