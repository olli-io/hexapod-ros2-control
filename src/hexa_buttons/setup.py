import os
from glob import glob

from setuptools import find_packages, setup

package_name = "hexa_buttons"

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
        "Front-panel GPIO buttons: battery/address and Bluetooth info screens "
        "on the face's OLED, and the pairing-scan request behind them."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "button_node = hexa_buttons.button_node:main",
        ],
    },
)
