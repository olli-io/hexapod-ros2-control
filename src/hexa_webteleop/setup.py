import os
from glob import glob

from setuptools import find_packages, setup

package_name = "hexa_webteleop"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "web"), glob("web/*.html")),
        (os.path.join("share", package_name, "web"), glob("web/*.css")),
        (os.path.join("share", package_name, "web"), glob("web/*.js")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Olli Moisio",
    maintainer_email="olli.moisio@protonmail.com",
    description="Web-app teleop for the hexapod: HTTP + WebSocket server hosting a phone/tablet control UI.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "webteleop_node = hexa_webteleop.webteleop_node:main",
        ],
    },
)
