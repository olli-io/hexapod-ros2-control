from setuptools import find_packages, setup

package_name = "hexa_kinematics"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Olli Moisio",
    maintainer_email="olli.moisio@protonmail.com",
    description="Forward and inverse kinematics for the hexapod.",
    license="Apache-2.0",
    tests_require=["pytest"],
)
