import os

from setuptools import find_packages
from setuptools import setup

package_name = "autoware_frenetix_planner"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (os.path.join("share", package_name), ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="Korbinian Moller",
    author_email="korbinian.moller@tum.de",
    maintainer="Maxime CLEMENT",
    maintainer_email="maxime.clement@tier4.jp",
    description="Node for the Frenetix planner",
    license="Apache License 2.0",
    entry_points={
        "console_scripts": [
            "frenetix_planner_node= autoware_frenetix_planner.frenetix_planner_node:main"
        ],
    },
)
