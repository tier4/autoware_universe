import os
from glob import glob
from setuptools import setup, find_packages

package_name = "autoware_frenetix_planner"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(where="src"),
    package_dir={"": "src"}, 
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=["setuptools", "rclpy", "frenetix", "shapely"],
    zip_safe=True,
    author="Korbinian Moller",
    author_email="korbinian.moller@tum.de",
    maintainer="Maxime CLEMENT",
    maintainer_email="maxime.clement@tier4.jp",
    description="Node for the Frenetix planner",
    license="Apache License 2.0",
    entry_points={
        "console_scripts": [
            "frenetix_planner_node = autoware_frenetix_planner.frenetix_planner_node:main"
        ],
    },
)
