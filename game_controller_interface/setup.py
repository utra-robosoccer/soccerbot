from glob import glob
from setuptools import find_packages, setup

package_name = "game_controller_interface"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),  # Optional: only if you have configs
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Anthony Pinson",
    maintainer_email="pinsonanthony@gmail.com",
    description="ROS 2 Game Controller Interface node",
    license="MIT",
    entry_points={
        "console_scripts": [
            "gc_interface = game_controller_interface.gc_interface:main",
            "gc_client = game_controller_interface.gc_client:main",
        ],
    },
)
