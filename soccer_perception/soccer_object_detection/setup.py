import os
from glob import glob

from setuptools import find_packages, setup

package_name = "soccer_object_detection"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Config files
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        # Launch files
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        # Models (OPTIONAL: if you use .pt model files like half_5.pt)
        (os.path.join("share", package_name, "models"), glob("models/*.pt")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Anthony Pinson",
    maintainer_email="pinsonanthony@gmail.com",
    description="soccer_object_detection ros2 package",
    license="BSD",
    entry_points={
        "console_scripts": [
            # Make sure this matches your actual main() function location
            "soccer_object_detection = soccer_object_detection.object_detect_node_ros:main",
        ],
    },
)
