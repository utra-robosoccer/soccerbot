from glob import glob

from setuptools import find_packages, setup

package_name = "soccer_firmware_interface"

setup(
    name=package_name,
    version="0.0.0",
    install_requires=["setuptools"],
    zip_safe=True,
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch")),
        ("share/" + package_name + "/launch", glob("launch/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    maintainer="Jonathan Spraggett",
    maintainer_email="jonathanspraggett@gmail.com",
    description="The soccer_firmware_interface package",
    license="BSD",
    entry_points={
        "console_scripts": [
            "soccer_firmware = soccer_firmware_interface.firmware_interface:main",
            "soccer_tf = soccer_firmware_interface.transform:main",
        ],
    },
)
