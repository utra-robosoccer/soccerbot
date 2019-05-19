from setuptools import setup, find_packages
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    package_dir={'control_center': 'control_center'},
    packages=find_packages(exclude=('test*', 'robot_config', 'launch'))
)

setup(**setup_args)
