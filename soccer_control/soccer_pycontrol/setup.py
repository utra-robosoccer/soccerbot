# from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup, find_packages

print(find_packages('src'))
# fmt: off
setup(
    packages=find_packages('src'),
    version="0.0.0",
    scripts=[],
    package_dir={'': 'src/'}
)
# fmt: on
