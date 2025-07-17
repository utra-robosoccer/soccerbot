from distutils.core import setup

from ros2_pkg.python_setup import generate_distutils_setup

# fmt: off
d = generate_distutils_setup(
    packages=['soccer_pycontrol'],
    scripts=[],
    package_dir={'': 'src'}
)
# fmt: on

setup(**d)
