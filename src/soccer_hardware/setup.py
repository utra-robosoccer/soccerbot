from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['soccer_hardware'],
    scripts=['nodes/soccer_hardware.py'],
    package_dir={'': 'src'},
    requires=['std_msgs', 'rospy', 'message_filters', 'gps_common', 'sensor_msgs', 'prettytable']
)

setup(**d)