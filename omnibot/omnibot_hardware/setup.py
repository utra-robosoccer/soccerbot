from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['soccer_communication'],
    package_dir={'': 'src'},
    requires=['soccer_msgs','serial','std_msgs', 'rospy', 'message_filters', 'gps_common', 'sensor_msgs', 'prettytable']
)

setup(**d)
