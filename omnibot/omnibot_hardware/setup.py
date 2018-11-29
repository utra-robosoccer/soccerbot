from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['omnibot_communication'],
    package_dir={'': 'src'},
    requires=['serial','std_msgs', 'rospy', 'message_filters', 'gps_common', 'prettytable']
)

setup(**d)
