# This message contains information provided by the RoboCup Protocol for team communication.
# Everything is in SI units (ROS standard).
# Set belief values to 0 if object was not recognized.
# More information here: https://github.com/bit-bots/RobocupProtocol

std_msgs/Header header

uint8 robot_id

uint8 STATE_UNKNOWN=0
uint8 STATE_UNPENALIZED=1
uint8 STATE_PENALIZED=2
uint8 state

# Absolute position value (in the map frame)
geometry_msgs/PoseWithCovariance robot_position

# ball position, rotation is not used (in the map frame)
geometry_msgs/PoseWithCovariance ball_absolute

