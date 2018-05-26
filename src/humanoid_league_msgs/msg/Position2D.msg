# The position system is the same as mitecom. The following part is taken from the mitecom documentation:
# https://github.com/fumanoids/mitecom
# The origin of the absolute coordinate center is the center of the middle
# circle (center of field). The x axis points towards the opponent goal, the
# y axis to the left.
#
#      y
#      ^       ______________________
#      |    M  |          |          |  O
#      |    Y  |_ -x, y   |   x, y  _|  P
#      |    G  | |        |        | |  P
# 0    +    O  | |       ( )       | |  G
#      |    A  |_|        |        |_|  O
#      |    L  |  -x,-y   |   x,-y   |  A
#      |       |__________|__________|  L
#      |
#      +------------------+--------------> x
#                         0
#
# The 0 value of the orientation is pointing to the opponent side (right side in the image).
# The value increases counter clockwise
# Everything in meters (because it is the ROS standard)

# The header is included to get the time stamp for later use in tf
std_msgs/Header header

geometry_msgs/Pose2D pose
# A certainty rating between 0 and 1, where 1 is the surest.
float32 confidence
