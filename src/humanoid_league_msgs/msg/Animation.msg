Header header

# This is a request to make HCM controllable, e.g. stop walking
bool request

# First message of this animation
bool first

# Last message of this animation
bool last

# Is this animation comming from the hardware control manager
bool hcm

# Joint goals
trajectory_msgs/JointTrajectory position
