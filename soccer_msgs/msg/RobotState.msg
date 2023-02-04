# This message contains information provided by the RoboCup Protocol for team communication.
# Everything is in SI units (ROS standard).
# Set belief values to 0 if object was not recognized.
# More information here: https://github.com/bit-bots/RobocupProtocol

std_msgs/Header header

uint8 player_id

uint8 STATUS_DISCONNECTED = 0
uint8 STATUS_READY = 1
uint8 STATUS_LOCALIZING = 2
uint8 STATUS_WALKING = 3
uint8 STATUS_TERMINATING_WALK = 4
uint8 STATUS_KICKING = 5
uint8 STATUS_FALLEN_FRONT = 6
uint8 STATUS_FALLEN_BACK = 7
uint8 STATUS_FALLEN_SIDE = 8
uint8 STATUS_PENALIZED = 9
uint8 STATUS_TRAJECTORY_IN_PROGRESS = 11
uint8 STATUS_STOPPED = 12
uint8 STATUS_DETERMINING_SIDE = 13
uint8 status

uint8 ROLE_UNASSIGNED = 0
uint8 ROLE_GOALIE = 1
uint8 ROLE_STRIKER = 2
uint8 ROLE_LEFT_WING = 3
uint8 ROLE_RIGHT_WING = 4
uint8 role

# Absolute position value (in the map frame)
geometry_msgs/Pose pose
bool localized

# ball position, rotation is not used (in the map frame)
geometry_msgs/Pose2D ball_pose
bool ball_detected

# Observed obstacles
geometry_msgs/Pose2D[] obstacles_poses
float32[] obstacles_sizes
float32[] obstacles_probabilities
