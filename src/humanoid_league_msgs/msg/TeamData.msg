# This message contains all information provided by the mitecom standard for team communication.
# Everything is in meters (ROS standard) not to be confused with millimeters (mitecom standard)!
# Set belief values to 0 if object was not recognized.
# More information here: https://github.com/fumanoids/mitecom

std_msgs/Header header

# Every value is an array because we can have multiple robots communicating with us.
# The values match with the robot ids
uint8[] robot_ids

uint8 ROLE_IDLING=0
uint8 ROLE_OTHER=1
uint8 ROLE_STRIKER=2
uint8 ROLE_SUPPORTER=3
uint8 ROLE_DEFENDER=4
uint8 ROLE_GOALIE=5
uint8[] role

uint8 ACTION_UNDEFINED=0
uint8 ACTION_POSITIONING=1
uint8 ACTION_GOING_TO_BALL=2
uint8 ACTION_TRYING_TO_SCORE=3
uint8 ACTION_WAITING=4
uint8[] action

uint8 STATE_INACTIVE=0
uint8 STATE_ACTIVE=1
uint8 STATE_PENALIZED=2
uint8[] state

# Absolute position values
geometry_msgs/Pose2D[] robot_positions

# Relative ball position, theta of Pose2D is not used
Position2D[] ball_relative

# Relative position of the opponent goal, theta of Pose2D is not used
# This is helpful if the robot has no global position, but sees the goal
Position2D[] oppgoal_relative

# Positions of opponent robots, if they are recognized
# The letter of the robot is arbitrary as the sending robot does not know the id of a seen robot
Position2D[] opponent_robot_a
Position2D[] opponent_robot_b
Position2D[] opponent_robot_c
Position2D[] opponent_robot_d

# Positions of team robots, if they are recognized
# The letter of the robot is arbitrary as the sending robot does not know the id of a seen robot
Position2D[] team_robot_a
Position2D[] team_robot_b
Position2D[] team_robot_c

float32[] avg_walking_speed
float32[] time_to_position_at_ball
float32[] max_kicking_distance

# Strategy over which side the team tries to attack
# Especially useful during a kickoff
uint8 UNSPECIFIED=0
uint8 LEFT=1
uint8 RIGHT=2
uint8 CENTER=3
uint8[] offensive_side