# This message provides information about the current strategy of the robot to the team communication so that it can be
# shared with other team robots

# Which role the robot has currently
uint8 ROLE_IDLING=0
uint8 ROLE_OTHER=1
uint8 ROLE_STRIKER=2
uint8 ROLE_SUPPORTER=3
uint8 ROLE_DEFENDER=4
uint8 ROLE_GOALIE=5
uint8 role

# The current action of the robot
uint8 ACTION_UNDEFINED=0
uint8 ACTION_POSITIONING=1
uint8 ACTION_GOING_TO_BALL=2
uint8 ACTION_TRYING_TO_SCORE=3
uint8 ACTION_WAITING=4
uint8 action

# Offensive strategy
uint8 SIDE_LEFT = 0
uint8 SIDE_MIDDLE = 1
uint8 SIDE_RIGHT = 2
uint8 offensive_side