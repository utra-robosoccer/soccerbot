# This message is used for communicating between the body behaviour and the head behaviour
# The body tells the head by this message what it shall do

# Search for Ball and track it if found
uint8 BALL_MODE=0
# Search for goal posts, mainly to locate the robot on the field
uint8 POST_MODE=1
# Track ball and goal by constantly switching between both
uint8 BALL_GOAL_TRACKING=2
# Look generally for all features on the field (ball, goals, corners, center point)
uint8 FIELD_FEATURES=3
# Look for features outside of the field (perimeter advertising, walls, etc).
# Can be used for localization using features on the ceiling.
uint8 NON_FIELD_FEATURES=4
# Simply look down to its feet.
uint8 LOOK_DOWN=5
# Simply look directly forward
uint8 LOOK_FORWARD=7
#Don't move the head
uint8 DONT_MOVE=8
# Look to the ceiling, for example for visual compas
uint8 LOOK_UP=9
uint8 headMode