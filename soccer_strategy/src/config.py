from soccer_msgs.msg import GameState
from robot import Robot

# Changes over time
field_side = True

# position where the robot is spawned at
FORMATIONS = {
    "initial": {
        Robot.Role.GOALIE: [-1.5, 0, 0],
        Robot.Role.STRIKER: [-2, 0, 1.57],
        Robot.Role.RIGHT_WING: [-1, 2.5, 0],
        Robot.Role.LEFT_WING: [-1.5, 1.75, 0]
    },
    "attack": {
        Robot.Role.GOALIE: [4.5, 0, 0],
        Robot.Role.STRIKER: [-2, 0, 0],
        Robot.Role.RIGHT_WING: [-2, 2.5, 0],
        Robot.Role.LEFT_WING: [-2, -2.5, 0]
    },
    "defensive": {
        Robot.Role.GOALIE: [4.5, 0, 0],
        Robot.Role.STRIKER: [3.5, 0, 0],
        Robot.Role.RIGHT_WING: [3.5, 2, 0],
        Robot.Role.LEFT_WING: [3, -2, 0]
    },
    "midfield": {
        Robot.Role.GOALIE: [4.5, 0, 0],
        Robot.Role.STRIKER: [0, 0, 0],
        Robot.Role.RIGHT_WING: [0, 3, 0],
        Robot.Role.LEFT_WING: [0, -3, 0]
    },
    "penalty_give": {
        Robot.Role.GOALIE: [-4.5, 0, 0],
        Robot.Role.STRIKER: [3.5, 0, 0]
    },
    "penalty_take": {
        Robot.Role.GOALIE: [-4.5, 0, 0]
    }
}

ENEMY_GOAL_POSITION = [4.8, 0]

def flip_positions():
    ENEMY_GOAL_POSITION[0] = -ENEMY_GOAL_POSITION[0]
    for formation in FORMATIONS:
        for role in FORMATIONS[formation]:
            FORMATIONS[formation][role][0] = -FORMATIONS[formation][role][0]
            FORMATIONS[formation][role][0] = -3.14
