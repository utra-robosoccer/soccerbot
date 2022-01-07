
INITIAL_POSITION =\
{
    "blue":
    {
        1: [-1.5, 0, 0],
        2: [-1, 0, 0],
        3: [-1.5, 1.75, 0]
    },
    "red":
    {
        1: [1.5, 0, -3.14],
        2: [1, -2.5, -3.14],
        3: [1.5, 1.75, -3.14]
    }
}

START_POSITION =\
{
    "blue":
    {
        1: [-4, -3.15, 1.57],
        2: [-1, -3.15, 1.57],
        3: [-4, 3.15, -1.57]
    },
    "red":
    {
        1: [4, -3.15, 1.57],
        2: [1, -3.15, 1.57],
        3: [4, 3.15, -1.57]
    }
}

PENALTYKICK_NON_KICKING_POSITION =\
{
    "blue":
    {
        1: [-4.5, 0, 0],
        2: [-1, -1, 0],
        3: [-1, 1, 0]
    },
    "red":
    {
        1: [4.5, 0, 3.14],
        2: [1, -1, 3.14],
        3: [1, 1, 3.14]
    }
}

PENALTYSHOOT_START_POSITION =\
{
    "kicker": [-2.6, 0, 3.14],
    "goalie": [-4.5, 0, 0]
}

ENEMY_GOAL_POSITION =\
{
    "blue": [5, 0],
    "red": [-5, 0],
    "penalty_shoot": [-4.5, 0]
}


def position_map(position_dict, team_color, first_half, robot_id):
    if first_half == 1:
        if team_color == 0:
            return position_dict["blue"][robot_id]
        elif team_color == 1:
            return position_dict["red"][robot_id]
    else:
        if team_color == 0:
            return position_dict["red"][robot_id]
        elif team_color == 1:
            return position_dict["blue"][robot_id]


def position_map_kickoff(position_dict, has_kickoff):
    if has_kickoff:
        return position_dict["kicker"]
    else:
        return position_dict["goalie"]


def position_map_goal(position_dict, team_color, first_half, penalty_shoot):
    if penalty_shoot:
        return position_dict["penalty_shoot"]

    if first_half == 1:
        if team_color == 0:
            return position_dict["blue"]
        elif team_color == 1:
            return position_dict["red"]
    else:
        if team_color == 0:
            return position_dict["red"]
        elif team_color == 1:
            return position_dict["blue"]
