# nav position for the robot before the game start


INITIAL_POSITION =\
{
    "Negative":
    {
        1: [-1.5, 0, -3.14],
        2: [-1, -2.5, -3.14],
        3: [-1, 2.5, -3.14],
        4: [-1.5, 1.75, -3.14]
    },
    "Positive":
    {
        1: [1.5, 0, -3.14],
        2: [1, -2.5, -3.14],
        3: [1, 2.5, -3.14],
        4: [1.5, 1.75, -3.14]
    }
}

# position where the robot is spawned at
START_POSITION =\
{
    "Negative":
    {
        1: [-4, -3.15, 1.57],
        2: [-1, -3.15, 1.57],
        3: [-1, 3.15, -1.57],
        4: [-4, 3.15, -1.57]
    },
    "Positive":
    {
        1: [4, -3.15, 1.57],
        2: [1, -3.15, 1.57],
        3: [1, 3.15, -1.57],
        4: [4, 3.15, -1.57]
    }
}

# todo need to be changed
PENALTYKICK_NON_KICKING_POSITION =\
{
    "Negative":
    {
        1: [-4.5, 0, 0],
        2: [-1, -1, 0],
        3: [-1, 1, 0]
    },
    "Positive":
    {
        1: [4.5, 0, 3.14],
        2: [1, -1, 3.14],
        3: [1, 1, 3.14]
    }
}
# todo need to be changed
PENALTYSHOOT_START_POSITION =\
{
    "kicker": [-2.6, 0, 3.14],
    "goalie": [-4.5, 0, 0]
}

ENEMY_GOAL_POSITION =\
{
    "Negative": [5, 0],
    "Positive": [-5, 0],
    "penalty_shoot": [-4.5, 0]
}


def position_map(position_dict, team_color, first_half, robot_id):
    if first_half == 1:
        if team_color == 0:
            return position_dict["Negative"][robot_id]
        elif team_color == 1:
            return position_dict["Positive"][robot_id]
    else:
        if team_color == 0:
            return position_dict["Positive"][robot_id]
        elif team_color == 1:
            return position_dict["Negative"][robot_id]


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
            return position_dict["Negative"]
        elif team_color == 1:
            return position_dict["Positive"]
    else:
        if team_color == 0:
            return position_dict["Positive"]
        elif team_color == 1:
            return position_dict["Negative"]



from decisions.is_closest_to_ball import IsClosestToBall
from actions.go_to_formation_position import GoToFormationPosition
from actions.go_to_ball import GoToBall
from tree.decision_tree import DecisionTree
from actions.kick import Kick
from decisions.can_kick import CanKick
from actions.stand_still import StandStill

#TODO different decision tree for each robot, ex: goalie

#don't think this is right because it will only create one instance
#proper way should be to write the high level data (XML) here then build the tree on the spot in the strategy constructor
BasicDecisionTree = DecisionTree(
            IsClosestToBall([
                CanKick([
                    Kick(),
                    GoToBall()
                ]),
                GoToFormationPosition()
            ])
        )

StationaryDecisionTree = DecisionTree(StandStill())
