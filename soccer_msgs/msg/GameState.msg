# This message provides all information from the game controller
# for additional information see documentation of the game controller
# https://github.com/bhuman/GameController


std_msgs/Header header

int8 GAMESTATE_INITIAL=0
int8 GAMESTATE_READY=1
int8 GAMESTATE_SET=2
int8 GAMESTATE_PLAYING=3
int8 GAMESTATE_FINISHED=4
int8 gamestate

# Secondary state, penaltyshoot is penalty shootout at the end of the game,
# penaltykick is a kick during the game
int8 STATE_NORMAL = 0
int8 STATE_PENALTYSHOOT = 1
int8 STATE_OVERTIME = 2
int8 STATE_TIMEOUT = 3
int8 STATE_DIRECT_FREEKICK = 4
int8 STATE_INDIRECT_FREEKICK = 5
int8 STATE_PENALTYKICK = 6
int8 STATE_CORNER_KICK = 7
int8 STATE_GOAL_KICK = 8
int8 STATE_THROW_IN = 9
int8 secondary_state

# For newest version of game controller
# Tells which team has the free kick or penalty kick
int8 secondary_state_team
# The secondary state contains a sub mode in which phase of execution the secondary state is
int8 MODE_PREPARATION = 0
int8 MODE_PLACING = 1
int8 MODE_END = 2
int8 secondary_state_mode

bool first_half
int8 own_score
int8 rival_score

# Seconds remaining for the game half
int16 seconds_remaining
# Seconds remaining for things like kickoff
int32 secondary_seconds_remaining

bool has_kick_off

uint8 PENALTY_NONE = 0
uint8 PENALTY_HL_BALL_MANIPULATION = 30
uint8 PENALTY_HL_PHYSICAL_CONTACT = 31
uint8 PENALTY_HL_ILLEGAL_ATTACK = 32
uint8 PENALTY_HL_ILLEGAL_DEFENSE = 33
uint8 PENALTY_HL_PICKUP_OR_INCAPABLE = 34
uint8 PENALTY_HL_SERVICE = 35
uint16 penalty

uint16 seconds_till_unpenalized

# Team colors
int8 TEAM_COLOR_BLUE = 0
int8 TEAM_COLOR_RED = 1
int8 team_color

bool drop_in_team
uint16 drop_in_time

# The number of the current penalty shot during penalty shootout
int8 penalty_shot
# a binary pattern indicating the successful penalty shots (1 for successful, 0 for unsuccessful)
uint16 single_shots

string coach_message
