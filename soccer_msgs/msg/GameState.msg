# This message provides all information from the game controller
# for additional information see documentation of the game controller
# https://github.com/bhuman/GameController


std_msgs/Header header

uint8 GAMESTATE_INITAL=0
uint8 GAMESTATE_READY=1
uint8 GAMESTATE_SET=2
uint8 GAMESTATE_PLAYING=3
uint8 GAMESTATE_FINISHED=4
uint8 gameState

# Secondary state, penaltyshoot is penalty shootout at the end of the game,
# penaltykick is a kick during the game
uint8 STATE_NORMAL = 0
uint8 STATE_PENALTYSHOOT = 1
uint8 STATE_OVERTIME = 2
uint8 STATE_TIMEOUT = 3
uint8 STATE_DIRECT_FREEKICK = 4
uint8 STATE_INDIRECT_FREEKICK = 5
uint8 STATE_PENALTYKICK = 6
uint8 STATE_CORNER_KICK = 7
uint8 STATE_GOAL_KICK = 8
uint8 STATE_THROW_IN = 9
uint8 secondaryState

# For newest version of game controller
# Tells which team has the free kick or penalty kick
uint8 secondaryStateTeam
# The secondary state contains a sub mode in which phase of execution the secondary state is
uint8 MODE_PREPARATION = 0
uint8 MODE_PLACING = 1
uint8 MODE_END = 2
uint8 secondaryStateMode

bool firstHalf
uint8 ownScore
uint8 rivalScore

# Seconds remaining for the game half
int16 secondsRemaining
# Seconds remaining for things like kickoff
uint16 secondary_seconds_remaining

bool hasKickOff
bool penalized
uint16 secondsTillUnpenalized
# Allowed to move is different from penalized.
# You can for example be not allowed to move due to the current state of the game
bool allowedToMove

# Team colors
uint8 BLUE = 0
uint8 RED = 1
uint8 teamColor

bool dropInTeam
uint16 dropInTime

# The number of the current penalty shot during penalty shootout
uint8 penaltyShot
# a binary pattern indicating the successful penalty shots (1 for successful, 0 for unsuccessful)
uint16 singleShots

string coach_message