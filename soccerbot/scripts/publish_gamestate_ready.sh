#!/usr/bin/env bash

rostopic pub -1 /robot1/gamestate soccer_msgs/GameState "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
gameState: 3
secondaryState: 0
secondaryStateTeam: 0
secondaryStateMode: 0
firstHalf: false
ownScore: 0
rivalScore: 0
secondsRemaining: 0
secondary_seconds_remaining: 0
hasKickOff: false
penalized: false
secondsTillUnpenalized: 0
teamColor: 0
dropInTeam: false
dropInTime: 0
penaltyShot: 0
singleShots: 0
coach_message: ''"