#!/usr/bin/env python
# -*- coding:utf-8 -*-

from construct import Byte, Struct, Enum, Bytes, Const, Array, Int16ul, Int32ul, PaddedString, Flag, Int16sl

Short = Int16ul

RobotInfo = "robot_info" / Struct(
    # define NONE                        0
    # define PENALTY_HL_KID_BALL_MANIPULATION    1
    # define PENALTY_HL_KID_PHYSICAL_CONTACT     2
    # define PENALTY_HL_KID_ILLEGAL_ATTACK       3
    # define PENALTY_HL_KID_ILLEGAL_DEFENSE      4
    # define PENALTY_HL_KID_REQUEST_FOR_PICKUP   5
    # define PENALTY_HL_KID_REQUEST_FOR_SERVICE  6
    # define PENALTY_HL_KID_REQUEST_FOR_PICKUP_2_SERVICE 7
    # define MANUAL                      15
    "penalty" / Byte,
    "secs_till_unpenalized" / Byte,
    "number_of_warnings" / Byte,
    "number_of_yellow_cards" / Byte,
    "number_of_red_cards" / Byte,
    "goalkeeper" / Flag
)

TeamInfo = "team" / Struct(
    "team_number" / Byte,
    "team_color" / Enum(Byte,
                        BLUE=0,
                        RED=1,
                        YELLOW=2,
                        BLACK=3,
                        WHITE=4,
                        GREEN=5,
                        ORANGE=6,
                        PURPLE=7,
                        BROWN=8,
                        GRAY=9
                        ),
    "score" / Byte,
    "penalty_shot" / Byte,  # penalty shot counter
    "single_shots" / Short,  # bits represent penalty shot success
    "coach_sequence" / Byte,
    "coach_message" / PaddedString(253, 'utf8'),
    "coach" / RobotInfo,
    "players" / Array(11, RobotInfo)
)

GameState = "gamedata" / Struct(
    "header" / Const(b'RGme'),
    "version" / Const(12, Short),
    "packet_number" / Byte,
    "players_per_team" / Byte,
    "game_type" / Byte,
    "game_state" / Enum(Byte,
                        STATE_INITIAL=0,
                        # auf startposition gehen
                        STATE_READY=1,
                        # bereithalten
                        STATE_SET=2,
                        # spielen
                        STATE_PLAYING=3,
                        # spiel zu ende
                        STATE_FINISHED=4
                        ),
    "first_half" / Flag,
    "kick_of_team" / Byte,
    "secondary_state" / Enum(Byte,
                             STATE_NORMAL=0,
                             STATE_PENALTYSHOOT=1,
                             STATE_OVERTIME=2,
                             STATE_TIMEOUT=3,
                             STATE_DIRECT_FREEKICK=4,
                             STATE_INDIRECT_FREEKICK=5,
                             STATE_PENALTYKICK=6,
                             STATE_CORNERKICK=7,
                             STATE_GOALKICK=8,
                             STATE_THROWIN=9,
                             DROPBALL=128,
                             UNKNOWN=255
                             ),
    "secondary_state_info" / Bytes(4),
    "drop_in_team" / Flag,
    "drop_in_time" / Short,
    "seconds_remaining" / Int16sl,
    "secondary_seconds_remaining" / Int16sl,
    "teams" / Array(2, "team" / TeamInfo)
)

GAME_CONTROLLER_RESPONSE_VERSION = 2

ReturnData = Struct(
    "header" / Const(b"RGrt"),
    "version" / Const(2, Byte),
    "team" / Byte,
    "player" / Byte,
    "message" / Byte
)