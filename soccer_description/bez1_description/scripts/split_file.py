import os
from os.path import expanduser

home = expanduser("~")

target_file = (
    home
    + "/ros2_ws/src/soccerbot/bez1_description/urdf/Bez1Robocup.proto"
    # home + "/ros2_ws/src/soccerbot/soccer_webots/robocup/protos/Bez1.proto"
)  # home + '/ros2_ws/src/soccerbot/soccer_webots/robocup/protos/Bez2.proto'


with open(target_file, "w") as proto_file:
    text = """#VRML_SIM R2022b utf8
# license: Apache License 2.0
# license url: http://www.apache.org/licenses/LICENSE-2.0
# This is a proto file for Webots for the WolfgangRobocup

EXTERNPROTO "Bez1_meshes/Bez1.proto"

PROTO Bez1Robocup [
  field  SFVec3f     translation     0 0 0
  field  SFRotation  rotation        0 1 0 0
  field  SFString    name            "red player 1"  # Is `Robot.name`.
  field  MFString    controllerArgs  []          # Is `Robot.controllerArgs`.
  field  SFString    customData      ""
]
{
Bez {
translation IS translation
    rotation IS rotation
    name IS name
    controllerArgs IS controllerArgs
    customData IS customData
    controller "player"
    }
}"""
    proto_file.write(text)
