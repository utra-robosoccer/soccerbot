import os

print("Modifying .proto file")

input_file = '/home/nam/catkin_ws/src/soccerbot/bez2_description/urdf/Bez2.proto'
target_file = '/home/nam/catkin_ws/src/soccerbot/soccer_webots/robocup/protos/Bez2.proto'
text = ''
with open(input_file, 'r') as proto_file:
    # update path names to STL files
    text = proto_file.read()

    # update STL paths
    text = text.replace('/home/nam/catkin_ws/src/soccerbot/bez2_description/meshes', 'Bez2_meshes')

    # set controller
    text = text.replace('field  SFString    controller      "void"  # Is `Robot.controller`.',
                        'field  SFString    controller      "player"  # Is `Robot.controller`.')

    print(text)

with open(target_file, 'w') as proto_file:
    proto_file.write(text)