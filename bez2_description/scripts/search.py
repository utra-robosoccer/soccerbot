import os
from os.path import expanduser

home = expanduser("~")

input_file = home + "/catkin_ws/src/soccerbot/bez2_description/urdf/Bez2.proto"
target_file = (
    home + "/catkin_ws/src/soccerbot/bez2_description/urdf/Bez2.proto"
)  # home + '/catkin_ws/src/soccerbot/soccer_webots/robocup/protos/Bez2.proto'
text = ""
key = "boundingObject Box {"
with open(input_file, "r") as proto_file:

    text = proto_file.readlines()
    for l_no, line in enumerate(text):
        # check if string present on a current line
        if line.find(key) != -1:
            init = text[l_no + 1].strip()[5:]
            print(init)
            temp = init.split(" ")
            temp = [float(i) * 0.001 for i in temp]
            temp = " ".join(str(x) for x in temp)
            print(temp)
            text[l_no + 1] = "size " + temp

with open(target_file, "w") as proto_file:
    proto_file.writelines(text)
