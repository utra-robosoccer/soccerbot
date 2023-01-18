import os
from os.path import expanduser

home = expanduser("~")

input_file = home + "/catkin_ws/src/soccerbot/bez2_description/urdf/Bez2.proto"
target_file = (
    home
    + "/catkin_ws/src/soccerbot/bez2_description/urdf/Bez2.proto"
    # home + "/catkin_ws/src/soccerbot/soccer_webots/robocup/protos/Bez1.proto"
)  # home + '/catkin_ws/src/soccerbot/soccer_webots/robocup/protos/Bez2.proto'
text = ""
key = "device"

# 1. Get all motor names
motor_names = []
with open(input_file, "r") as proto_file:
    text = proto_file.readlines()
    for l_no, line in enumerate(text):
        # check if string present on a current line
        if line.find(key) != -1:
            init = text[l_no + 2].strip()[6:-1]
            motor_names.append(init)
            # print(init)
            # temp = init.split(" ")
            # temp = [float(i) * 0.001 for i in temp]
            # temp = " ".join(str(x) for x in temp)
            # print(temp)
            # text[l_no + 1] = "size " + temp


# 2. Classify. 3 = xc430, 2 = xm430, 1 = mx64
motor_type = [3, 3, 3, 3, 2, 2, 2, 1, 1, 1, 2, 2, 2, 1, 1, 1, 3, 3]
motor_min = [
    -1.57079632679,
    0,
    -1.57079632679,
    0,
    -1.309,
    -0.785398163397,
    -0.785398163397,
    -2.793,
    -0.785398163397,
    -0.785398163397,
    -1.309,
    -0.785398163397,
    -0.785398163397,
    -2.793,
    -0.785398163397,
    -0.785398163397,
    -1.57079632679,
    -2.35619449019,
]
motor_max = [
    3.92699081699,
    3.14159265359,
    3.92699081699,
    3.14159265359,
    0.524,
    1.57079632679,
    2.35619449019,
    0,
    1.57079632679,
    0.785398163397,
    0.524,
    1.57079632679,
    2.35619449019,
    0,
    1.57079632679,
    0.785398163397,
    1.57079632679,
    2.35619449019,
]
# 3. Add motor specs
motor_count = 0

key2 = "device"
with open(input_file, "r") as proto_file:
    text = proto_file.readlines()
    for l_no, line in enumerate(text):
        # check if string present on a current line
        if line.find(key2) != -1:

            if motor_count >= 18:
                break

            # 1. verify name
            init = text[l_no + 2].strip()[6:-1]
            if init == motor_names[motor_count]:
                hinge_init = -1
                hinge_end = -1
                ps_init = -1
                ps_end = -1
                d_end = -1

                for i in range(0, 10):
                    if text[l_no - i].find("HingeJoint {") != -1:
                        hinge_init = l_no - i
                        break

                    if text[l_no - i].find("}") != -1:
                        hinge_end = l_no - i

                for i in range(0, 10):
                    if text[l_no + i].find("PositionSensor {") != -1:
                        ps_init = l_no + i

                    if text[l_no + i].find("}") != -1 and ps_init >= 0:
                        ps_end = l_no + i
                        break

                for i in range(0, 10):

                    if text[l_no + i].find("maxTorque 10000") != -1 and ps_init >= 0:
                        d_end = l_no + i
                        break

                if motor_type[motor_count] == 1:  # mx64
                    text[hinge_init] = "HingeJointWithBacklash {\n"
                    text[hinge_end] = (
                        "      dampingConstant %{= mx64_damping_constant }%\n     staticFriction %{"
                        "= mx64_static_friction }%\n}\n backlash %{= mx64_backlash }%\n"
                    )

                    text[d_end] = (
                        "      maxVelocity %{= mx64_max_velocity }%\n     maxTorque %{= mx64_max_torque }%"
                        "\n        minPosition " + str(motor_min[motor_count]) + "\n      maxPosition " + str(motor_max[motor_count]) + "\n"
                    )
                    text[ps_end] = "     resolution %{= mx64_resolution }%\n}\n"
                elif motor_type[motor_count] == 2:  # xm430
                    text[hinge_init] = "HingeJointWithBacklash {\n"
                    text[hinge_end] = (
                        "      dampingConstant %{= xm430_damping_constant }%\n     staticFriction %{"
                        "= xm430_static_friction }%\n}\n backlash %{= xm430_backlash }%\n"
                    )

                    text[d_end] = (
                        "      maxVelocity %{= xm430_max_velocity }%\n     maxTorque %{= xm430_max_torque }%"
                        "\n        minPosition " + str(motor_min[motor_count]) + "\n      maxPosition " + str(motor_max[motor_count]) + "\n"
                    )
                    text[ps_end] = "     resolution %{= xm430_resolution }%\n}\n"
                elif motor_type[motor_count] == 3:  # xc430
                    text[hinge_init] = "HingeJointWithBacklash {\n"
                    text[hinge_end] = (
                        "      dampingConstant %{= xc430_damping_constant }%\n     staticFriction %{"
                        "= xc430_static_friction }%\n}\n backlash %{= xc430_backlash }%\n"
                    )

                    text[d_end] = (
                        "      maxVelocity %{= xc430_max_velocity }%\n     maxTorque %{= xc430_max_torque }%"
                        "\n        minPosition " + str(motor_min[motor_count]) + "\n      maxPosition " + str(motor_max[motor_count]) + "\n"
                    )
                    text[ps_end] = "     resolution %{= xc430_resolution }%\n}\n"
                motor_count += 1

with open(target_file, "w") as proto_file:
    proto_file.writelines(text)
