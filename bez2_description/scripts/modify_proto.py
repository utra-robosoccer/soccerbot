import os

print("Modifying .proto file")

input_file = '/home/nam/catkin_ws/src/soccerbot/bez2_description/urdf/Bez2.proto'
target_file = '/home/nam/catkin_ws/src/soccerbot/soccer_webots/robocup/protos/Bez2.proto'
text = ''
with open(input_file, 'r') as proto_file:
    text = proto_file.read()

    # update STL paths
    text = text.replace('/home/nam/catkin_ws/src/soccerbot/bez2_description/meshes', 'Bez2_meshes')

    # set controller
    text = text.replace('"left_arm_motor_0"', '"left_arm_motor_0 [shoulder]"')
    text = text.replace('"right_arm_motor_0"', '"right_arm_motor_0 [shoulder]"')
    text = text.replace('"left_leg_motor_1"', '"left_leg_motor_1 [hip]"')
    text = text.replace('"right_leg_motor_1"', '"right_leg_motor_1 [hip]"')

    # add tags to the motors
    text = text.replace('field  SFString    controller      "void"  # Is `Robot.controller`.',
                        'field  SFString    controller      "player"  # Is `Robot.controller`.')

    # insert camera
    target_text = '''children [
                            Transform {
                              translation -0.075313 -0.007794 -0.229048
                              scale 0.001000 0.001000 0.001000
                              children [
                                Shape {
                                  appearance USE steel_satin
                                  geometry DEF camera_point_1 Mesh {
                                    url "Bez2_meshes/camera_point_1.stl"
                                  }
                                }
                              ]
                            }
                          ]'''
    repalce_text = '''rotation 0.000000 1.000000 -0.000000 3.14159
                          children [
                            Transform {
                              translation -0.075313 -0.007794 -0.229048
                              scale 0.001000 0.001000 0.001000
                              children [
                                Shape {
                                  appearance USE steel_satin
                                  geometry DEF camera_point_1 Mesh {
                                    url "Bez2_meshes/camera_point_1.stl"
                                  }
                                }
                              ]
                            }
                          ]
                          children [
                            DEF CAMERA Camera {
                                rotation  1 0 0 -1.5708
                                name "camera"
                                fieldOfView 1.39626
                                width 640
                                height 480
                                noise 0.007000
                                near 0.2
                                far 10
                            }
                          ]'''
    text = text.replace(target_text, repalce_text)

    # add imu
    target_text = '''children [
                Shape {
                  appearance USE steel_satin
                  geometry DEF imu_1 Mesh {
                    url "Bez2_meshes/imu_1.stl"
                  }
                }
              ]'''
    repalce_text = '''children [
                Accelerometer {
                  name "imu accelerometer"
                  lookupTable [
                  -19.62 -32768 0.00203
                  19.62 32767 0.00203
                  ]
                  resolution 1
                }
                Gyro {
                  name "imu gyro"
                  lookupTable [
                    -8.7266 -32768 0.00698
                    8.7266 32767 0.00698
                  ]
                  resolution 1
                }
                Shape {
                  appearance USE steel_satin
                  geometry DEF imu_1 Mesh {
                    url "Bez2_meshes/imu_1.stl"
                  }
                }
              ]'''
    text = text.replace(target_text, repalce_text)

with open(target_file, 'w') as proto_file:
    proto_file.write(text)