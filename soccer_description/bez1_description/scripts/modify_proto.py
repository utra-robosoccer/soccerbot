import os
from os.path import expanduser

print("Modifying .proto file")
home = expanduser("~")

input_file = home + "/ros2_ws/src/soccerbot/bez1_description/urdf/Bez1.proto"
target_file = (
    home + "/ros2_ws/src/soccerbot/bez1_description/urdf/Bez1.proto"
)  # home + '/ros2_ws/src/soccerbot/soccer_webots/robocup/protos/Bez2.proto'
text = ""
with open(input_file, "r") as proto_file:
    text = proto_file.read()

    # Export
    target_text = """PROTO Bez1 ["""

    replace_text = """EXTERNPROTO "HingeJointWithBacklash.proto"
EXTERNPROTO "JerseyFront.proto"
EXTERNPROTO "JerseyBack.proto"

PROTO Bez1 ["""
    text = text.replace(target_text, replace_text)

    # Jersey
    target_text = """Shape {
        appearance DEF black PBRAppearance {
          baseColor 0.500000 0.500000 0.500000
          roughness 1.000000
          metalness 0
        }
        geometry DEF torso Mesh {
          url "/home/manx52/ros2_ws/src/soccerbot/bez1_description/bez1_description/meshes/torso.stl"
        }
      }"""

    replace_text = """children [
          %{
            print (fields.name.value)
            if fields.name.value ~= '' then

            -- name is supposed to be something like "red player 2" or "blue player 1"
            local words = {}
            for word in fields.name.value:gmatch("%w+") do table.insert(words, word) end

                color = words[1]
                number = words[3]
                else
                color = "blue"
                number = 3
                end

                local text1 = '"textures/bez_' .. number .. '_' .. color .. '.png"'
                local text2 = '"textures/bez_back_' .. number .. '_' .. color .. '.png"'
          }%
        Transform {
            translation 0.010000 0.000000 0.0100000
            rotation 0.000000 0.000000 0.000000 0
            children [
              JerseyFront {
                jerseyTexture [
                 %{=text1}%
                ]
              }
            ]
          }

          Transform {
            translation -0.020000 0.000000 0.0100000
            rotation 0.000000 0.000000 0.000000 0
            children [
              JerseyBack {
                jerseyTexture [
                 %{=text2}%
                ]
              }
            ]
          }

          Shape {
        appearance DEF black PBRAppearance {
          baseColor 0.500000 0.500000 0.500000
          roughness 1.000000
          metalness 0
        }
        geometry DEF torso Mesh {
          url "/home/manx52/ros2_ws/src/soccerbot/bez1_description/bez1_description/meshes/torso.stl"
        }
      }
            """
    text = text.replace(target_text, replace_text)

    # Bounding boxes
    target_text = """name "Arm_Assembly-bez3_v12_2"
                boundingObject Transform {
                  translation -0.007000 0.024337 -0.078700
                  rotation 0.000000 -1.000000 -0.000000 3.141593
                  children [
                    Box {
                       size 0.050000 0.056674 0.197500
                    }
                  ]
                }"""
    replace_text = """name "Arm_Assembly-bez3_v12_2"
                boundingObject Transform {
                  translation -0.012 0.024337 -0.1
                  rotation 0.000000 -1.000000 -0.000000 3.141593
                  children [
                    Box {
                       size 0.04 0.04 0.150
                    }
                  ]
                }"""
    text = text.replace(target_text, replace_text)

    target_text = """name "Arm_Assembly-bez3_v12_1"
                boundingObject Transform {
                  translation -0.007000 -0.024337 -0.078700
                  rotation 0.000000 -1.000000 -0.000000 3.141593
                  children [
                    Box {
                       size 0.050000 0.056674 0.197500
                    }
                  ]
                }"""
    replace_text = """name "Arm_Assembly-bez3_v12_1"
                boundingObject Transform {
                  translation -0.012 -0.024337 -0.1
                  rotation 0.000000 -1.000000 -0.000000 3.141593
                  children [
                    Box {
                       size 0.04 0.04 0.150
                    }
                  ]
                }"""

    text = text.replace(target_text, replace_text)

    target_text = """name "New_right_foot_upperhalf_v3_1"
                                        boundingObject Transform {
                                          translation 0.052160 0.015000 -0.005500
                                          rotation -0.707107 0.000000 0.707107 3.141593
                                          children [
                                            Box {
                                               size 0.043000 0.090000 0.170000
                                            }
                                          ]
                                        }"""
    replace_text = """name "New_right_foot_upperhalf_v3_1"
                                        boundingObject Transform {
                                          translation 0.052160 0.015000 -0.0255
                                          rotation -0.707107 0.000000 0.707107 3.141593
                                          children [
                                            Box {
                                               size 0.004 0.090000 0.170000
                                            }
                                          ]
                                        }"""

    text = text.replace(target_text, replace_text)

    target_text = """name "New_left_foot_upperhalf_v5_1"
                                        boundingObject Transform {
                                          translation 0.052160 -0.015000 -0.005500
                                          rotation 0.000000 1.000000 -0.000000 1.570796
                                          children [
                                            Box {
                                               size 0.043000 0.090000 0.170000
                                            }
                                          ]
                                        }"""
    replace_text = """name "New_left_foot_upperhalf_v5_1 [foot]"
                                        boundingObject Transform {
                                          translation 0.052160 -0.015000 -0.0255
                                          rotation 0.000000 1.000000 -0.000000 1.570796
                                          children [
                                            Box {
                                               size 0.004 0.090000 0.170000
                                            }
                                          ]
                                        }"""

    text = text.replace(target_text, replace_text)

    target_text = """name IS name
    boundingObject Transform {
      translation 0.006000 -0.000000 -0.034713
      rotation 0.000000 1.000000 0.000000 3.141593
      children [
        Box {
           size 0.124000 0.145000 0.220674
        }
      ]
    }"""
    replace_text = """name IS name
    boundingObject Transform {
      translation 0.006000 -0.000000 -0.034713
      rotation 0.000000 1.000000 0.000000 3.141593
      children [
        Box {
           size 0.124000 0.145000 0.15
        }
      ]
    }"""

    text = text.replace(target_text, replace_text)

    # insert camera
    target_text = """HingeJoint {
                    jointParameters HingeJointParameters {
                      anchor 0.034400 0.025674 0.048000
                    }
                    device [
                      RotationalMotor {
                        name "head_camera"
                        maxTorque 10000
                      }
                      PositionSensor {
                        name "head_camera_sensor"
                      }
                    ]
                    endPoint Solid {
                      translation 0.034400 0.025674 0.048000
                      children [
                        Transform {
                          translation -0.067650 -0.003154 -0.157441
                          children [
                            Shape {
                              appearance USE steel_satin
                              geometry DEF camera_point_1 Mesh {
                                url "/home/manx52/ros2_ws/src/soccerbot/bez2_description/meshes/camera_point_1.stl"
                              }
                            }
                          ]
                        }
                      ]
                      name "camera_point_1"
                      boundingObject Transform {
                        translation -0.001000 -0.000000 0.000000
                        rotation 0.707107 0.707107 0.000000 3.141593
                        children [
                          Box {
                             size 0.004000 0.004000 0.004000
                          }
                        ]
                      }
                      physics Physics {
                        density -1
                        mass 0.000502
                        centerOfMass [ -0.001000 -0.000000 0.000000 ]
                      }
                    }
                  }"""
    replace_text = """Solid {
                      translation 0.015000 0.001450 0.047400
                      rotation 0.0 0.0 -1.0 1.5708
                      children [
                        DEF CAMERA Camera {
                            rotation 0.0 0.0 1.0 1.5708
                            name "camera"
                            fieldOfView 1.39626
                            width 640
                            height 480
                            noise 0.007000
                            near 0.2
                            far 10

                        }
                      ]
                      name "camera"
                      physics Physics {
                        density -1
                        mass 0.010000
                        centerOfMass [ 0.000000 0.000000 0.000000 ]
                        inertiaMatrix [
                          1.000000e-09 1.000000e-09 1.000000e-09
                          0.000000e+00 0.000000e+00 0.000000e+00
                        ]
                      }
                  }"""
    text = text.replace(target_text, replace_text)

    # add imu
    target_text = """HingeJoint {
        jointParameters HingeJointParameters {
          axis 0.000000 0.000000 1.000000
          anchor 0.013000 0.000000 0.028950
        }
        device [
          RotationalMotor {
            name "torso_imu"
            maxTorque 10000
          }
          PositionSensor {
            name "torso_imu_sensor"
          }
        ]
        endPoint Solid {
          translation 0.013000 0.000000 0.028950
          children [
            Transform {
              translation -0.013000 0.000000 -0.028950
              children [
                Shape {
                  appearance USE steel_satin
                  geometry DEF imu_1 Mesh {
                    url "/home/manx52/ros2_ws/src/soccerbot/bez2_description/meshes/imu_1.stl"
                  }
                }
              ]
            }
          ]
          name "imu_1"
          boundingObject Transform {
            translation 0.002000 0.000000 -0.043000
            rotation 0.000000 1.000000 -0.000000 1.570796
            children [
              Box {
                 size 0.004000 0.004000 0.004000
              }
            ]
          }
          physics Physics {
            density -1
            mass 0.000502
            centerOfMass [ 0.002000 -0.000000 -0.043000 ]
          }
        }
      }"""
    replace_text = """Solid {
              translation 0.000000 0.000000 0.000000
              rotation 0.000000 1.000000 0.000000 0.000000
              children [

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
              ]
              name "imu_link"
              physics Physics {
                density -1
                mass 0.010000
                centerOfMass [ 0.000000 0.000000 0.000000 ]
                inertiaMatrix [
                  1.000000e-09 1.000000e-09 1.000000e-09
                  0.000000e+00 0.000000e+00 0.000000e+00
                ]
              }
            }"""
    text = text.replace(target_text, replace_text)

    # update main proto fields
    text = text.replace('field  SFString    name            "Bez2"', 'field  SFString    name            ""')
    text = text.replace("  field  SFBool      selfCollision   FALSE   ", "  field  SFBool      selfCollision   TRUE     ")

    # Motor Specs
    target_text = """Robot {
    translation IS translation
    rotation IS rotation
    controller IS controller
    controllerArgs IS controllerArgs
    customData IS customData
    supervisor IS supervisor
    synchronization IS synchronization
    selfCollision IS selfCollision"""

    replace_text = """%{
    local mx64_backlash = 0.01
    local mx64_max_torque = 6.00
    local mx64_max_velocity = 6.60
    local mx64_damping_constant = 0.66
    local mx64_static_friction = 1.42
    local mx64_resolution = 0.00153

    local xm430_backlash = 0.01
    local xm430_max_torque = 4.1
    local xm430_max_velocity = 4.82
    local xm430_damping_constant = 0.721165
    local xm430_static_friction = 0.777
    local xm430_resolution = 0.00153

    local xc430_backlash = 0.01
    local xc430_max_torque = 1.9
    local xc430_max_velocity = 7.33
    local xc430_damping_constant = 0.159155
    local xc430_static_friction = 0.80667
    local xc430_resolution = 0.00153
  }%

  Robot {
    translation IS translation
    rotation IS rotation
    controller IS controller
    controllerArgs IS controllerArgs
    customData IS customData
    supervisor IS supervisor
    synchronization IS synchronization
    selfCollision IS selfCollision
    """
    text = text.replace(target_text, replace_text)

    # update STL paths
    text = text.replace(home + "/ros2_ws/src/soccerbot/bez2_description/meshes/", "")

    # set hands
    target_text = """children [
                  Transform {
                    translation -0.000000 -0.080500 0.090150
                    children [
                      Shape {
                        appearance DEF abs_white PBRAppearance {
                          baseColor 0.500000 0.500000 0.500000
                          roughness 1.000000
                          metalness 0
                        }
                        geometry DEF Arm_Assembly-bez3_v12_2 Mesh {
                          url "Arm_Assembly-bez3_v12_2.stl"
                        }
                      }
                    ]
                  }
                ]"""

    replace_text = """children [
                  Transform {
                    translation -0.000000 -0.080500 0.090150
                    children [
                      Shape {
                        appearance DEF abs_white PBRAppearance {
                          baseColor 0.500000 0.500000 0.500000
                          roughness 1.000000
                          metalness 0
                        }
                        geometry DEF Arm_Assembly-bez3_v12_2 Mesh {
                          url "Arm_Assembly-bez3_v12_2.stl"
                        }
                      }
                    ]
                  }
                  Solid {
                    translation -0.01 0.025 -0.18
                    rotation 0.000000 1.000000 0.000000 0.000000
                    name "left_hand [hand]"
                    children [
                      Shape {
                        appearance PBRAppearance {
                          baseColor 0 0 0
                          roughness 1
                          metalness 0
                        }
                        geometry Box {
                          size 0.005 0.02 0.005
                        }
                      }
                    ]
                    boundingObject Transform {
                      translation 0 0 0
                      rotation 0.000000 1.000000 0.000000 0.000000
                      children [
                        Box {
                          size 0.005 0.02 0.005
                        }
                      ]
                    }
                    physics Physics {
                      density -1
                      mass 0.0001
                      centerOfMass [ 0 0 0 ]
                      inertiaMatrix []
                    }
                  }
                ]"""
    text = text.replace(target_text, replace_text)

    target_text = """children [
                  Transform {
                    translation 0.000000 0.080500 0.090150
                    children [
                      Shape {
                        appearance USE abs_white
                        geometry DEF Arm_Assembly-bez3_v12_1 Mesh {
                          url "Arm_Assembly-bez3_v12_1.stl"
                        }
                      }
                    ]
                  }
                ]"""

    replace_text = """children [
                  Transform {
                    translation 0.000000 0.080500 0.090150
                    children [
                      Shape {
                        appearance USE abs_white
                        geometry DEF Arm_Assembly-bez3_v12_1 Mesh {
                          url "Arm_Assembly-bez3_v12_1.stl"
                        }
                      }
                    ]
                  }
                  Solid {
                    translation -0.01 -0.025 -0.18
                    rotation 0.000000 1.000000 0.000000 0.000000
                    name "right_hand [hand]"
                    children [
                      Shape {
                        appearance PBRAppearance {
                          baseColor 0 0 0
                          roughness 1
                          metalness 0
                        }
                        geometry Box {
                          size 0.005 0.02 0.005
                        }
                      }
                    ]
                    boundingObject Transform {
                      translation 0 0 0
                      rotation 0.000000 1.000000 0.000000 0.000000
                      children [
                        Box {
                          size 0.005 0.02 0.005
                        }
                      ]
                    }
                    physics Physics {
                      density -1
                      mass 0.0001
                      centerOfMass [ 0 0 0 ]
                      inertiaMatrix []
                    }
                  }
                ]"""
    text = text.replace(target_text, replace_text)

    # set Pressure sensors
    target_text = """children [
                                          Transform {
                                            translation 0.069680 -0.036000 0.376050
                                            children [
                                              Shape {
                                                appearance USE abs_white
                                                geometry DEF New_right_foot_upperhalf_v3_1 Mesh {
                                                  url "New_right_foot_upperhalf_v3_1.stl"
                                                }
                                              }
                                            ]
                                          }
                                        ]"""

    replace_text = """children [
                                          Transform {
                                            translation 0.069680 -0.036000 0.376050
                                            children [
                                              Shape {
                                                appearance USE abs_white
                                                geometry DEF New_right_foot_upperhalf_v3_1 Mesh {
                                                  url "New_right_foot_upperhalf_v3_1.stl"
                                                }
                                              }
                                            ]
                                          }
                                          TouchSensor {
                                            translation 0.12 0.052 -0.03
                                            rotation 1.000000 0.000000 0.000000 1.57
                                            name "left_leg_foot_sensor_1"
                                            type "bumper"
                                            resolution -1
                                                children [
                                                  Shape {
                                                    appearance PBRAppearance {
                                                        baseColor 0 0 0
                                                        roughness 1
                                                        metalness 0
                                                        }
                                                      geometry Capsule {
                                                          height 0.005
                                                          radius 0.005
                                                      }
                                                  }
                                                ]
                          			            boundingObject Transform {
                                              children [
                                                  Capsule {
                                                        height 0.005
                                                        radius 0.005
                                                    }
                                             ]
                                            }
                                            physics Physics {
                                              density -1
                                              mass 0.02
                                              centerOfMass [0 0 0]
                                              inertiaMatrix []
                                            }

					                                }
                                          TouchSensor {
                                            translation 0.12 -0.022 -0.03
                                            rotation 1.000000 0.000000 0.000000 1.57
                                            name "left_leg_foot_sensor_2"
                                            type "bumper"
                                            resolution -1
                                            children [
                                                Shape {
                                                  appearance PBRAppearance {
                                                      baseColor 0 0 0
                                                      roughness 1
                                                      metalness 0
                                                  }
                                                  geometry Capsule {
                                                      height 0.005
                                                      radius 0.005
                                                  }
                                                }
                                            ]
                                            boundingObject Transform {

                                              children [
                                                 Capsule {
                                                   height 0.005
                                                  radius 0.005
                                                }
                                             ]
                                            }
                                            physics Physics {
                                              density -1
                                              mass 0.02
                                              centerOfMass [0 0 0]
                                              inertiaMatrix []
                                            }
                                          }
                                          TouchSensor {
                                            translation -0.025 -0.022 -0.03
                                            rotation 1.000000 0.000000 0.000000 1.57
                                            name "left_leg_foot_sensor_3"
                                            type "bumper"
                                            resolution -1
                                            children [
                                              Shape {
                                                appearance PBRAppearance {
                                                    baseColor 0 0 0
                                                    roughness 1
                                                    metalness 0
                                                }
                                                geometry Capsule {
                                                    height 0.005
                                                    radius 0.005
                                                }
                                              }
                                            ]
                                            boundingObject Transform {

                                              children [
                                                 Capsule {
                                                   height 0.005
                                                  radius 0.005
                                                }
                                             ]
                                            }
                                            physics Physics {
                                              density -1
                                              mass 0.02
                                              centerOfMass [0 0 0]
                                              inertiaMatrix []
                                            }
                                          }
                                          TouchSensor {
                                            translation -0.025 0.052 -0.03
                                            rotation 1.000000 0.000000 0.000000 1.57
                                            name "left_leg_foot_sensor_4"
                                            type "bumper"
                                            resolution -1
                                            children [
                                              Shape {
                                                appearance PBRAppearance {
                                                  baseColor 0 0 0
                                                    roughness 1
                                                    metalness 0
                                                    }
                                                geometry Capsule {
                                                  height 0.005
                                                    radius 0.005
                                                }
                                              }
                                            ]
                                            boundingObject Transform {

                                              children [
                                                 Capsule {
                                                   height 0.005
                                                  radius 0.005
                                                }
                                             ]
                                            }
                                            physics Physics {
                                              density -1
                                              mass 0.02
                                              centerOfMass [0 0 0]
                                              inertiaMatrix []
                                            }
                                          }
                                        ]"""
    text = text.replace(target_text, replace_text)

    target_text = """children [
                                          Transform {
                                            translation 0.068180 0.035100 0.376050
                                            children [
                                              Shape {
                                                appearance USE abs_white
                                                geometry DEF New_left_foot_upperhalf_v5_1 Mesh {
                                                  url "New_left_foot_upperhalf_v5_1.stl"
                                                }
                                              }
                                            ]
                                          }
                                        ]"""

    replace_text = """children [
                                          Transform {
                                            translation 0.068180 0.035100 0.376050
                                            children [
                                              Shape {
                                                appearance USE abs_white
                                                geometry DEF New_left_foot_upperhalf_v5_1 Mesh {
                                                  url "New_left_foot_upperhalf_v5_1.stl"
                                                }
                                              }
                                            ]
                                          }
                                          TouchSensor {
                                            translation 0.12 0.023 -0.03
                                            rotation 1.000000 0.000000 0.000000 1.57
                                            name "right_leg_foot_sensor_1"
                                            type "bumper"
                                            resolution -1
                                                children [
                                                  Shape {
                                                    appearance PBRAppearance {
                                                        baseColor 0 0 0
                                                        roughness 1
                                                        metalness 0
                                                    }
                                                    geometry Capsule {
                                                        height 0.005
                                                        radius 0.005
                                                    }
                                                  }
                                                ]
                          			            boundingObject Transform {

                                              children [
                                                 Capsule {
                                                   height 0.005
                                                  radius 0.005
                                                }
                                             ]
                                            }
                                            physics Physics {
                                              density -1
                                              mass 0.02
                                              centerOfMass [0 0 0]
                                              inertiaMatrix []
                                            }
					                                }
                                          TouchSensor {
                                            translation 0.12 -0.051 -0.03
                                            rotation 1.000000 0.000000 0.000000 1.57
                                            name "right_leg_foot_sensor_2"
                                            type "bumper"
                                            resolution -1
                                            children [
                                              Shape {
                                                  appearance PBRAppearance {
                                                      baseColor 0 0 0
                                                      roughness 1
                                                      metalness 0
                                                  }
                                                  geometry Capsule {
                                                      height 0.005
                                                      radius 0.005
                                                  }
                                              }
                                            ]
                                            boundingObject Transform {

                                              children [
                                                 Capsule {
                                                   height 0.005
                                                  radius 0.005
                                                }
                                             ]
                                            }
                                            physics Physics {
                                              density -1
                                              mass 0.02
                                              centerOfMass [0 0 0]
                                              inertiaMatrix []
                                            }
                                          }
                                          TouchSensor {
                                            translation -0.025 -0.051 -0.03
                                            rotation 1.000000 0.000000 0.000000 1.57
                                            name "right_leg_foot_sensor_3"
                                            type "bumper"
                                            resolution -1
                                            children [
                                                Shape {
                                                  appearance PBRAppearance {
                                                      baseColor 0 0 0
                                                      roughness 1
                                                      metalness 0
                                                  }
                                                  geometry Capsule {
                                                      height 0.005
                                                      radius 0.005
                                                  }
                                                }
                                            ]
                                            boundingObject Transform {

                                              children [
                                                 Capsule {
                                                   height 0.005
                                                  radius 0.005
                                                }
                                             ]
                                            }
                                            physics Physics {
                                              density -1
                                              mass 0.02
                                              centerOfMass [0 0 0]
                                              inertiaMatrix []
                                            }
                                          }
                                          TouchSensor {
                                            translation -0.025 0.022 -0.03
                                            rotation 1.000000 0.000000 0.000000 1.57
                                            name "right_leg_foot_sensor_4"
                                            type "bumper"
                                            resolution -1
                                            children [
                                              Shape {
                                                appearance PBRAppearance {
                                                    baseColor 0 0 0
                                                    roughness 1
                                                    metalness 0
                                                }
                                                geometry Capsule {
                                                    height 0.005
                                                    radius 0.005
                                                }
                                              }
                                            ]
                                            boundingObject Transform {

                                              children [
                                                 Capsule {
                                                   height 0.005
                                                  radius 0.005
                                                }
                                             ]
                                            }
                                            physics Physics {
                                              density -1
                                              mass 0.02
                                              centerOfMass [0 0 0]
                                              inertiaMatrix []
                                            }
                                          }
                                        ]"""
    text = text.replace(target_text, replace_text)

    # set controller
    text = text.replace('"left_arm_motor_0"', '"left_arm_motor_0 [shoulder]"')
    text = text.replace('"right_arm_motor_0"', '"right_arm_motor_0 [shoulder]"')
    text = text.replace('"left_leg_motor_1"', '"left_leg_motor_1 [hip]"')
    text = text.replace('"right_leg_motor_1"', '"right_leg_motor_1 [hip]"')

    # set arms
    text = text.replace('name "biceps_v3_1"', 'name "biceps_v3_1 [arm]"')
    text = text.replace('name "Arm_Assembly-bez3_v12_2"', 'name "Arm_Assembly-bez3_v12_2 [arm]"')
    text = text.replace('name "biceps_v3_2"', 'name "biceps_v3_2 [arm]"')
    text = text.replace('name "Arm_Assembly-bez3_v12_1"', 'name "Arm_Assembly-bez3_v12_1 [arm]"')

    # set foot
    text = text.replace('name "New_right_foot_upperhalf_v3_1"', 'name "New_right_foot_upperhalf_v3_1 [foot]"')
    text = text.replace('name "New_left_foot_upperhalf_v5_1"', 'name "New_left_foot_upperhalf_v5_1 [foot]"')
    text = text.replace("baseColor 0.500000 0.500000 0.500000", "baseColor 0 0 0")
    # add tags to the motors
    text = text.replace(
        'field  SFString    controller      "void"  # Is `Robot.controller`.', 'field  SFString    controller      "player"  # Is `Robot.controller`.'
    )

with open(target_file, "w") as proto_file:
    proto_file.write(text)
