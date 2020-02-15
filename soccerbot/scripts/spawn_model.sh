#!/usr/bin/env bash

MODEL_NAME=$1
XPOS=$2
YPOS=$3
ROBOT_NAMESPACE=$MODEL_NAME
REFERENCE_FRAME="world"

rosservice call --wait /gazebo/spawn_urdf_model "{
  model_name: '$MODEL_NAME',
  model_xml: $(rosparam get robot_description),
  robot_namespace: '/$ROBOT_NAMESPACE',
  initial_pose: {
    position: {
      x: $XPOS,
      y: $YPOS,
      z: $(rosparam get initial_state/z)
    },
    orientation: {
      x: $(rosparam get initial_state/orientation/x),
      y: $(rosparam get initial_state/orientation/y),
      z: $(rosparam get initial_state/orientation/z),
      w: $(rosparam get initial_state/orientation/w)
    }
  },
  reference_frame: '$REFERENCE_FRAME'
}"

rosservice call --wait /gazebo/set_model_configuration "{
  model_name: '$MODEL_NAME',
  urdf_param_name: $(rosparam get robot_description),
  joint_names: [
    'right_leg_motor_0',
    'right_leg_motor_1',
    'right_leg_motor_2',
    'right_leg_motor_3',
    'right_leg_motor_4',
    'right_leg_motor_5',
    'left_leg_motor_0',
    'left_leg_motor_1',
    'left_leg_motor_2',
    'left_leg_motor_3',
    'left_leg_motor_4',
    'left_leg_motor_5',
    'head_motor_0',
    'head_motor_1',
    'left_arm_motor_0',
    'left_arm_motor_1',
    'right_arm_motor_0',
    'right_arm_motor_1'
  ],
  joint_positions: [
     $(rosparam get initial_state/right_leg_motor_0),
     $(rosparam get initial_state/right_leg_motor_1),
     $(rosparam get initial_state/right_leg_motor_2),
     $(rosparam get initial_state/right_leg_motor_3),
     $(rosparam get initial_state/right_leg_motor_4),
     $(rosparam get initial_state/right_leg_motor_5),
     $(rosparam get initial_state/left_leg_motor_0),
     $(rosparam get initial_state/left_leg_motor_1),
     $(rosparam get initial_state/left_leg_motor_2),
     $(rosparam get initial_state/left_leg_motor_3),
     $(rosparam get initial_state/left_leg_motor_4),
     $(rosparam get initial_state/left_leg_motor_5),
     $(rosparam get initial_state/head_motor_0),
     $(rosparam get initial_state/head_motor_1),
     $(rosparam get initial_state/left_arm_motor_0),
     $(rosparam get initial_state/left_arm_motor_1),
     $(rosparam get initial_state/right_arm_motor_0),
     $(rosparam get initial_state/right_arm_motor_1)
  ]
}"