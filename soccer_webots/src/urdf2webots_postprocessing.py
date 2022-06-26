#!/usr/bin/env python3
import os

print(os.getcwd())
os.system(
    "python3 -m urdf2webots.importer --robot-name=Bez3 --input=../../bez3_description/urdf/bez3.urdf --output=../robocup/protos/Bez3.proto --box-collision"
)

# rename  25t_servo_hub to t_servo_hub
# controller > "player"
# some rotational joints need [hip] [shoulder]
# add camera, imu, foot sensors

# selfcollision > TRUE
# define motor values: torque etc
