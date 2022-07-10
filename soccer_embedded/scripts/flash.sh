#!/bin/bash
rm ~/catkin_ws/src/soccerbot/soccer_embedded/Robot/Build/F4/Robot_F4.elf
rm ~/catkin_ws/src/soccerbot/soccer_embedded/Robot/Build/F4/Robot_F4.hex
make
openocd -f interface/stlink-v2-1.cfg -f target/stm32f4x.cfg -c "program ./Build/F4/Robot_F4.elf verify reset exit"
