#!/bin/bash

cd Robot || exit
openocd -f interface/stlink-v2-1.cfg -f target/stm32f4x.cfg -c "program ./Build/F4/Robot_F4.elf verify reset exit"
