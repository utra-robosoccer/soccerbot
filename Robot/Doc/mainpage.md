@mainpage Welcome


# Introduction
This is the documentation for the embedded software for UTRA's soccerbot.
This page contains some high-level descriptions of
how this software works. More details for each module can be found by browsing the Modules and Data Structures sections.
To return to UTRA's website, <a href="http://utrahumanoid.ca/">click here</a>.

Our robot consists of 2 main computing solutions: a NVidia Jetson TX2 which runs the high-level software for controls and AI, and a STMicroelectronics 32-bit microcontroller. At a glance, the microcontroller software is responsible for:
-# Controlling the 18 actuators which form the robot's joints
-# Gathering sensor data
-# PC communication for receiving actuator commands and transmitting sensor data

It is important to note that all of these tasks must be accomplished with high efficiency and reliability.


# High-level Description
A STM32F446RE microcontroller on a custom PCB designed by the electrical team was chosen as the platform for the embedded systems. The software runs ontop of FreeRTOS, a real-time OS with a priority-based preemptive scheduler. All IO is DMA-based where possible, and interrupt-based otherwise. The peripherals involved are:
- MPU6050 inertial measurement unit
- Dynamixel MX-28 servo motor (x12)
- Dynamixel AX-12A servo motor (x6)

## Threads
The software consists of 6 threads (note: priority = 6 is the highest priority while priority = 0 is the lowest priority):
-# **PC RX, priority 6**: event-based, receives command packets from PC 
-# **PC TX, priority 5**: sends packets to the PC consisting of sensor data
-# **MPU6050, priority 3**: reads acceleration and angular velocity
-# **default, priority 0**: CubeMX-generated function that immediately sleeps

## Principle of Operation
The image below shows some of the possible interactions between the components of the system once the scheduler has started. Note how the program flow is largely sequential—not much happens until a command packet is received from the PC. This is an area we are working on improving in our next design iteration

<img src="Media\Design_RoboCup_2018\Design_RoboCup_2018.png" width="80%" alt="Steady-state flow">


# Plans for the Future
<!-- Describe current plans here -->