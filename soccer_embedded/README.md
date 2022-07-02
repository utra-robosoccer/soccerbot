[![Build Status](https://travis-ci.org/utra-robosoccer/soccer-embedded.svg?branch=master)](https://travis-ci.org/utra-robosoccer/soccer-embedded)
[![Total alerts](https://img.shields.io/lgtm/alerts/g/utra-robosoccer/soccer-embedded.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/utra-robosoccer/soccer-embedded/alerts/)

# soccer-embedded
Collection of embedded programs for an autonomous humanoid soccer robot.

# Common
All the software components we've developed which go into the robot's embedded systems.

# CubeMX
Contains the files required to auto-generate code for each of our target MCUs.

# Development
Contains development folders for ongoing projects. Examples include developing specific features that will one day be integrated into the main robot code (such as Ethernet-based communication), and developing helper programs dedicated to expediting development and testing.

# Diagrams
UML diagrams for our software (not kept up-to-date).

# docs
Documentation for the code in the Common directory. Extracted using Doxygen.

# Robot
The program controlling the embedded systems of the robot. The target MCU is specified by setting TARGET=F4 (default) or TARGET=F7.

# RobotTest
Unit test runner (GTest/GMock).

# Templates
Contains C and C++ source file and header file templates and examples illustrating their usage.

# Testing
Contains various test programs serving as "checkpoints" for integrating features as well as other projects for scratchwork.

# Tutorials
Contains tutorials for setting up your development environment and learning the basics of embedded development.
