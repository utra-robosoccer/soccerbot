/*
 * MX28.cpp
 *
 *  Created on: Mar 17, 2018
 *      Author: Admin
 */




/********************************** Includes **********************************/
#include "MX28.h"




/********************************* Functions *********************************/
MX28::MX28(MotorInitData* motorInitData) :
	Dynamixel(motorInitData)
{
	// TODO Auto-generated constructor stub
	lastTick = 0;
	lastPWM = 0;
	lastVelocityTrajectory = 0;
	lastPositionTrajectory = 0;
}

MX28::~MX28() {
	// TODO Auto-generated destructor stub
}

