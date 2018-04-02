/*
 * MX28.cpp
 *
 *  Created on: Mar 17, 2018
 *      Author: Admin
 */




/********************************** Includes **********************************/
#include "MX28.h"




/********************************* Constants *********************************/
/* Register definitions. */
// TODO
const uint8_t MX28_REG_ID 					    	= 7;		// Motor ID register
const uint8_t MX28_REG_BAUD_RATE					= 8;		// Baud rate register
const uint8_t MX28_REG_RETURN_DELAY_TIME			= 9;		// Status packet return delay time register
//const uint8_t MX28_REG_CW_ANGLE_LIMIT		    	= 0x06;		// Clockwise angle limit register (0x06 = low byte, 0x07 = high byte)
//const uint8_t MX28_REG_CCW_ANGLE_LIMIT		    = 0x08;		// Counter-clockwise angle limit register (0x08 = low byte, 0x09 = high byte)
//const uint8_t MX28_REG_HIGH_VOLTAGE_LIMIT	    	= 0x0C;		// Maximum voltage limit register
//const uint8_t MX28_REG_LOW_VOLTAGE_LIMIT			= 0x0D;		// Minimum voltage limit register
//const uint8_t MX28_REG_MAX_TORQUE			    	= 0x0E;		// Maximum torque limit register (0x0E = low byte, 0x0F = high byte)
//const uint8_t MX28_REG_STATUS_RETURN_LEVEL	    = 0x10;		// Status packet return condition(s) register
//const uint8_t MX28_REG_ALARM_LED					= 0xxx;		// Alarm LED condition(s) register
//const uint8_t MX28_REG_ALARM_SHUTDOWN		    	= 0x12;		// Alarm shutdown condition(s) register
const uint8_t MX28_REG_TORQUE_ENABLE 		    	= 64;		// Motor power control register
const uint8_t MX28_REG_LED_ENABLE			    	= 65;		// LED control register
//const uint8_t MX28_REG_CW_COMPLIANCE_MARGIN		= 0x1A;		// Clockwise compliance margin register
//const uint8_t MX28_REG_CCW_COMPLIANCE_MARGIN		= 0x1B;		// Counter-clockwise compliance margin register
//const uint8_t MX28_REG_CW_COMPLIANCE_SLOPE	    = 0x1C;		// Clockwise compliance slope register
//const uint8_t MX28_REG_CCW_COMPLIANCE_SLOPE    	= 0x1D;		// Counter-clockwise compliance slope register
const uint8_t MX28_REG_GOAL_POSITION		    	= 116;		// Goal position register (0x1E = low byte, 0x1F = high byte)
const uint8_t MX28_REG_GOAL_VELOCITY		    	= 104;		// Goal velocity register (0x20 = low byte, 0x21 = high byte)
//const uint8_t MX28_REG_GOAL_TORQUE			    = 0x22;		// Goal torque register (0x22 = low byte, 0x23 = high byte)
//const uint8_t MX28_REG_LOCK_EEPROM 	 	    	= 0x2F;		// EEPROM lock register
//const uint8_t MX28_REG_PUNCH 	 			    	= 0x30;		// Punch (0x30 = low register, 0x31 = high register)
//const uint8_t MX28_REG_CURRENT_POSITION 	    	= 0x24;		// Current position register (0x24 = low byte, 0x25 = high byte)
//const uint8_t MX28_REG_CURRENT_VELOCITY 	    	= 0x26;		// Current velocity register (0x26 = low byte, 0x27 = high byte)
//const uint8_t MX28_REG_CURRENT_LOAD 		    	= 0x28;		// Current load register (0x28 = low byte, 0x29 = high byte)
//const uint8_t MX28_REG_CURRENT_VOLTAGE 	    	= 0x2A;		// Current voltage register
//const uint8_t MX28_REG_CURRENT_TEMPERATURE     	= 0x2B;		// Current temperature register
//const uint8_t MX28_REG_REGISTERED 			    = 0x2C;		// Command execution status register
//const uint8_t MX28_REG_MOVING 				    = 0x2E;		// Motor motion register

/* Default register value definitions. */
//TODO




/********************************* Functions *********************************/
MX28::MX28(MotorInitData* motorInitData) :
	Dynamixel(motorInitData)
{
	// TODO Auto-generated constructor stub
	lastTick = 0;
	lastPWM = 0;
	lastVelocityTrajectory = 0;
	lastPositionTrajectory = 0;


	this -> angleResolution = 4095; // 2^12

	this -> REG_ID = MX28_REG_ID;
	this -> REG_BAUD_RATE = MX28_REG_BAUD_RATE;
	this -> REG_GOAL_POSITION = MX28_REG_GOAL_POSITION;
	this -> REG_GOAL_VELOCITY = MX28_REG_GOAL_VELOCITY;
	this -> REG_TORQUE_ENABLE = MX28_REG_TORQUE_ENABLE;
	this -> REG_RETURN_DELAY_TIME = MX28_REG_RETURN_DELAY_TIME;
	this -> REG_LED_ENABLE = MX28_REG_LED_ENABLE;
}

MX28::~MX28() {
	// TODO Auto-generated destructor stub
}

