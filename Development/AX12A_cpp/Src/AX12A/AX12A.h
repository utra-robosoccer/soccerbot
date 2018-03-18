/*
 * AX12A.h
 *
 *  Created on: Mar 16, 2018
 *      Author: Admin
 */

#ifndef AX12A_H_
#define AX12A_H_

#include "Dynamixel.h"



/* Value limit definitions. */
const double MAX_VELOCITY = 114;	// Maximum angular velocity (RPM)
const double MIN_VELOCITY = 1;		// Minimum angular velocity (RPM)
const uint16_t MAX_ANGLE = 300;		// Maximum angular position (joint mode)
const uint8_t MIN_ANGLE = 0;		// Minimum angular position (joint mode)
const uint8_t MAX_TORQUE = 100;		// Maximum torque (percent of maximum)
const uint8_t MIN_TORQUE = 0;		// Minimum torque (percent of maximum)
const uint8_t MAX_VOLTAGE = 14;		// Maximum operating voltage
const uint8_t MIN_VOLTAGE = 6;		// Minimum operating voltage
const uint16_t MAX_PUNCH = 1023;	// Maximum punch (proportional to minimum current)
const uint8_t MIN_PUNCH = 0;		// Minimum punch (proportional to minimum current)

/* Instruction set definitions. */
#define INST_PING				0x01	// Gets a status packet
#define INST_READ_DATA			0x02	// Reads data from a motor register
#define INST_WRITE_DATA			0x03	// Writes data for immediate execution
#define INST_REG_WRITE			0x04	// Registers an instruction to be executed at a later time
#define INST_ACTION				0x05	// Triggers instructions registered by INST_REG_WRITE
#define INST_RESET				0x06	// Resets the Dynamixel actuator(s) specified
#define INST_SYNC_WRITE			0x83	// Used to send commands concurrently to a set of specified motors

/* Register definitions. */
#define REG_ID 						0x03		// Motor ID register
#define REG_BAUD_RATE				0x04		// Baud rate register
#define REG_RETURN_DELAY_TIME		0x05		// Status packet return delay time register
#define REG_CW_ANGLE_LIMIT			0x06		// Clockwise angle limit register (0x06 = low byte, 0x07 = high byte)
#define REG_CCW_ANGLE_LIMIT			0x08		// Counter-clockwise angle limit register (0x08 = low byte, 0x09 = high byte)
#define REG_HIGH_VOLTAGE_LIMIT		0x0C		// Maximum voltage limit register
#define REG_LOW_VOLTAGE_LIMIT		0x0D		// Minimum voltage limit register
#define REG_MAX_TORQUE				0x0E		// Maximum torque limit register (0x0E = low byte, 0x0F = high byte)
#define REG_STATUS_RETURN_LEVEL		0x10		// Status packet return condition(s) register
#define REG_ALARM_LED				0x11		// Alarm LED condition(s) register
#define REG_ALARM_SHUTDOWN			0x12		// Alarm shutdown condition(s) register
#define REG_TORQUE_ENABLE 			0x18		// Motor power control register
#define REG_LED_ENABLE				0x19		// LED control register
#define REG_CW_COMPLIANCE_MARGIN	0x1A		// Clockwise compliance margin register
#define REG_CCW_COMPLIANCE_MARGIN	0x1B		// Counter-clockwise compliance margin register
#define REG_CW_COMPLIANCE_SLOPE		0x1C		// Clockwise compliance slope register
#define REG_CCW_COMPLIANCE_SLOPE	0x1D		// Counter-clockwise compliance slope register
#define REG_GOAL_POSITION			0x1E		// Goal position register (0x1E = low byte, 0x1F = high byte)
#define REG_GOAL_VELOCITY			0x20		// Goal velocity register (0x20 = low byte, 0x21 = high byte)
#define REG_GOAL_TORQUE				0x22		// Goal torque register (0x22 = low byte, 0x23 = high byte)
#define REG_LOCK_EEPROM 	 		0x2F		// EEPROM lock register
#define REG_PUNCH 	 				0x30		// Punch (0x30 = low register, 0x31 = high register)
#define REG_CURRENT_POSITION 		0x24		// Current position register (0x24 = low byte, 0x25 = high byte)
#define REG_CURRENT_VELOCITY 	 	0x26		// Current velocity register (0x26 = low byte, 0x27 = high byte)
#define REG_CURRENT_LOAD 			0x28		// Current load register (0x28 = low byte, 0x29 = high byte)
#define REG_CURRENT_VOLTAGE 		0x2A		// Current voltage register
#define REG_CURRENT_TEMPERATURE 	0x2B		// Current temperature register
#define REG_REGISTERED 				0x2C		// Command execution status register
#define REG_MOVING 					0x2E		// Motor motion register

/* Default register value definitions. */
#define BROADCAST_ID					0xFE	// Motor broadcast ID (i.e. messages sent to this ID will be sent to all motors on the bus)
#define DEFAULT_ID						0x01	// Default motor ID
#define DEFAULT_BAUD_RATE				0x01	// Default baud rate
#define DEFAULT_RETURN_DELAY			0xFA	// Default time motor waits before returning status packet (microseconds)
#define DEFAULT_CW_ANGLE_LIMIT			0x0000	// Default clockwise angle limit
#define DEFAULT_CCW_ANGLE_LIMIT			0x03FF	// Default counter-clockwise angle limit
#define DEFAULT_HIGH_VOLTAGE_LIMIT		0xBE	// Default permitted maximum voltage (0xBE = 140 -> 14.0 V)
#define DEFAULT_LOW_VOLTAGE_LIMIT		0x3C	// Default permitted minimum voltage (0x3C = 60 -> 6.0 V)
#define DEFAULT_MAXIMUM_TORQUE			0x03FF	// Default maximum torque limit (10-bit resolution percentage)
#define DEFAULT_STATUS_RETURN_LEVEL		0x02	// Default condition(s) under which a status packet will be returned (all)
#define DEFAULT_ALARM_LED				0x24	// Default condition(s) under which the alarm LED will be set
#define DEFAULT_ALARM_SHUTDOWN			0x24	// Default condition(s) under which the motor will shut down due to an alarm
#define DEFAULT_TORQUE_ENABLE			0x00	// Default motor power state
#define DEFAULT_LED_ENABLE				0x00	// Default LED state
#define DEFAULT_CW_COMPLIANCE_MARGIN	0x01	// Default clockwise compliance margin (position error)
#define DEFAULT_CCW_COMPLIANCE_MARGIN	0x01	// Default counter-clockwise compliance margin (position error)
#define DEFAULT_CW_COMPLIANCE_SLOPE		0x20	// Default clockwise compliance slope (torque near goal position)
#define DEFAULT_CCW_COMPLIANCE_SLOPE	0x20	// Default counter-clockwise compliance slope (torque near goal position)
#define DEFAULT_EEPROM_LOCK				0x00	// Default value for the EEPROM lock
#define DEFAULT_PUNCH					0x0020	// Default punch



class AX12A: public Dynamixel {
public:
	/********** Properties **********/
	uint8_t					_isJointMode;			/*!< 1 if motor is joint mode, 0 if wheel mode		*/


	/********** Methods **********/
	// Constructor and destructor
	AX12A();
	virtual ~AX12A();

	// Low-level transmission and reception
	void dataWriter(uint8_t arrSize, uint8_t writeAddr, uint8_t param1, uint8_t param2);
	uint16_t dataReader(uint8_t readAddr, uint8_t readLength);
	void computeChecksum(uint8_t *arr, int length);

	// Setters (use the WRITE DATA instruction)
	void setGoalTorque(double goalTorque); // (RAM)
	void setMaxTorque(double maxTorque); // (EEPROM)
	void setAlarmLED(uint8_t alarm_LED_data); // (EEPROM)
	void setAlarmShutdown(uint8_t alarm_shutdown_data); // (EEPROM)
	void setCWComplianceMargin(uint8_t CWcomplianceMargin); // (RAM)
	void setCCWComplianceMargin(uint8_t CCWcomplianceMargin); // (RAM)
	void setCWComplianceSlope(uint8_t CWcomplianceSlope); // (RAM)
	void setCCWComplianceSlope(uint8_t CCWcomplianceSlope); // (RAM)
	void setEEPROMLock(); // (RAM)
	void setPunch(double punch); // (RAM)
	void reset();

	// Getters (use the READ DATA instruction)
	uint8_t isJointMode();

	// Interfaces for previously-defined functions
	void revive(uint8_t ID); // TODO: Check this over for correctness in the context of several motors
	void broadcastRevive(uint8_t ID); // TODO: Check this over for correctness in the context of several motors
	void enterWheelMode(double goalVelocity);
	void enterJointMode();
	void setComplianceSlope(uint8_t complianceSlope);
	void setComplianceMargin(uint8_t complianceMargin);
};

#endif /* AX12A_H_ */
