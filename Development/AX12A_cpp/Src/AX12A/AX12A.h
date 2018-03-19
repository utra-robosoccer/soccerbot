/*
 * AX12A.h
 *
 *  Created on: Mar 16, 2018
 *      Author: Admin
 */

#ifndef AX12A_H_
#define AX12A_H_

#include "../Dynamixel/Dynamixel.h"

class AX12A: public Dynamixel {
public:
	/********** Register Addresses **********/
	/* Value limit definitions. */
	static constexpr double MAX_VELOCITY            = 114;		// Maximum angular velocity (RPM)
	static constexpr double MIN_VELOCITY            = 1;		// Minimum angular velocity (RPM)
	static constexpr uint16_t MAX_ANGLE             = 300;		// Maximum angular position (joint mode)
	static constexpr uint8_t MIN_ANGLE              = 0;		// Minimum angular position (joint mode)
	static constexpr uint8_t MAX_TORQUE             = 100;		// Maximum torque (percent of maximum)
	static constexpr uint8_t MIN_TORQUE             = 0;		// Minimum torque (percent of maximum)
	static constexpr uint8_t MAX_VOLTAGE            = 14;		// Maximum operating voltage
	static constexpr uint8_t MIN_VOLTAGE            = 6;		// Minimum operating voltage
	static constexpr uint16_t MAX_PUNCH             = 1023;		// Maximum punch (proportional to minimum current)
	static constexpr uint8_t MIN_PUNCH              = 0;		// Minimum punch (proportional to minimum current)

	/* Instruction set definitions. */
	static constexpr uint8_t INST_PING				= 0x01;	    // Gets a status packet
	static constexpr uint8_t INST_READ_DATA			= 0x02;	    // Reads data from a motor register
	static constexpr uint8_t INST_WRITE_DATA		= 0x03;	    // Writes data for immediate execution
	static constexpr uint8_t INST_REG_WRITE			= 0x04;	    // Registers an instruction to be executed at a later time
	static constexpr uint8_t INST_ACTION			= 0x05;	    // Triggers instructions registered by INST_REG_WRITE
	static constexpr uint8_t INST_RESET				= 0x06;	    // Resets the Dynamixel actuator(s) specified
	static constexpr uint8_t INST_SYNC_WRITE		= 0x83;	    // Used to send commands concurrently to a set of specified motors

	/* Register definitions. */
	static constexpr uint8_t REG_ID 					= 0x03;		// Motor ID register
	static constexpr uint8_t REG_BAUD_RATE				= 0x04;		// Baud rate register
	static constexpr uint8_t REG_RETURN_DELAY_TIME		= 0x05;		// Status packet return delay time register
	static constexpr uint8_t REG_CW_ANGLE_LIMIT			= 0x06;		// Clockwise angle limit register (0x06 = low byte, 0x07 = high byte)
	static constexpr uint8_t REG_CCW_ANGLE_LIMIT		= 0x08;		// Counter-clockwise angle limit register (0x08 = low byte, 0x09 = high byte)
	static constexpr uint8_t REG_HIGH_VOLTAGE_LIMIT		= 0x0C;		// Maximum voltage limit register
	static constexpr uint8_t REG_LOW_VOLTAGE_LIMIT		= 0x0D;		// Minimum voltage limit register
	static constexpr uint8_t REG_MAX_TORQUE				= 0x0E;		// Maximum torque limit register (0x0E = low byte, 0x0F = high byte)
	static constexpr uint8_t REG_STATUS_RETURN_LEVEL	= 0x10;		// Status packet return condition(s) register
	static constexpr uint8_t REG_ALARM_LED				= 0x11;		// Alarm LED condition(s) register
	static constexpr uint8_t REG_ALARM_SHUTDOWN			= 0x12;		// Alarm shutdown condition(s) register
	static constexpr uint8_t REG_TORQUE_ENABLE 			= 0x18;		// Motor power control register
	static constexpr uint8_t REG_LED_ENABLE				= 0x19;		// LED control register
	static constexpr uint8_t REG_CW_COMPLIANCE_MARGIN	= 0x1A;		// Clockwise compliance margin register
	static constexpr uint8_t REG_CCW_COMPLIANCE_MARGIN	= 0x1B;		// Counter-clockwise compliance margin register
	static constexpr uint8_t REG_CW_COMPLIANCE_SLOPE	= 0x1C;		// Clockwise compliance slope register
	static constexpr uint8_t REG_CCW_COMPLIANCE_SLOPE	= 0x1D;		// Counter-clockwise compliance slope register
	static constexpr uint8_t REG_GOAL_POSITION			= 0x1E;		// Goal position register (0x1E = low byte, 0x1F = high byte)
	static constexpr uint8_t REG_GOAL_VELOCITY			= 0x20;		// Goal velocity register (0x20 = low byte, 0x21 = high byte)
	static constexpr uint8_t REG_GOAL_TORQUE			= 0x22;		// Goal torque register (0x22 = low byte, 0x23 = high byte)
	static constexpr uint8_t REG_LOCK_EEPROM 	 		= 0x2F;		// EEPROM lock register
	static constexpr uint8_t REG_PUNCH 	 				= 0x30;		// Punch (0x30 = low register, 0x31 = high register)
	static constexpr uint8_t REG_CURRENT_POSITION 		= 0x24;		// Current position register (0x24 = low byte, 0x25 = high byte)
	static constexpr uint8_t REG_CURRENT_VELOCITY 	 	= 0x26;		// Current velocity register (0x26 = low byte, 0x27 = high byte)
	static constexpr uint8_t REG_CURRENT_LOAD 			= 0x28;		// Current load register (0x28 = low byte, 0x29 = high byte)
	static constexpr uint8_t REG_CURRENT_VOLTAGE 		= 0x2A;		// Current voltage register
	static constexpr uint8_t REG_CURRENT_TEMPERATURE 	= 0x2B;		// Current temperature register
	static constexpr uint8_t REG_REGISTERED 			= 0x2C;		// Command execution status register
	static constexpr uint8_t REG_MOVING 				= 0x2E;		// Motor motion register

	/* Default register value definitions. */
	static constexpr uint8_t BROADCAST_ID					= 0xFE;	    // Motor broadcast ID (i.e. messages sent to this ID will be sent to all motors on the bus)
	static constexpr uint8_t DEFAULT_ID						= 0x01;	    // Default motor ID
	static constexpr uint8_t DEFAULT_BAUD_RATE				= 0x01;	    // Default baud rate
	static constexpr uint8_t DEFAULT_RETURN_DELAY			= 0xFA;	    // Default time motor waits before returning status packet (microseconds)
	static constexpr uint16_t DEFAULT_CW_ANGLE_LIMIT		= 0x0000;	// Default clockwise angle limit
	static constexpr uint16_t DEFAULT_CCW_ANGLE_LIMIT		= 0x03FF;	// Default counter-clockwise angle limit
	static constexpr uint8_t DEFAULT_HIGH_VOLTAGE_LIMIT		= 0xBE;	    // Default permitted maximum voltage (0xBE = 140 -> 14.0 V)
	static constexpr uint8_t DEFAULT_LOW_VOLTAGE_LIMIT		= 0x3C;	    // Default permitted minimum voltage (0x3C = 60 -> 6.0 V)
	static constexpr uint16_t DEFAULT_MAXIMUM_TORQUE		= 0x03FF;	// Default maximum torque limit (10-bit resolution percentage)
	static constexpr uint8_t DEFAULT_STATUS_RETURN_LEVEL	= 0x02;	    // Default condition(s) under which a status packet will be returned (all)
	static constexpr uint8_t DEFAULT_ALARM_LED				= 0x24;	    // Default condition(s) under which the alarm LED will be set
	static constexpr uint8_t DEFAULT_ALARM_SHUTDOWN			= 0x24;	    // Default condition(s) under which the motor will shut down due to an alarm
	static constexpr uint8_t DEFAULT_TORQUE_ENABLE			= 0x00;	    // Default motor power state
	static constexpr uint8_t DEFAULT_LED_ENABLE				= 0x00;	    // Default LED state
	static constexpr uint8_t DEFAULT_CW_COMPLIANCE_MARGIN	= 0x01;	    // Default clockwise compliance margin (position error)
	static constexpr uint8_t DEFAULT_CCW_COMPLIANCE_MARGIN	= 0x01;	    // Default counter-clockwise compliance margin (position error)
	static constexpr uint8_t DEFAULT_CW_COMPLIANCE_SLOPE	= 0x20;	    // Default clockwise compliance slope (torque near goal position)
	static constexpr uint8_t DEFAULT_CCW_COMPLIANCE_SLOPE	= 0x20;	    // Default counter-clockwise compliance slope (torque near goal position)
	static constexpr uint8_t DEFAULT_EEPROM_LOCK			= 0x00;	    // Default value for the EEPROM lock
	static constexpr uint16_t DEFAULT_PUNCH					= 0x0020;	// Default punch


	/********** Properties **********/
	bool isJointMode;			/*!< 1 if motor is joint mode, 0 if wheel mode		*/


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
	bool isInJointMode();

	// Interfaces for previously-defined functions
	void revive(uint8_t ID); // TODO: Check this over for correctness in the context of several motors
	void broadcastRevive(uint8_t ID); // TODO: Check this over for correctness in the context of several motors
	void enterWheelMode(double goalVelocity);
	void enterJointMode();
	void setComplianceSlope(uint8_t complianceSlope);
	void setComplianceMargin(uint8_t complianceMargin);
};

#endif /* AX12A_H_ */
