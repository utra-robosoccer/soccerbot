/* This file holds common header code for the AX12A library and MX28 library. It is
 * somewhat generic in that any Dynamixel actuator using protocol version 1.0 should
 * be able to be integrated with little effort, as the instructions and register
 * addresses are very similar for all actuators using this protocol version.
 *
 * Author: Tyler
 */

/******************** Define to prevent recursive inclusion *******************/
#ifndef __DYNAMIXEL_PROTOCOL_V1_H__
#define __DYNAMIXEL_PROTOCOL_V1_H__

/********************************** Includes **********************************/
#ifdef stm32f4xx_hal
	#include "stm32f4xx_hal.h"
	#include "stm32f4xx_hal_conf.h"
#endif

#ifdef stm32h7xx_hal
	#include "stm32h7xx_hal.h"
	#include "stm32h7xx_hal_conf.h"
#endif

/* I/O */
#include "gpio.h"
#include "usart.h"

/* Types */
#include <stdint.h>

/* Dynamixel actuators */
#include "Dynamixel_HandleTypeDef.h"
#include "MX28/MX28.h"
#include "AX12A/AX12A.h"

/*********************************** Macros ***********************************/
#define __DYNAMIXEL_TRANSMIT(port, pinNum) HAL_GPIO_WritePin(port, pinNum, 1) // Set data direction pin high (TX)
#define __DYNAMIXEL_RECEIVE(port, pinNum) HAL_GPIO_WritePin(port, pinNum, 0) // Set data direction pin low (RX)

/* Communications */
#define TRANSMIT_TIMEOUT 		10 		// Timeout for blocking UART transmissions, in milliseconds
#define RECEIVE_TIMEOUT			10 		// Timeout for blocking UART receptions, in milliseconds
#define NUM_MOTORS				18		// Used to determine buffer sizes
#define BUFF_SIZE_RX			8		// Receive buffer size for UART receptions (number of bytes)
#define TX_PACKET_SIZE			9		// Maximum packet size for regular motor commands (exclusion: sync write)
#define NUM_UARTS				6		// Number of UARTs available for motor communication

/* Instruction set definitions */
#define INST_PING				0x01	    // Gets a status packet
#define INST_READ_DATA			0x02	    // Reads data from a motor register
#define INST_WRITE_DATA			0x03	    // Writes data for immediate execution
#define INST_REG_WRITE			0x04	    // Registers an instruction to be executed at a later time
#define INST_ACTION				0x05	    // Triggers instructions registered by INST_REG_WRITE
#define INST_RESET				0x06	    // Resets the control tables of the Dynamixel actuator(s) specified
#define INST_SYNC_WRITE			0x83	    // Writes on a specified address with a specified data length on multiple devices

/* Register addresses */
#define REG_ID 					    		0x03	// Motor ID register
#define REG_BAUD_RATE						0x04	// Baud rate register
#define REG_RETURN_DELAY_TIME				0x05	// Status packet return delay time register
#define REG_CW_ANGLE_LIMIT		    		0x06	// Clockwise angle limit register (0x06 = low byte, 0x07 = high byte)
#define REG_CCW_ANGLE_LIMIT		    		0x08	// Counter-clockwise angle limit register (0x08 = low byte, 0x09 = high byte)
#define REG_HIGH_VOLTAGE_LIMIT	    		0x0C	// Maximum voltage limit register
#define REG_LOW_VOLTAGE_LIMIT				0x0D	// Minimum voltage limit register
#define REG_MAX_TORQUE			    		0x0E	// Maximum torque limit register (0x0E = low byte, 0x0F = high byte)
#define REG_STATUS_RETURN_LEVEL	    		0x10	// Status packet return condition(s) register
#define REG_ALARM_LED						0x11	// Alarm LED condition(s) register
#define REG_ALARM_SHUTDOWN		    		0x12	// Alarm shutdown condition(s) register
#define REG_TORQUE_ENABLE 		    		0x18	// Motor power control register
#define REG_LED_ENABLE			    	    0x19    // LED control register
#define REG_GOAL_POSITION		    	    0x1E    // Goal position register (0x1E = low byte, 0x1F = high byte)
#define REG_GOAL_VELOCITY		    	    0x20    // Goal velocity register (0x20 = low byte, 0x21 = high byte)
#define REG_GOAL_TORQUE			    	    0x22	// Goal torque register (0x22 = low byte, 0x23 = high byte)
#define REG_LOCK_EEPROM 	 	    	    0x2F	// EEPROM lock register
#define REG_PUNCH 	 			    		0x30	// Punch (0x30 = low register, 0x31 = high register)
#define REG_CURRENT_POSITION 	    		0x24	// Current position register (0x24 = low byte, 0x25 = high byte)
#define REG_CURRENT_VELOCITY 	    		0x26	// Current velocity register (0x26 = low byte, 0x27 = high byte)
#define REG_CURRENT_LOAD 		    		0x28	// Current load register (0x28 = low byte, 0x29 = high byte)
#define REG_CURRENT_VOLTAGE 	    	    0x2A	// Current voltage register
#define REG_CURRENT_TEMPERATURE     	    0x2B	// Current temperature register
#define REG_REGISTERED 			    		0x2C	// Command execution status register
#define REG_MOVING 				    		0x2E	// Motor motion register

/* Default register values */
#define BROADCAST_ID				 0xFE	    // Motor broadcast ID (i.e. messages sent to this ID will be sent to all motors on the bus)
#define DEFAULT_ID				0x01	    // Default motor ID
#define DEFAULT_RETURN_DELAY		 0xFA	    // Default time motor waits before returning status packet (microseconds)
#define DEFAULT_CW_ANGLE_LIMIT		 0x0000		// Default clockwise angle limit
#define DEFAULT_LOW_VOLTAGE_LIMIT	 0x3C		// Default permitted minimum voltage (0x3C = 60 -> 6.0 V)
#define DEFAULT_MAXIMUM_TORQUE		 0x03FF	    // Default maximum torque limit (10-bit resolution percentage)
#define DEFAULT_STATUS_RETURN_LEVEL	 0x02	    // Default condition(s) under which a status packet will be returned (all)
#define DEFAULT_ALARM_LED			 0x24	    // Default condition(s) under which the alarm LED will be set
#define DEFAULT_ALARM_SHUTDOWN		 0x24	    // Default condition(s) under which the motor will shut down due to an alarm
#define DEFAULT_TORQUE_ENABLE		 0x00	    // Default motor power state
#define DEFAULT_LED_ENABLE			 0x00	    // Default LED state
#define DEFAULT_EEPROM_LOCK			 0x00	// Default value for the EEPROM lock

/* Value limit definitions */
#define MIN_VELOCITY          1		// Minimum angular velocity (RPM)
#define MAX_ANGLE             300	// Maximum angular position (joint mode)
#define MIN_ANGLE             0		// Minimum angular position (joint mode)
#define MAX_TORQUE            100	// Maximum torque (percent of maximum)
#define MIN_TORQUE            0		// Minimum torque (percent of maximum)
#define MAX_VOLTAGE           14	// Maximum operating voltage
#define MIN_VOLTAGE           6		// Minimum operating voltage
#define MAX_PUNCH             1023	// Maximum punch (proportional to minimum current)
#define MIN_PUNCH             0		// Minimum punch (proportional to minimum current)

/******************************* Public Variables *******************************/
/* Buffer for data received from motors. */
extern uint8_t arrReceive[NUM_MOTORS][BUFF_SIZE_RX];

/* Bytes to be transmitted to motors are written to this array. */
extern uint8_t arrTransmit[NUM_MOTORS + 1][TX_PACKET_SIZE];

/***************************** Function prototypes ****************************/
// Setters (use the WRITE DATA instruction)
void Dynamixel_SetID(Dynamixel_HandleTypeDef* hdynamixel, uint8_t ID); // (EEPROM)
void Dynamixel_SetReturnDelayTime(Dynamixel_HandleTypeDef* hdynamixel, uint16_t microSec); // (EEPROM)
void Dynamixel_SetCWAngleLimit(Dynamixel_HandleTypeDef* hdynamixel, double minAngle); // (EEPROM)
void Dynamixel_SetCCWAngleLimit(Dynamixel_HandleTypeDef* hdynamixel, double maxAngle); // (EEPROM)
void Dynamixel_SetHighestVoltageLimit(Dynamixel_HandleTypeDef* hdynamixel, double highestVoltage); // (EEPROM)
void Dynamixel_SetLowestVoltageLimit(Dynamixel_HandleTypeDef* hdynamixel, double lowestVoltage); // (EEPROM)
void Dynamixel_SetMaxTorque(Dynamixel_HandleTypeDef* hdynamixel, double maxTorque); // (EEPROM)
void Dynamixel_SetStatusReturnLevel(Dynamixel_HandleTypeDef* hdynamixel, uint8_t status_data); // (EEPROM)
void Dynamixel_SetAlarmLED(Dynamixel_HandleTypeDef* hdynamixel, uint8_t alarm_LED_data); // (EEPROM)
void Dynamixel_SetAlarmShutdown(Dynamixel_HandleTypeDef* hdynamixel, uint8_t alarm_shutdown_data); // (EEPROM)

void Dynamixel_TorqueEnable(Dynamixel_HandleTypeDef* hdynamixel, uint8_t isEnabled); // (RAM)
void Dynamixel_LEDEnable(Dynamixel_HandleTypeDef* hdynamixel, uint8_t isEnabled); // (RAM)
void Dynamixel_SetGoalPosition(Dynamixel_HandleTypeDef* hdynamixel, double goalAngle); // (RAM)
void Dynamixel_SetGoalVelocity(Dynamixel_HandleTypeDef* hdynamixel, double goalVelocity); // (RAM)
void Dynamixel_SetGoalTorque(Dynamixel_HandleTypeDef* hdynamixel, double goalTorque); // (RAM)
void Dynamixel_LockEEPROM(Dynamixel_HandleTypeDef* hdynamixel); // (RAM)
void Dynamixel_SetPunch(Dynamixel_HandleTypeDef* hdynamixel, double punch); // (RAM)

// Getters (use READ DATA instruction)
void Dynamixel_GetPosition(Dynamixel_HandleTypeDef* hdynamixel);
void Dynamixel_GetVelocity(Dynamixel_HandleTypeDef* hdynamixel);
void Dynamixel_GetLoad(Dynamixel_HandleTypeDef* hdynamixel);
float Dynamixel_GetVoltage(Dynamixel_HandleTypeDef* hdynamixel);
uint8_t Dynamixel_GetTemperature(Dynamixel_HandleTypeDef* hdynamixel);
uint8_t Dynamixel_IsRegistered(Dynamixel_HandleTypeDef* hdynamixel);
uint8_t Dynamixel_IsMoving(Dynamixel_HandleTypeDef* hdynamixel);
uint8_t Dynamixel_IsJointMode(Dynamixel_HandleTypeDef* hdynamixel);

// Transmission & Reception
void Dynamixel_DataWriter(Dynamixel_HandleTypeDef* hdynamixel, uint8_t* args, uint8_t numArgs);
uint16_t Dynamixel_DataReader(Dynamixel_HandleTypeDef* hdynamixel, uint8_t readAddr, uint8_t readLength);

// Other motor instructions (low level control with timing from different WRITE DATA instruction)
void Dynamixel_RegWrite(Dynamixel_HandleTypeDef* hdynamixel, uint8_t arrSize, uint8_t writeAddr, uint8_t param1, uint8_t param2);
void Dynamixel_Action(Dynamixel_HandleTypeDef* hdynamixel);
int8_t Dynamixel_Ping(Dynamixel_HandleTypeDef* hdynamixel);

// Setup
void Dynamixel_Init(Dynamixel_HandleTypeDef* hdynamixel, uint8_t ID, UART_HandleTypeDef* UART_Handle,\
		GPIO_TypeDef* DataDirPort, uint16_t DataDirPinNum, enum motorTypes_e motorType);
void Dynamixel_Reset(Dynamixel_HandleTypeDef* hdynamixel);

// Interfaces for previously-defined functions
void Dynamixel_EnterWheelMode(Dynamixel_HandleTypeDef* hdynamixel, double goalVelocity);
void Dynamixel_EnterJointMode(Dynamixel_HandleTypeDef* hdynamixel);

#endif /* __DYNAMIXEL_PROTOCOL_V1_H__ */
