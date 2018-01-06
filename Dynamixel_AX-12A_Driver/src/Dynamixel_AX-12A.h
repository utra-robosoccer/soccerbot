/****************** Drivers for Dynamixel AX-12A Servo Motor ******************/

/******************** Define to prevent recursive inclusion *******************/
#ifndef __DYNAMIXEL_AX_12A_H__
#define __DYNAMIXEL_AX_12A_H__

/********************************** Includes **********************************/
#ifdef stm32f4xx_hal.h
	#include "stm32f4xx_hal.h"
	#include "stm32f4xx_hal_conf.h"
#endif
#ifdef stm32h7xx_hal.h
	#include "stm32h7xx_hal.h"
	#include "stm32h7xx_hal_conf.h"
#endif
/******************************(**** Macros ******************************(****/
#define __DYNAMIXEL_TRANSMIT(port, pinNum) HAL_GPIO_WritePin(port, pinNum, 1) // Set data direction pin high (TX)
#define __DYNAMIXEL_RECEIVE(port, pinNum) HAL_GPIO_WritePin(port, pinNum, 0) // Set data direction pin low (RX)

/*********************************** Defines **********************************/
/* Communications. */
#define TRANSMIT_IT				0		// 1 if using interrupts for transmit, otherwise 0 (polling)
#define NUM_MOTORS				18		// Used to determine buffer sizes
#define BUFF_SIZE_RX			8		// Receive buffer size for UART receptions (number of bytes)
#define TX_PACKET_SIZE			9		// Maximum packet size for regular motor commands (exclusion: sync write)
#define BUFF_SIZE_SYNC_WRITE	64		// Maximum packet size for sync write
#define NUM_UARTS				6		// Number of UARTs available for motor communication
const uint8_t TRANSMIT_TIMEOUT = 10; 	// Timeout for blocking UART transmissions, in milliseconds
const uint8_t RECEIVE_TIMEOUT = 10;		// Timeout for blocking UART receptions, in milliseconds

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

/******************************* Public Variables *******************************/
/* Buffer for data received from motors. */
uint8_t arrReceive[NUM_MOTORS][BUFF_SIZE_RX] = {{0}};

/* Bytes to be transmitted to motors are written to this array. */
uint8_t arrTransmit[NUM_MOTORS + 1][TX_PACKET_SIZE] = {
	{0xFF, 0xFF, 0xFE, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 1, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 2, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 3, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 4, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 5, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 6, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 7, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 8, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 9, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 10, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 11, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 12, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 13, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 14, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 15, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 16, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 17, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 18, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00}
};

/* Sync write buffer. */
uint8_t arrSyncWrite[NUM_UARTS][BUFF_SIZE_SYNC_WRITE];

/*********************************** Types ************************************/
typedef struct{
	uint8_t					_ID;					/*!< Motor identification (0-252)					*/
	uint32_t				_BaudRate;				/*!< UART communication baud rate					*/
	uint16_t				_lastPosition;			/*!< Position read from motor						*/
	float					_lastVelocity;			/*!< Velocity read from motor						*/
	uint8_t					_lastLoad;				/*!< Load read from motor							*/
	uint8_t					_lastLoadDirection;		/*!< 1 -> CW | 0 -> CCW								*/
	float					_lastVoltage;			/*!< Voltage read from motor						*/
	uint8_t					_lastTemperature;		/*!< Temperature read from motor					*/
	uint8_t					_isJointMode;			/*!< 1 if motor is joint mode, 0 if wheel mode		*/
	UART_HandleTypeDef*		_UART_Handle;			/*!< UART handle for motor							*/
	GPIO_TypeDef*			_dataDirPort;			/*!< Port data direction pin is on					*/
	uint16_t				_dataDirPinNum;			/*!< Data direction pin number						*/
}Dynamixel_HandleTypeDef;

/***************************** Function prototypes ****************************/
// Setters (use the WRITE DATA instruction)
void Dynamixel_SetID(Dynamixel_HandleTypeDef* hdynamixel, uint8_t ID); // (EEPROM)
void Dynamixel_SetBaudRate(Dynamixel_HandleTypeDef* hdynamixel, double baud); // (EEPROM)
void Dynamixel_SetReturnDelayTime(Dynamixel_HandleTypeDef* hdynamixel, double microSec); // (EEPROM)
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
void Dynamixel_SetCWComplianceMargin(Dynamixel_HandleTypeDef* hdynamixel, uint8_t CWcomplianceMargin); // (RAM)
void Dynamixel_SetCCWComplianceMargin(Dynamixel_HandleTypeDef* hdynamixel, uint8_t CCWcomplianceMargin); // (RAM)
void Dynamixel_SetCWComplianceSlope(Dynamixel_HandleTypeDef* hdynamixel, uint8_t CWcomplianceSlope); // (RAM)
void Dynamixel_SetCCWComplianceSlope(Dynamixel_HandleTypeDef* hdynamixel, uint8_t CCWcomplianceSlope); // (RAM)
void Dynamixel_SetGoalPosition(Dynamixel_HandleTypeDef* hdynamixel, double goalAngle); // (RAM)
void Dynamixel_SetGoalVelocity(Dynamixel_HandleTypeDef* hdynamixel, double goalVelocity); // (RAM)
void Dynamixel_SetGoalTorque(Dynamixel_HandleTypeDef* hdynamixel, double goalTorque); // (RAM)
void Dynamixel_LockEEPROM(Dynamixel_HandleTypeDef* hdynamixel); // (RAM)
void Dynamixel_SetPunch(Dynamixel_HandleTypeDef* hdynamixel, double punch); // (RAM)

// Getters (use READ DATA instruction)
void Dynamixel_GetPosition(Dynamixel_HandleTypeDef* hdynamixel);
void Dynamixel_GetVelocity(Dynamixel_HandleTypeDef* hdynamixel);
void Dynamixel_GetLoad(Dynamixel_HandleTypeDef* hdynamixel);
void Dynamixel_GetVoltage(Dynamixel_HandleTypeDef* hdynamixel);
void Dynamixel_GetTemperature(Dynamixel_HandleTypeDef* hdynamixel);
uint8_t Dynamixel_IsRegistered(Dynamixel_HandleTypeDef* hdynamixel);
uint8_t Dynamixel_IsMoving(Dynamixel_HandleTypeDef* hdynamixel);
uint8_t Dynamixel_IsJointMode(Dynamixel_HandleTypeDef* hdynamixel);

// Other motor instructions (low level control with timing from different WRITE DATA instruction)
void Dynamixel_RegWrite(Dynamixel_HandleTypeDef* hdynamixel, uint8_t arrSize, uint8_t writeAddr, uint8_t param1, uint8_t param2);
void Dynamixel_Action(Dynamixel_HandleTypeDef* hdynamixel);

// Computation
inline uint8_t Dynamixel_ComputeChecksum(uint8_t *arr, int length){
	/* Compute the checksum for data to be transmitted.
	 *
	 * Arguments: arr, the array to be transmitted and ran through the checksum function
	 * 			  length, the total length of the array arr
	 *
	 * Returns: the 1-byte number that is the checksum
	 */

	/* Local variable declaration. */
	uint8_t accumulate = 0;

	/* Loop through the array starting from the 2nd element of the array and finishing before the last
	 * since the last is where the checksum will be stored. */
	for(uint8_t i = 2; i < length - 1; i++){
		accumulate += arr[i];
	}

	return (~accumulate) & 0xFF; // Lower 8 bits of the logical NOT of the sum
}

// Transmission & Reception
uint8_t Dynamixel_Ping(Dynamixel_HandleTypeDef* hdynamixel);
void Dynamixel_DataWriter(Dynamixel_HandleTypeDef* hdynamixel, uint8_t arrSize, uint8_t writeAddr, uint8_t param1, uint8_t param2);
void Dynamixel_SyncWriter(Dynamixel_HandleTypeDef* hdynamixel, uint8_t uartIndex, uint8_t arrSize, uint8_t *params); // UNIMPLEMENTED
uint16_t Dynamixel_DataReader(Dynamixel_HandleTypeDef* hdynamixel, uint8_t readAddr, uint8_t readLength);

// Initialization
void Dynamixel_Init(Dynamixel_HandleTypeDef* hdynamixel, uint8_t ID, UART_HandleTypeDef* UART_Handle,\
		GPIO_TypeDef* DataDirPort, uint16_t DataDirPinNum);
void Dynamixel_Reset(Dynamixel_HandleTypeDef* hdynamixel);

// Error handling
void Dynamixel_ErrorHandler(uint8_t);

// Interfaces for previously-defined functions
void Dynamixel_Revive(Dynamixel_HandleTypeDef* hdynamixel, uint8_t ID);
void Dynamixel_BroadcastRevive(Dynamixel_HandleTypeDef* hdynamixel, uint8_t ID);
void Dynamixel_EnterWheelMode(Dynamixel_HandleTypeDef* hdynamixel, double goalVelocity);
void Dynamixel_EnterJointMode(Dynamixel_HandleTypeDef* hdynamixel);

// Testing
void Dynamixel_TestAll(Dynamixel_HandleTypeDef** arrHdynamixel, uint8_t arrSize);

#endif /* __DYNAMIXEL_AX-12A_H__ */
