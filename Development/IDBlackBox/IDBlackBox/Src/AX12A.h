/****************** Drivers for Dynamixel AX-12A Servo Motor ******************/

/******************** Define to prevent recursive inclusion *******************/
#ifndef __DYNAMIXEL_AX_12A_H__
#define __DYNAMIXEL_AX_12A_H__

/********************************** Includes **********************************/
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_conf.h"

#include "gpio.h"

#include <stdint.h>
/*********************************** Macros ***********************************/
#define __DYNAMIXEL_TRANSMIT(port, pinNum) HAL_GPIO_WritePin(port, pinNum, GPIO_PIN_SET) // Set data direction pin high (TX)
#define __DYNAMIXEL_RECEIVE(port, pinNum) HAL_GPIO_WritePin(port, pinNum, GPIO_PIN_RESET) // Set data direction pin low (RX)

/*********************************** Defines **********************************/
/* Communications. */
extern const uint8_t TRANSMIT_TIMEOUT; 	// Timeout for blocking UART transmissions, in milliseconds
extern const uint8_t RECEIVE_TIMEOUT;		// Timeout for blocking UART receptions, in milliseconds

/* Instruction set definitions. */
#define INST_WRITE_DATA			0x03	// Writes data for immediate execution
#define INST_RESET				0x06	// Resets the Dynamixel actuator(s) specified

/* Register definitions. */
#define REG_ID 						0x03		// Motor ID register
#define REG_LED_ENABLE				0x19		// LED control register
#define REG_GOAL_POSITION			0x1E		// Goal position register (0x1E = low byte, 0x1F = high byte)
#define REG_GOAL_VELOCITY			0x20		// Goal velocity register (0x20 = low byte, 0x21 = high byte)


/* Default register value definitions. */
#define BROADCAST_ID					0xFE	// Motor broadcast ID (i.e. messages sent to this ID will be sent to all motors on the bus)
#define DEFAULT_ID						0x01	// Default motor ID
#define DEFAULT_LED_ENABLE				0x00	// Default LED state

extern const  double MAX_VELOCITY;	// Maximum angular velocity (RPM)
extern const double MIN_VELOCITY;		// Minimum angular velocity (RPM)
extern const uint16_t MAX_ANGLE ;		// Maximum angular position (joint mode)
extern const uint8_t MIN_ANGLE;		// Minimum angular position (joint mode)

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
void Dynamixel_LEDEnable(Dynamixel_HandleTypeDef* hdynamixel, uint8_t isEnabled); // (RAM)
void Dynamixel_SetGoalPosition(Dynamixel_HandleTypeDef* hdynamixel, double goalAngle); // (RAM)
void Dynamixel_SetGoalVelocity(Dynamixel_HandleTypeDef* hdynamixel, double goalVelocity); // (RAM)


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
void Dynamixel_DataWriter(Dynamixel_HandleTypeDef* hdynamixel, uint8_t arrSize, uint8_t writeAddr, uint8_t param1, uint8_t param2);

// Initialization
void Dynamixel_Init(Dynamixel_HandleTypeDef* hdynamixel, uint8_t ID, UART_HandleTypeDef* UART_Handle,\
		GPIO_TypeDef* DataDirPort, uint16_t DataDirPinNum);
void Dynamixel_Reset(Dynamixel_HandleTypeDef* hdynamixel);

// Interfaces for previously-defined functions
void Dynamixel_BroadcastRevive(Dynamixel_HandleTypeDef* hdynamixel, uint8_t ID);

#endif /* __DYNAMIXEL_AX-12A_H__ */
