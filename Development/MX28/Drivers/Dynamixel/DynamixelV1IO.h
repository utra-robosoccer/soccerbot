/* This header is for IO-related functions and data structures that need to be
 * included across all Dynamixel files.
 *
 * Author: Tyler
 */

/******************** Define to prevent recursive inclusion *******************/
#ifndef __DYNAMIXEL_V1_IO_H__
#define __DYNAMIXEL_V1_IO_H__

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

/* Default register values */
#define DEFAULT_ID				0x01	    // Default motor ID

/******************************* Public Variables *******************************/
/* Buffer for data received from motors. */
extern uint8_t arrReceive[NUM_MOTORS][BUFF_SIZE_RX];

/* Bytes to be transmitted to motors are written to this array. */
extern uint8_t arrTransmit[NUM_MOTORS + 1][TX_PACKET_SIZE];

/*********************************** Types ************************************/
enum motorTypes_e{
	AX12ATYPE,
	MX28TYPE
};

// TODO: Should also have a resolution divider field for MX28
typedef struct{
	enum motorTypes_e		_motorType;				/*!< Identifies motor as AX12A, MX28, etc.			*/
	uint8_t					_protocolVersion;
	uint8_t					_ID;					/*!< Motor identification (0-252)					*/
	float					_lastPosition;			/*!< Position read from motor						*/
	float					_lastVelocity;			/*!< Velocity read from motor						*/
	float					_lastLoad;				/*!< Load read from motor (% of max torque)			*/
	uint8_t					_lastLoadDirection;		/*!< 1 -> CW | 0 -> CCW								*/
	uint8_t					_isJointMode;			/*!< 1 if motor is joint mode, 0 if wheel mode		*/
	UART_HandleTypeDef*		_UART_Handle;			/*!< UART handle for motor							*/
	GPIO_TypeDef*			_dataDirPort;			/*!< Port data direction pin is on					*/
	uint16_t				_dataDirPinNum;			/*!< Data direction pin number						*/
}Dynamixel_HandleTypeDef;

/***************************** Function prototypes ****************************/
// Transmission & Reception
void Dynamixel_DataWriter(Dynamixel_HandleTypeDef* hdynamixel, uint8_t arrSize, uint8_t writeAddr, uint8_t param1, uint8_t param2);
uint16_t Dynamixel_DataReader(Dynamixel_HandleTypeDef* hdynamixel, uint8_t readAddr, uint8_t readLength);

// Other motor instructions (low level control with timing from different WRITE DATA instruction)
void Dynamixel_RegWrite(Dynamixel_HandleTypeDef* hdynamixel, uint8_t arrSize, uint8_t writeAddr, uint8_t param1, uint8_t param2);
void Dynamixel_Action(Dynamixel_HandleTypeDef* hdynamixel);
int8_t Dynamixel_Ping(Dynamixel_HandleTypeDef* hdynamixel);

// Setup
void Dynamixel_Reset(Dynamixel_HandleTypeDef* hdynamixel);

#endif /* __DYNAMIXEL_V1_IO_H__ */
