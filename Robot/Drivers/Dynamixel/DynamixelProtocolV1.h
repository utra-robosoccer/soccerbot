/**
  ******************************************************************************
  * @file    DynamixelProtocolV1.h
  * @author  Tyler
  * @brief   Common header code for the AX12A library and MX28 library.
  * 		 It is generic in that any Dynamixel actuator using protocol
  * 		 version 1.0 should be able to be integrated with little effort, as
  * 		 the instructions and register addresses are very similar for all
  * 		 actuators using this protocol version
  *
  * @defgroup DynamixelProtocolV1Header Dynamixel Protocol V1.0 (header)
  * @brief    Header for Dynamixel Protocol V1.0, showing the public content
  * @ingroup  DynamixelProtocolV1
  * @{
  ******************************************************************************
  */




/******************** Define to prevent recursive inclusion ******************/
#ifndef __DYNAMIXEL_PROTOCOL_V1_H__
#define __DYNAMIXEL_PROTOCOL_V1_H__

#ifdef __cplusplus
extern "C" {
#endif




/********************************* Includes **********************************/
// I/O
#include "gpio.h"
#include "usart.h"

// Types, math constants, and math functions
#include <stdint.h>
#include <math.h>

// Dynamixel actuators
#include "Dynamixel_HandleTypeDef.h"
#include "MX28/MX28.h"
#include "AX12A/AX12A.h"

// System-level
#include "sharedMacros.h"
#include "cmsis_os.h"




/********************************** Macros ***********************************/
// Library parameters
#define NUM_MOTORS				18		/**< Used to determine buffer sizes */
#define NUM_UARTS				6		/**< Number of UARTs available for motor communication */




/********************************* Constants *********************************/
// Default register values
extern const uint8_t BROADCAST_ID;
extern const uint8_t DEFAULT_ID;
extern const uint8_t DEFAULT_RETURN_DELAY;
extern const uint16_t DEFAULT_CW_ANGLE_LIMIT;
extern const uint8_t DEFAULT_LOW_VOLTAGE_LIMIT;
extern const uint16_t DEFAULT_MAXIMUM_TORQUE;
extern const uint8_t DEFAULT_STATUS_RETURN_LEVEL;
extern const uint8_t DEFAULT_ALARM_LED;
extern const uint8_t DEFAULT_ALARM_SHUTDOWN;
extern const uint8_t DEFAULT_TORQUE_ENABLE;
extern const uint8_t DEFAULT_LED_ENABLE;
extern const uint8_t DEFAULT_EEPROM_LOCK;

// Value limit definitions
extern const float MIN_VELOCITY;
extern const float MAX_ANGLE;
extern const float MIN_ANGLE;
extern const float MAX_TORQUE;
extern const float MIN_TORQUE;
extern const float MAX_VOLTAGE;
extern const float MIN_VOLTAGE;
extern const uint16_t MAX_PUNCH;
extern const uint16_t MIN_PUNCH;




/*************************** Library configuration ***************************/
/** Enumerates the low-level I/O modes the library supports */
enum IO_FLAGS{
    IO_DMA,     /**< Direct memory access */
    IO_POLL,    /**< Polled I/O           */
    IO_IT       /**< Interrupt-based I/O  */
};




/***************************** Function prototypes ***************************/
// Setters (use the WRITE DATA instruction)
void Dynamixel_SetID(Dynamixel_HandleTypeDef* hdynamixel, uint8_t ID); // (EEPROM)
void Dynamixel_SetBaudRate(Dynamixel_HandleTypeDef* hdynamixel, uint32_t baud); // (EEPROM)
void Dynamixel_SetReturnDelayTime(Dynamixel_HandleTypeDef* hdynamixel, uint16_t microSec); // (EEPROM)
void Dynamixel_SetCWAngleLimit(Dynamixel_HandleTypeDef* hdynamixel, float minAngle); // (EEPROM)
void Dynamixel_SetCCWAngleLimit(Dynamixel_HandleTypeDef* hdynamixel, float maxAngle); // (EEPROM)
void Dynamixel_SetHighestVoltageLimit(Dynamixel_HandleTypeDef* hdynamixel, float highestVoltage); // (EEPROM)
void Dynamixel_SetLowestVoltageLimit(Dynamixel_HandleTypeDef* hdynamixel, float lowestVoltage); // (EEPROM)
void Dynamixel_SetMaxTorque(Dynamixel_HandleTypeDef* hdynamixel, float maxTorque); // (EEPROM)
void Dynamixel_SetStatusReturnLevel(Dynamixel_HandleTypeDef* hdynamixel, uint8_t status_data); // (EEPROM)
void Dynamixel_SetAlarmLED(Dynamixel_HandleTypeDef* hdynamixel, uint8_t alarm_LED_data); // (EEPROM)
void Dynamixel_SetAlarmShutdown(Dynamixel_HandleTypeDef* hdynamixel, uint8_t alarm_shutdown_data); // (EEPROM)
void Dynamixel_TorqueEnable(Dynamixel_HandleTypeDef* hdynamixel, uint8_t isEnabled); // (RAM)
void Dynamixel_LEDEnable(Dynamixel_HandleTypeDef* hdynamixel, uint8_t isEnabled); // (RAM)
void Dynamixel_SetGoalPosition(Dynamixel_HandleTypeDef* hdynamixel, float goalAngle); // (RAM)
void Dynamixel_SetGoalVelocity(Dynamixel_HandleTypeDef* hdynamixel, float goalVelocity); // (RAM)
void Dynamixel_SetGoalTorque(Dynamixel_HandleTypeDef* hdynamixel, float goalTorque); // (RAM)
void Dynamixel_LockEEPROM(Dynamixel_HandleTypeDef* hdynamixel); // (RAM)
void Dynamixel_SetPunch(Dynamixel_HandleTypeDef* hdynamixel, float punch); // (RAM)

// Getters (use READ DATA instruction)
void Dynamixel_GetPosition(Dynamixel_HandleTypeDef* hdynamixel);
void Dynamixel_GetVelocity(Dynamixel_HandleTypeDef* hdynamixel);
void Dynamixel_GetLoad(Dynamixel_HandleTypeDef* hdynamixel);
float Dynamixel_GetVoltage(Dynamixel_HandleTypeDef* hdynamixel);
uint8_t Dynamixel_GetTemperature(Dynamixel_HandleTypeDef* hdynamixel);
bool Dynamixel_IsRegistered(Dynamixel_HandleTypeDef* hdynamixel);
bool Dynamixel_IsMoving(Dynamixel_HandleTypeDef* hdynamixel);
bool Dynamixel_IsJointMode(Dynamixel_HandleTypeDef* hdynamixel);

// Library configuration functions
void Dynamixel_SetIOType(enum IO_FLAGS type);
enum IO_FLAGS Dynamixel_GetIOType();

// Low-level transmission and reception functions
void Dynamixel_DataWriter(Dynamixel_HandleTypeDef* hdynamixel, uint8_t* args, uint8_t numArgs);
uint16_t Dynamixel_DataReader(Dynamixel_HandleTypeDef* hdynamixel, uint8_t readAddr, uint8_t readLength);

// Other motor instruction functions
void Dynamixel_RegWrite(Dynamixel_HandleTypeDef* hdynamixel, uint8_t arrSize, uint8_t writeAddr, uint8_t param1, uint8_t param2);
void Dynamixel_Action(Dynamixel_HandleTypeDef* hdynamixel);
int8_t Dynamixel_Ping(Dynamixel_HandleTypeDef* hdynamixel);

// Setup functions
void Dynamixel_Init(Dynamixel_HandleTypeDef* hdynamixel, uint8_t ID, UART_HandleTypeDef* UART_Handle,\
		GPIO_TypeDef* DataDirPort, uint16_t DataDirPinNum, enum motorTypes_e motorType);
void Dynamixel_Reset(Dynamixel_HandleTypeDef* hdynamixel);

// Interfaces for previously-defined functions
void Dynamixel_EnterWheelMode(Dynamixel_HandleTypeDef* hdynamixel, float goalVelocity);
void Dynamixel_EnterJointMode(Dynamixel_HandleTypeDef* hdynamixel);

/**
 * @}
 */
/* end DynamixelProtocolV1Header */

#ifdef __cplusplus
}
#endif

#endif /* __DYNAMIXEL_PROTOCOL_V1_H__ */
