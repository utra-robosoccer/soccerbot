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
  * @defgroup DynamixelProtocolV1Header Header
  * @ingroup  DynamixelProtocolV1
  * @{
  ******************************************************************************
  */




/******************** Define to prevent recursive inclusion ******************/
#ifndef __DYNAMIXEL_PROTOCOL_V1_H__
#define __DYNAMIXEL_PROTOCOL_V1_H__




/********************************* Includes **********************************/
// I/O
#include "gpio.h"
#include "usart.h"

// Types, math constants, and math functions
#include <stdint.h>
#include <math.h>

// Dynamixel actuators
#include "h/MX28.h"
#include "h/AX12A.h"
#include "h/Dynamixel_Types.h"
#include "h/Dynamixel_Data.h"




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
void Dynamixel_SetIOType(ioFlags_t type);
ioFlags_t Dynamixel_GetIOType();

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

#endif /* __DYNAMIXEL_PROTOCOL_V1_H__ */
