
/******************** Define to prevent recursive inclusion *******************/
#ifndef __DYNAMIXEL_HANDLE_TYPE_DEF_H__
#define __DYNAMIXEL_HANDLE_TYPE_DEF_H__

/********************************** Includes **********************************/
#include <stdint.h>
#include "gpio.h"
#include "main.h"

/*********************************** Types ************************************/
enum motorTypes_e{
	AX12ATYPE,
	MX28TYPE
};

// TODO: Should also have a resolution divider field for MX28
typedef struct{
	enum motorTypes_e		_motorType;				/*!< Identifies motor as AX12A, MX28, etc.			*/
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

#define MAX_MOTORS_PER_UART 6
typedef struct{
	UART_HandleTypeDef* _UART_Handle; // UART handle in common to all motors here
	Dynamixel_HandleTypeDef* motorHandles[MAX_MOTORS_PER_UART]; // Array of motor handles
	float motorArgs[2 * MAX_MOTORS_PER_UART]; /* Buffer for motor commands
											   *
											   * Example: sending position commands for 3 motors
											   * 	motorArgs = {motor1pos, motor2pos, motor3pos}
											   *
											   * Example: sending position and velocity commands for 3 motors
											   * 	motorArgs = {motor1pos, motor1vel, motor2pos, motor2vel, motor3pos, motor3vel}
											   */
	uint8_t numMotors; // Number of motors in the motorHandles array
}SyncWriteBlock_t;

#endif /* __DYNAMIXEL_HANDLE_TYPE_DEF_H__ */
