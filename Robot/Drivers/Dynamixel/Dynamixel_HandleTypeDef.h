/**
  ******************************************************************************
  * @file    Dynamixel_Types.h
  * @author  Tyler
  * @brief   This file defines the data structure used by all Dynamixel
  *          actuators, as well as user-defined types it uses
  *
  * @ingroup Dynamixel
  ******************************************************************************
  */




/******************** Define to prevent recursive inclusion ******************/
#ifndef __DYNAMIXEL_HANDLE_TYPE_DEF_H__
#define __DYNAMIXEL_HANDLE_TYPE_DEF_H__

#ifdef __cplusplus
extern "C" {
#endif




/********************************* Includes **********************************/
#include <stdint.h>
#include <stdbool.h>
#include "gpio.h"




/********************************** Types ************************************/
/** Enumerates the types of Dynamixel actuators the library supports */
typedef enum motorTypes_e{
	AX12ATYPE,      /**< AX12A actuator */
	MX28TYPE        /**< MX28 actuator  */
}motorTypes_t;

// TODO: Should also have a resolution divider field for MX28
/** Organizes all the information relevant to a motor */
typedef struct{
    motorTypes_t            _motorType;             /**< Identifies motor as AX12A, MX28, etc.          */
    uint8_t                 _ID;                    /**< Motor identification (0-252)                   */
    bool                    _lastReadIsValid;       /**< 1 if checksum verified for last read,
	 	 	 	 	 	 	 	 	 	 	 	 	 	 0 if checksum mismatch                         */
    float                   _lastPosition;          /**< Position read from motor                       */
    float                   _lastVelocity;          /**< Velocity read from motor                       */
    float                   _lastLoad;              /**< Load read from motor (% of max torque)         */
    uint8_t                 _lastLoadDirection;     /**< 1 implies CW | 0 implies CCW                   */
    bool                    _isJointMode;           /**< 1 if motor is joint mode, 0 if wheel mode      */
    UART_HandleTypeDef*     _UART_Handle;           /**< UART handle for motor                          */
    GPIO_TypeDef*           _dataDirPort;           /**< Port data direction pin is on                  */
uint16_t                _dataDirPinNum;             /**< Data direction pin number                      */
}Dynamixel_HandleTypeDef;

#ifdef __cplusplus
}
#endif

#endif /* __DYNAMIXEL_HANDLE_TYPE_DEF_H__ */
