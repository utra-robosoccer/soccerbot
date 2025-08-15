#ifndef CAN_TEST
#define CAN_TEST

// This CAN TEST lib is only for Cubmars
/*
CUBEMARs uses CAN Exd ID
[28 : 8] Control mode id
[7  : 0] Driver ID (Can be viewed using upper computer)
*/
#include "main.h"
#include "stm32f4xx_hal_can.h"
#include "stm32f4xx_hal_def.h"
#include <stdint.h>
#include <sys/_intsup.h>
#include <sys/types.h>

#define  TEST_MOTOR_ID 1

#define MAX_POS 12.5
#define MIN_POS -12.5
#define MAX_RPM 6.0
#define MIN_RPM -6.0
#define MAX_TORQUE  34.0
#define MIN_TORQUE -34.0
#define MIN_KD_KP 0
#define MAX_KD 5
#define MAX_KP 500

#define TEST_KD 5
#define TEST_KP 100
#define TEST_RPM 6.0

typedef enum{
    CAN_PACKET_SET_DUTY = 0,
    CAN_PACKETY_SET_CURRENT,
    CAN_PACKET_SET_CURRENT_BRAKE, 
    CAN_PACKET_SET_RPM,
    CAN_PACKET_SET_POS,
    CAN_PACKET_SET_ORIGIN_HERE,
    CAN_PACKET_SET_POS_SPD,
} CAN_EXD_PACKET_CONTROLMODE_ID;

HAL_StatusTypeDef CUBEMARS_set_torque(CAN_HandleTypeDef *phcan, uint32_t* pTxMailBox, float current);
HAL_StatusTypeDef CUBEMARS_set_pos(CAN_HandleTypeDef *phcan, uint32_t* pTXmailBox, float pos);
HAL_StatusTypeDef CUBEMARS_enable_motion_ctrl(CAN_HandleTypeDef *phcan, uint32_t* pTxmailBox);
HAL_StatusTypeDef CUBEMARS_disable_motion_ctrl(CAN_HandleTypeDef *phcan, uint32_t* pTxmailBox);
HAL_StatusTypeDef CUBEMARS_set_origin(CAN_HandleTypeDef *phcan, uint32_t* pTxmailBox);
HAL_StatusTypeDef CUBEMARS_set_motion_ctrl_parameters(CAN_HandleTypeDef *phcan, uint32_t* pTxMailBox,
                float pos, float rpm, float kp, float kd, float torque);
#endif