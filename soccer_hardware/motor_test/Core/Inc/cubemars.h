#ifndef CAN_TEST
#define CAN_TEST
#include "main.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>

// This CAN TEST lib is only for Cubmars
/*
CUBEMARs uses CAN Exd ID
[28 : 8] Control mode id
[7  : 0] Driver ID (Can be viewed using upper computer)
*/

#define  TEST_MOTOR_ID 1
#define  CAN_MOTION_CTRL 8

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
#define TEST_KP 200
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

extern uint16_t pos_int;
extern uint16_t spd_int;
extern uint16_t torq_int;
extern uint8_t motor_temp;
extern uint8_t error_code;

extern float pos_float;
extern float spd_float;
extern float torq_float;
extern float motor_temp_float;

HAL_StatusTypeDef CUBEMARS_set_torque(CAN_HandleTypeDef *phcan, uint32_t* pTxMailBox, float current);
HAL_StatusTypeDef CUBEMARS_set_pos(CAN_HandleTypeDef *phcan, uint32_t* pTXmailBox, float pos);
HAL_StatusTypeDef CUBEMARS_enable_motion_ctrl(CAN_HandleTypeDef *phcan, uint32_t* pTxmailBox);
HAL_StatusTypeDef CUBEMARS_disable_motion_ctrl(CAN_HandleTypeDef *phcan, uint32_t* pTxmailBox);
HAL_StatusTypeDef CUBEMARS_set_origin(CAN_HandleTypeDef *phcan, uint32_t* pTxmailBox);
void CUBARMARS_unpack_mit_ctrl_parameters(uint8_t* recv_msg, int recv_msg_len);
HAL_StatusTypeDef CUBEMARS_set_motion_ctrl_parameters(CAN_HandleTypeDef *phcan, uint32_t* pTxMailBox,
                float pos, float rpm, float kp, float kd, float torque);
#endif
