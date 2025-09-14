#include "robostride_test.h"
#include "main.h"
#include "stm32f4xx_hal_can.h"
#include "stm32f4xx_hal_def.h"
#include <stdint.h>

CAN_TxHeaderTypeDef rs_can_tx_header = {
    .StdId = 0x0,
    .ExtId = 0xff, //dummy value
    .IDE   = CAN_ID_EXT,
    .RTR   = CAN_RTR_DATA,
    .DLC   = 8
};

CAN_RxHeaderTypeDef rs_can_rx_header;

//Expand the .extid to self defined struct. defined in .h file
 #define txCanIdEx (*((exCanIdInfo*)&(rs_can_tx_header.ExtId)))
 #define rxCanIdEx (*((exCanIdInfo*)&(rs_can_rx_header.ExtId)))

//Helper Function Definitions
static int float_to_uint(float x, float x_min, float x_max, unsigned int bits)
{  
/// Converts a float to an int, given range and number of bits ///  
    float span = x_max - x_min;  
    if(x < x_min) x = x_min; 
    else if(x > x_max) x = x_max; 
    return (int) ((x- x_min)*((float)((1<<bits)/span))); 
} 

static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
 /// converts unsigned int to float, given range and number of bits ///
    float span = x_max- x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

//Comm Type 3
HAL_StatusTypeDef can_enable_motor(uint8_t id, uint16_t master_id)
{
    char msg[8];
    txCanIdEx.mode = 3;
    txCanIdEx.id = id;
    txCanIdEx.res = 0;
    txCanIdEx.data = master_id; 
    // txCanIdEx.data = 0; //For some reason we have this on the datasheet
    rs_can_tx_header.DLC = 8;

    return HAL_CAN_AddTxMessage(&hcan1, &rs_can_tx_header, (const uint8_t*)msg, &TxMailbox);   
}

//Comm Type 1
HAL_StatusTypeDef can_mit_control_set(uint8_t id, float torque, float MechPosition, float speed, float kp, float kd)
{
    uint8_t msg[8];
    //set txID
    txCanIdEx.mode = 1;
    txCanIdEx.id = id;
    txCanIdEx.data = float_to_uint(torque, T_MIN, T_MAX, 16);
    txCanIdEx.res = 0;

    rs_can_tx_header.DLC = 8;

    msg[0] = float_to_uint(MechPosition, P_MIN, P_MAX, 16) >> 8;
    msg[1] = float_to_uint(MechPosition, P_MIN, P_MAX, 16) & 0xFF;
    msg[2] = float_to_uint(speed, V_MIN, V_MAX, 16) >> 8;
    msg[3] = float_to_uint(speed, V_MIN, V_MAX, 16) & 0xFF;
    msg[4] = float_to_uint(kp, KP_MIN, KP_MAX, 16) >> 8;
    msg[5] = float_to_uint(kp, KP_MIN, KP_MAX, 16) & 0xFF;
    msg[6] = float_to_uint(kd, KD_MIN, KD_MAX, 16) >> 8;
    msg[7] = float_to_uint(kd, KD_MIN, KD_MAX, 16) & 0xFF;
    return HAL_CAN_AddTxMessage(&hcan1, &rs_can_tx_header, msg, &TxMailbox);
}

//Comm Type 4
HAL_StatusTypeDef can_disable_motor(uint8_t id, uint16_t master_id)
{
    txCanIdEx.mode = 4;
    txCanIdEx.id = id;
    txCanIdEx.data = master_id;
    txCanIdEx.res = 0;

    uint8_t msg[8] = {0x0};

    rs_can_tx_header.DLC = 8;

    return HAL_CAN_AddTxMessage(&hcan1, &rs_can_tx_header, msg, &TxMailbox);
}

//Comm Type 18 -> Special
HAL_StatusTypeDef can_change_motor_mode(uint8_t id, uint16_t master_id, rs_runmode_t rs_runmode)
{
    uint16_t reg_idx = 0x7005; //Consider using enums in the future
    txCanIdEx.mode = 0x12; //comm type 18
    txCanIdEx.id = id;
    txCanIdEx.data = master_id;
    txCanIdEx.res = 0;

    uint8_t msg[8] = {0x0};

    rs_can_tx_header.DLC = 8;

    msg[0] = reg_idx & 0xff;
    msg[1] = reg_idx >> 8;
    //msg[2] and [3] set to 0
    msg[4] = rs_runmode & 0xff;

    return HAL_CAN_AddTxMessage(&hcan1, &rs_can_tx_header, msg, &TxMailbox);
}

//Comm Type 2 -> Rx side. Called when verified Can Msg mode 0x2 in the Callback function  
HAL_StatusTypeDef can_unpack_motor_feedback(motor_t* motor, uint8_t* recv_buf) //may need to revise 
{
    if ((rxCanIdEx.data & MOTOR_FEEDBACK_MASK_ID) >> MOTOR_FEEDBACK_ID_OFFSET != motor -> id){
        return HAL_ERROR;
    }
    //update error status
    motor -> motor_errors.driver_fault = (rxCanIdEx.data & MOTOR_ERROR_MASK_DRIVER_FAULT)>>MOTOR_ERROR_OFFSET_DRIVER_FAULT;
    motor -> motor_errors.uncalibrated = (rxCanIdEx.data & MOTOR_ERROR_MASK_UNCALIBRATED) >> MOTOR_ERROR_OFFSET_UNCALIBRATED;
    motor -> motor_errors.stall_overload = (rxCanIdEx.data & MOTOR_ERROR_MASK_STALL_OVERLOAD) >> MOTOR_ERROR_OFFSET_STALL_OVERLOAD;
    motor -> motor_errors.encoder_fault = (rxCanIdEx.data & MOTOR_ERROR_MASK_ENCODER_FAULT) >> MOTOR_ERROR_OFFSET_ENCODER_FAULT;
    motor -> motor_errors.overheat = (rxCanIdEx.data & MOTOR_ERROR_MASK_OVERHEAT) >> MOTOR_ERROR_OFFSET_OVERHEAT;
    motor -> motor_errors.undervoltage = (rxCanIdEx.data & MOTOR_ERROR_MASK_UNDERVOLTAGE) >> MOTOR_ERROR_OFFSET_UNDERVOLTAGE;

    motor -> pos = uint_to_float(recv_buf[1] << 8 | recv_buf[0], P_MIN, P_MAX, 16);
    motor -> rpm = uint_to_float(recv_buf[3] << 8 | recv_buf[2], V_MIN, V_MAX, 16);
    motor -> torq = uint_to_float(recv_buf[5] << 8 | recv_buf[4], T_MIN, T_MAX, 16);
    motor -> temperature = (float)(recv_buf[7] << 8 | recv_buf[6]) / 10.0;
    return HAL_OK;
}

//Comm Type 6
HAL_StatusTypeDef can_set_mech_zero(uint8_t id, uint16_t master_id)
{
    txCanIdEx.mode = 0x6;
    txCanIdEx.data = master_id;
    txCanIdEx.id = id;
    txCanIdEx.res = 0;

    uint8_t msg[8] = {0};
    msg[0] = 0x1;

    rs_can_tx_header.DLC = 8;
    return HAL_CAN_AddTxMessage(&hcan1, &rs_can_tx_header, msg, &TxMailbox);
}
