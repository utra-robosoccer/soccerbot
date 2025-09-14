#include "cubemars.h"

uint16_t pos_int;
uint16_t spd_int;
uint16_t torq_int;
uint8_t motor_temp;
uint8_t error_code;

float pos_float;
float spd_float;
float torq_float;
float motor_temp_float;

HAL_StatusTypeDef CAN_transmit_extid(CAN_HandleTypeDef *phcan, uint32_t* pTXmailBox, 
                                    uint32_t control_mode, uint8_t motor_id, const uint8_t *msg, uint8_t packet_len)
{
    // id should be ext id
    if (packet_len > 8){
        //Paket len max is 8 bytes
        packet_len = 8;
    }

    uint32_t eid = (control_mode << 8) | motor_id;

    CAN_TxHeaderTypeDef TxHeader;
    TxHeader.StdId = 0;
    TxHeader.ExtId = eid & 0x1ffffff;
    TxHeader.IDE = CAN_ID_EXT;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = packet_len;

    return HAL_CAN_AddTxMessage(phcan, &TxHeader, msg, pTXmailBox);
}

HAL_StatusTypeDef CAN_transmit_stdid(CAN_HandleTypeDef *phcan, uint32_t* pTXmailBox,
                                    uint32_t motor_id, const uint8_t *msg, uint8_t packet_len)
{
    if (packet_len > 8){
        //Paket len max is 8 bytes
        packet_len = 8;
    }
    uint16_t stdid = motor_id & 0x7FF;

    CAN_TxHeaderTypeDef TxHeader;
    TxHeader.StdId = stdid;
    TxHeader.ExtId = 0;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = packet_len;

    return HAL_CAN_AddTxMessage(phcan, &TxHeader, msg, pTXmailBox);
}

void buffer_fill_int32(uint8_t* tx_buf, int32_t data)
{
    for (int8_t i = 3, pos_bit = 0; i >= 0; i --, pos_bit ++){
        tx_buf[i] = (data & (0xff << pos_bit * 8)) >> (pos_bit * 8);
    }
}

HAL_StatusTypeDef CUBEMARS_set_torque(CAN_HandleTypeDef *phcan, uint32_t* pTxMailBox, float current)
{
    uint8_t tx_buf[4];
    int32_t current_for_motor = current * 1000;
    buffer_fill_int32(tx_buf, current_for_motor);
    return CAN_transmit_extid(phcan, pTxMailBox, CAN_PACKETY_SET_CURRENT, TEST_MOTOR_ID, tx_buf, sizeof(tx_buf));
}

HAL_StatusTypeDef CUBEMARS_set_pos(CAN_HandleTypeDef *phcan, uint32_t* pTXmailBox, float pos)
{
    uint8_t tx_buf[4]; //pos mode only require 4 bytes
    int32_t pos_for_motor = pos * 10000.0;
    buffer_fill_int32(tx_buf, pos_for_motor);
    return CAN_transmit_extid(phcan, pTXmailBox, CAN_PACKET_SET_POS, TEST_MOTOR_ID, tx_buf, sizeof(tx_buf));
}

HAL_StatusTypeDef CUBEMARS_enable_motion_ctrl(CAN_HandleTypeDef *phcan, uint32_t* pTxmailBox)
{
    uint8_t tx_buf[8];
    tx_buf[0] = 0xFF;
    tx_buf[1] = 0xFF;
    tx_buf[2] = 0xFF;
    tx_buf[3] = 0xFF;
    tx_buf[4] = 0xFF;
    tx_buf[5] = 0xFF;
    tx_buf[6] = 0xFF;
    tx_buf[7] = 0xFC;
    

    return CAN_transmit_stdid(phcan, pTxmailBox, TEST_MOTOR_ID, tx_buf, sizeof(tx_buf));
}

HAL_StatusTypeDef CUBEMARS_disable_motion_ctrl(CAN_HandleTypeDef *phcan, uint32_t* pTxmailBox)
{
    uint8_t tx_buf[8];
    tx_buf[0] = 0xFF;
    tx_buf[1] = 0xFF;
    tx_buf[2] = 0xFF;
    tx_buf[3] = 0xFF;
    tx_buf[4] = 0xFF;
    tx_buf[5] = 0xFF;
    tx_buf[6] = 0xFF;
    tx_buf[7] = 0xFD;

    return CAN_transmit_stdid(phcan, pTxmailBox, TEST_MOTOR_ID, tx_buf, sizeof(tx_buf));
}

HAL_StatusTypeDef CUBEMARS_set_origin(CAN_HandleTypeDef *phcan, uint32_t* pTxmailBox)
{
    uint8_t tx_buf[8];
    tx_buf[0] = 0xFF;
    tx_buf[1] = 0xFF;
    tx_buf[2] = 0xFF;
    tx_buf[3] = 0xFF;
    tx_buf[4] = 0xFF;
    tx_buf[5] = 0xFF;
    tx_buf[6] = 0xFF;
    tx_buf[7] = 0xFE;

    return CAN_transmit_stdid(phcan, pTxmailBox, TEST_MOTOR_ID, tx_buf, sizeof(tx_buf));
}

static int float_to_int(float x, float x_min, float x_max, unsigned int bits)
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

void CUBARMARS_unpack_mit_ctrl_parameters(uint8_t* recv_msg, int recv_msg_len)
{
    pos_int = (recv_msg[1] << 8) | recv_msg[2];
    spd_int = (recv_msg[3] << 4) | (recv_msg[4] >> 4);
    torq_int = ((recv_msg[4] & 0xF) << 8) | recv_msg[5];
    motor_temp = recv_msg[6];
    error_code = recv_msg[7];

    pos_float = uint_to_float(pos_int, MIN_POS, MAX_POS, 16);
    spd_float = uint_to_float(spd_int, MIN_RPM, MAX_RPM, 12);
    torq_float = uint_to_float(torq_int, MIN_TORQUE, MAX_TORQUE, 12);
    motor_temp_float = (float)motor_temp;
    
}

HAL_StatusTypeDef CUBEMARS_set_motion_ctrl_parameters(CAN_HandleTypeDef *phcan, uint32_t* pTxMailBox,
                float pos, float rpm, float kp, float kd, float torque)
{
    //Check if all parameters are legal
    if(
        pos > MAX_POS || pos < MIN_POS ||
        rpm > MAX_RPM || rpm < MIN_RPM ||
        kp > MAX_KP || kp < MIN_KD_KP ||
        kd > MAX_KD || kd < MIN_KD_KP ||
        torque > MAX_TORQUE || torque < MIN_TORQUE
    ){
        return HAL_ERROR;
    }

    uint8_t tx_buf[8];
    int32_t pos_int = float_to_int(pos, MIN_POS, MAX_POS, 16);
    int32_t rpm_int = float_to_int(rpm, MIN_RPM, MAX_RPM, 12);
    int32_t kp_int = float_to_int(kp, MIN_KD_KP, MAX_KP, 12);
    int32_t kd_int = float_to_int(kd, MIN_KD_KP, MAX_KD, 12);
    int32_t torque_int = float_to_int(torque, MIN_TORQUE, MAX_TORQUE, 12);

    tx_buf[0] = (pos_int >> 8) & 0xFF;
    tx_buf[1] = pos_int & 0xFF;
    tx_buf[2] = (rpm_int >> 8) & 0xFF;
    tx_buf[3] = ((rpm_int & 0xF) << 4) | ((kp_int >> 8) & 0xF); 
    tx_buf[4] = (kp_int & 0xFF);
    tx_buf[5] = (kd_int >> 4) & 0xFF;
    tx_buf[6] = ((kd_int & 0xF) << 4) | (torque_int >> 8);
    tx_buf[7] = torque_int & 0xFF;
    
    return CAN_transmit_stdid(phcan, pTxMailBox, TEST_MOTOR_ID, tx_buf, sizeof(tx_buf));
}
