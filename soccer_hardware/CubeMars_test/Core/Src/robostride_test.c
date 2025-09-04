#include "robostride_test.h"

CAN_TxHeaderTypeDef rs_can_tx_header= {
    .StdId = 0x0,
    .ExtId = 0xff,
    .IDE   = CAN_ID_EXT,
    .RTR   = CAN_RTR_DATA,
    .DLC   = 8
};

#define macro()