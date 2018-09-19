/**
  *****************************************************************************
  * @file    tx_helpers.c
  * @author  Tyler
  *
  * @defgroup TX_Helpers TX Helpers
  *****************************************************************************
  */




/********************************* Includes **********************************/
#include <stdbool.h>
#include "tx_helpers.h"
#include "helpers.h"
#include "lfsr.h"
#include "usart.h"
#include "cmsis_os.h"




/******************************** Constants **********************************/
static const uint8_t NOTIFIED_FROM_TX_ISR = 0x80;




/****************************** Public Variables *****************************/
extern osThreadId TXHandle;




/***************************** Private Variables *****************************/
static volatile uint8_t buf[8] = {0xFF, 0xFF, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00};




/******************************** Functions **********************************/
void update_buffer_contents(Data_t* data){
    static uint32_t lfsr = 0x2F;
    static uint32_t polynomial = POLY_MASK_PERIOD_63;

    lfsr_get_noise(&lfsr, polynomial);
    int8_t noise = (lfsr >> 2) - 16;

    buf[2] = data->id;
    buf[5] = (data->pos & 0xFF) + noise; // low byte + noise
    buf[6] = (data->pos >> 8) & 0xFF; // high byte
    buf[7] = Dynamixel_ComputeChecksum((uint8_t*)buf, sizeof(buf));
}

void transmit_buffer_contents(void){
    bool status;
    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)buf, 8);
    status = waitUntilNotifiedOrTimeout(NOTIFIED_FROM_TX_ISR, 2);
    if(!status){
        HAL_UART_AbortTransmit(&huart1);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if(huart == &huart1){
        xTaskNotifyFromISR(
            TXHandle,
            NOTIFIED_FROM_TX_ISR,
            eSetBits,
            &xHigherPriorityTaskWoken
        );
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
