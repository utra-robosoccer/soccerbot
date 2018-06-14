/******************** Define to prevent recursive inclusion *******************/
#ifndef __UART_HANDLER_H
#define __UART_HANDLER_H


/********************************** Includes **********************************/
#include "stm32h7xx_hal.h"
#include "../Drivers/Motors/AX12A/AX12A.h"


/* USER CODE BEGIN Private defines */
#define cmdREAD			0
#define cmdWRITE		1

typedef struct {
	uint8_t 					type;
	Dynamixel_HandleTypeDef*	motorHandle;
	uint16_t 					position;
	uint16_t 					velocity;
	QueueHandle_t				qHandle;
}UARTcmd;

typedef struct {
	Dynamixel_HandleTypeDef*	motorHandle;
	uint16_t 					position;
	uint16_t 					velocity;
}UARTrx;
/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#endif
