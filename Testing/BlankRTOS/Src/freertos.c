/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include <robotGoal.h>
#include <robotState.h>
#include <stdint.h>
#include "serial.h"
#include "string.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;
osThreadId rxHandle;
uint32_t rxBuffer[ 128 ];
osStaticThreadDef_t rxControlBlock;
osThreadId txHandle;
uint32_t txBuffer[ 128 ];
osStaticThreadDef_t txControlBlock;

/* USER CODE BEGIN Variables */
volatile RobotGoal robotGoal, *robotGoalPtr;
volatile unsigned char robotGoalData[sizeof(RobotGoal)];
volatile unsigned char *robotGoalDataPtr;

volatile unsigned startSeqCount;
volatile unsigned totalBytesRead;

volatile RobotState robotState, *robotStatePtr;

#define __DYNAMIXEL_TRANSMIT(port, pinNum) HAL_GPIO_WritePin(port, pinNum, 1) // Set data direction pin high (TX)
#define __DYNAMIXEL_RECEIVE(port, pinNum) HAL_GPIO_WritePin(port, pinNum, 0) // Set data direction pin low (RX)

volatile uint8_t buffRx[92];
volatile uint8_t buffTx[92];
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void StartRx(void const * argument);
void StartTx(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* Hook prototypes */

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}                   
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  	__DYNAMIXEL_RECEIVE(GPIOA, GPIO_PIN_4);

	// Receiving
	robotGoal.id = 0;
	robotGoalPtr = &robotGoal;
	robotGoalDataPtr = robotGoalData;
	startSeqCount = 0;
	totalBytesRead = 0;

	// Sending
	robotState.id = 0;
	robotStatePtr = &robotState;
	robotState.start_seq = UINT32_MAX;
	robotState.end_seq = 0;
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128, defaultTaskBuffer, &defaultTaskControlBlock);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of rx */
  osThreadStaticDef(rx, StartRx, osPriorityRealtime, 0, 128, rxBuffer, &rxControlBlock);
  rxHandle = osThreadCreate(osThread(rx), NULL);

  /* definition and creation of tx */
  osThreadStaticDef(tx, StartTx, osPriorityHigh, 0, 128, txBuffer, &txControlBlock);
  txHandle = osThreadCreate(osThread(tx), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */

	uint32_t notification;
  /* Infinite loop */
  for(;;)
  {
	  // This simulates waiting on new trajectories to be received
      do{
          xTaskNotifyWait(0, 0x40, &notification, portMAX_DELAY);
      }while((notification & 0x40) != 0x40);

      // This simulates acquiring new present positions
	  for(uint8_t i = 0; i < 80; i++){
		  robotState.msg[i] = robotGoal.msg[i];
	  }

	  // This simulates telling the TX task there is data to send
	  xTaskNotify(txHandle, 0x40, eSetBits);
  }
  /* USER CODE END StartDefaultTask */
}

/* StartRx function */
void StartRx(void const * argument)
{
  /* USER CODE BEGIN StartRx */
	uint32_t notification;
  /* Infinite loop */
  for(;;)
  {
		HAL_UART_Receive_DMA(&huart5, buffRx, sizeof(buffRx));

		do{
			xTaskNotifyWait(0, 0x80, &notification, portMAX_DELAY);
		}while(((notification & 0x80) != 0x80) &&
				((notification & 0x20) != 0x20)
			);

		if((notification & 0x20) == 0x20){
			xTaskNotify(defaultTaskHandle, 0x40, eSetBits);
		}
  }
  /* USER CODE END StartRx */
}

/* StartTx function */
void StartTx(void const * argument)
{
  /* USER CODE BEGIN StartTx */

	uint32_t notification;
  /* Infinite loop */
  for(;;)
  {
	do{
		xTaskNotifyWait(0, 0x40, &notification, portMAX_DELAY);
	}while((notification & 0x40) != 0x40);

	HAL_UART_Transmit_DMA(&huart5, (uint8_t*)&robotState, sizeof(RobotState));

	do{
		xTaskNotifyWait(0, 0x80, &notification, portMAX_DELAY);
	}while((notification & 0x80) != 0x80);
  }
  /* USER CODE END StartTx */
}

/* USER CODE BEGIN Application */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	uint32_t notificationValue = 0x80;
	if (huart == &huart5) {
		for (uint8_t i = 0; i < sizeof(buffRx); i++) {
			if (startSeqCount == 4) {
				*robotGoalDataPtr = buffRx[i];
				robotGoalDataPtr++;
				totalBytesRead++;

				if (totalBytesRead == sizeof(RobotGoal)) {

					// Process RobotGoal here
					memcpy(&robotGoal, robotGoalData, sizeof(RobotGoal));

					robotGoalDataPtr = robotGoalData;
					startSeqCount = 0;
					totalBytesRead = 0;

					notificationValue = 0x20;
					continue;
				}
			} else {
				if (buffRx[i] == 255)
					startSeqCount++;
				else
					startSeqCount = 0;
			}
		}
		xTaskNotifyFromISR(rxHandle, notificationValue, eSetBits, &xHigherPriorityTaskWoken);

		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef * huart){
	if (huart == &huart5) {
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xTaskNotifyFromISR(txHandle, 0x80, eSetBits, &xHigherPriorityTaskWoken);

		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
