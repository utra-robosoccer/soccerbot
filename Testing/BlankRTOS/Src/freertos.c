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

/* USER CODE BEGIN Variables */
volatile RobotGoal robotGoal, *robotGoalPtr;
volatile unsigned char robotGoalData[sizeof(RobotGoal)];
volatile unsigned char *robotGoalDataPtr;

volatile unsigned char buf[23];

volatile unsigned startSeqCount;
volatile unsigned totalBytesRead;

volatile RobotState robotState, *robotStatePtr;

#define __DYNAMIXEL_TRANSMIT(port, pinNum) HAL_GPIO_WritePin(port, pinNum, 1) // Set data direction pin high (TX)
#define __DYNAMIXEL_RECEIVE(port, pinNum) HAL_GPIO_WritePin(port, pinNum, 0) // Set data direction pin low (RX)

volatile uint8_t flagRx, flagTx;
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);

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

	uint8_t buff[92];
  /* Infinite loop */
  for(;;)
  {
	    flagRx = 0;
		HAL_UART_Receive_DMA(&huart5, buff, sizeof(buff));
		while(!flagRx);
		flagTx = 0;
		HAL_UART_Transmit_DMA(&huart5, buff, sizeof(buff));
		while(!flagTx);
//		flag = 0;
//		HAL_UART_Receive_IT(&huart5, (unsigned char *) buf, sizeof(buf));
//		while(!flag);
//
//		robotState.id = robotGoal.id;
//		for (int i = 0; i < 80; ++i) {
//			robotState.msg[i] = robotGoal.msg[i];
//		}
//		HAL_UART_Transmit(&huart5, (uint8_t *)robotStatePtr, sizeof(RobotState), 100);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Application */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart) {
	if (huart == &huart5) {
		flagRx = 1;
	}
//	if (huart == &huart5) {
////		HAL_UART_Abort_IT(&huart5);
//		for (int i = 0; i < sizeof(buf); i++) {
//			if (startSeqCount == 4) {
//				*robotGoalDataPtr = buf[i];
//				robotGoalDataPtr++;
//				totalBytesRead++;
//
//				if (totalBytesRead == sizeof(RobotGoal)) {
//
//					// Process RobotGoal here
//					memcpy(&robotGoal, robotGoalData, sizeof(RobotGoal));
//
//					robotGoalDataPtr = robotGoalData;
//					startSeqCount = 0;
//					totalBytesRead = 0;
//					flag = 1;
//					continue;
//				}
//			} else {
//				if (buf[i] == 255)
//					startSeqCount++;
//				else
//					startSeqCount = 0;
//			}
//		}
//
//		HAL_UART_Receive_IT(&huart5, (unsigned char *) buf, sizeof(buf));
//	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef * huart){
	if (huart == &huart5) {
		flagTx = 1;
	}
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
