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
#include "UART_Handler.h"
#include "../Drivers/Dynamixel/DynamixelProtocolV1.h"
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

enum motorNames {MOTOR1, MOTOR2, MOTOR3, MOTOR4, MOTOR5,
				 MOTOR6, MOTOR7, MOTOR8, MOTOR9, MOTOR10,
				 MOTOR11, MOTOR12, MOTOR13, MOTOR14, MOTOR15,
				 MOTOR16, MOTOR17, MOTOR18
};

Dynamixel_HandleTypeDef Motor1,Motor2,Motor3,Motor4,Motor5,Motor6,Motor7,Motor8,Motor9,Motor10,Motor11,Motor12,Motor13,Motor14,Motor15,Motor16,Motor17,Motor18;
const double PI = 3.141592654;
volatile UARTcmd Motorcmd[18];
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
	Dynamixel_Init(&Motor1, 1, &huart6, GPIOC, GPIO_PIN_8, AX12ATYPE);
	Dynamixel_Init(&Motor2, 2, &huart6, GPIOC, GPIO_PIN_8, AX12ATYPE);
	Dynamixel_Init(&Motor3, 3, &huart6, GPIOC, GPIO_PIN_8, AX12ATYPE);
	Dynamixel_Init(&Motor4, 4, &huart1, GPIOA, GPIO_PIN_8, AX12ATYPE);
	Dynamixel_Init(&Motor5, 5, &huart1, GPIOA, GPIO_PIN_8, AX12ATYPE);
	Dynamixel_Init(&Motor6, 6, &huart1, GPIOA, GPIO_PIN_8, AX12ATYPE);
	Dynamixel_Init(&Motor7, 7, &huart4, GPIOC, GPIO_PIN_3, AX12ATYPE);
	Dynamixel_Init(&Motor8, 8, &huart4, GPIOC, GPIO_PIN_3, AX12ATYPE);
	Dynamixel_Init(&Motor9, 9, &huart4, GPIOC, GPIO_PIN_3, AX12ATYPE);
	Dynamixel_Init(&Motor10, 10, &huart2, GPIOA, GPIO_PIN_4, AX12ATYPE);
	Dynamixel_Init(&Motor11, 11, &huart2, GPIOA, GPIO_PIN_4, AX12ATYPE);
	Dynamixel_Init(&Motor12, 12, &huart2, GPIOA, GPIO_PIN_4, AX12ATYPE);
	Dynamixel_Init(&Motor13, 13, &huart3, GPIOB, GPIO_PIN_2, AX12ATYPE);
	Dynamixel_Init(&Motor14, 14, &huart3, GPIOB, GPIO_PIN_2, AX12ATYPE);
	Dynamixel_Init(&Motor15, 15, &huart3, GPIOB, GPIO_PIN_2, AX12ATYPE);
	Dynamixel_Init(&Motor16, 16, &huart3, GPIOB, GPIO_PIN_2, AX12ATYPE);
	Dynamixel_Init(&Motor17, 17, &huart3, GPIOB, GPIO_PIN_2, AX12ATYPE);
	Dynamixel_Init(&Motor18, 18, &huart3, GPIOB, GPIO_PIN_2, AX12ATYPE);


	Dynamixel_HandleTypeDef* arrDynamixel[18] = {&Motor1,&Motor2,&Motor3,&Motor4,
			&Motor5,&Motor6,&Motor7,&Motor8,&Motor9,&Motor10,&Motor11,&Motor12,
			&Motor13,&Motor14,&Motor15,&Motor16,&Motor17,&Motor18};


	for(uint8_t i = MOTOR1; i < MOTOR18; i++) {
        // Configure motor to return status packets only for read commands
        Dynamixel_SetStatusReturnLevel(arrDynamixel[i], 1);
        osDelay(10);

        // Configure motor to return status packets with minimal latency
        Dynamixel_SetReturnDelayTime(arrDynamixel[i], 2);
        osDelay(10);

        // Enable motor torque
        Dynamixel_TorqueEnable(arrDynamixel[i], 1);
        osDelay(10);

        // Set compliance slope such that it has torque setting 7 near the goal position
		AX12A_SetComplianceSlope(arrDynamixel[i], 7);
		osDelay(10);

		(Motorcmd[i]).motorHandle = arrDynamixel[i];
		(Motorcmd[i]).type = cmdWRITE;
		(Motorcmd[i]).velocity = 10;
	}

//	(Motorcmd[0]).qHandle = UART6_reqHandle;
//	(Motorcmd[1]).qHandle = UART6_reqHandle;
//	(Motorcmd[2]).qHandle = UART6_reqHandle;
//	(Motorcmd[3]).qHandle = UART1_reqHandle;
//	(Motorcmd[4]).qHandle = UART1_reqHandle;
//	(Motorcmd[5]).qHandle = UART1_reqHandle;
//	(Motorcmd[6]).qHandle = UART4_reqHandle;
//	(Motorcmd[7]).qHandle = UART4_reqHandle;
//	(Motorcmd[8]).qHandle = UART4_reqHandle;
//	(Motorcmd[9]).qHandle = UART2_reqHandle;
//	(Motorcmd[10]).qHandle = UART2_reqHandle;
//	(Motorcmd[11]).qHandle = UART2_reqHandle;
//	(Motorcmd[12]).qHandle = UART3_reqHandle;
//	(Motorcmd[13]).qHandle = UART3_reqHandle;
//	(Motorcmd[14]).qHandle = UART3_reqHandle;
//	(Motorcmd[15]).qHandle = UART3_reqHandle;
//	(Motorcmd[16]).qHandle = UART3_reqHandle;
//	(Motorcmd[17]).qHandle = UART3_reqHandle;

  uint32_t notification = 0;
  /* Infinite loop */
  float positions[12];
  for(;;)
  {
	  // This simulates waiting on new trajectories to be received
      do{
          xTaskNotifyWait(0, 0x40, &notification, portMAX_DELAY);
      }while((notification & 0x40) != 0x40);

      // acquiring new present positions
		for(uint8_t i = 0; i < 12; i++){
			uint8_t* ptr = &positions[i];
			for(uint8_t j = 0; j < 4; j++){
				*ptr = robotGoal.msg[i * 4 + j];
				ptr++;
			}
		}

		for(uint8_t i = MOTOR1; i <= MOTOR12; i++){ // NB: i begins at 0 (i.e. Motor1 corresponds to i = 0)
			switch(i){
			    case MOTOR1: (Motorcmd[i]).position = -1*positions[i]*180/PI + 150 - 1;
			  				 break;
			    case MOTOR2: (Motorcmd[i]).position = -1*positions[i]*180/PI + 150 + 3;
			  				 break;
			    case MOTOR3: (Motorcmd[i]).position = -1*positions[i]*180/PI + 150 + 1;
			  				 break;
			    case MOTOR4: (Motorcmd[i]).position = positions[i]*180/PI + 150 + 2;
			  				 break;
			    case MOTOR5: (Motorcmd[i]).position = positions[i]*180/PI + 150 - 0;
			  				 break;
			    case MOTOR6: (Motorcmd[i]).position = -1*positions[i]*180/PI + 150 + 0;
			  				 break;
			    case MOTOR7: (Motorcmd[i]).position = positions[i]*180/PI + 150 + 0;
			  				 break;
			    case MOTOR8: (Motorcmd[i]).position = positions[i]*180/PI + 150 - 3;
			  				 break;
			    case MOTOR9: (Motorcmd[i]).position = -1*positions[i]*180/PI + 150 - 0;
			  				 break;
			    case MOTOR10: (Motorcmd[i]).position = positions[i]*180/PI + 150 + 4;
			  				 break;
			    case MOTOR11: (Motorcmd[i]).position = positions[i]*180/PI + 150 + 1;
			  				 break;
			    case MOTOR12: (Motorcmd[i]).position = -1*positions[i]*180/PI + 150 + 3;
							 break;
            }
        }
		Dynamixel_GetPosition(&Motor9);
		for(uint8_t i = MOTOR1; i < MOTOR12; i++){
			strcpy(&robotState.msg[i * 4], &(arrDynamixel[i]->_lastPosition));
		}
		Dynamixel_SetGoalPosition(&Motor9, Motorcmd[MOTOR9].position);
		do{
			xTaskNotifyWait(0, 0x80, &notification, portMAX_DELAY);
		}while((notification & 0x80) != 0x80);

	    memcpy(robotState.msg, robotGoal.msg, sizeof(robotGoal.msg));

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

	HAL_UART_Receive_DMA(&huart5, buffRx, sizeof(buffRx));
  /* Infinite loop */
  for(;;)
  {
		do{
			xTaskNotifyWait(0, 0x80, &notification, portMAX_DELAY);
		}while((notification & 0x80) != 0x80);

		HAL_UART_Receive_DMA(&huart5, buffRx, sizeof(buffRx));

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

					xTaskNotify(defaultTaskHandle, 0x40, eSetBits);
					continue;
				}
			} else {
				if (buffRx[i] == 255)
					startSeqCount++;
				else
					startSeqCount = 0;
			}
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
	if (huart == &huart5) {
		xTaskNotifyFromISR(rxHandle, 0x80, eSetBits, &xHigherPriorityTaskWoken);
	}
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef * huart){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if (huart == &huart5) {
		xTaskNotifyFromISR(txHandle, 0x80, eSetBits, &xHigherPriorityTaskWoken);
	}
	if(huart == &huart4){
//		xTaskNotifyFromISR(defaultTaskHandle, 0x80, eSetBits, &xHigherPriorityTaskWoken);
	}
//	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
