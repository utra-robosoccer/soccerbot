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
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "gpio.h"
#include "i2c.h"
#include "../Drivers/MPU6050/MPU6050.h"
#include "sharedMacros.h"
#include "UART_Handler.h"
#include "../Drivers/Dynamixel/DynamixelProtocolV1.h"
#include "../Drivers/Communication/Communication.h"

/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[ 512 ];
osStaticThreadDef_t defaultTaskControlBlock;
osThreadId UART1_Handle;
uint32_t UART1_Buffer[ 128 ];
osStaticThreadDef_t UART1_ControlBlock;
osThreadId UART2_Handle;
uint32_t UART2_Buffer[ 128 ];
osStaticThreadDef_t UART2_ControlBlock;
osThreadId UART3_Handle;
uint32_t UART3_Buffer[ 128 ];
osStaticThreadDef_t UART3_ControlBlock;
osThreadId UART4_Handle;
uint32_t UART4_Buffer[ 128 ];
osStaticThreadDef_t UART4_ControlBlock;
osThreadId UART6_Handle;
uint32_t UART6_Buffer[ 128 ];
osStaticThreadDef_t UART6_ControlBlock;
osThreadId IMUTaskHandle;
uint32_t IMUTaskBuffer[ 128 ];
osStaticThreadDef_t IMUTaskControlBlock;
osThreadId rxTaskHandle;
uint32_t rxTaskBuffer[ 512 ];
osStaticThreadDef_t rxTaskControlBlock;
osThreadId txTaskHandle;
uint32_t txTaskBuffer[ 512 ];
osStaticThreadDef_t txTaskControlBlock;
osMessageQId UART1_reqHandle;
uint8_t UART1_reqBuffer[ 16 * sizeof( UARTcmd ) ];
osStaticMessageQDef_t UART1_reqControlBlock;
osMessageQId UART2_reqHandle;
uint8_t UART2_reqBuffer[ 16 * sizeof( UARTcmd ) ];
osStaticMessageQDef_t UART2_reqControlBlock;
osMessageQId UART3_reqHandle;
uint8_t UART3_reqBuffer[ 16 * sizeof( UARTcmd ) ];
osStaticMessageQDef_t UART3_reqControlBlock;
osMessageQId UART4_reqHandle;
uint8_t UART4_reqBuffer[ 16 * sizeof( UARTcmd ) ];
osStaticMessageQDef_t UART4_reqControlBlock;
osMessageQId UART6_reqHandle;
uint8_t UART6_reqBuffer[ 16 * sizeof( UARTcmd ) ];
osStaticMessageQDef_t UART6_reqControlBlock;
osMessageQId UART_rxHandle;
uint8_t UART_rxBuffer[ 32 * sizeof( UARTcmd ) ];
osStaticMessageQDef_t UART_rxControlBlock;
osMutexId PCUARTHandle;
osStaticMutexDef_t PCUARTControlBlock;

/* USER CODE BEGIN Variables */
const double PI = 3.141592654;

enum motorNames {MOTOR1, MOTOR2, MOTOR3, MOTOR4, MOTOR5,
				 MOTOR6, MOTOR7, MOTOR8, MOTOR9, MOTOR10,
				 MOTOR11, MOTOR12, MOTOR13, MOTOR14, MOTOR15,
				 MOTOR16, MOTOR17, MOTOR18
};

Dynamixel_HandleTypeDef Motor1, Motor2, Motor3 ,Motor4, Motor5,
						Motor6, Motor7, Motor8, Motor9, Motor10,
						Motor11, Motor12, Motor13, Motor14, Motor15,
						Motor16, Motor17, Motor18;

MPU6050_HandleTypeDef IMUdata;

bool setupIsDone = false;
static volatile uint32_t error;
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void UART1_Handler(void const * argument);
void UART2_Handler(void const * argument);
void UART3_Handler(void const * argument);
void UART4_Handler(void const * argument);
void UART6_Handler(void const * argument);
void StartIMUTask(void const * argument);
void StartRxTask(void const * argument);
void StartTxTask(void const * argument);

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

  /* Create the mutex(es) */
  /* definition and creation of PCUART */
  osMutexStaticDef(PCUART, &PCUARTControlBlock);
  PCUARTHandle = osMutexCreate(osMutex(PCUART));

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
  osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 512, defaultTaskBuffer, &defaultTaskControlBlock);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of UART1_ */
  osThreadStaticDef(UART1_, UART1_Handler, osPriorityNormal, 0, 128, UART1_Buffer, &UART1_ControlBlock);
  UART1_Handle = osThreadCreate(osThread(UART1_), NULL);

  /* definition and creation of UART2_ */
  osThreadStaticDef(UART2_, UART2_Handler, osPriorityNormal, 0, 128, UART2_Buffer, &UART2_ControlBlock);
  UART2_Handle = osThreadCreate(osThread(UART2_), NULL);

  /* definition and creation of UART3_ */
  osThreadStaticDef(UART3_, UART3_Handler, osPriorityNormal, 0, 128, UART3_Buffer, &UART3_ControlBlock);
  UART3_Handle = osThreadCreate(osThread(UART3_), NULL);

  /* definition and creation of UART4_ */
  osThreadStaticDef(UART4_, UART4_Handler, osPriorityNormal, 0, 128, UART4_Buffer, &UART4_ControlBlock);
  UART4_Handle = osThreadCreate(osThread(UART4_), NULL);

  /* definition and creation of UART6_ */
  osThreadStaticDef(UART6_, UART6_Handler, osPriorityNormal, 0, 128, UART6_Buffer, &UART6_ControlBlock);
  UART6_Handle = osThreadCreate(osThread(UART6_), NULL);

  /* definition and creation of IMUTask */
  osThreadStaticDef(IMUTask, StartIMUTask, osPriorityNormal, 0, 128, IMUTaskBuffer, &IMUTaskControlBlock);
  IMUTaskHandle = osThreadCreate(osThread(IMUTask), NULL);

  /* definition and creation of rxTask */
  osThreadStaticDef(rxTask, StartRxTask, osPriorityRealtime, 0, 512, rxTaskBuffer, &rxTaskControlBlock);
  rxTaskHandle = osThreadCreate(osThread(rxTask), NULL);

  /* definition and creation of txTask */
  osThreadStaticDef(txTask, StartTxTask, osPriorityHigh, 0, 512, txTaskBuffer, &txTaskControlBlock);
  txTaskHandle = osThreadCreate(osThread(txTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of UART1_req */
  osMessageQStaticDef(UART1_req, 16, UARTcmd, UART1_reqBuffer, &UART1_reqControlBlock);
  UART1_reqHandle = osMessageCreate(osMessageQ(UART1_req), NULL);

  /* definition and creation of UART2_req */
  osMessageQStaticDef(UART2_req, 16, UARTcmd, UART2_reqBuffer, &UART2_reqControlBlock);
  UART2_reqHandle = osMessageCreate(osMessageQ(UART2_req), NULL);

  /* definition and creation of UART3_req */
  osMessageQStaticDef(UART3_req, 16, UARTcmd, UART3_reqBuffer, &UART3_reqControlBlock);
  UART3_reqHandle = osMessageCreate(osMessageQ(UART3_req), NULL);

  /* definition and creation of UART4_req */
  osMessageQStaticDef(UART4_req, 16, UARTcmd, UART4_reqBuffer, &UART4_reqControlBlock);
  UART4_reqHandle = osMessageCreate(osMessageQ(UART4_req), NULL);

  /* definition and creation of UART6_req */
  osMessageQStaticDef(UART6_req, 16, UARTcmd, UART6_reqBuffer, &UART6_reqControlBlock);
  UART6_reqHandle = osMessageCreate(osMessageQ(UART6_req), NULL);

  /* definition and creation of UART_rx */
  osMessageQStaticDef(UART_rx, 32, UARTcmd, UART_rxBuffer, &UART_rxControlBlock);
  UART_rxHandle = osMessageCreate(osMessageQ(UART_rx), NULL);

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
//	Dynamixel_Init(&Motor7, 7, &huart4, GPIOC, GPIO_PIN_3, AX12ATYPE);
	Dynamixel_Init(&Motor7, 7, &huart4, GPIOC, GPIO_PIN_3, MX28TYPE);
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

	UARTcmd Motorcmd[18];
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

        // Set compliance slope such that it has torque setting 7
        // (affects torque near the goal position)
		AX12A_SetComplianceSlope(arrDynamixel[i], 7);
		osDelay(10);

		(Motorcmd[i]).motorHandle = arrDynamixel[i];
		(Motorcmd[i]).type = cmdWRITE;
		(Motorcmd[i]).velocity = 10;
	}

	(Motorcmd[MOTOR1]).qHandle = UART6_reqHandle;
	(Motorcmd[MOTOR2]).qHandle = UART6_reqHandle;
	(Motorcmd[MOTOR3]).qHandle = UART6_reqHandle;
	(Motorcmd[MOTOR4]).qHandle = UART1_reqHandle;
	(Motorcmd[MOTOR5]).qHandle = UART1_reqHandle;
	(Motorcmd[MOTOR6]).qHandle = UART1_reqHandle;
	(Motorcmd[MOTOR7]).qHandle = UART4_reqHandle;
	(Motorcmd[MOTOR8]).qHandle = UART4_reqHandle;
	(Motorcmd[MOTOR9]).qHandle = UART4_reqHandle;
	(Motorcmd[MOTOR10]).qHandle = UART2_reqHandle;
	(Motorcmd[MOTOR11]).qHandle = UART2_reqHandle;
	(Motorcmd[MOTOR12]).qHandle = UART2_reqHandle;
	(Motorcmd[MOTOR13]).qHandle = UART3_reqHandle;
	(Motorcmd[MOTOR14]).qHandle = UART3_reqHandle;
	(Motorcmd[MOTOR15]).qHandle = UART3_reqHandle;
	(Motorcmd[MOTOR16]).qHandle = UART3_reqHandle;
	(Motorcmd[MOTOR17]).qHandle = UART3_reqHandle;
	(Motorcmd[MOTOR18]).qHandle = UART3_reqHandle;

	// IMU initialization
	IMUdata._I2C_Handle = &hi2c1;
	MPU6050_init(&IMUdata);
	MPU6050_manually_set_offsets(&IMUdata);
	MPU6050_set_LPF(&IMUdata, 4);

	// Set setupIsDone and unblock the higher-priority tasks
	setupIsDone = true;
	xTaskNotify(rxTaskHandle, 1UL, eNoAction);
	xTaskNotify(txTaskHandle, 1UL, eNoAction);
	xTaskNotify(UART1_Handle, 1UL, eNoAction);
	xTaskNotify(UART2_Handle, 1UL, eNoAction);
	xTaskNotify(UART3_Handle, 1UL, eNoAction);
	xTaskNotify(UART4_Handle, 1UL, eNoAction);
	xTaskNotify(UART6_Handle, 1UL, eNoAction);
	xTaskNotify(IMUTaskHandle, 1UL, eNoAction);

	/* Infinite loop */
	uint8_t i;
	float positions[18];
	while(1){
		// Wait until notified by the PC RX task (i.e. until a new RobotGoal
		// is received)
		xTaskNotifyWait(0, UINT32_MAX, NULL, portMAX_DELAY);

		// Convert raw bytes from robotGoal received from PC into floats
		for(uint8_t i = 0; i < 12; i++){
			uint8_t* ptr = (uint8_t*)&positions[i];
			for(uint8_t j = 0; j < 4; j++){
				*ptr = robotGoal.msg[i * 4 + j];
				ptr++;
			}
		}

		// Send each goal position to the queue, where the UART handler
		// thread that's listening will receive it and send it to the motor
		for(i = MOTOR1; i <= MOTOR12; i++){ // NB: i begins at 0 (i.e. Motor1 corresponds to i = 0)
			switch(i){
			    case MOTOR1: (Motorcmd[i]).position = -1*positions[i]*180/PI + 150 - 1;
			  				 break;
			    case MOTOR2: (Motorcmd[i]).position = -1*positions[i]*180/PI + 150 + 3 + 55;
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
			    default:
			    	break;
            }
			Motorcmd[i].type = cmdWRITE;
			xQueueSend(Motorcmd[i].qHandle, &Motorcmd[i], 0);
			Motorcmd[i].type = cmdREAD;
			xQueueSend(Motorcmd[i].qHandle, &Motorcmd[i], 0);
        }
		robotState.id = robotGoal.id;
    }
  /* USER CODE END StartDefaultTask */
}

/* UART1_Handler function */
void UART1_Handler(void const * argument)
{
  /* USER CODE BEGIN UART1_Handler */
  // Here, we use task notifications to block this task from running until a notification
  // is received. This allows one-time setup to complete in a low-priority task.
  xTaskNotifyWait(UINT32_MAX, UINT32_MAX, NULL, portMAX_DELAY);

  /* Infinite loop */
  UARTcmd cmdMessage;
  TXData_t dataToSend;
  dataToSend.eDataType = eMotorData;
  uint32_t notification;

  for(;;)
	{
	  while(xQueueReceive(UART1_reqHandle, &cmdMessage, portMAX_DELAY) != pdTRUE);
	  if(cmdMessage.type == cmdREAD) {
		  // Simulate acquiring sensor data
		  cmdMessage.motorHandle->_lastPosition = cmdMessage.position;
		  dataToSend.pData = cmdMessage.motorHandle;

		  // TODO: get reading to work
//		  Dynamixel_GetPosition(cmdMessage.motorHandle);
//
//		  do{
//			  xTaskNotifyWait(0, NOTIFIED_FROM_ISR, &notification, MAX_DELAY_TIME);
//		  }while((notification & NOTIFIED_FROM_ISR) != NOTIFIED_FROM_ISR);
//
//		  dataToSend.pData = cmdMessage.motorHandle;

		  xQueueSend(UART_rxHandle, &dataToSend, 0);
	  }
	  else if(cmdMessage.type == cmdWRITE){
		  Dynamixel_SetGoalPosition(cmdMessage.motorHandle, cmdMessage.position);

		  do{
			  xTaskNotifyWait(0, NOTIFIED_FROM_ISR, &notification, MAX_DELAY_TIME);
		  }while((notification & NOTIFIED_FROM_ISR) != NOTIFIED_FROM_ISR);
	  }
  }
  /* USER CODE END UART1_Handler */
}

/* UART2_Handler function */
void UART2_Handler(void const * argument)
{
  /* USER CODE BEGIN UART2_Handler */
  // Here, we use task notifications to block this task from running until a notification
  // is received. This allows one-time setup to complete in a low-priority task.
  xTaskNotifyWait(UINT32_MAX, UINT32_MAX, NULL, portMAX_DELAY);

  /* Infinite loop */
  UARTcmd cmdMessage;
  TXData_t dataToSend;
  dataToSend.eDataType = eMotorData;
  uint32_t notification;

  for(;;)
	{
	  while(xQueueReceive(UART2_reqHandle, &cmdMessage, portMAX_DELAY) != pdTRUE);
	  if(cmdMessage.type == cmdREAD) {
		  // Simulate acquiring sensor data
		  cmdMessage.motorHandle->_lastPosition = cmdMessage.position;
		  dataToSend.pData = cmdMessage.motorHandle;

		  // TODO: get reading to work
//		  Dynamixel_GetPosition(cmdMessage.motorHandle);
//
//		  do{
//			  xTaskNotifyWait(0, NOTIFIED_FROM_ISR, &notification, MAX_DELAY_TIME);
//		  }while((notification & NOTIFIED_FROM_ISR) != NOTIFIED_FROM_ISR);
//
//		  dataToSend.pData = cmdMessage.motorHandle;

		  xQueueSend(UART_rxHandle, &dataToSend, 0);
	  }
	  else if(cmdMessage.type == cmdWRITE) {
		  Dynamixel_SetGoalPosition(cmdMessage.motorHandle, cmdMessage.position);

		  do{
			  xTaskNotifyWait(0, NOTIFIED_FROM_ISR, &notification, MAX_DELAY_TIME);
		  }while((notification & NOTIFIED_FROM_ISR) != NOTIFIED_FROM_ISR);
	  }
  }
  /* USER CODE END UART2_Handler */
}

/* UART3_Handler function */
void UART3_Handler(void const * argument)
{
  /* USER CODE BEGIN UART3_Handler */
  // Here, we use task notifications to block this task from running until a notification
  // is received. This allows one-time setup to complete in a low-priority task.
  xTaskNotifyWait(UINT32_MAX, UINT32_MAX, NULL, portMAX_DELAY);

  /* Infinite loop */
  UARTcmd cmdMessage;
  TXData_t dataToSend;
  dataToSend.eDataType = eMotorData;
  uint32_t notification;

  for(;;)
	{
	  while(xQueueReceive(UART3_reqHandle, &cmdMessage, portMAX_DELAY) != pdTRUE);
	  if(cmdMessage.type == cmdREAD) {
		  // Simulate acquiring sensor data
		  cmdMessage.motorHandle->_lastPosition = cmdMessage.position;
		  dataToSend.pData = cmdMessage.motorHandle;

		  // TODO: get reading to work
//		  Dynamixel_GetPosition(cmdMessage.motorHandle);
//
//		  do{
//			  xTaskNotifyWait(0, NOTIFIED_FROM_ISR, &notification, MAX_DELAY_TIME);
//		  }while((notification & NOTIFIED_FROM_ISR) != NOTIFIED_FROM_ISR);
//
//		  dataToSend.pData = cmdMessage.motorHandle;

		  xQueueSend(UART_rxHandle, &dataToSend, 0);
	  }
	  else if(cmdMessage.type == cmdWRITE) {
		  Dynamixel_SetGoalPosition(cmdMessage.motorHandle, cmdMessage.position);

		  do{
			  xTaskNotifyWait(0, NOTIFIED_FROM_ISR, &notification, MAX_DELAY_TIME);
		  }while((notification & NOTIFIED_FROM_ISR) != NOTIFIED_FROM_ISR);
	  }
  }
  /* USER CODE END UART3_Handler */
}

/* UART4_Handler function */
void UART4_Handler(void const * argument)
{
  /* USER CODE BEGIN UART4_Handler */
  // Here, we use task notifications to block this task from running until a notification
  // is received. This allows one-time setup to complete in a low-priority task.
  xTaskNotifyWait(UINT32_MAX, UINT32_MAX, NULL, portMAX_DELAY);

  /* Infinite loop */
  UARTcmd cmdMessage;
  TXData_t dataToSend;
  dataToSend.eDataType = eMotorData;
  uint32_t notification;

  for(;;)
	{
	  while(xQueueReceive(UART4_reqHandle, &cmdMessage, portMAX_DELAY) != pdTRUE);
	  if(cmdMessage.type == cmdREAD) {
		  // Simulate acquiring sensor data
		  cmdMessage.motorHandle->_lastPosition = cmdMessage.position;
		  dataToSend.pData = cmdMessage.motorHandle;

		  // TODO: get reading to work
//		  Dynamixel_GetPosition(cmdMessage.motorHandle);
//
//		  do{
//			  xTaskNotifyWait(0, NOTIFIED_FROM_ISR, &notification, MAX_DELAY_TIME);
//		  }while((notification & NOTIFIED_FROM_ISR) != NOTIFIED_FROM_ISR);
//
//		  dataToSend.pData = cmdMessage.motorHandle;

		  xQueueSend(UART_rxHandle, &dataToSend, 0);
	  }
	  else if(cmdMessage.type == cmdWRITE) {
		  Dynamixel_SetGoalPosition(cmdMessage.motorHandle, cmdMessage.position);

		  do{
			  xTaskNotifyWait(0, NOTIFIED_FROM_ISR, &notification, MAX_DELAY_TIME);
		  }while((notification & NOTIFIED_FROM_ISR) != NOTIFIED_FROM_ISR);
	  }
  }
  /* USER CODE END UART4_Handler */
}

/* UART6_Handler function */
void UART6_Handler(void const * argument)
{
  /* USER CODE BEGIN UART6_Handler */
  // Here, we use task notifications to block this task from running until a notification
  // is received. This allows one-time setup to complete in a low-priority task.
  xTaskNotifyWait(UINT32_MAX, UINT32_MAX, NULL, portMAX_DELAY);

  /* Infinite loop */
  UARTcmd cmdMessage;
  TXData_t dataToSend;
  dataToSend.eDataType = eMotorData;
  uint32_t notification;

  for(;;)
	{
	  while(xQueueReceive(UART6_reqHandle, &cmdMessage, portMAX_DELAY) != pdTRUE);
	  if(cmdMessage.type == cmdREAD) {
		  // Simulate acquiring sensor data
		  cmdMessage.motorHandle->_lastPosition = cmdMessage.position;
		  dataToSend.pData = cmdMessage.motorHandle;

		  // TODO: get reading to work
//		  Dynamixel_GetPosition(cmdMessage.motorHandle);
//
//		  do{
//			  xTaskNotifyWait(0, NOTIFIED_FROM_ISR, &notification, MAX_DELAY_TIME);
//		  }while((notification & NOTIFIED_FROM_ISR) != NOTIFIED_FROM_ISR);
//
//		  dataToSend.pData = cmdMessage.motorHandle;

		  xQueueSend(UART_rxHandle, &dataToSend, 0);
	  }
	  else if(cmdMessage.type == cmdWRITE) {
		  Dynamixel_SetGoalPosition(cmdMessage.motorHandle, cmdMessage.position);

		  do{
			  xTaskNotifyWait(0, NOTIFIED_FROM_ISR, &notification, MAX_DELAY_TIME);
		  }while((notification & NOTIFIED_FROM_ISR) != NOTIFIED_FROM_ISR);
	  }
  }
  /* USER CODE END UART6_Handler */
}

/* StartIMUTask function */
void StartIMUTask(void const * argument)
{
  /* USER CODE BEGIN StartIMUTask */
  // Here, we use task notifications to block this task from running until a notification
  // is received. This allows one-time setup to complete in a low-priority task.
  xTaskNotifyWait(UINT32_MAX, UINT32_MAX, NULL, portMAX_DELAY);

  /* Infinite loop */
  TXData_t dataToSend;
  dataToSend.eDataType = eIMUData;

  uint32_t notification;

  for(;;)
  {
	  do{
	      xTaskNotifyWait(0, NOTIFIED_FROM_TASK, &notification, portMAX_DELAY);
	  }while((notification & NOTIFIED_FROM_TASK) != NOTIFIED_FROM_TASK);

      // Note that it takes < 1 ms total for the sensor to read both accel and gyro
	  MPU6050_Read_Accelerometer_Withoffset_IT(&IMUdata); // Also updates pitch and roll
	  MPU6050_Read_Gyroscope_Withoffset_IT(&IMUdata);

	  dataToSend.pData = &IMUdata;
	  xQueueSend(UART_rxHandle, &dataToSend, 0);
  }
  /* USER CODE END StartIMUTask */
}

/* StartRxTask function */
void StartRxTask(void const * argument)
{
  /* USER CODE BEGIN StartRxTask */
	uint8_t robotGoalData[sizeof(RobotGoal)];
	uint8_t *robotGoalDataPtr;
	uint8_t buffRx[92];
	uint8_t startSeqCount;
	uint8_t totalBytesRead;

	// Receiving
	robotGoal.id = 0;
	robotGoalDataPtr = robotGoalData;
	startSeqCount = 0;
	totalBytesRead = 0;

	// Sending
	robotState.id = 0;
	robotState.start_seq = UINT32_MAX;
	robotState.end_seq = 0;

	xTaskNotifyWait(UINT32_MAX, UINT32_MAX, NULL, portMAX_DELAY);

	HAL_StatusTypeDef status;

	uint32_t notification;

	HAL_UART_Receive_DMA(&huart5, (uint8_t*)buffRx, sizeof(buffRx));

	/* Infinite loop */
	for (;;) {
		// Wait until notified from ISR. Clear no bits on entry in case the notification
		// comes before this statement is executed (which is rather unlikely as long as
		// this task has the highest priority, but overall this is a better decision in
		// case priorities are changed in the future and someone forgets about this.
		do{
			xTaskNotifyWait(0, NOTIFIED_FROM_ISR, &notification, portMAX_DELAY);
		}while((notification & NOTIFIED_FROM_ISR) != NOTIFIED_FROM_ISR);

		do{
			// This do-while loop with the mutex inside of it makes calls to the UART module
			// responsible for PC communication atomic. This attempts to solve the following
			// scenario: the TX thread is in the middle of executing the call to HAL_UART_Transmit
			// when suddenly the RX thread is unblocked. The RX thread calls HAL_UART_Receive, and
			// returns immediately when it detects that the uart module is already locked. Then
			// the RX thread blocks itself and never wakes up since a RX transfer was never
			// initialized.
			xSemaphoreTake(PCUARTHandle, 1);
			status = HAL_UART_Receive_DMA(&huart5, (uint8_t*)buffRx, sizeof(buffRx));
			xSemaphoreGive(PCUARTHandle);
		}while(status != HAL_OK);

		for (uint8_t i = 0; i < sizeof(buffRx); i++) {
			if (startSeqCount == 4) {
				// This control block is entered when the header sequence of
				// 0xFFFFFFFF has been received; thus we know the data we
				// receive will be in tact

				*robotGoalDataPtr = buffRx[i];
				robotGoalDataPtr++;
				totalBytesRead++;

				if (totalBytesRead == sizeof(RobotGoal)) {
					// If, after the last couple of receive interrupts, we have
					// received sizeof(RobotGoal) bytes, then we copy the data
					// buffer into the robotGoal structure and wake the control
					// thread to distribute states to each actuator
					memcpy(&robotGoal, robotGoalData, sizeof(RobotGoal));

					// Reset the variables to help with reception of a RobotGoal
					robotGoalDataPtr = robotGoalData;
					startSeqCount = 0;
					totalBytesRead = 0;

					xTaskNotify(defaultTaskHandle, NOTIFIED_FROM_TASK, eSetBits); // Wake control task
					xTaskNotify(IMUTaskHandle, NOTIFIED_FROM_TASK, eSetBits); // Wake MPU task
					continue;
				}
			}else{
				// This control block is used to verify that the data header is in tact
				if (buffRx[i] == 0xFF) {
					startSeqCount++;
				} else {
					startSeqCount = 0;
				}
			}
		}
	}
  /* USER CODE END StartRxTask */
}

/* StartTxTask function */
void StartTxTask(void const * argument)
{
  /* USER CODE BEGIN StartTxTask */
  xTaskNotifyWait(UINT32_MAX, UINT32_MAX, NULL, portMAX_DELAY);

  TXData_t receivedData;
  Dynamixel_HandleTypeDef* motorPtr = NULL;
  MPU6050_HandleTypeDef* imuPtr = NULL;
  char* const pIMUXGyroData = &robotState.msg[ROBOT_STATE_MPU_DATA_OFFSET];

  HAL_StatusTypeDef status;
  uint32_t notification;
  uint32_t dataReadyFlags = 0; // Bits in this are set based on which sensor data is ready

  // TODO: In the future, this "12" should be replaced with NUM_MOTORS. We will
  // be ready for this once all 18 motors can be ready from.
  uint32_t NOTIFICATION_MASK = 0x80000000;
  for(uint8_t i = 1; i <= 9; i++){
	  NOTIFICATION_MASK |= (1 << i);
  }

  /* Infinite loop */
  for(;;)
  {
	  // TODO: Test this
	  while((dataReadyFlags & NOTIFICATION_MASK) != NOTIFICATION_MASK){
		  while(xQueueReceive(UART_rxHandle, &receivedData, portMAX_DELAY) != pdTRUE);

		  switch(receivedData.eDataType){
			  case eMotorData:
				  motorPtr = (Dynamixel_HandleTypeDef*)receivedData.pData;

				  if(motorPtr == NULL){ break; }

				  // Validate data and store it in robotState
				  if(motorPtr->_ID <= NUM_MOTORS){
					  // Copy sensor data for this motor into its section of robotState.msg
					  memcpy(&robotState.msg[4 * (motorPtr->_ID - 1)], &(motorPtr->_lastPosition), sizeof(float));

					  // Set flag indicating the motor with this id has reported in with position data
					  dataReadyFlags |= (1 << motorPtr->_ID);
				  }
				  break;
			  case eIMUData:
				  imuPtr = (MPU6050_HandleTypeDef*)receivedData.pData;

				  if(imuPtr == NULL){ break; }

				  // Copy sensor data into the IMU data section of robotState.msg
				  memcpy(pIMUXGyroData, (&imuPtr->_X_GYRO), 6 * sizeof(float));

				  // Set flag indicating IMU data has reported in
				  dataReadyFlags |= 0x80000000;
				  break;
			  default:
				  break;
		  }
	  }
	  dataReadyFlags = 0; // Clear all flags

	  do{
	      // This do-while loop with the mutex inside of it makes calls to the UART module
	      // responsible for PC communication atomic. This attempts to solve the following
	      // scenario: the TX thread is in the middle of executing the call to HAL_UART_Transmit
	      // when suddenly the RX thread is unblocked. The RX thread calls HAL_UART_Receive, and
	      // returns immediately when it detects that the uart module is already locked. Then
	      // the RX thread blocks itself and never wakes up since a RX transfer was never
	      // initialized.
	      xSemaphoreTake(PCUARTHandle, 1);
	      status = HAL_UART_Transmit_DMA(&huart5, (uint8_t*)&robotState, sizeof(RobotState));
	      xSemaphoreGive(PCUARTHandle);
	  }while(status != HAL_OK);

	  // Wait until notified from ISR. Clear no bits on entry in case the notification
	  // came while a higher priority task was executing.
	  do{
		  xTaskNotifyWait(0, NOTIFIED_FROM_ISR, &notification, portMAX_DELAY);
	  }while((notification & NOTIFIED_FROM_ISR) != NOTIFIED_FROM_ISR);
  }
  /* USER CODE END StartTxTask */
}

/* USER CODE BEGIN Application */

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	// This callback runs after the interrupt data transfer from the sensor to the mcu is finished
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xTaskNotifyFromISR(IMUTaskHandle, NOTIFIED_FROM_ISR, eSetBits, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart){
	if(setupIsDone){
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		if(huart == &huart5){
			xTaskNotifyFromISR(txTaskHandle, NOTIFIED_FROM_ISR, eSetBits, &xHigherPriorityTaskWoken);
		}
		if(huart == &huart1){
			xTaskNotifyFromISR(UART1_Handle, NOTIFIED_FROM_ISR, eSetBits, &xHigherPriorityTaskWoken);
		}
		else if(huart == &huart2){
			xTaskNotifyFromISR(UART2_Handle, NOTIFIED_FROM_ISR, eSetBits, &xHigherPriorityTaskWoken);
		}
		else if(huart == &huart3){
			xTaskNotifyFromISR(UART3_Handle, NOTIFIED_FROM_ISR, eSetBits, &xHigherPriorityTaskWoken);
		}
		else if(huart == &huart4){
			xTaskNotifyFromISR(UART4_Handle, NOTIFIED_FROM_ISR, eSetBits, &xHigherPriorityTaskWoken);
		}
		else if(huart == &huart6){
			xTaskNotifyFromISR(UART6_Handle, NOTIFIED_FROM_ISR, eSetBits, &xHigherPriorityTaskWoken);
		}
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if (huart == &huart5) {
		xTaskNotifyFromISR(rxTaskHandle, NOTIFIED_FROM_ISR, eSetBits, &xHigherPriorityTaskWoken);
	}
	if(huart == &huart1){
		xTaskNotifyFromISR(UART1_Handle, NOTIFIED_FROM_ISR, eSetBits, &xHigherPriorityTaskWoken);
	}
	else if(huart == &huart2){
		xTaskNotifyFromISR(UART2_Handle, NOTIFIED_FROM_ISR, eSetBits, &xHigherPriorityTaskWoken);
	}
	else if(huart == &huart3){
		xTaskNotifyFromISR(UART3_Handle, NOTIFIED_FROM_ISR, eSetBits, &xHigherPriorityTaskWoken);
	}
	else if(huart == &huart4){
		xTaskNotifyFromISR(UART4_Handle, NOTIFIED_FROM_ISR, eSetBits, &xHigherPriorityTaskWoken);
	}
	else if(huart == &huart6){
		xTaskNotifyFromISR(UART6_Handle, NOTIFIED_FROM_ISR, eSetBits, &xHigherPriorityTaskWoken);
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
     error = HAL_UART_GetError(huart);
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
