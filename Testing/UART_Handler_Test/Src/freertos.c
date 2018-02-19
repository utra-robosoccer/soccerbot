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
  * Copyright (c) 2017 STMicroelectronics International N.V. 
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
#include "stm32f4xx_hal.h"
//#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "UART_Handler.h"
#include "C:/Users/Gokul/SkyDrive/Documents/Work/stm32/Dynamixel_AX-12A_Driver/src/Dynamixel_AX_12A.c"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;

/* USER CODE BEGIN Variables */
Dynamixel_HandleTypeDef Motor0,Motor1,Motor2,Motor3,Motor4,Motor5,Motor6,Motor7,Motor8,Motor9,Motor10,Motor11;

osThreadId defaultTaskHandle;
osThreadId UART2_HandlerHandle;
osThreadId UART4_HandlerHandle;
osThreadId UART5_HandlerHandle;
osThreadId UART6_HandlerHandle;
osMessageQId UART2_reqHandle;
osMessageQId UART4_reqHandle;
osMessageQId UART5_reqHandle;
osMessageQId UART6_reqHandle;
osMessageQId UART_rxHandle;



const double motorPosArr[12][1000] = {
		{},
		{},
		{},
		{},
		{},
		{},
		{},
		{},
		{},
		{},
		{},
		{},
};


/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
void UART2_Handler(void const * argument);
void UART4_Handler(void const * argument);
void UART5_Handler(void const * argument);
void UART6_Handler(void const * argument);
//void UART_Send(void const * argument);
/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  osThreadDef(UART2_Handler, UART2_Handler, osPriorityRealtime, 0, 128);
  UART2_HandlerHandle = osThreadCreate(osThread(UART2_Handler), NULL);
  osThreadDef(UART4_Handler, UART4_Handler, osPriorityRealtime, 0, 128);
  UART4_HandlerHandle = osThreadCreate(osThread(UART4_Handler), NULL);
  osThreadDef(UART5_Handler, UART5_Handler, osPriorityRealtime, 0, 128);
  UART5_HandlerHandle = osThreadCreate(osThread(UART5_Handler), NULL);
  osThreadDef(UART6_Handler, UART6_Handler, osPriorityRealtime, 0, 128);
  UART6_HandlerHandle = osThreadCreate(osThread(UART6_Handler), NULL);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  osMessageQDef(UART2_req, 20, UARTcmd);
  UART2_reqHandle = osMessageCreate(osMessageQ(UART2_req), NULL);
  osMessageQDef(UART4_req, 20, UARTcmd);
  UART4_reqHandle = osMessageCreate(osMessageQ(UART4_req), NULL);
  osMessageQDef(UART5_req, 20, UARTcmd);
  UART5_reqHandle = osMessageCreate(osMessageQ(UART5_req), NULL);
  osMessageQDef(UART6_req, 20, UARTcmd);
  UART6_reqHandle = osMessageCreate(osMessageQ(UART6_req), NULL);
  osMessageQDef(UART_rx, 20, UART_HandleTypeDef*);
  UART_rxHandle = osMessageCreate(osMessageQ(UART_rx), NULL);
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
int size = 1000, i, j;
UARTcmd Motorcmd[12];
Dynamixel_Init(&Motor0, 1, &huart2, GPIOD, GPIO_PIN_7);
Dynamixel_Init(&Motor1, 2, &huart2, GPIOD, GPIO_PIN_7);
Dynamixel_Init(&Motor2, 3, &huart2, GPIOD, GPIO_PIN_7);
Dynamixel_Init(&Motor3, 4, &huart2, GPIOD, GPIO_PIN_7);
Dynamixel_Init(&Motor4, 5, &huart2, GPIOD, GPIO_PIN_7);
Dynamixel_Init(&Motor5, 6, &huart2, GPIOD, GPIO_PIN_7);
Dynamixel_Init(&Motor6, 7, &huart2, GPIOD, GPIO_PIN_7);
Dynamixel_Init(&Motor7, 8, &huart2, GPIOD, GPIO_PIN_7);
Dynamixel_Init(&Motor8, 9, &huart2, GPIOD, GPIO_PIN_7);
Dynamixel_Init(&Motor9, 10, &huart2, GPIOD, GPIO_PIN_7);
Dynamixel_Init(&Motor10, 11, &huart2, GPIOD, GPIO_PIN_7);
Dynamixel_Init(&Motor11, 12, &huart2, GPIOD, GPIO_PIN_7);

(Motorcmd[0]).motorHandle = &Motor0;
(Motorcmd[1]).motorHandle = &Motor1;
(Motorcmd[2]).motorHandle = &Motor2;
(Motorcmd[3]).motorHandle = &Motor3;
(Motorcmd[4]).motorHandle = &Motor4;
(Motorcmd[5]).motorHandle = &Motor5;
(Motorcmd[6]).motorHandle = &Motor6;
(Motorcmd[7]).motorHandle = &Motor7;
(Motorcmd[8]).motorHandle = &Motor8;
(Motorcmd[9]).motorHandle = &Motor9;
(Motorcmd[10]).motorHandle = &Motor10;
(Motorcmd[11]).motorHandle = &Motor11;
(Motorcmd[0]).qHandle = UART2_reqHandle;
(Motorcmd[1]).qHandle = UART2_reqHandle;
(Motorcmd[2]).qHandle = UART2_reqHandle;
(Motorcmd[3]).qHandle = UART2_reqHandle;
(Motorcmd[4]).qHandle = UART2_reqHandle;
(Motorcmd[5]).qHandle = UART2_reqHandle;
(Motorcmd[6]).qHandle = UART2_reqHandle;
(Motorcmd[7]).qHandle = UART2_reqHandle;
(Motorcmd[8]).qHandle = UART2_reqHandle;
(Motorcmd[9]).qHandle = UART2_reqHandle;
(Motorcmd[10]).qHandle = UART2_reqHandle;
(Motorcmd[11]).qHandle = UART2_reqHandle;

for(i=0; i<12; i++) {
	(Motorcmd[i]).type = cmdWRITE;
	(Motorcmd[i]).velocity = MAX_VELOCITY * 0.75;
}
  /* Infinite loop */
  for(i=0;i<size;i++)
  {
	  for(j=0;j<12;j++) {
		  (Motorcmd[j]).position = motorPosArr[j][i];
		  xQueueSend((Motorcmd[j]).qHandle,&(Motorcmd[j]),10);
	  }

    osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Application */

void UART2_Handler(void const * argument)
{
  /* USER CODE BEGIN UART_Handler */
  /* Infinite loop */
  UARTcmd 	cmdMessage;
  for(;;)
  {
	  while(xQueueReceive(UART2_reqHandle,&(cmdMessage),portMAX_DELAY) != pdTRUE);
	  if(cmdMessage.type == cmdREAD) {
		  /*SEND READ COMMAND TO MOTOR*/
		  Dynamixel_GetPosition(cmdMessage.motorHandle);
		  //Dynamixel_GetVelocity(cmdMessage.motorHandle);
		  xQueueSend(UART_rxHandle,&(cmdMessage.motorHandle),0);
	  }
	  else if(cmdMessage.type == cmdWRITE) {
		  Dynamixel_SetGoalPosition(cmdMessage.motorHandle,cmdMessage.position);
		  Dynamixel_SetGoalVelocity(cmdMessage.motorHandle,cmdMessage.velocity);
	  }
	  osDelay(1);
  }
  /* USER CODE END UART_Handler */
}
void UART4_Handler(void const * argument)
{
  /* USER CODE BEGIN UART_Handler */
  /* Infinite loop */
  UARTcmd 	cmdMessage;
  for(;;)
  {
	  while(xQueueReceive(UART4_reqHandle,&(cmdMessage),portMAX_DELAY) != pdTRUE);
	  if(cmdMessage.type == cmdREAD) {
		  /*SEND READ COMMAND TO MOTOR*/
		  Dynamixel_GetPosition(cmdMessage.motorHandle);
		  //Dynamixel_GetVelocity(cmdMessage.motorHandle);
		  xQueueSend(UART_rxHandle,&(cmdMessage.motorHandle),0);
	  }
	  else if(cmdMessage.type == cmdWRITE) {
		  Dynamixel_SetGoalPosition(cmdMessage.motorHandle,cmdMessage.position);
		  Dynamixel_SetGoalVelocity(cmdMessage.motorHandle,cmdMessage.velocity);
	  }
	  osDelay(1);
  }
  /* USER CODE END UART_Handler */
}
void UART5_Handler(void const * argument)
{
  /* USER CODE BEGIN UART_Handler */
  /* Infinite loop */
  UARTcmd 	cmdMessage;
  for(;;)
  {
	  while(xQueueReceive(UART5_reqHandle,&(cmdMessage),portMAX_DELAY) != pdTRUE);
	  if(cmdMessage.type == cmdREAD) {
		  /*SEND READ COMMAND TO MOTOR*/
		  Dynamixel_GetPosition(cmdMessage.motorHandle);
		  //Dynamixel_GetVelocity(cmdMessage.motorHandle);
		  xQueueSend(UART_rxHandle,&(cmdMessage.motorHandle),0);
	  }
	  else if(cmdMessage.type == cmdWRITE) {
		  Dynamixel_SetGoalPosition(cmdMessage.motorHandle,cmdMessage.position);
		  Dynamixel_SetGoalVelocity(cmdMessage.motorHandle,cmdMessage.velocity);
	  }
	  osDelay(1);
  }
  /* USER CODE END UART_Handler */
}
void UART6_Handler(void const * argument)
{
  /* USER CODE BEGIN UART_Handler */
  /* Infinite loop */
  UARTcmd 	cmdMessage;
  for(;;)
  {
	  while(xQueueReceive(UART6_reqHandle,&(cmdMessage),portMAX_DELAY) != pdTRUE);
	  if(cmdMessage.type == cmdREAD) {
		  /*SEND READ COMMAND TO MOTOR*/
		  Dynamixel_GetPosition(cmdMessage.motorHandle);
		  //Dynamixel_GetVelocity(cmdMessage.motorHandle);
		  xQueueSend(UART_rxHandle,&(cmdMessage.motorHandle),0);
	  }
	  else if(cmdMessage.type == cmdWRITE) {
		  Dynamixel_SetGoalPosition(cmdMessage.motorHandle,cmdMessage.position);
		  Dynamixel_SetGoalVelocity(cmdMessage.motorHandle,cmdMessage.velocity);
	  }
	  osDelay(1);
  }
  /* USER CODE END UART_Handler */
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
