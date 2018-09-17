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

/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;
osThreadId UART1TaskHandle;
uint32_t UART1TaskBuffer[ 128 ];
osStaticThreadDef_t UART1TaskControlBlock;
osThreadId UART2TaskHandle;
uint32_t UART2TaskBuffer[ 128 ];
osStaticThreadDef_t UART2TaskControlBlock;
osThreadId UART5TaskHandle;
uint32_t UART5TaskBuffer[ 128 ];
osStaticThreadDef_t UART5TaskControlBlock;
osThreadId UART4TaskHandle;
uint32_t UART4TaskBuffer[ 128 ];
osStaticThreadDef_t UART4TaskControlBlock;
osThreadId UART6TaskHandle;
uint32_t UART6TaskBuffer[ 128 ];
osStaticThreadDef_t UART6TaskControlBlock;
osThreadId IMUTaskHandle;
uint32_t IMUTaskBuffer[ 128 ];
osStaticThreadDef_t IMUTaskControlBlock;
osThreadId CommandTaskHandle;
uint32_t CommandTaskBuffer[ 512 ];
osStaticThreadDef_t CommandTaskControlBlock;
osThreadId RxTaskHandle;
uint32_t RxTaskBuffer[ 512 ];
osStaticThreadDef_t RxTaskControlBlock;
osThreadId TxTaskHandle;
uint32_t TxTaskBuffer[ 128 ];
osStaticThreadDef_t TxTaskControlBlock;
osMessageQId UART1_reqHandle;
uint8_t UART1_reqBuffer[ 16 * sizeof( UARTcmd_t ) ];
osStaticMessageQDef_t UART1_reqControlBlock;
osMessageQId UART2_reqHandle;
uint8_t UART2_reqBuffer[ 16 * sizeof( UARTcmd_t ) ];
osStaticMessageQDef_t UART2_reqControlBlock;
osMessageQId UART5_reqHandle;
uint8_t UART5_reqBuffer[ 16 * sizeof( UARTcmd_t ) ];
osStaticMessageQDef_t UART5_reqControlBlock;
osMessageQId UART4_reqHandle;
uint8_t UART4_reqBuffer[ 16 * sizeof( UARTcmd_t ) ];
osStaticMessageQDef_t UART4_reqControlBlock;
osMessageQId UART6_reqHandle;
uint8_t UART6_reqBuffer[ 16 * sizeof( UARTcmd_t ) ];
osStaticMessageQDef_t UART6_reqControlBlock;
osMessageQId TXQueueHandle;
uint8_t TXQueueBuffer[ 32 * sizeof( TXData_t ) ];
osStaticMessageQDef_t TXQueueControlBlock;
osMutexId PCUARTHandle;
osStaticMutexDef_t PCUARTControlBlock;

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
extern void StartUART1Task(void const * argument);
extern void StartUART2Task(void const * argument);
void StartUART5Task(void const * argument);
extern void StartUART4Task(void const * argument);
extern void StartUART6Task(void const * argument);
extern void StartIMUTask(void const * argument);
extern void StartCommandTask(void const * argument);
extern void StartRxTask(void const * argument);
extern void StartTxTask(void const * argument);

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
  osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128, defaultTaskBuffer, &defaultTaskControlBlock);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of UART1Task */
  osThreadStaticDef(UART1Task, StartUART1Task, osPriorityNormal, 0, 128, UART1TaskBuffer, &UART1TaskControlBlock);
  UART1TaskHandle = osThreadCreate(osThread(UART1Task), NULL);

  /* definition and creation of UART2Task */
  osThreadStaticDef(UART2Task, StartUART2Task, osPriorityNormal, 0, 128, UART2TaskBuffer, &UART2TaskControlBlock);
  UART2TaskHandle = osThreadCreate(osThread(UART2Task), NULL);

  /* definition and creation of UART5Task */
  osThreadStaticDef(UART5Task, StartUART5Task, osPriorityNormal, 0, 128, UART5TaskBuffer, &UART5TaskControlBlock);
  UART5TaskHandle = osThreadCreate(osThread(UART5Task), NULL);

  /* definition and creation of UART4Task */
  osThreadStaticDef(UART4Task, StartUART4Task, osPriorityNormal, 0, 128, UART4TaskBuffer, &UART4TaskControlBlock);
  UART4TaskHandle = osThreadCreate(osThread(UART4Task), NULL);

  /* definition and creation of UART6Task */
  osThreadStaticDef(UART6Task, StartUART6Task, osPriorityNormal, 0, 128, UART6TaskBuffer, &UART6TaskControlBlock);
  UART6TaskHandle = osThreadCreate(osThread(UART6Task), NULL);

  /* definition and creation of IMUTask */
  osThreadStaticDef(IMUTask, StartIMUTask, osPriorityNormal, 0, 128, IMUTaskBuffer, &IMUTaskControlBlock);
  IMUTaskHandle = osThreadCreate(osThread(IMUTask), NULL);

  /* definition and creation of CommandTask */
  osThreadStaticDef(CommandTask, StartCommandTask, osPriorityBelowNormal, 0, 512, CommandTaskBuffer, &CommandTaskControlBlock);
  CommandTaskHandle = osThreadCreate(osThread(CommandTask), NULL);

  /* definition and creation of RxTask */
  osThreadStaticDef(RxTask, StartRxTask, osPriorityRealtime, 0, 512, RxTaskBuffer, &RxTaskControlBlock);
  RxTaskHandle = osThreadCreate(osThread(RxTask), NULL);

  /* definition and creation of TxTask */
  osThreadStaticDef(TxTask, StartTxTask, osPriorityHigh, 0, 128, TxTaskBuffer, &TxTaskControlBlock);
  TxTaskHandle = osThreadCreate(osThread(TxTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of UART1_req */
  osMessageQStaticDef(UART1_req, 16, UARTcmd_t, UART1_reqBuffer, &UART1_reqControlBlock);
  UART1_reqHandle = osMessageCreate(osMessageQ(UART1_req), NULL);

  /* definition and creation of UART2_req */
  osMessageQStaticDef(UART2_req, 16, UARTcmd_t, UART2_reqBuffer, &UART2_reqControlBlock);
  UART2_reqHandle = osMessageCreate(osMessageQ(UART2_req), NULL);

  /* definition and creation of UART5_req */
  osMessageQStaticDef(UART5_req, 16, UARTcmd_t, UART5_reqBuffer, &UART5_reqControlBlock);
  UART5_reqHandle = osMessageCreate(osMessageQ(UART5_req), NULL);

  /* definition and creation of UART4_req */
  osMessageQStaticDef(UART4_req, 16, UARTcmd_t, UART4_reqBuffer, &UART4_reqControlBlock);
  UART4_reqHandle = osMessageCreate(osMessageQ(UART4_req), NULL);

  /* definition and creation of UART6_req */
  osMessageQStaticDef(UART6_req, 16, UARTcmd_t, UART6_reqBuffer, &UART6_reqControlBlock);
  UART6_reqHandle = osMessageCreate(osMessageQ(UART6_req), NULL);

  /* definition and creation of TXQueue */
  osMessageQStaticDef(TXQueue, 32, TXData_t, TXQueueBuffer, &TXQueueControlBlock);
  TXQueueHandle = osMessageCreate(osMessageQ(TXQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* StartUART5Task function */
void StartUART5Task(void const * argument)
{
  /* USER CODE BEGIN StartUART5Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartUART5Task */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
