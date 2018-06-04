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
uint32_t txTaskBuffer[ 128 ];
osStaticThreadDef_t txTaskControlBlock;
osMessageQId UART1_reqHandle;
uint8_t UART1_reqBuffer[ 20 * sizeof( UARTcmd ) ];
osStaticMessageQDef_t UART1_reqControlBlock;
osMessageQId UART2_reqHandle;
uint8_t UART2_reqBuffer[ 20 * sizeof( UARTcmd ) ];
osStaticMessageQDef_t UART2_reqControlBlock;
osMessageQId UART3_reqHandle;
uint8_t UART3_reqBuffer[ 20 * sizeof( UARTcmd ) ];
osStaticMessageQDef_t UART3_reqControlBlock;
osMessageQId UART4_reqHandle;
uint8_t UART4_reqBuffer[ 20 * sizeof( UARTcmd ) ];
osStaticMessageQDef_t UART4_reqControlBlock;
osMessageQId UART6_reqHandle;
uint8_t UART6_reqBuffer[ 20 * sizeof( UARTcmd ) ];
osStaticMessageQDef_t UART6_reqControlBlock;
osMessageQId UART_rxHandle;
uint8_t UART_rxBuffer[ 20 * sizeof( UARTcmd ) ];
osStaticMessageQDef_t UART_rxControlBlock;
osMessageQId IMUQueueHandle;
uint8_t IMUQueueBuffer[ 20 * sizeof( uint32_t ) ];
osStaticMessageQDef_t IMUQueueControlBlock;
osMutexId PCUARTHandle;
osStaticMutexDef_t PCUARTControlBlock;
osSemaphoreId semUART1TxHandle;
osStaticSemaphoreDef_t semUART1TxControlBlock;
osSemaphoreId semUART2TxHandle;
osStaticSemaphoreDef_t semUART2TxControlBlock;
osSemaphoreId semUART3TxHandle;
osStaticSemaphoreDef_t semUART3TxControlBlock;
osSemaphoreId semUART4TxHandle;
osStaticSemaphoreDef_t semUART4TxControlBlock;
osSemaphoreId semUART6TxHandle;
osStaticSemaphoreDef_t semUART6TxControlBlock;

/* USER CODE BEGIN Variables */

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

  /* Create the semaphores(s) */
  /* definition and creation of semUART1Tx */
  osSemaphoreStaticDef(semUART1Tx, &semUART1TxControlBlock);
  semUART1TxHandle = osSemaphoreCreate(osSemaphore(semUART1Tx), 1);

  /* definition and creation of semUART2Tx */
  osSemaphoreStaticDef(semUART2Tx, &semUART2TxControlBlock);
  semUART2TxHandle = osSemaphoreCreate(osSemaphore(semUART2Tx), 1);

  /* definition and creation of semUART3Tx */
  osSemaphoreStaticDef(semUART3Tx, &semUART3TxControlBlock);
  semUART3TxHandle = osSemaphoreCreate(osSemaphore(semUART3Tx), 1);

  /* definition and creation of semUART4Tx */
  osSemaphoreStaticDef(semUART4Tx, &semUART4TxControlBlock);
  semUART4TxHandle = osSemaphoreCreate(osSemaphore(semUART4Tx), 1);

  /* definition and creation of semUART6Tx */
  osSemaphoreStaticDef(semUART6Tx, &semUART6TxControlBlock);
  semUART6TxHandle = osSemaphoreCreate(osSemaphore(semUART6Tx), 1);

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
  osThreadStaticDef(txTask, StartTxTask, osPriorityHigh, 0, 128, txTaskBuffer, &txTaskControlBlock);
  txTaskHandle = osThreadCreate(osThread(txTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of UART1_req */
  osMessageQStaticDef(UART1_req, 20, UARTcmd, UART1_reqBuffer, &UART1_reqControlBlock);
  UART1_reqHandle = osMessageCreate(osMessageQ(UART1_req), NULL);

  /* definition and creation of UART2_req */
  osMessageQStaticDef(UART2_req, 20, UARTcmd, UART2_reqBuffer, &UART2_reqControlBlock);
  UART2_reqHandle = osMessageCreate(osMessageQ(UART2_req), NULL);

  /* definition and creation of UART3_req */
  osMessageQStaticDef(UART3_req, 20, UARTcmd, UART3_reqBuffer, &UART3_reqControlBlock);
  UART3_reqHandle = osMessageCreate(osMessageQ(UART3_req), NULL);

  /* definition and creation of UART4_req */
  osMessageQStaticDef(UART4_req, 20, UARTcmd, UART4_reqBuffer, &UART4_reqControlBlock);
  UART4_reqHandle = osMessageCreate(osMessageQ(UART4_req), NULL);

  /* definition and creation of UART6_req */
  osMessageQStaticDef(UART6_req, 20, UARTcmd, UART6_reqBuffer, &UART6_reqControlBlock);
  UART6_reqHandle = osMessageCreate(osMessageQ(UART6_req), NULL);

  /* definition and creation of UART_rx */
  osMessageQStaticDef(UART_rx, 20, UARTcmd, UART_rxBuffer, &UART_rxControlBlock);
  UART_rxHandle = osMessageCreate(osMessageQ(UART_rx), NULL);

  /* definition and creation of IMUQueue */
  osMessageQStaticDef(IMUQueue, 20, uint32_t, IMUQueueBuffer, &IMUQueueControlBlock);
  IMUQueueHandle = osMessageCreate(osMessageQ(IMUQueue), NULL);

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

/* UART1_Handler function */
void UART1_Handler(void const * argument)
{
  /* USER CODE BEGIN UART1_Handler */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END UART1_Handler */
}

/* UART2_Handler function */
void UART2_Handler(void const * argument)
{
  /* USER CODE BEGIN UART2_Handler */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END UART2_Handler */
}

/* UART3_Handler function */
void UART3_Handler(void const * argument)
{
  /* USER CODE BEGIN UART3_Handler */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END UART3_Handler */
}

/* UART4_Handler function */
void UART4_Handler(void const * argument)
{
  /* USER CODE BEGIN UART4_Handler */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END UART4_Handler */
}

/* UART6_Handler function */
void UART6_Handler(void const * argument)
{
  /* USER CODE BEGIN UART6_Handler */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END UART6_Handler */
}

/* StartIMUTask function */
void StartIMUTask(void const * argument)
{
  /* USER CODE BEGIN StartIMUTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartIMUTask */
}

/* StartRxTask function */
void StartRxTask(void const * argument)
{
  /* USER CODE BEGIN StartRxTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartRxTask */
}

/* StartTxTask function */
void StartTxTask(void const * argument)
{
  /* USER CODE BEGIN StartTxTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTxTask */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
