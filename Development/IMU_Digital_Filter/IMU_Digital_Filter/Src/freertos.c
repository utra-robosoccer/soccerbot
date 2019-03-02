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
#include "../Drivers/MPU6050/MPU6050.h"
#include "sharedMacros.h"
#include "../Drivers/MPU6050/MPUFilter.h"

typedef struct{
    float ax;
    float ay;
    float az;
    float vx;
    float vy;
    float vz;
}IMUData_t;
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[ 512 ];
osStaticThreadDef_t defaultTaskControlBlock;
osThreadId IMUHandle;
uint32_t IMUBuffer[ 512 ];
osStaticThreadDef_t IMUControlBlock;
osThreadId TXHandle;
uint32_t TXBuffer[ 512 ];
osStaticThreadDef_t TXControlBlock;
osMessageQId pcQueueHandle;
uint8_t pcQueueBuffer[ 1 * sizeof( IMUData_t ) ];
osStaticMessageQDef_t pcQueueControlBlock;

/* USER CODE BEGIN Variables */
MPU6050_HandleTypeDef IMUdata;
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void StartIMU(void const * argument);
void StartTX(void const * argument);

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
  osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512, defaultTaskBuffer, &defaultTaskControlBlock);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of IMU */
  osThreadStaticDef(IMU, StartIMU, osPriorityNormal, 0, 512, IMUBuffer, &IMUControlBlock);
  IMUHandle = osThreadCreate(osThread(IMU), NULL);

  /* definition and creation of TX */
  osThreadStaticDef(TX, StartTX, osPriorityHigh, 0, 512, TXBuffer, &TXControlBlock);
  TXHandle = osThreadCreate(osThread(TX), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of pcQueue */
  osMessageQStaticDef(pcQueue, 1, IMUData_t, pcQueueBuffer, &pcQueueControlBlock);
  pcQueueHandle = osMessageCreate(osMessageQ(pcQueue), NULL);

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

/* StartIMU function */
void StartIMU(void const * argument)
{
  /* USER CODE BEGIN StartIMU */
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  const TickType_t IMU_CYCLE_TIME_MS = 2;
  uint8_t i = 0;

  IMUdata._I2C_Handle = &hi2c1;
  MPU6050_init(&IMUdata);
  MPU6050_manually_set_offsets(&IMUdata);
  MPU6050_set_LPF(&IMUdata, 4);

  IMUData_t data;

  MPUFilter_InitAllFilters();

  /* Infinite loop */
  for(;;)
  {
      // Service this thread every 2 ms for a 500 Hz sample rate
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(IMU_CYCLE_TIME_MS));

      MPU6050_Read_Accelerometer_Withoffset_IT(&IMUdata); // Also updates pitch and roll
      MPUFilter_FilterAcceleration(&IMUdata);

      // Gyroscope data is much more volatile/sensitive to changes than
      // acceleration data. To compensate, we feed in samples to the filter
      // slower. Good DSP practise? Not sure. To compensate for the high
      // delays, we also use a filter with fewer taps than the acceleration
      // filters. Ideally: we would sample faster to reduce aliasing, then
      // use a filter with a smaller cutoff frequency. However, the filter
      // designer we are using does not allow us to generate such filters in
      // the free version, so this is the best we can do unless we use other
      // software.
      if(i % 16 == 0){
          MPU6050_Read_Gyroscope_Withoffset_IT(&IMUdata);
          MPUFilter_FilterAngularVelocity(&IMUdata);
      }
      i++;

      data.ax = IMUdata._X_ACCEL;
      data.ay = IMUdata._Y_ACCEL;
      data.az = IMUdata._Z_ACCEL;

      data.vx = IMUdata._X_GYRO;
      data.vy = IMUdata._Y_GYRO;
      data.vz = IMUdata._Z_GYRO;

      xQueueOverwrite(pcQueueHandle, &data);
    }
  /* USER CODE END StartIMU */
}

/* StartTX function */
void StartTX(void const * argument)
{
  /* USER CODE BEGIN StartTX */
  IMUData_t recvData = {0};
  uint8_t packet[24] = {0};

  const uint8_t ax_off = 0;
  const uint8_t ay_off = 4;
  const uint8_t az_off = 8;
  const uint8_t vx_off = 12;
  const uint8_t vy_off = 16;
  const uint8_t vz_off = 20;

  volatile int8_t val = 0;

  uint32_t notification;

  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  /* Infinite loop */
  for(;;)
  {
      while(xQueueReceive(pcQueueHandle, &recvData, portMAX_DELAY) != pdTRUE);

      // Service this thread every 10 ms
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));

      memcpy(packet, &recvData, 6 * sizeof(float));

      switch(val){
          case 0:
              HAL_UART_Transmit_DMA(&huart2, &packet[ax_off], sizeof(float));
              break;
          case 1:
              HAL_UART_Transmit_DMA(&huart2, &packet[ay_off], sizeof(float));
              break;
          case 2:
              HAL_UART_Transmit_DMA(&huart2, &packet[az_off], sizeof(float));
              break;
          case 3:
              HAL_UART_Transmit_DMA(&huart2, &packet[vx_off], sizeof(float));
              break;
          case 4:
              HAL_UART_Transmit_DMA(&huart2, &packet[vy_off], sizeof(float));
              break;
          case 5:
              HAL_UART_Transmit_DMA(&huart2, &packet[vz_off], sizeof(float));
              break;
          default:
              break;
      }

      do{
          xTaskNotifyWait(0, NOTIFIED_FROM_TX_ISR, &notification, portMAX_DELAY);
      }while((notification & NOTIFIED_FROM_TX_ISR) != NOTIFIED_FROM_TX_ISR);
  }
  /* USER CODE END StartTX */
}

/* USER CODE BEGIN Application */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    // This callback runs after the interrupt data transfer from the sensor to the mcu is finished
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(IMUHandle, NOTIFIED_FROM_RX_ISR, eSetBits, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart){
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if(huart == &huart2){
        xTaskNotifyFromISR(TXHandle, NOTIFIED_FROM_TX_ISR, eSetBits, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
