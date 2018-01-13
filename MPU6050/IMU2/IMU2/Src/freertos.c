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
#include "stm32h7xx_hal.h"
#include "MPU6050.h"
#include "usart.h"
#include <stdio.h>
#include <stdlib.h>
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId IMUtaskHandle;

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartIMUtask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

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
  /* definition and creation of IMUtask */
  osThreadDef(IMUtask, StartIMUtask, osPriorityNormal, 0, 128);
  IMUtaskHandle = osThreadCreate(osThread(IMUtask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartIMUtask function */
void StartIMUtask(void const * argument)
{

  /* USER CODE BEGIN StartIMUtask */
    uint8_t output_buffer[6];
    uint8_t output_buffer_2[6];
    MPU6050_HandleTypeDef IMUdata;
    IMUdata._I2C_Handle = &hi2c3;
    uint16_t test=5;
    MPU6050_RESET_SENSOR_REG(IMUdata);
    MPU6050_init(&IMUdata);

  /* Infinite loop */
  for(;;)
  {
	  MPU6050_READ_DATA(&IMUdata, MPU6050_RA_ACCEL_XOUT_H,output_buffer);
	  			  	uint16_t Xa = abs((int16_t)(output_buffer[0]<<8|output_buffer[1]));
	  			  	uint16_t Ya = abs((int16_t)(output_buffer[2]<<8|output_buffer[3]));
	  			  	uint16_t Za = abs((int16_t)(output_buffer[4]<<8|output_buffer[5]));

	  			  	//Now read the gyroscope values:

	  				MPU6050_READ_DATA(&IMUdata, MPU6050_RA_GYRO_XOUT_H,output_buffer_2);
	  				uint16_t Xg = abs((int16_t)(output_buffer_2[0]<<8|output_buffer_2[1]));
	  				uint16_t Yg = abs((int16_t)(output_buffer_2[2]<<8|output_buffer_2[3]));
	  				uint16_t Zg = abs((int16_t)(output_buffer_2[4]<<8|output_buffer_2[5]));


	  				//Now store in struct:
	  				IMUdata._X_ACCEL = Xa;
	  				IMUdata._Y_ACCEL = Ya;
	  				IMUdata._Z_ACCEL = Za;

	  				IMUdata._X_GYRO = Xg;
	  				IMUdata._Y_GYRO = Yg;
	  				IMUdata._Z_GYRO = Zg;

	  				//TEST: Read a specific register

	  				//Now enqueue the struct
	  				//NOTE: determine how large the queue should be

	  				//xQueueSend( IMUqueueHandle, &IMUdata, 1000);
	  				HAL_UART_Transmit(&huart3, &Xg, 2, 10);
	  				//HAL_UART_Transmit(&huart3, &test, 2, 10);
	  				osDelay(1000);
  }
  /* USER CODE END StartIMUtask */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
