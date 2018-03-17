
/**
  ******************************************************************************
  * @file           : main.cpp
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_hal.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
/* Motor driver. */
#include <math.h>
#include "../../../../../control/soccer-control/PID.h"  // Control routines & helpers
#include "../../../../../control/soccer-control/rtwtypes.h"  // Type definitions from auto-gen code
//#include "../../../../soccer-control/angles.h"  // Joint trajectories

/* Other */
#include <stdio.h>
#include "AX12Aold.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
const int DIM = 16;
const int SIZE = 400;
const double MOTORANGLES[14][400] = {0};


enum motorNames {MOTOR1, MOTOR2, MOTOR3, MOTOR4, MOTOR5,
					   MOTOR6, MOTOR7, MOTOR8, MOTOR9, MOTOR10,
					   MOTOR11, MOTOR12, MOTOR13, MOTOR14, MOTOR15,
					   MOTOR16, MOTOR17, MOTOR18
};

const double PI = M_PI; // From math.h

Dynamixel_HandleTypeDef Motor1;
Dynamixel_HandleTypeDef Motor2;
Dynamixel_HandleTypeDef Motor3;
Dynamixel_HandleTypeDef Motor4;
Dynamixel_HandleTypeDef Motor5;
Dynamixel_HandleTypeDef Motor6;
Dynamixel_HandleTypeDef Motor7;
Dynamixel_HandleTypeDef Motor8;
Dynamixel_HandleTypeDef Motor9;
Dynamixel_HandleTypeDef Motor10;
Dynamixel_HandleTypeDef Motor11;
Dynamixel_HandleTypeDef Motor12;
Dynamixel_HandleTypeDef Motor13;
Dynamixel_HandleTypeDef Motor14;
Dynamixel_HandleTypeDef Motor15;
Dynamixel_HandleTypeDef Motor16;
Dynamixel_HandleTypeDef Motor17;
Dynamixel_HandleTypeDef Motor18;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_UART7_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_UART5_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */
  Dynamixel_Init(&Motor1, 1, &huart2, GPIOD, GPIO_PIN_7);
  Dynamixel_Init(&Motor2, 2, &huart2, GPIOD, GPIO_PIN_7);
  Dynamixel_Init(&Motor3, 3, &huart2, GPIOD, GPIO_PIN_7);
  Dynamixel_Init(&Motor4, 4, &huart1, GPIOB, GPIO_PIN_10);
  Dynamixel_Init(&Motor5, 5, &huart1, GPIOB, GPIO_PIN_10);
  Dynamixel_Init(&Motor6, 6, &huart1, GPIOB, GPIO_PIN_10);
  Dynamixel_Init(&Motor7, 7, &huart7, GPIOA, GPIO_PIN_15);
  Dynamixel_Init(&Motor8, 8, &huart7, GPIOA, GPIO_PIN_15);
  Dynamixel_Init(&Motor9, 9, &huart7, GPIOA, GPIO_PIN_15);
  Dynamixel_Init(&Motor10, 10, &huart5, GPIOB, GPIO_PIN_0);
  Dynamixel_Init(&Motor11, 11, &huart5, GPIOB, GPIO_PIN_0);
  Dynamixel_Init(&Motor12, 12, &huart5, GPIOB, GPIO_PIN_0);
  Dynamixel_Init(&Motor13, 13, &huart4, GPIOA, GPIO_PIN_0);
  Dynamixel_Init(&Motor14, 14, &huart4, GPIOA, GPIO_PIN_0);
  Dynamixel_Init(&Motor15, 15, &huart4, GPIOA, GPIO_PIN_0);
  Dynamixel_Init(&Motor16, 16, &huart4, GPIOA, GPIO_PIN_0);
  Dynamixel_Init(&Motor17, 17, &huart4, GPIOA, GPIO_PIN_0);
  Dynamixel_Init(&Motor18, 18, &huart4, GPIOA, GPIO_PIN_0);

  Dynamixel_HandleTypeDef* arrDynamixel[18];
  arrDynamixel[0] = &Motor1;
  arrDynamixel[1] = &Motor2;
  arrDynamixel[2] = &Motor3;
  arrDynamixel[3] = &Motor4;
  arrDynamixel[4] = &Motor5;
  arrDynamixel[5] = &Motor6;
  arrDynamixel[6] = &Motor7;
  arrDynamixel[7] = &Motor8;
  arrDynamixel[8] = &Motor9;
  arrDynamixel[9] = &Motor10;
  arrDynamixel[10] = &Motor11;
  arrDynamixel[11] = &Motor12;
  arrDynamixel[12] = &Motor13;
  arrDynamixel[13] = &Motor14;
  arrDynamixel[14] = &Motor15;
  arrDynamixel[15] = &Motor16;
  arrDynamixel[16] = &Motor17;
  arrDynamixel[17] = &Motor18;

  for(int i = 0; i < 18; i++){
	  Dynamixel_SetGoalVelocity(arrDynamixel[i], 100);
	  Dynamixel_SetCWComplianceSlope(arrDynamixel[i], 4);
	  Dynamixel_SetCCWComplianceSlope(arrDynamixel[i], 4);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	  for(int j = 0; j < SIZE; j ++){
		  for(int i = MOTOR1; i <= MOTOR12; i++){ // NB: i begins at 0 (i.e. Motor1 corresponds to i = 0)
			  switch(i){
				  case MOTOR1:	  Dynamixel_SetGoalPosition(arrDynamixel[i], -1*MOTORANGLES[i][j]*180/PI + 150 - 1);
								  break;
				  case MOTOR2:	  Dynamixel_SetGoalPosition(arrDynamixel[i], -1*MOTORANGLES[i][j]*180/PI + 150 + 3);
								  break;
				  case MOTOR3:	  Dynamixel_SetGoalPosition(arrDynamixel[i], -1*MOTORANGLES[i][j]*180/PI + 150 + 1);
								  break;
				  case MOTOR4:	  Dynamixel_SetGoalPosition(arrDynamixel[i], MOTORANGLES[i][j]*180/PI + 150 + 2);
								  break;
				  case MOTOR5:	  Dynamixel_SetGoalPosition(arrDynamixel[i], MOTORANGLES[i][j]*180/PI + 150 - 0);
								  break;
				  case MOTOR6:	  Dynamixel_SetGoalPosition(arrDynamixel[i], -1*MOTORANGLES[i][j]*180/PI + 150 + 0);
								  break;
				  case MOTOR7:	  Dynamixel_SetGoalPosition(arrDynamixel[i], MOTORANGLES[i][j]*180/PI + 150 + 0);
								  break;
				  case MOTOR8:	  Dynamixel_SetGoalPosition(arrDynamixel[i], MOTORANGLES[i][j]*180/PI + 150 - 3);
								  break;
				  case MOTOR9:	  Dynamixel_SetGoalPosition(arrDynamixel[i], -1*MOTORANGLES[i][j]*180/PI + 150 - 0);
								  break;
				  case MOTOR10:	  Dynamixel_SetGoalPosition(arrDynamixel[i], MOTORANGLES[i][j]*180/PI + 150 + 4);
								  break;
				  case MOTOR11:   Dynamixel_SetGoalPosition(arrDynamixel[i], MOTORANGLES[i][j]*180/PI + 150 + 1);
								  break;
				  case MOTOR12:	  Dynamixel_SetGoalPosition(arrDynamixel[i], -1*MOTORANGLES[i][j]*180/PI + 150 + 3);
								  break;
			  }
		  }
		  HAL_Delay(10); // Default from Lukas: 10
	  }
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Supply configuration update enable 
    */
  MODIFY_REG(PWR->CR3, PWR_CR3_SCUEN, 0);

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while ((PWR->D3CR & (PWR_D3CR_VOSRDY)) != PWR_D3CR_VOSRDY) 
  {
    
  }
    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_UART4|RCC_PERIPHCLK_UART7
                              |RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_UART5
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
  PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(SystemCoreClock/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
