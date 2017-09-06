/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
/* Motor driver. */
#include "../../../Dynamixel_AX-12A_Driver/src/Dynamixel_AX-12A.h"
#include "../../../Dynamixel_AX-12A_Driver/src/Dynamixel_AX-12A.c"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* Directions are relative to the robot's perspective. */
Dynamixel_HandleTypeDef Motor1; // Right motor to rotate left leg about vertical axis
Dynamixel_HandleTypeDef Motor2;
Dynamixel_HandleTypeDef Motor3;
Dynamixel_HandleTypeDef Motor4; // Left motor to rotate right leg about vertical axis
Dynamixel_HandleTypeDef Motor5;
Dynamixel_HandleTypeDef Motor6;
Dynamixel_HandleTypeDef Motor7; // Right knee
Dynamixel_HandleTypeDef Motor8; // Controls up/down motion of right foot
Dynamixel_HandleTypeDef Motor9; // Controls side-to-side motion of right foot
Dynamixel_HandleTypeDef Motor10; // Left knee
Dynamixel_HandleTypeDef Motor11; // Controls up/down motion of left foot
Dynamixel_HandleTypeDef Motor12; // Controls side-to-side motion of left foot

// UN-ID'd:
Dynamixel_HandleTypeDef Motor13; // Right shoulder
Dynamixel_HandleTypeDef Motor14; // Right elbow
Dynamixel_HandleTypeDef Motor15; // Left shoulder
Dynamixel_HandleTypeDef Motor16; // Left elbow
Dynamixel_HandleTypeDef Motor17; // Neck
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

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
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_UART5_Init();
  MX_TIM10_Init();
  MX_UART4_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();

  /* USER CODE BEGIN 2 */
  /* UART2 --> Motors: 1, 2, 3	|	Data Direction Pin: D7 (STM32) / D51 (Nucleo446ZE). */
  	Dynamixel_Init(&Motor1, 1, &huart2, GPIOD, GPIO_PIN_7);
  	Dynamixel_Init(&Motor2, 2, &huart2, GPIOD, GPIO_PIN_7);
  	Dynamixel_Init(&Motor3, 3, &huart2, GPIOD, GPIO_PIN_7);

  	/* UART4 --> Motors: 4, 5, 6	|	Data Direction Pin: C12 (STM32) / D47 (Nucleo446ZE). */
  	Dynamixel_Init(&Motor4, 4, &huart4, GPIOC, GPIO_PIN_12);
  	Dynamixel_Init(&Motor5, 5, &huart4, GPIOC, GPIO_PIN_12);
  	Dynamixel_Init(&Motor6, 6, &huart4, GPIOC, GPIO_PIN_12);

  	/* UART5 --> Motors: 7, 8, 9	|	Data Direction Pin: F10 (STM32) / A5 (Nucleo446ZE). */
  	Dynamixel_Init(&Motor7, 7, &huart5, GPIOF, GPIO_PIN_10);
  	Dynamixel_Init(&Motor8, 8, &huart5, GPIOF, GPIO_PIN_10);
  	Dynamixel_Init(&Motor9, 9, &huart5, GPIOF, GPIO_PIN_10);

  	/* UART6 --> Motors: 10, 11, 12	|	Data Direction Pin: F15 (STM32) / D2 (Nucleo446ZE). */
  	Dynamixel_Init(&Motor10, 10, &huart6, GPIOF, GPIO_PIN_15);
  	Dynamixel_Init(&Motor11, 11, &huart6, GPIOF, GPIO_PIN_15);
  	Dynamixel_Init(&Motor12, 12, &huart6, GPIOF, GPIO_PIN_15);

  	Dynamixel_HandleTypeDef* arrDynamixel[12];
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

	for(uint8_t i = 0; i < 12; i++){
//		Dynamixel_SetGoalVelocity(arrDynamixel[i], MAX_VELOCITY*0.2);
  		Dynamixel_LEDEnable(arrDynamixel[i], 1);
  		HAL_Delay(1000);
  		Dynamixel_LEDEnable(arrDynamixel[i], 0);
  		HAL_Delay(10);
	}
	while(1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** NVIC Configuration
*/
static void MX_NVIC_Init(void)
{
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USART6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART6_IRQn);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
