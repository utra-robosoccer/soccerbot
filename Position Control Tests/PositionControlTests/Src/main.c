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

/* USER CODE BEGIN Includes */
/* Motor driver. */
#include "../../../Dynamixel_AX-12A_Driver/src/Dynamixel_AX-12A.h"
#include "../../../Dynamixel_AX-12A_Driver/src/Dynamixel_AX-12A.c"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
Dynamixel_HandleTypeDef Motor1;

const double SimuData[] =
  /* Expression: OutValues
   * Referenced by: '<S2>/Vector'
   */
  { 0.0, 0.024541228522912288, 0.049067674327418015, 0.073564563599667426,
    0.0980171403295606, 0.1224106751992162, 0.14673047445536175,
    0.17096188876030122, 0.19509032201612825, 0.2191012401568698,
    0.24298017990326387, 0.26671275747489837, 0.29028467725446233,
    0.31368174039889152, 0.33688985339222005, 0.35989503653498811,
    0.38268343236508978, 0.40524131400498986, 0.42755509343028208,
    0.44961132965460654, 0.47139673682599764, 0.49289819222978404,
    0.51410274419322166, 0.53499761988709715, 0.55557023301960218,
    0.57580819141784534, 0.59569930449243336, 0.61523159058062682,
    0.63439328416364549, 0.65317284295377676, 0.67155895484701833,
    0.68954054473706683, 0.70710678118654746, 0.72424708295146689,
    0.74095112535495911, 0.75720884650648457, 0.77301045336273688,
    0.78834642762660623, 0.80320753148064483, 0.81758481315158371,
    0.83146961230254524, 0.844853565249707, 0.85772861000027212,
    0.87008699110871135, 0.88192126434835494, 0.89322430119551532,
    0.90398929312344334, 0.91420975570353069, 0.92387953251128674,
    0.93299279883473885, 0.94154406518302081, 0.94952818059303667,
    0.95694033573220894, 0.96377606579543984, 0.970031253194544,
    0.97570213003852857, 0.98078528040323043, 0.98527764238894122,
    0.989176509964781, 0.99247953459871, 0.99518472667219682,
    0.99729045667869021, 0.99879545620517241, 0.99969881869620425, 1.0,
    0.99969881869620425, 0.99879545620517241, 0.99729045667869021,
    0.99518472667219693, 0.99247953459871, 0.989176509964781,
    0.98527764238894122, 0.98078528040323043, 0.97570213003852857,
    0.970031253194544, 0.96377606579543984, 0.95694033573220894,
    0.94952818059303667, 0.94154406518302081, 0.93299279883473885,
    0.92387953251128674, 0.91420975570353069, 0.90398929312344345,
    0.89322430119551521, 0.881921264348355, 0.87008699110871146,
    0.85772861000027212, 0.84485356524970723, 0.83146961230254535,
    0.81758481315158371, 0.80320753148064494, 0.78834642762660634,
    0.7730104533627371, 0.75720884650648468, 0.740951125354959,
    0.72424708295146689, 0.70710678118654757, 0.689540544737067,
    0.67155895484701855, 0.65317284295377664, 0.63439328416364549,
    0.61523159058062693, 0.59569930449243347, 0.57580819141784545,
    0.55557023301960218, 0.53499761988709715, 0.51410274419322177,
    0.49289819222978415, 0.47139673682599786, 0.44961132965460687,
    0.42755509343028203, 0.40524131400498992, 0.38268343236508989,
    0.35989503653498833, 0.33688985339222033, 0.31368174039889141,
    0.29028467725446239, 0.26671275747489848, 0.24298017990326407,
    0.21910124015687005, 0.19509032201612861, 0.17096188876030122,
    0.1467304744553618, 0.12241067519921635, 0.098017140329560826,
    0.073564563599667732, 0.049067674327417966, 0.024541228522912326,
    1.2246467991473532E-16, -0.02454122852291208, -0.049067674327417724,
    -0.0735645635996675, -0.09801714032956059, -0.1224106751992161,
    -0.14673047445536158, -0.17096188876030097, -0.19509032201612836,
    -0.2191012401568698, -0.24298017990326382, -0.26671275747489825,
    -0.29028467725446211, -0.31368174039889118, -0.33688985339222005,
    -0.35989503653498811, -0.38268343236508967, -0.40524131400498969,
    -0.42755509343028181, -0.44961132965460665, -0.47139673682599764,
    -0.49289819222978393, -0.51410274419322155, -0.53499761988709693,
    -0.555570233019602, -0.57580819141784534, -0.59569930449243325,
    -0.61523159058062671, -0.63439328416364527, -0.65317284295377653,
    -0.67155895484701844, -0.68954054473706683, -0.70710678118654746,
    -0.72424708295146678, -0.74095112535495888, -0.75720884650648423,
    -0.77301045336273722, -0.78834642762660589, -0.803207531480645,
    -0.81758481315158327, -0.83146961230254524, -0.84485356524970712,
    -0.857728610000272, -0.87008699110871135, -0.88192126434835494,
    -0.89322430119551521, -0.90398929312344312, -0.9142097557035308,
    -0.92387953251128652, -0.932992798834739, -0.94154406518302081,
    -0.94952818059303667, -0.95694033573220882, -0.96377606579543984,
    -0.970031253194544, -0.97570213003852846, -0.98078528040323032,
    -0.98527764238894111, -0.989176509964781, -0.99247953459871,
    -0.99518472667219693, -0.99729045667869021, -0.99879545620517241,
    -0.99969881869620425, -1.0, -0.99969881869620425, -0.99879545620517241,
    -0.99729045667869021, -0.99518472667219693, -0.99247953459871,
    -0.98917650996478113, -0.98527764238894122, -0.98078528040323043,
    -0.97570213003852857, -0.970031253194544, -0.96377606579544,
    -0.95694033573220894, -0.94952818059303679, -0.94154406518302092,
    -0.93299279883473907, -0.92387953251128663, -0.91420975570353091,
    -0.90398929312344334, -0.89322430119551532, -0.881921264348355,
    -0.87008699110871146, -0.85772861000027223, -0.84485356524970723,
    -0.83146961230254546, -0.81758481315158393, -0.80320753148064528,
    -0.78834642762660612, -0.77301045336273688, -0.75720884650648457,
    -0.74095112535495911, -0.724247082951467, -0.70710678118654768,
    -0.68954054473706716, -0.67155895484701866, -0.65317284295377709,
    -0.63439328416364593, -0.61523159058062737, -0.59569930449243325,
    -0.57580819141784523, -0.55557023301960218, -0.53499761988709726,
    -0.51410274419322188, -0.49289819222978426, -0.47139673682599792,
    -0.449611329654607, -0.42755509343028253, -0.40524131400499042,
    -0.38268343236509039, -0.359895036534988, -0.33688985339222,
    -0.31368174039889152, -0.2902846772544625, -0.26671275747489859,
    -0.24298017990326418, -0.21910124015687016, -0.19509032201612872,
    -0.17096188876030177, -0.14673047445536239, -0.12241067519921603,
    -0.0980171403295605, -0.073564563599667412, -0.049067674327418091,
    -0.024541228522912448, -2.4492935982947064E-16
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM10_Init(void);

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
  MX_USART2_UART_Init();
  MX_TIM10_Init();

  /* USER CODE BEGIN 2 */
  Dynamixel_Init(&Motor1, 1, &huart2, GPIOD, GPIO_PIN_7);
  Dynamixel_SetGoalVelocity(&Motor1, MAX_VELOCITY);
  Dynamixel_SetCWComplianceSlope(&Motor1, 7);
  Dynamixel_SetCCWComplianceSlope(&Motor1, 7);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  int i = 0;
	  while(i < 256){
		  Dynamixel_SetGoalPosition(&Motor1, (SimuData[i]+1.0)*90.0 + 60);
		  HAL_Delay(10);
		  i++;
	  }
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

/* TIM10 init function */
static void MX_TIM10_Init(void)
{

  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 90;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 9999;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 1000000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PD8   ------> USART3_TX
     PD9   ------> USART3_RX
     PA8   ------> USB_OTG_FS_SOF
     PA9   ------> USB_OTG_FS_VBUS
     PA10   ------> USB_OTG_FS_ID
     PA11   ------> USB_OTG_FS_DM
     PA12   ------> USB_OTG_FS_DP
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : STLK_RX_Pin STLK_TX_Pin */
  GPIO_InitStruct.Pin = STLK_RX_Pin|STLK_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim){
	if(htim->Instance == TIM10){
	}
}
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
