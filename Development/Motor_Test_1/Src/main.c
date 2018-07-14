/**
 ******************************************************************************
 * @file           : main.c
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
#include "stm32l4xx_hal.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "DynamixelProtocolV1.h"
#include "AX12A.h"
#include "Dynamixel_HandleTypeDef.h"
#include "string.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
char rx_buf[4] = "...."; //receives chars from uart, 1)AX##, 2)M1##, 3)M2##
uint8_t rx_valid = -1; //stores valid id in non-string format

Dynamixel_HandleTypeDef motorAX;
Dynamixel_HandleTypeDef motorMX;

int valid_rx(char *rx);
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
int main(void) {
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
	MX_USART1_UART_Init(); //default baud rate is 1M

	/* USER CODE BEGIN 2 */
	enum cases {
		INVALID, VALID_AX, // e.g. AX01
		VALID_MX1, // e.g. M101
		VALID_MX2, // e.g. M201
		QUERY, // e.g. ?...
		NA
	} currCase, pastCase;

	uint8_t id_show = -1;
	currCase = NA; //case 1:AX12 at 1M, case 2: MX28 at 57600, case 3: MX28 at 1M
	pastCase = NA; //needed for isInitialized condition

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		HAL_StatusTypeDef status;
		do {
			status = HAL_UART_Receive(&huart2, (unsigned char *)rx_buf, 4, 1000);
		} while (status != HAL_OK);

		currCase = valid_rx(rx_buf);

		switch (currCase) {
		case INVALID:
			HAL_UART_Transmit(&huart2, (unsigned char *) "invalid", 7, 1000);
			break;

		case VALID_AX: //1. if valid id is received, motortype AX
			MX_USART1_UART_Init_X(1000000);
			Dynamixel_Init(&motorAX, 0xFE, &huart1, Motor1_GPIO_Port,
			Motor1_Pin, AX12ATYPE);
			Dynamixel_SetID(&motorAX, rx_valid);

			Dynamixel_SetBaudRate(&motorAX, 1000000);
			HAL_UART_Transmit(&huart2, (unsigned char *) "valid, AX\n", 10, 1000);
			pastCase = VALID_AX;
			break;

		case VALID_MX1: //2. if valid id is received, motortype MX at 57600 bps
			MX_USART1_UART_Init_X(57600);
			Dynamixel_Init(&motorMX, 0xFE, &huart1, Motor1_GPIO_Port,
			Motor1_Pin, MX28TYPE);
			Dynamixel_SetID(&motorMX, rx_valid);

			Dynamixel_SetBaudRate(&motorMX, 1000000);
			MX_USART1_UART_Init_X(1000000);

			HAL_UART_Transmit(&huart2, (unsigned char *) "valid, MX1\n", 11, 1000);
			pastCase = VALID_MX1;
			break;

		case VALID_MX2:
			//3. if valid id is received, motortype MX at 1M bps
			MX_USART1_UART_Init_X(1000000);
			Dynamixel_Init(&motorAX, 0xFE, &huart1, Motor1_GPIO_Port,
			Motor1_Pin, MX28TYPE);
			Dynamixel_SetID(&motorMX, rx_valid);

			Dynamixel_SetBaudRate(&motorMX, 1000000);
			HAL_UART_Transmit(&huart2, (unsigned char *) "valid, MX1\n", 11, 1000);
			pastCase = VALID_MX2;
			break;

		case QUERY:
			//if ID is requested, displays the last stored motor's ID
			//1. show motor type and store the IDs specific to the motor
			if (pastCase == VALID_AX) {
				id_show = motorAX._ID;
				HAL_UART_Transmit(&huart2, (unsigned char *) "Motor:AX\t", 9, 1000);
			} else if (pastCase == VALID_MX1 || pastCase == VALID_MX2) {
				id_show = motorMX._ID;
				HAL_UART_Transmit(&huart2, (unsigned char *) "Motor:MX\t", 9, 1000);
			}

			//1. show motor IDs
			if (id_show == 0xFE) {
				HAL_UART_Transmit(&huart2, (unsigned char *) "ID:0xFE\n", 8, 1000);
			} else if(id_show != -1) { //if ID was assigned
				char mes_id[10];
				uint8_t strlength;
				strlength = sprintf(mes_id, "ID: %d\n", id_show);
				HAL_UART_Transmit(&huart2, (uint8_t*) mes_id, strlength, 1000);
			}
			break;
		default:
			break;
		}

	}
	/* USER CODE END 3 */

}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	/**Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();

	__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE
			| RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	RCC_OscInitStruct.PLL.PLLN = 16;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1
			| RCC_PERIPHCLK_USART2;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/**Enable MSI Auto calibration
	 */
	HAL_RCCEx_EnableMSIPLLMode();

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
int valid_rx(char *rx) {
	/* Checks if received buffer is a valid ID input
	 *
	 * Returns: 0: INVALID / not a valid input
	 * 			1: VALID_AX / valid id, motor type is AX
	 * 			2: VALID_MX1 / valid id, motor type is MX, baudrate at 57600 for IDing
	 * 			3. VALID_MX2 / valid id, motor type is MX
	 *
	 */
	int idCheck = 0;
	/*1. return if query case*/
	if (rx[0] == '?') {
		return 4;
	}
	/*2. check if received ID is valid*/
	//NOTED: atoi(CHARACTER) instead of STRING causes problems
	uint16_t id_num = (rx[2] - '0') * 10 + (rx[3] - '0');
	if (id_num > 0 && id_num < 100) { //valid id is 1-18 at the moment, but set to 1-100 for now
		rx_valid = id_num;
		idCheck = 1;
	} else if (rx_buf[2] == 'F' && rx_buf[3] == 'E') { //0xFE broadcast
		rx_valid = 0xFE;
		idCheck = 1;
	} else {
		idCheck = 0;
		return 0;
	}
	/*3. check if received motor type is valid*/
	if (idCheck == 1) { //if ID id valid, return cases 1, 2, or 3
		if (rx_buf[0] == 'A' && rx_buf[1] == 'X') {
			return 1; //case 1: AX motor, 1M
		} else if (rx_buf[0] == 'M' && rx_buf[1] == '1') {
			return 2; //case 2: MX motor, 57600
		} else if (rx_buf[0] == 'M' && rx_buf[1] == '2') {
			return 3; //case 3: MX motor, 1M
		}
	}
	return 0; //all other cases, return 0, invalid
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
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
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
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
