/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "crc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "crc.h"
#include "../app/imu_helper.h"
#include "../component/MPU6050/MPU6050.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <cmath>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 #define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

 #define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define UART_RX_BUF_SIZE 39
volatile uint8_t uart_tx_buf[32] = { 0 },
		         uart_rx_buf[UART_RX_BUF_SIZE] = { 0 },
				 uart_rx_crc_buf[UART_RX_BUF_SIZE] = { 0 };
				 // uart_rx_valid_buf[UART_RX_BUF_SIZE] = { 0 };
volatile size_t uart_rx_buf_idx = 0,
		        uart_rx_munched_idx = 0;

extern CRC_HandleTypeDef hcrc;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;

#define NUM_PWM_SERVOS 6
volatile uint32_t *servo_ccrs[NUM_PWM_SERVOS] = {
	&TIM1->CCR1,
	&TIM1->CCR2,
	&TIM1->CCR3,
	&TIM1->CCR4,
	&TIM15->CCR2,
	&TIM16->CCR1,
};
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_I2C1_Init();
  MX_TIM6_Init();
  MX_TIM1_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start(&htim6);
  HAL_TIM_Base_Start(&htim15);
  HAL_TIM_Base_Start(&htim16);

  soccerbot::app::initImuProcessor();

  for(uint8_t i = 0; i < NUM_PWM_SERVOS; i++) {
	  *servo_ccrs[i] = 0; // __HAL_TIM_GET_AUTORELOAD(&htim1) / 2;
  }
  htim1.Instance->BDTR |= TIM_BDTR_MOE;
  htim1.Instance->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
  htim15.Instance->BDTR |= TIM_BDTR_MOE;
  htim15.Instance->CCER |= TIM_CCER_CC2E;
  htim16.Instance->BDTR |= TIM_BDTR_MOE;
  htim16.Instance->CCER |= TIM_CCER_CC1E;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    imu::ImuStruct_t imu_data = { 0 };
	imu::MPU6050 imu(&hi2c1);
	constexpr uint8_t IMU_DIGITAL_LOWPASS_FILTER_SETTING = 6;
	imu.init(IMU_DIGITAL_LOWPASS_FILTER_SETTING);
	uint32_t last_tx_tick = 0, last_rx_tick = 0, num_samples = 0;
	float PACKET_HEADER = NAN; // nanf("");
	for (uint32_t iter = 0;; iter++)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		uint32_t tick = HAL_GetTick();
		{
			// WRITE BLOCK
			if(tick > last_tx_tick + 1) { // reliable enough 500Hz if most of the time is spent in the busy wait and the conditional stuff doesn't take more than a few 100 us
				last_tx_tick = tick;
				uint8_t valid_gyro = soccerbot::app::readFromSensor(imu, &num_samples);
				imu.Fill_Struct(&imu_data);
				if(valid_gyro) {
					soccerbot::app::processImuData(imu_data);
				}
				if(1) {
					// HAL_UART_Transmit_IT(&huart2, (uint8_t*)&PACKET_HEADER, sizeof(PACKET_HEADER));
					memcpy((void*)uart_tx_buf, &PACKET_HEADER, sizeof(PACKET_HEADER));
					memcpy((void*)(&uart_tx_buf[sizeof(PACKET_HEADER)]), &imu_data.x_Gyro, sizeof(imu_data));
					HAL_UART_Transmit_IT(&huart2, (uint8_t*)uart_tx_buf, sizeof(PACKET_HEADER) + sizeof(imu_data));
				}
				if(0) {
					char buf[64] = { 0 };
					size_t n = sprintf(buf, "%ld\t%.2f\t%.2f\r\n", HAL_GetTick(), imu_data.z_Accel, imu_data.z_Gyro);
					HAL_UART_Transmit_IT(&huart2, (const uint8_t*)buf, n);
				}
			}
		}

		if(1) {
			if(tick > last_rx_tick) { // ~1kHz poll
				last_rx_tick = tick;
				// READ BLOCK
				volatile uint16_t last_read_size = huart2.RxXferSize - huart2.RxXferCount;
				volatile uint8_t next_read_size = UART_RX_BUF_SIZE; // min(, UART_RX_BUF_SIZE - ((uart_rx_buf_idx + last_read_size) % UART_RX_BUF_SIZE));
				HAL_StatusTypeDef status = HAL_UART_Receive_IT(&huart2, (uint8_t*)&uart_rx_buf[(uart_rx_buf_idx + last_read_size) % UART_RX_BUF_SIZE], next_read_size);
				if(status == HAL_OK) {
					uart_rx_buf_idx += last_read_size;
				}

				uint16_t cur_read_size = huart2.RxXferSize - huart2.RxXferCount;
				uint16_t eot = uart_rx_buf_idx + cur_read_size;
				for(uint8_t i = uart_rx_munched_idx; i < eot; i++) {
					if(uart_rx_buf[i % UART_RX_BUF_SIZE] & 0x40) {
						// crc_buf = rx_buf[munched_idx:] + rx_buf[:munched_idx]
						uint16_t bytes_to_end = UART_RX_BUF_SIZE - (uart_rx_munched_idx % UART_RX_BUF_SIZE);
						memcpy((void*)uart_rx_crc_buf, (void*)&uart_rx_buf[uart_rx_munched_idx % UART_RX_BUF_SIZE], bytes_to_end);
						memcpy((void*)&uart_rx_crc_buf[bytes_to_end], (void*)uart_rx_buf, uart_rx_munched_idx % UART_RX_BUF_SIZE);

						uint16_t packet_size = max(0, (int16_t)eot - (int16_t)uart_rx_munched_idx - 1) % UART_RX_BUF_SIZE;
						memset((void*)&uart_rx_crc_buf[packet_size], 0, UART_RX_BUF_SIZE - packet_size); // excludes bytes past data (including master-calculated CRC)

						volatile uint16_t n_crc32 = ((max(packet_size, 1) - 1) / 4) + 1;
						volatile uint8_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t*)uart_rx_crc_buf, n_crc32);
						if(((crc ^ uart_rx_buf[i % UART_RX_BUF_SIZE]) & 0x3F) == 0) {
							for(uint8_t servo_idx = 0; servo_idx < NUM_PWM_SERVOS; servo_idx++) {
								uint16_t ccr = uart_rx_buf[(uart_rx_munched_idx + servo_idx * 2) % UART_RX_BUF_SIZE];
								ccr |= (uint16_t)uart_rx_buf[(uart_rx_munched_idx + servo_idx * 2 + 1) % UART_RX_BUF_SIZE] << 6;
								*servo_ccrs[servo_idx] = SERVO0 + ccr * SERVO_RANGE / 0xFFF; // (uint32_t)(uart_rx_buf[(uart_rx_munched_idx + servo_idx) % UART_RX_BUF_SIZE] & 0x7F) * htim1.Instance->ARR / 0x7F;
							}
						}
						// possibly count bad packets
						uart_rx_munched_idx = i + 1;
						break;
					}
				}
			}
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
