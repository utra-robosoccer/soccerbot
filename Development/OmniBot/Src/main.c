
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define		WHEEL_DIAMETER   0.1

enum motor_direction {
    MOTOR_STOP = 0, MOTOR_CLOCKWISE = 1, MOTOR_COUNTERCLOCKWISE = 2
};

char rx_buf[6] = "......"; //Each _ _ _ 3 digits represent x_vel or y_vel(pwm)

float x_vel = 0;
float y_vel = 0;
float w_vel = 0;

uint32_t pwm_value = 110; //anything below 100 will not turn the motor
uint32_t pwm_value_1 = 110;
uint32_t pwm_value_2 = 110;
uint32_t pwm_value_3 = 110;
uint32_t pwm_value_4 = 110;
uint8_t motor1_dir = 0;
uint8_t motor2_dir = 0;
uint8_t motor3_dir = 0;
uint8_t motor4_dir = 0;
uint8_t isStop = 0;

HAL_StatusTypeDef status;
/* Private variables ---------------------------------------------------------*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
int convertToInt(char *rx);
float convertToFloat(char *rx);
//void user_pwm_setvalue(uint16_t value, uint16_t channel);

void driveMotor(uint16_t pwm_value, uint8_t motor_num, uint16_t dir);
void setMotor1Dir();
void setMotor2Dir();
void setMotor3Dir();
void setMotor4Dir();

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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {

        /*1. Receive all pwm inputs from PC, and assign motor_wheel_directions */
        do {
            status = HAL_UART_Receive(&huart2, (unsigned char *) rx_buf, 6,
                    1000);
            setMotor1Dir();
            setMotor2Dir();
            setMotor3Dir();
            setMotor4Dir();
        } while (status != HAL_OK);
        status = HAL_ERROR;

        /*2. Convert the character in buffers to floats/integers, Update Globals */
        x_vel = convertToInt(rx_buf); //0-900 (100 < x < 200)
        char substr[3];
        strncpy(substr, rx_buf + 3, 3);
        y_vel = convertToInt(substr);



        pwm_value = 200;
        /*3. Given the input information, determine the globals(motor_dir, speed, etc.)*/
        //THERE SHOULD BE A FUNCTION HERE THAT ASSIGNS PWM FOR MOTORS AND SPEEDS
        motor1_dir = MOTOR_CLOCKWISE;
        motor2_dir = MOTOR_CLOCKWISE;
        motor3_dir = MOTOR_CLOCKWISE;
        motor4_dir = MOTOR_CLOCKWISE;

        /*4. With the updated globals, actuate the motors*/
        driveMotor(pwm_value, 1, motor1_dir);
        driveMotor(pwm_value, 2, motor2_dir);
        driveMotor(pwm_value, 3, motor3_dir);
        driveMotor(pwm_value, 4, motor4_dir);

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
        HAL_UART_Transmit(&huart2, (unsigned char *) "next", 4, 1000);
        HAL_Delay(100);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure LSE Drive Capability 
    */
  HAL_PWR_EnableBkUpAccess();

  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

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
int convertToInt(char *rx) {
    uint8_t hund = rx[0] - '0';
    uint8_t ten = rx[1] - '0';
    uint8_t one = rx[2] - '0';
    int res = hund * 100 + ten * 10 + one;
    return res;
}

float convertToFloat(char *rx) {
    float res = 0.0;
    uint8_t ten = rx[0] - '0';
    uint8_t one = rx[1] - '0';
    uint8_t dec_ten = rx[2] - '0';
    uint8_t dec_hun = rx[3] - '0';
    res = ten * 10 + one * 1 + dec_ten * 0.1 + dec_hun * 0.01;
    return res;
}

void setMotor1Dir() {
    switch (motor1_dir) {
    case MOTOR_CLOCKWISE:
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
        break;
    case MOTOR_COUNTERCLOCKWISE:
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
        break;
    case MOTOR_STOP:
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
        break;
    default:
        break;
    }
}

void setMotor2Dir() {
    switch (motor2_dir) {
    case MOTOR_CLOCKWISE:
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
        break;
    case MOTOR_COUNTERCLOCKWISE:
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
        break;
    case MOTOR_STOP:
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
        break;
    default:
        break;
    }
}

void setMotor3Dir() {
    switch (motor3_dir) {
    case MOTOR_CLOCKWISE:
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        break;
    case MOTOR_COUNTERCLOCKWISE:
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        break;
    case MOTOR_STOP:
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

void setMotor4Dir() {
    switch (motor4_dir) {
    case MOTOR_CLOCKWISE:
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
        break;
    case MOTOR_COUNTERCLOCKWISE:
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
        break;
    case MOTOR_STOP:
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
        break;
    default:
        break;
    }
}

/*void setPWMValue(uint16_t value, uint16_t channel) {
 //HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
 TIM_OC_InitTypeDef sConfigOC;

 sConfigOC.OCMode = TIM_OCMODE_PWM1;
 sConfigOC.Pulse = value;
 sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
 sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

 HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
 HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
 }*/

void driveMotor(uint16_t pwm_value, uint8_t motor_num, uint16_t dir) {
//Set PWM values and Motor Directions
    switch (motor_num) {
    case 1:
        motor1_dir = dir;
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        break;
    case 2:
        motor2_dir = dir;
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
        break;
    case 3:
        motor3_dir = dir;
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
        break;
    case 4:
        motor4_dir = dir;
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
        break;
    default:
        break;
    }
}

/*int* get_motor_dir(uint8_t dir){
 //motor_dir[4] = [0:motor0, 1:motor2, 2:motor3, 3:motor3]
 int motor_dir[4] = {-1};
 switch (dir) {
 case FORWARD:
 motor_dir[0] = MOTOR_CLOCKWISE;
 motor_dir[1] = MOTOR_CLOCKWISE;
 motor_dir[2] = MOTOR_CLOCKWISE;
 motor_dir[3] = MOTOR_CLOCKWISE;
 break;
 case BACKWARD:
 motor_dir[0] = MOTOR_COUNTERCLOCKWISE;
 motor_dir[1] = MOTOR_COUNTERCLOCKWISE;
 motor_dir[2] = MOTOR_COUNTERCLOCKWISE;
 motor_dir[3] = MOTOR_COUNTERCLOCKWISE;
 break;
 case LEFT:
 motor_dir[0] = MOTOR_COUNTERCLOCKWISE;
 motor_dir[1] = MOTOR_CLOCKWISE;
 motor_dir[2] = MOTOR_CLOCKWISE;
 motor_dir[3] = MOTOR_COUNTERCLOCKWISE;
 break;
 case RIGHT:
 motor_dir[0] = MOTOR_CLOCKWISE;
 motor_dir[1] = MOTOR_COUNTERCLOCKWISE;
 motor_dir[2] = MOTOR_COUNTERCLOCKWISE;
 motor_dir[3] = MOTOR_CLOCKWISE;
 break;
 case STOP:
 motor_dir[0] = MOTOR_STOP;
 motor_dir[1] = MOTOR_STOP;
 motor_dir[2] = MOTOR_STOP;
 motor_dir[3] = MOTOR_STOP;
 break;
 default:
 break;
 }
 return motor_dir;
 }
 */

//int get_pwm_speed(uint8_t speed){
//	uint8_t rpm = speed/(WHEEL_DIAMETER *)
//}
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
