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
#include "Dynamixel_Data.h"
#include "AX12A.h"
#include "MX28.h"
#include "Dynamixel_Types.h"
#include "DynamixelProtocolV1_IO.h"
#include "DynamixelProtocolV1.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define		WHEEL_DIAMETER   0.1
#define		SPEED 			 1 //rps



enum motor_direction {
    MOTOR_STOP = 0, MOTOR_CLOCKWISE = 1, MOTOR_COUNTERCLOCKWISE = 2
};

uint8_t rx_buf[3];
//rx_buf2 = 0, rx_buf3 = 0;

uint8_t x_case = 0; //0-stop, 1-0.1m/s, 2-0.2m/s, 3-0.3m/s
uint8_t y_case = 0;
uint8_t w_case = 0;

uint8_t w_dir, x_dir, y_dir;
float x_vel;
float y_vel;

uint32_t pwm_value_w = 0;
uint32_t pwm_value_1 = 0;
uint32_t pwm_value_2 = 0;

uint8_t motor1_dir = 0;
uint8_t motor2_dir = 0;
uint8_t motor3_dir = 0;
uint8_t motor4_dir = 0;
uint8_t isStop = 0;

double joint1_angle;
double joint2_angle;

uint8_t joint1_id = 10;
uint8_t joint2_id = 16;

HAL_StatusTypeDef status_b1, status_b2, status_b3;
/* Private variables ---------------------------------------------------------*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void extractW(uint8_t rx_buf_1);
void extractX(uint8_t rx_buf_1);
void extractY(uint8_t rx_buf_1);

double extractAngle(uint8_t rx_buf);
void joint_horizontal_control(double angle, uint8_t joint_id);
void joint_vertical_control(double angle, uint8_t joint_id);

int convertToInt(char *rx);
//uint32_t get_pwm(float vel);

void driveMotor(uint16_t pwm_value, uint8_t motor_num, uint16_t dir);
void setMotor1Dir();
void setMotor2Dir();
void setMotor3Dir();
void setMotor4Dir();

void move_to_pos();


Dynamixel_HandleTypeDef motorAX_hor;
Dynamixel_HandleTypeDef motorAX_vert;

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
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
    MX_TIM1_Init();
    /* USER CODE BEGIN 2 */

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        int prev = 0;

        do {
            /* while huart2 does not receive commands, in other words
             * while status_b1 is NOT HAL_OK, stay in this DO loop to
             * set motor directions and actuate them(PWM)
             */
            status_b1 = HAL_UART_Receive(&huart2, rx_buf, 3*sizeof(uint8_t), 10);
            //status_b2 = HAL_UART_Receive(&huart2, &rx_buf2, sizeof(uint8_t), 10);
            //status_b3 = HAL_UART_Receive(&huart2, &rx_buf3, sizeof(uint8_t), 10);

            setMotor1Dir();
            setMotor2Dir();
            setMotor3Dir();
            setMotor4Dir();

            if (prev == 0) { //only start pwm when it has been stopped
                HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
                HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
                HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
                HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
                prev = 1;
            }
        } while (status_b1 != HAL_OK);


        status_b1 = HAL_ERROR; //reset
        status_b2 = HAL_ERROR; //reset
        status_b3 = HAL_ERROR; //reset

        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);

        /* 1. Received all pwm inputs from PC. Once outside the do-while loop,
         * the global variable rx_buf has been updated from the PC command  */

        /* 2. Convert the character in the buffer to floats/integers */
        /* Assumed: at max speed, the wheel is turning 1 rev/sec => 6.28*0.05*/
        // Global variables including x_dir, w_dir, y_dir, and pwm's are updated
        extractW(rx_buf[0]);
        extractX(rx_buf[0]);
        extractY(rx_buf[0]);

        /* 3. With the updated globals, set the appropriate speeds(PWM) for
         * all motors using SET_COMPARE function*/
        move_to_pos();


        /*extract second byte*/
        MX_USART2_UART_Init_X(1000000);
        joint1_angle = extractAngle(rx_buf[1]);
        joint_horizontal_control(joint1_angle, joint1_id);

        /*extract third byte*/
        joint2_angle = extractAngle(rx_buf[2]);
        joint_vertical_control(joint2_angle, joint2_id);

        MX_USART2_UART_Init_X(115200);

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        //HAL_UART_Transmit(&huart2, rx_buf, 3, 10);
        //HAL_UART_Transmit(&huart2, (unsigned char *) "done", 4, 1000);
        // HAL_Delay(100);
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
    RCC_OscInitStruct.PLL.PLLM = 1;
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

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
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
void extractW(uint8_t rx_buf) {
    //SIGN 1/0 _ _ _ _ _ _
    //updates 0b_ _ , first element being 1(CW) or 0(CCW), and second element being 1(turn) or 0(stop)
    uint8_t w_case = rx_buf >> 6;
    switch (w_case) {
    case 1: //0b01: CCW, rotate
        w_dir = 2;
        pwm_value_w = 150;
        break;
    case 3: //0b11: CW, rotate
        w_dir = 1;
        pwm_value_w = 150;
        break;
    default:
        w_dir = 0;
        pwm_value_w = 0;
        break;
    }
}

void extractX(uint8_t rx_buf) {
    //_ _ SIGN X1 X2 _ _ _
    //updates 0b_ _ _,first element being 1(CW) or 0(CCW), and second/third element being the
    uint8_t x_case = (rx_buf >> 3) & 0b00000111;
    switch (x_case) {
    case 1: //0b001: -dir, 0.1m/s
        x_dir = 2;
        pwm_value_1 = 130;
        break;
    case 2: //0b010: -dir, 0.2m/s
        x_dir = 2;
        pwm_value_1 = 160;
        break;
    case 3: //0b011: -dir, 0.3m/s
        x_dir = 2;
        pwm_value_1 = 200;
        break;
    case 5: //0b101: +dir, 0.1m/s
        x_dir = 1;
        pwm_value_1 = 130;
        break;
    case 6: //0b110: +dir, 0.2m/s
        x_dir = 1;
        pwm_value_1 = 160;
        break;
    case 7: //0b111: +dir, 0.3m/s
        x_dir = 1;
        pwm_value_1 = 200;
        break;
    default:
        pwm_value_1 = 0;
        x_dir = 0;
        break;
    }
}

void extractY(uint8_t rx_buf) {
    //_ _ _ _ _ SIGN Y1 Y2
    uint8_t y_case = rx_buf & 0b00000111;
    switch (y_case) {
    case 1: //0b001: -dir, 0.1m/s
        y_dir = 2;
        pwm_value_2 = 130;
        break;
    case 2: //0b010: -dir, 0.2m/s
        y_dir = 2;
        pwm_value_2 = 160;
        break;
    case 3: //0b011: -dir, 0.3m/s
        y_dir = 2;
        pwm_value_2 = 200;
        break;
    case 5: //0b101: +dir, 0.1m/s
        y_dir = 1;
        pwm_value_2 = 130;
        break;
    case 6: //0b110: +dir, 0.2m/s
        y_dir = 1;
        pwm_value_2 = 160;
        break;
    case 7: //0b111: +dir, 0.3m/s
        y_dir = 1;
        pwm_value_2 = 200;
        break;
    default:
        y_dir = 0;
        pwm_value_2 = 0;
        break;
    }
}

int convertToInt(char *rx) {
    uint8_t hund = rx[0] - '0';
    uint8_t ten = rx[1] - '0';
    uint8_t one = rx[2] - '0';
    int res = hund * 100 + ten * 10 + one;
    return res;
}

/*uint32_t get_pwm(float vel) {
 float abs_vel = vel;
 if (vel < 0) {
 abs_vel = -vel;
 }

 float rps = abs_vel / (WHEEL_DIAMETER * 3.1416);
 uint32_t pwm_value = 130 + (rps / SPEED) * 70;
 return pwm_value;
 }*/

void move_to_pos() {
    if (w_dir == 1) { //CW
        driveMotor(pwm_value_w, 1, MOTOR_COUNTERCLOCKWISE);
        driveMotor(pwm_value_w, 2, MOTOR_COUNTERCLOCKWISE);
        driveMotor(pwm_value_w, 3, MOTOR_COUNTERCLOCKWISE);
        driveMotor(pwm_value_w, 4, MOTOR_COUNTERCLOCKWISE);
    } else if (w_dir == 2) { //CCW
        driveMotor(pwm_value_w, 1, MOTOR_CLOCKWISE);
        driveMotor(pwm_value_w, 2, MOTOR_CLOCKWISE);
        driveMotor(pwm_value_w, 3, MOTOR_CLOCKWISE);
        driveMotor(pwm_value_w, 4, MOTOR_CLOCKWISE);
    } else if (y_dir == 1) {
        driveMotor(pwm_value_2, 1, MOTOR_STOP);
        driveMotor(pwm_value_2, 2, MOTOR_COUNTERCLOCKWISE);
        driveMotor(pwm_value_2, 3, MOTOR_STOP);
        driveMotor(pwm_value_2, 4, MOTOR_CLOCKWISE);
    } else if (y_dir == 2) {
        driveMotor(pwm_value_2, 1, MOTOR_STOP);
        driveMotor(pwm_value_2, 2, MOTOR_CLOCKWISE);
        driveMotor(pwm_value_2, 3, MOTOR_STOP);
        driveMotor(pwm_value_2, 4, MOTOR_COUNTERCLOCKWISE);
    } else if (x_dir == 1) {
        driveMotor(pwm_value_1, 1, MOTOR_COUNTERCLOCKWISE);
        driveMotor(pwm_value_1, 2, MOTOR_STOP);
        driveMotor(pwm_value_1, 3, MOTOR_CLOCKWISE);
        driveMotor(pwm_value_1, 4, MOTOR_STOP);
    } else if (x_dir == 2) {
        driveMotor(pwm_value_1, 1, MOTOR_CLOCKWISE);
        driveMotor(pwm_value_1, 2, MOTOR_STOP);
        driveMotor(pwm_value_1, 3, MOTOR_COUNTERCLOCKWISE);
        driveMotor(pwm_value_1, 4, MOTOR_STOP);
    }
    else {
        driveMotor(pwm_value_w, 1, MOTOR_STOP);
        driveMotor(pwm_value_w, 2, MOTOR_STOP);
        driveMotor(pwm_value_w, 3, MOTOR_STOP);
        driveMotor(pwm_value_w, 4, MOTOR_STOP);
    }
    return;
}

/* setMotor1Dir() takes in the global variable motor1_dir
 * to set or reset two specific output pins to control
 * the direction of motor 1*/
void setMotor1Dir() {
    switch (motor1_dir) {
    case MOTOR_CLOCKWISE:
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        break;
    case MOTOR_COUNTERCLOCKWISE:
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
        break;
    case MOTOR_STOP:
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        break;
    default:
        break;
    }
}

void setMotor2Dir() {
    switch (motor2_dir) {
    case MOTOR_CLOCKWISE:
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
        break;
    case MOTOR_COUNTERCLOCKWISE:
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
        break;
    case MOTOR_STOP:
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
        break;
    default:
        break;
    }
}

void setMotor3Dir() {
    switch (motor3_dir) {
    case MOTOR_CLOCKWISE:
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        break;
    case MOTOR_COUNTERCLOCKWISE:
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        break;
    case MOTOR_STOP:
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    }
}

void setMotor4Dir() {
    switch (motor4_dir) {
    case MOTOR_CLOCKWISE:
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
        break;
    case MOTOR_COUNTERCLOCKWISE:
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
        break;
    case MOTOR_STOP:
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
        break;
    default:
        break;
    }
}

void driveMotor(uint16_t pwm_value, uint8_t motor_num, uint16_t dir) {
//Set PWM values and Motor Directions
    switch (motor_num) {
    case 1:
        motor1_dir = dir;
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwm_value);
        break;
    case 2:
        motor2_dir = dir;
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, pwm_value);
        break;
    case 3:
        motor3_dir = dir;
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, pwm_value);
        break;
    case 4:
        motor4_dir = dir;
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, pwm_value);
        break;
    default:
        break;
    }
}

double extractAngle(uint8_t rx_buf){
	double angle = ((double)rx_buf/(double)255.00)*(double)300.00;
	return angle;

}

void joint_horizontal_control(double angle, uint8_t joint_id){ //joint_id = 10
	//MX_USART2_UART_Init();
	Dynamixel_Init(&motorAX_hor, joint_id, &huart2, Motor_GPIO_Port,
				Motor1_Pin, AX12ATYPE);
	Dynamixel_SetGoalPosition(&motorAX_hor, angle);
}

void joint_vertical_control(double angle, uint8_t joint_id){ //joint_id = 16
    Dynamixel_Init(&motorAX_vert, joint_id, &huart2, Motor_GPIO_Port,
                    Motor2_Pin, AX12ATYPE);
    Dynamixel_SetGoalPosition(&motorAX_vert, angle);
}

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
