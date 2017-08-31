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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "MPU-REGS.h"
#include <stdlib.h>
#include <stdio.h>
#define printf(x)	HAL_UART_Transmit(&huart2,x,sizeof(x),10)
#define printnl(a)	printf(a); \
					printf("\n\r")
#define INT_COEF 16384.0f
#define REM_COEF 16384

uint16_t total_count;

uint8_t Acc_X, Acc_Y, Acc_Z;
float acc_X;
int Gyro_X, Gyro_Y, Gyro_Z;
int Rem_X_Accel,Rem_Y_Accel,Rem_Z_Accel;
int Rem_X_Gyro,Rem_Y_Gyro,Rem_Z_Gyro;
char Sign_X_Accel, Sign_Y_Accel, Sign_Z_Accel;
char Sign_X_Gyro, Sign_Y_Gyro, Sign_Z_Gyro;


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
void MPU6050_READ_DATA(uint8_t Reg_addr,uint8_t* sensor_buffer){
	uint8_t status = HAL_I2C_Mem_Read(&hi2c3,(uint16_t) MPU6050_ADDR,(uint16_t) Reg_addr, 1 , sensor_buffer, 6,1000);

}

void MPU6050_WRITE_REG(uint8_t reg_addr, uint8_t data){
	HAL_I2C_Mem_Write(&hi2c3, (uint16_t) MPU6050_ADDR, (uint16_t) reg_addr, 1, &data, 1, 10);
}

void print_8bit(uint8_t eightbit){
	for(int i = 7; i >= 0; i--)
			  HAL_UART_Transmit(&huart2,(((eightbit >> i) & 0b01) ? ("1") : ("0")) ,1, 10);
			  HAL_UART_Transmit(&huart2,"\n\r" ,2 , 10);
}

void MPU6050_READ_REG(uint8_t reg_addr){
	uint8_t receivebyte;
	uint8_t status = HAL_I2C_Mem_Read(&hi2c3,(uint16_t) MPU6050_ADDR,(uint16_t) reg_addr, 1,  &receivebyte, 1,1000);
	print_8bit(receivebyte);
}

void MPU6050_init(){
	MPU6050_WRITE_REG(MPU6050_RA_ACCEL_CONFIG, 0);
	MPU6050_WRITE_REG(MPU6050_RA_GYRO_CONFIG, 1<<3);
	MPU6050_WRITE_REG(MPU6050_RA_PWR_MGMT_1, 0);
	MPU6050_WRITE_REG(MPU6050_RA_PWR_MGMT_2, 0);
	MPU6050_WRITE_REG(MPU6050_RA_SMPLRT_DIV, 249);
}

void RESET_FIFO(){
	MPU6050_WRITE_REG(MPU6050_RA_USER_CTRL, 1<<MPU6050_USERCTRL_FIFO_RESET_BIT);
}

void set_Accel_Gyro_FIFO_Enabled(){

	MPU6050_WRITE_REG(MPU6050_RA_FIFO_EN, MPU6050_XG_FIFO_EN_BIT | 1<<MPU6050_YG_FIFO_EN_BIT | 1<<MPU6050_ZG_FIFO_EN_BIT | 1<<MPU6050_ACCEL_FIFO_EN_BIT);
	MPU6050_WRITE_REG(MPU6050_RA_USER_CTRL, 1<<6);
}

void clear_int(){
	MPU6050_WRITE_REG(MPU6050_RA_INT_ENABLE, 0);
}

void data_ready_int(){
	MPU6050_WRITE_REG(MPU6050_RA_INT_ENABLE, 1);
}

void get_val_gyro(){
	Read_Gyroscope();
	print_Angular_Velocity();
}


void print_Angular_Velocity(){
	char buffer_X[20];
	char buffer_Y[20];
	char buffer_Z[20];
	char buffer_remx[20];
	char buffer_remy[20];
	char buffer_remz[20];


	itoa(Gyro_X, buffer_X, 10);
	itoa(Gyro_Y, buffer_Y, 10);
	itoa(Gyro_Z, buffer_Z, 10);
	itoa(Rem_X_Gyro, buffer_remx, 10);
	itoa(Rem_Y_Gyro, buffer_remy, 10);
	itoa(Rem_Z_Gyro, buffer_remz, 10);

	HAL_UART_Transmit(&huart2, &Sign_X_Gyro ,1, 10);
	HAL_UART_Transmit(&huart2, buffer_X ,3, 10);
	HAL_UART_Transmit(&huart2,"." ,1 , 10);
	HAL_UART_Transmit(&huart2,buffer_remx,1 , 10);
	HAL_UART_Transmit(&huart2,"    " ,4 , 10);

	HAL_UART_Transmit(&huart2, &Sign_Y_Gyro ,1, 10);
	HAL_UART_Transmit(&huart2, buffer_Y ,3, 10);
	HAL_UART_Transmit(&huart2,"." ,1 , 10);
	HAL_UART_Transmit(&huart2,buffer_remy,1 , 10);
	HAL_UART_Transmit(&huart2,"    " ,4 , 10);

	HAL_UART_Transmit(&huart2, &Sign_Z_Gyro ,1, 10);
	HAL_UART_Transmit(&huart2, buffer_Z ,3, 10);
	HAL_UART_Transmit(&huart2,"." ,1 , 10);
	HAL_UART_Transmit(&huart2,buffer_remz,1, 10);
	HAL_UART_Transmit(&huart2,"    " ,4 , 10);
	HAL_UART_Transmit(&huart2,"\n\r" ,2 , 10);

}


void print_Acceleration(){
	char buffer_X[20];
	char buffer_Y[20];
	char buffer_Z[20];
	char buffer_remx[20];
	char buffer_remy[20];
	char buffer_remz[20];

	itoa(Acc_X, buffer_X, 10);
	itoa(Acc_Y, buffer_Y, 10);
	itoa(Acc_Z, buffer_Z, 10);
	itoa(Rem_X_Accel, buffer_remx, 10);
	itoa(Rem_Y_Accel, buffer_remy, 10);
	itoa(Rem_Z_Accel, buffer_remz, 10);

	HAL_UART_Transmit(&huart2, &Sign_X_Accel ,1, 10);
	HAL_UART_Transmit(&huart2, buffer_X ,3, 10);
	HAL_UART_Transmit(&huart2,"." ,1 , 10);
	HAL_UART_Transmit(&huart2,buffer_remx,3 , 10);
	HAL_UART_Transmit(&huart2,"    " ,4 , 10);

	HAL_UART_Transmit(&huart2, &Sign_Y_Accel ,1, 10);
	HAL_UART_Transmit(&huart2, buffer_Y ,3, 10);
	HAL_UART_Transmit(&huart2,"." ,1 , 10);
	HAL_UART_Transmit(&huart2,buffer_remy,3 , 10);
	HAL_UART_Transmit(&huart2,"    " ,4 , 10);

	HAL_UART_Transmit(&huart2, &Sign_Z_Accel ,1, 10);
	HAL_UART_Transmit(&huart2, buffer_Z ,3, 10);
	HAL_UART_Transmit(&huart2,"." ,1 , 10);
	HAL_UART_Transmit(&huart2,buffer_remz,3, 10);
	HAL_UART_Transmit(&huart2,"    " ,4 , 10);
	HAL_UART_Transmit(&huart2,"\n\r" ,2 , 10);

}
void Read_Gyroscope(){

	uint8_t output_buffer[6];
	MPU6050_READ_DATA(MPU6050_RA_GYRO_XOUT_H,output_buffer);
	uint16_t X = abs((int16_t)(output_buffer[0]<<8|output_buffer[1]));
	uint16_t Y = abs((int16_t)(output_buffer[2]<<8|output_buffer[3]));
	uint16_t Z = abs((int16_t)(output_buffer[4]<<8|output_buffer[5]));
	Sign_X_Gyro = (output_buffer[0] >> 7) ? '-' : '+';
	Sign_Y_Gyro = (output_buffer[2] >> 7) ? '-' : '+';
	Sign_Z_Gyro = (output_buffer[4] >> 7) ? '-' : '+';

	Gyro_X = X/131;
	Gyro_Y = Y/131;
	Gyro_Z = Z/131;
	Rem_X_Gyro = (int)(X % 131)*10;
	Rem_Y_Gyro = (int)(Y % 131)*10;
	Rem_Z_Gyro = (int)(Z % 131)*10;
}

void Read_Accelerometer(){

	uint8_t output_buffer[6];
	uint16_t X = abs((int16_t)(output_buffer[0]<<8|output_buffer[1]));
	uint16_t Y = abs((int16_t)(output_buffer[2]<<8|output_buffer[3]));
	uint16_t Z = abs((int16_t)(output_buffer[4]<<8|output_buffer[5]));
	Sign_X_Accel = (output_buffer[0] >> 7) ? '-' : '+';
	Sign_Y_Accel = (output_buffer[2] >> 7) ? '-' : '+';
	Sign_Z_Accel = (output_buffer[4] >> 7) ? '-' : '+';

	Acc_X = X/INT_COEF;
	acc_X =  X/INT_COEF;;
	Acc_Y = Y/INT_COEF;
	Acc_Z = Z/INT_COEF;
	Rem_X_Accel = (int)(X % REM_COEF)*10;
	Rem_Y_Accel = (int)(Y % REM_COEF)*10;
	Rem_Z_Accel = (int)(Z % REM_COEF)*10;
}

void get_val_Accel(){
	Read_Accelerometer();
    print_Acceleration();

}



void Read_FIFO_REG(uint8_t* buffer_gyro,uint8_t* buffer_accel){
	    // HAL_I2C_Mem_Read(&hi2c3,(uint16_t) 0b11010000,(uint16_t) MPU6050_RA_FIFO_R_W, 1, buffer_gyro , 6 , 100);
	    HAL_I2C_Mem_Read(&hi2c3,(uint16_t) 0b11010000,(uint16_t) MPU6050_RA_FIFO_R_W, 1, &buffer_gyro[0] , 1 , 100);
	    HAL_I2C_Mem_Read(&hi2c3,(uint16_t) 0b11010000,(uint16_t) MPU6050_RA_FIFO_R_W, 1, &buffer_gyro[1] , 1 , 100);
	    HAL_I2C_Mem_Read(&hi2c3,(uint16_t) 0b11010000,(uint16_t) MPU6050_RA_FIFO_R_W, 1, &buffer_gyro[2] , 1 , 100);
	    HAL_I2C_Mem_Read(&hi2c3,(uint16_t) 0b11010000,(uint16_t) MPU6050_RA_FIFO_R_W, 1, &buffer_gyro[3] , 1 , 100);
	    HAL_I2C_Mem_Read(&hi2c3,(uint16_t) 0b11010000,(uint16_t) MPU6050_RA_FIFO_R_W, 1, &buffer_gyro[4] , 1 , 100);
	    HAL_I2C_Mem_Read(&hi2c3,(uint16_t) 0b11010000,(uint16_t) MPU6050_RA_FIFO_R_W, 1, &buffer_gyro[5] , 1 , 100);
	    HAL_I2C_Mem_Read(&hi2c3,(uint16_t) 0b11010000,(uint16_t) MPU6050_RA_FIFO_R_W, 1, buffer_accel[0] ,1 , 100);
	    HAL_I2C_Mem_Read(&hi2c3,(uint16_t) 0b11010000,(uint16_t) MPU6050_RA_FIFO_R_W, 1, buffer_accel[1] ,1 , 100);
	    HAL_I2C_Mem_Read(&hi2c3,(uint16_t) 0b11010000,(uint16_t) MPU6050_RA_FIFO_R_W, 1, buffer_accel[2] ,1 , 100);
	    HAL_I2C_Mem_Read(&hi2c3,(uint16_t) 0b11010000,(uint16_t) MPU6050_RA_FIFO_R_W, 1, buffer_accel[3] ,1 , 100);
	    HAL_I2C_Mem_Read(&hi2c3,(uint16_t) 0b11010000,(uint16_t) MPU6050_RA_FIFO_R_W, 1, buffer_accel[4] ,1 , 100);
	    HAL_I2C_Mem_Read(&hi2c3,(uint16_t) 0b11010000,(uint16_t) MPU6050_RA_FIFO_R_W, 1, buffer_accel[5] ,1 , 100);

	    	uint16_t X = abs((int16_t)(buffer_gyro[0]<<8|buffer_gyro[1]));
	    	uint16_t Y = abs((int16_t)(buffer_gyro[2]<<8|buffer_gyro[3]));
	    	uint16_t Z = abs((int16_t)(buffer_gyro[4]<<8|buffer_gyro[5]));
	    	Sign_X_Gyro = (buffer_gyro[0] >> 7) ? '-' : '+';
	    	Sign_Y_Gyro = (buffer_gyro[2] >> 7) ? '-' : '+';
	    	Sign_Z_Gyro = (buffer_gyro[4] >> 7) ? '-' : '+';

	    	Gyro_X = X/131.0;
	    	Gyro_Y = Y/131.0;
	    	Gyro_Z = Z/131.0;
	    	Rem_X_Gyro = (int)(X % 131)*10;
	    	Rem_Y_Gyro = (int)(Y % 131)*10;
	    	Rem_Z_Gyro = (int)(Z % 131)*10;

	    	X = abs((int16_t)(buffer_accel[0]<<8|buffer_accel[1]));
	    	Y = abs((int16_t)(buffer_accel[2]<<8|buffer_accel[3]));
	    	Z = abs((int16_t)(buffer_accel[4]<<8|buffer_accel[5]));
	    	Sign_X_Accel = (buffer_accel[0] >> 7) ? '-' : '+';
	    	Sign_Y_Accel = (buffer_accel[2] >> 7) ? '-' : '+';
	   		Sign_Z_Accel = (buffer_accel[4] >> 7) ? '-' : '+';

    		Acc_X = X/INT_COEF;
	   		Acc_Y = Y/INT_COEF;
	   		Acc_Z = Z/INT_COEF;
	  		Rem_X_Accel = (int)(X % REM_COEF)*10;
	   		Rem_Y_Accel = (int)(Y % REM_COEF)*10;
	   		Rem_Z_Accel = (int)(Z % REM_COEF)*10;
}

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
  MX_I2C3_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  MPU6050_init();
  RESET_FIFO();
  set_Accel_Gyro_FIFO_Enabled();
  clear_int();


  HAL_Delay(1000);
  /* USER CODE END 2 */
  uint8_t zero =0;
  int j = 0;
  for(;j<50;j++){
	  HAL_Delay(50);
	  HAL_UART_Transmit(&huart2,&zero,1,100);
  }
  zero = 0xFF;
  HAL_UART_Transmit(&huart2,&zero,1,100);
  data_ready_int();
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

//	  HAL_UART_Transmit(&huart2,&zero,1,100);



  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
 *
*/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_1){
		uint8_t output[2];
		MPU6050_READ_DATA(MPU6050_RA_ACCEL_XOUT_H,output);
		HAL_UART_Transmit(&huart2,&output[1],1,100);
		HAL_UART_Transmit(&huart2,&output[0],1,100);

	}
}

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
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
