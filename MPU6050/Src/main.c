
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "MPU-REGS.h"
#include <stdlib.h>
#include <stdio.h>
#define printf(x)	HAL_UART_Transmit(&huart2,x,sizeof(x),10)
#define printnl(a)	printf(a); \
					printf("\n\r")
#define INT_COEF 16384.0f
#define REM_COEF 16384

uint16_t TOTAL_COUNT;
uint8_t Acc_X, Acc_Y, Acc_Z;
float acc_X;
int Gyro_X, Gyro_Y, Gyro_Z;
int Rem_X_Accel,Rem_Y_Accel,Rem_Z_Accel;
int Rem_X_Gyro,Rem_Y_Gyro,Rem_Z_Gyro;
char Sign_X_Accel, Sign_Y_Accel, Sign_Z_Accel;
char Sign_X_Gyro, Sign_Y_Gyro, Sign_Z_Gyro;

void SystemClock_Config(void);

/* Reads data stored in sensor output registers and stores data into a buffer
   
   Parameters: Reg_addr: address of register required to be read from
   	       sensor_buffer: 
void MPU6050_READ_DATA(uint8_t Reg_addr, uint8_t* sensor_buffer){
	uint8_t status = HAL_I2C_Mem_Read(&hi2c3,(uint16_t) MPU6050_ADDR,(uint16_t) Reg_addr, 1 , sensor_buffer, 6,1000);
}

/* Write one-byte to sensor register
 * Returns: None
 */
void MPU6050_WRITE_REG(uint8_t reg_addr, uint8_t data){
	HAL_I2C_Mem_Write(&hi2c3, (uint16_t) MPU6050_ADDR, (uint16_t) reg_addr, 1, &data, 1, 10);
}


/* Reads data from registers via I2C3 and prints out the 8-bit data value via USART2
   Return: None 
 */
void MPU6050_READ_REG(uint8_t reg_addr){
	uint8_t receivebyte;
	uint8_t status = HAL_I2C_Mem_Read(&hi2c3,(uint16_t) MPU6050_ADDR,(uint16_t) reg_addr, 1,  &receivebyte, 1,1000);
	print_8bit(receivebyte);
}


/* Initializes registers of sensor control:
   MPU6050_RA_GYRO_CONFIG:  register address: 1B
   		            configuration: Disables self-test mode for gyroscope, and sets gyroscope full scale range to ± 250 °/s
   MPU6050_RA_ACCEL_CONFIG: register address：1C
   		  	    configuration: Disables self-test mode for accelerometer, and sets acceerometer  full scale range to ± 2g
   MPU6050_RA_PWR_MGMT_1:   register address: 6B
   			    configuration: Disables sleep mode, set clock source as internal 8MHz oscillator
   MPU6050_RA_PWR_MGMT_1:   register address: 6C
   			    configuration: Disables standby mode for both sensors
   MPU6050_RA_SMPLRT_DIV:   register address: 19
   			    configuration: Specifies the divider from the gyroscope output rate to generate the Sample Rate for MPU-6050
			    		   Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
   Return: None */
void MPU6050_init(){
	MPU6050_WRITE_REG(MPU6050_RA_ACCEL_CONFIG, 0);
	MPU6050_WRITE_REG(MPU6050_RA_GYRO_CONFIG, 0);
	MPU6050_WRITE_REG(MPU6050_RA_PWR_MGMT_1, 0);
	MPU6050_WRITE_REG(MPU6050_RA_PWR_MGMT_2, 0);
	MPU6050_WRITE_REG(MPU6050_RA_SMPLRT_DIV, 249);
}

/* Resets the signal paths for all sensors (gyroscopes, accelerometers, and temperature sensor). This operation will also clear the sensor registers. This bit automatically clears to 0 after the reset has been triggered.
 *Return: None
*/
void MPU6050_RESET_SENSOR_REG(){
	MPU6050_WRITE_REG(MPU6050_RA_USER_CTRL, 1);
}


/* Disables all interrupts 
   Register address： 38
   Returns : None
 */
void MPU6050_Clear_Int(){
	MPU6050_WRITE_REG(MPU6050_RA_INT_ENABLE, 0);
}


/* Enables data ready interrupt
   Register address： 38
   Returns : None
*/
void MPU6050_Data_Ready_Int(){
	MPU6050_WRITE_REG(MPU6050_RA_INT_ENABLE, 1);
}


/* Reads output data stored in gyroscope output registers and print out angular velocity
    Returns : None
*/
void MPU6050_Get_Val_Gyro(){
	MPU6050_Read_Gyroscope();
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

/*Reads output data stored in gyroscope output registers
  Returns : None*/
void MPU6050_Read_Gyroscope(){

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


/*Reads output data stored in accelerometer output registers and converts the data in 2's complement to decimal numbers
  Returns : None*/
void MPU6050_Read_Accelerometer(){

	uint8_t output_buffer[6];
	MPU6050_READ_DATA(MPU6050_RA_ACCEL_XOUT_H,output_buffer);
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



/*Reads output data stored in accelerometer output registers(in decimal form) and prints out the value via USART2
  Returns: None
 */
void MPU6050_Get_Val_Accel(){
	MPU6050_Read_Accelerometer();
        print_Acceleration();
}


/*This function is called automatically when a interrupt is generated. Any instructions from the user given a specific interrupt is to be written in this function.
 The following instructions show an example of dealing with a data ready interrupt
 
 Parameter：GPIO_Pin: This parameter is automatically inputted by EXTI(X)_HANDLER function and indicates upon which pin the interrupt is generated. By physically 
 connecting interrupt pins on MPU6050 to a pin on microcontroller(requires pre-configuration beforehand), the user will be able to tell if the interrupt is generated
 by MPU6050 or other devices if possible.
 
 Return: None
*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
		uint8_t output[2];
		HAL_I2C_Mem_Read(&hi2c3,(uint16_t) MPU6050_ADDR, MPU6050_RA_GYRO_YOUT_H, 1 , output, 2,1000);
		HAL_UART_Transmit(&huart2,&output[0],1,100);
		HAL_UART_Transmit(&huart2,&output[1],1,100);


}

************************************************ FIFO ******************************************************************************************************************
void MPU6050_Set_Gyro_FIFO_Enabled(){
	//MPU6050_WRITE_REG(MPU6050_RA_FIFO_EN, MPU6050_XG_FIFO_EN_BIT | 1<<MPU6050_YG_FIFO_EN_BIT | 1<<MPU6050_ZG_FIFO_EN_BIT);
	MPU6050_WRITE_REG(MPU6050_RA_FIFO_EN, MPU6050_XG_FIFO_EN_BIT);
	MPU6050_WRITE_REG(MPU6050_RA_USER_CTRL, 1<<6);
}

void set_Accel_FIFO_Enabled(){

	MPU6050_WRITE_REG(MPU6050_RA_FIFO_EN, MPU6050_XG_FIFO_EN_BIT | 1<<MPU6050_YG_FIFO_EN_BIT | 1<<MPU6050_ZG_FIFO_EN_BIT | 1<<MPU6050_ACCEL_FIFO_EN_BIT);
	MPU6050_WRITE_REG(MPU6050_RA_USER_CTRL, 1<<6);
}

void FIFOcount(){
	uint8_t count_H,count_L;
	HAL_I2C_Mem_Read(&hi2c3,(uint16_t) MPU6050_ADDR,(uint16_t) MPU6050_RA_FIFO_COUNTH, 1 , &count_H, 6,1000);
	HAL_I2C_Mem_Read(&hi2c3,(uint16_t) MPU6050_ADDR,(uint16_t) MPU6050_RA_FIFO_COUNTL, 1 , &count_L, 6,1000);

	TOTAL_COUNT=(count_H<<8|count_L);
}
/* Clear FIFO buffer
   Return: None 
 */
void MPU6050_RESET_FIFO(){
	MPU6050_WRITE_REG(MPU6050_RA_USER_CTRL, 1<<MPU6050_USERCTRL_FIFO_RESET_BIT);
}
void MPU6050_Read_FIFO_REG(uint8_t* buffer_gyro,uint8_t* buffer_accel){
	    // HAL_I2C_Mem_Read(&hi2c3,(uint16_t) 0b11010000,(uint16_t) MPU6050_RA_FIFO_R_W, 1, buffer_gyro , 6 , 100);
	    HAL_I2C_Mem_Read(&hi2c3,(uint16_t) 0b11010000,(uint16_t) MPU6050_RA_FIFO_R_W, 1, &buffer_gyro[0] , 1 , 100);
	    HAL_I2C_Mem_Read(&hi2c3,(uint16_t) 0b11010000,(uint16_t) MPU6050_RA_FIFO_R_W, 1, &buffer_gyro[1] , 1 , 100);
	    HAL_I2C_Mem_Read(&hi2c3,(uint16_t) 0b11010000,(uint16_t) MPU6050_RA_FIFO_R_W, 1, &buffer_gyro[2] , 1 , 100);
	    HAL_I2C_Mem_Read(&hi2c3,(uint16_t) 0b11010000,(uint16_t) MPU6050_RA_FIFO_R_W, 1, &buffer_gyro[3] , 1 , 100);
	    HAL_I2C_Mem_Read(&hi2c3,(uint16_t) 0b11010000,(uint16_t) MPU6050_RA_FIFO_R_W, 1, &buffer_gyro[4] , 1 , 100);
	    HAL_I2C_Mem_Read(&hi2c3,(uint16_t) 0b11010000,(uint16_t) MPU6050_RA_FIFO_R_W, 1, &buffer_gyro[5] , 1 , 100);

	    HAL_I2C_Mem_Read(&hi2c3,(uint16_t) 0b11010000,(uint16_t) MPU6050_RA_FIFO_R_W, 1, &buffer_accel[0] ,1 , 100);
	    HAL_I2C_Mem_Read(&hi2c3,(uint16_t) 0b11010000,(uint16_t) MPU6050_RA_FIFO_R_W, 1, &buffer_accel[1] ,1 , 100);
	    HAL_I2C_Mem_Read(&hi2c3,(uint16_t) 0b11010000,(uint16_t) MPU6050_RA_FIFO_R_W, 1, &buffer_accel[2] ,1 , 100);
	    HAL_I2C_Mem_Read(&hi2c3,(uint16_t) 0b11010000,(uint16_t) MPU6050_RA_FIFO_R_W, 1, &buffer_accel[3] ,1 , 100);
	    HAL_I2C_Mem_Read(&hi2c3,(uint16_t) 0b11010000,(uint16_t) MPU6050_RA_FIFO_R_W, 1, &buffer_accel[4] ,1 , 100);
	    HAL_I2C_Mem_Read(&hi2c3,(uint16_t) 0b11010000,(uint16_t) MPU6050_RA_FIFO_R_W, 1, &buffer_accel[5] ,1 , 100);

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

