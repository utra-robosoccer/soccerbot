#include "MPU6050.h"
#include "Freertos.h"
#include <math.h>
#include <stdio.h>


void MPU6050_READ_DATA(MPU6050_HandleTypeDef *sMPU6050, uint8_t Reg_addr, uint8_t* sensor_buffer){
	/* Reads data stored in sensor output registers and stores data into a buffer

	   Parameters: Reg_addr: address of register required to be read from
	   	       sensor_buffer: an 8-bit array used to store sensor output data. Number of bytes aims to be stored in the buffer at a time is selected by
		 		      	  	  the user.
	*/
	uint8_t status = HAL_I2C_Mem_Read(sMPU6050 -> _I2C_Handle ,(uint16_t) MPU6050_ADDR,(uint16_t) Reg_addr, 1 , sensor_buffer, 6,1000);
}


void MPU6050_WRITE_REG(MPU6050_HandleTypeDef *sMPU6050,uint8_t reg_addr, uint8_t data){
	/* Write one-byte to sensor register
	 * Returns: None
	 */
	HAL_I2C_Mem_Write(sMPU6050 -> _I2C_Handle, (uint16_t) MPU6050_ADDR, (uint16_t) reg_addr, 1, &data, 1, 10);
}

uint8_t MPU6050_READ_REG(MPU6050_HandleTypeDef *sMPU6050, uint8_t reg_addr){
	uint8_t receivebyte;
	uint8_t status = HAL_I2C_Mem_Read(sMPU6050 -> _I2C_Handle,(uint16_t) MPU6050_ADDR,(uint16_t) reg_addr, 1,  &receivebyte, 1,1000);
	return receivebyte;
}


void MPU6050_init(MPU6050_HandleTypeDef *sMPU6050){
	MPU6050_WRITE_REG(sMPU6050, MPU6050_RA_ACCEL_CONFIG, 0);
	MPU6050_WRITE_REG(sMPU6050, MPU6050_RA_GYRO_CONFIG, 0);
	MPU6050_WRITE_REG(sMPU6050, MPU6050_RA_PWR_MGMT_1, 0);
	MPU6050_WRITE_REG(sMPU6050, MPU6050_RA_PWR_MGMT_2, 0);
	MPU6050_WRITE_REG(sMPU6050, MPU6050_RA_SMPLRT_DIV, MPU6050_CLOCK_DIV_296);
	sMPU6050 -> _Sample_Rate = 8000/ (1+ MPU6050_CLOCK_DIV_296);
	sMPU6050 -> _X_ACCEL_OFFSET=69.1;
	sMPU6050 -> _Y_ACCEL_OFFSET=0;
	sMPU6050 -> _Z_ACCEL_OFFSET=0;

	sMPU6050 -> _X_GYRO_OFFSET=0;
	sMPU6050 -> _Y_GYRO_OFFSET= 0;
	sMPU6050 -> _Z_GYRO_OFFSET= 0;
}



void MPU6050_RESET_SENSOR_REG(MPU6050_HandleTypeDef *sMPU6050){
	/* Resets the signal paths for all sensors (gyroscopes, accelerometers, and temperature sensor). This operation will also clear the sensor registers.
	   This bit automatically clears to 0 after the reset has been triggered.

	   register address: 6A
	   Return: None */

	MPU6050_WRITE_REG(sMPU6050, MPU6050_RA_USER_CTRL, 1);
}

void MPU6050_manually_set_offsets(MPU6050_HandleTypeDef *sMPU6050){

	sMPU6050 -> _X_ACCEL_OFFSET= -520/((float)IMU_ACC_CONVERSION);
	sMPU6050 -> _Y_ACCEL_OFFSET=-830/((float)IMU_ACC_CONVERSION);
	sMPU6050 -> _Z_ACCEL_OFFSET=16315/((float)IMU_ACC_CONVERSION) - 9.81;

	sMPU6050 -> _X_GYRO_OFFSET= 240/((float)IMU_GY_CONVERSION);
	sMPU6050 -> _Y_GYRO_OFFSET= -760/((float)IMU_GY_CONVERSION);
	sMPU6050 -> _Z_GYRO_OFFSET= -150/((float)IMU_GY_CONVERSION);
}

void MPU6050_Clear_Int(MPU6050_HandleTypeDef *sMPU6050){
	/* Disables all interrupts
	   Register address： 38
	   Returns : None
	 */
	MPU6050_WRITE_REG(sMPU6050, MPU6050_RA_INT_ENABLE, 0);
}



void MPU6050_Data_Ready_Int(MPU6050_HandleTypeDef *sMPU6050){
	/* Enables data ready interrupt
	   Register address： 38
	   Returns : None
	*/
	MPU6050_WRITE_REG(sMPU6050, MPU6050_RA_INT_ENABLE, 1);
}



void MPU6050_Read_Gyroscope(MPU6050_HandleTypeDef *sMPU6050){
	/*Reads output data stored in gyroscope output registers, and converts the data in 2's complement to decimal numbers
	  Returns : None*/
	uint8_t output_buffer[6];
	MPU6050_READ_DATA(sMPU6050, MPU6050_RA_GYRO_XOUT_H,output_buffer);
	uint16_t X = ((int16_t)(output_buffer[0]<<8|output_buffer[1]));
	uint16_t Y = ((int16_t)(output_buffer[2]<<8|output_buffer[3]));
	uint16_t Z = ((int16_t)(output_buffer[4]<<8|output_buffer[5]));
	sMPU6050 ->_X_GYRO = X;
	sMPU6050 ->_Y_GYRO = Y;
	sMPU6050 ->_Z_GYRO = Z;

	/*************The following part modifies outputs for printing prpose**********/
	X = abs((int16_t)(output_buffer[0]<<8|output_buffer[1]));
	Y = abs((int16_t)(output_buffer[2]<<8|output_buffer[3]));
	Z = abs((int16_t)(output_buffer[4]<<8|output_buffer[5]));
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

void MPU6050_Read_Gyroscope_Withoffset(MPU6050_HandleTypeDef *sMPU6050){
	uint8_t output_buffer[6];
	MPU6050_READ_DATA(sMPU6050, MPU6050_RA_GYRO_XOUT_H,output_buffer);
	uint16_t X = ((int16_t)(output_buffer[0]<<8|output_buffer[1]));
	uint16_t Y = ((int16_t)(output_buffer[2]<<8|output_buffer[3]));
	uint16_t Z = ((int16_t)(output_buffer[4]<<8|output_buffer[5]));
	sMPU6050 ->_X_GYRO = X-(sMPU6050 ->_X_GYRO_OFFSET);
	sMPU6050 ->_Y_GYRO = Y-(sMPU6050 ->_Y_GYRO_OFFSET);
	sMPU6050 ->_Z_GYRO = Z-(sMPU6050 ->_Z_GYRO_OFFSET);


}

void MPU6050_Read_Accelerometer_Withoffset(MPU6050_HandleTypeDef *sMPU6050){

	//IMPORTANT: IN THE UPRIGHT POSITION, ALL 6 CALIBRATED POSITIONS SHOULD BE 0
	// (because even though there is some acceleration from gravity, this is just the reference.)

	//Also, this function updates the angles in the struct. It is also important that the offsets
	// are calibrated before calling this since angle conversions use the peak value of the z
	// acceleration, ie. _Z_GYRO_OFFSET

	uint8_t output_buffer[6];
	//Get the raw values
	MPU6050_READ_DATA(sMPU6050, MPU6050_RA_ACCEL_XOUT_H,output_buffer);
	uint16_t X_A = (int16_t)(output_buffer[0]<<8|output_buffer[1]);
	uint16_t Y_A = (int16_t)(output_buffer[2]<<8|output_buffer[3]);
	uint16_t Z_A  = (int16_t)(output_buffer[4]<<8|output_buffer[5]);
	sMPU6050 ->_X_ACCEL = X_A-(sMPU6050 ->_X_ACCEL_OFFSET);
	sMPU6050 ->_Y_ACCEL = Y_A-(sMPU6050 ->_Y_ACCEL_OFFSET);
	sMPU6050 ->_Z_ACCEL = Z_A;//-(sMPU6050 ->_Z_ACCEL_OFFSET);

	//Now find angles: consult pg 10 of https://www.nxp.com/docs/en/application-note/AN3461.pdf
	// for a sketch of what each angle means
	int X= sMPU6050 ->_X_ACCEL;
	int Y=sMPU6050 ->_Y_ACCEL;
	int Z=sMPU6050 -> _Z_ACCEL;
	float pitch, roll;
	pitch = atan2(Y, Z) * 180/M_PI;
	roll = atan2(-X, sqrt(Y*Y + Z*Z)) * 180/M_PI;
	sMPU6050 ->_ROLL = pitch;
	sMPU6050 ->_PITCH= roll;
}

void MPU6050_set_LPF(MPU6050_HandleTypeDef *sMPU6050, uint8_t lpf){
	/* uint8_t lpf : this input is between 0 and 7
	 *
	 * this function allows you to set the value of the LPF manually.
	 * Please see https://cdn.sparkfun.com/datasheets/Sensors/Accelerometers/RM-MPU-6000A.pdf
	 * for descriptions of what each value does.
	 *
	 */
	// the LPF reg is decimal 26

	uint8_t current_value;
	uint8_t bitmask=7; // 00000111

	current_value=MPU6050_READ_REG(sMPU6050, 26); //read the current value of the LPF reg
	//note that this reg also contains unneeded fsync data which should be preserved

	current_value = (current_value & (~bitmask)) | lpf;
	MPU6050_WRITE_REG(sMPU6050, 26, current_value);
}



void MPU6050_10sec_calibration(MPU6050_HandleTypeDef *sMPU6050){
// A user calibration test which lasts for 10 seconds. place the sensor upright (+z upwards)
// currently no timer is implemented, so it is not for any fixed duration. In the future
// implement a timed calibration to increase usability.

//NOTE: FOR SOME REASON OFFSETS ARE BEHAVING ODDLY
/*
	int max_iterations=1;
	uint8_t output_buffer[6];
	uint8_t output_buffer2[6];
	uint16_t X_A[max_iterations], Y_A[max_iterations], Z_A[max_iterations];
	uint16_t X[max_iterations], Y[max_iterations], Z[max_iterations];
	uint16_t offsets[6];

	for (int i=0; i<max_iterations; i++){

		//read acceleration values:
		MPU6050_READ_DATA(sMPU6050, MPU6050_RA_ACCEL_XOUT_H,output_buffer);
		X_A[i] = (int16_t)(output_buffer[0]<<8|output_buffer[1]);
		Y_A[i] = (int16_t)(output_buffer[2]<<8|output_buffer[3]);
		Z_A[i]  = (int16_t)(output_buffer[4]<<8|output_buffer[5]);
		osDelay(100);

		//read gyroscope values:
		MPU6050_READ_DATA(sMPU6050, MPU6050_RA_GYRO_XOUT_H,output_buffer2);
		X[i] = ((int16_t)(output_buffer2[0]<<8|output_buffer2[1]));
		Y[i] = ((int16_t)(output_buffer2[2]<<8|output_buffer2[3]));
		Z[i] = ((int16_t)(output_buffer2[4]<<8|output_buffer2[5]));
		osDelay(100);

	}

	//average the values:
	for (int i=0; i<max_iterations; i++){
		offsets[0]+=X_A[i];
		offsets[1]+=Y_A[i];
		offsets[2]+=Z_A[i];
		offsets[3]+=X[i];
		offsets[4]+=Y[i];
		offsets[5]+=Z[i];
	}

	//store the values in the struct:
	sMPU6050 -> _X_ACCEL_OFFSET= offsets[0]/max_iterations;
	sMPU6050 -> _Y_ACCEL_OFFSET= offsets[1]/max_iterations;
	sMPU6050 -> _Z_ACCEL_OFFSET= offsets[2]/max_iterations;
	sMPU6050 -> _X_GYRO_OFFSET= offsets[3]/max_iterations;
	sMPU6050 -> _Y_GYRO_OFFSET= offsets[4]/max_iterations;
	sMPU6050 -> _Z_GYRO_OFFSET= offsets[5]/max_iterations;
	*/


	uint8_t output_buffer[6];
	//Gyro:
	MPU6050_READ_DATA(sMPU6050, MPU6050_RA_GYRO_XOUT_H,output_buffer);
	uint16_t X = ((int16_t)(output_buffer[0]<<8|output_buffer[1]));
	uint16_t Y = ((int16_t)(output_buffer[2]<<8|output_buffer[3]));
	uint16_t Z = ((int16_t)(output_buffer[4]<<8|output_buffer[5]));

	sMPU6050 -> _X_GYRO_OFFSET=X;
	sMPU6050 -> _Y_GYRO_OFFSET=Y;
	sMPU6050 -> _Z_GYRO_OFFSET=Z;

	//Accel:
	MPU6050_READ_DATA(sMPU6050, MPU6050_RA_ACCEL_XOUT_H,output_buffer);
	uint16_t X_A = (int16_t)(output_buffer[0]<<8|output_buffer[1]);
	uint16_t Y_A = (int16_t)(output_buffer[2]<<8|output_buffer[3]);
	uint16_t Z_A  = (int16_t)(output_buffer[4]<<8|output_buffer[5]);

	sMPU6050 -> _X_ACCEL_OFFSET=X_A;
	sMPU6050 -> _Y_ACCEL_OFFSET=Y_A;
	sMPU6050 -> _Z_ACCEL_OFFSET=Z_A;

}


void MPU6050_Read_Accelerometer(MPU6050_HandleTypeDef *sMPU6050){
	/*Reads output data stored in accelerometer output registers, and converts the data in 2's complement to decimal numbers
	  Returns : None*/
	uint8_t output_buffer[6];
	MPU6050_READ_DATA(sMPU6050, MPU6050_RA_ACCEL_XOUT_H,output_buffer);
	uint16_t X_A = (int16_t)(output_buffer[0]<<8|output_buffer[1]);
	uint16_t Y_A = (int16_t)(output_buffer[2]<<8|output_buffer[3]);
	uint16_t Z_A  = (int16_t)(output_buffer[4]<<8|output_buffer[5]);
	sMPU6050 ->_X_ACCEL = X_A;
	sMPU6050 ->_Y_ACCEL = Y_A;
	sMPU6050 ->_Z_ACCEL = Z_A;

	/*************The following part modifies outputs for printing prpose**********/
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


//I am not sure what this next function does:
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	/*This function is called automatically when a interrupt is generated. Any instructions from the user given a specific interrupt is to be written in this function.
	 The following instructions show an example of dealing with a data ready interrupt

	 Parameter：GPIO_Pin: This parameter is automatically inputted by EXTI(X)_HANDLER function and indicates upon which pin the interrupt is generated. By physically
	 connecting interrupt pins on MPU6050 to a pin on microcontroller(requires pre-configuration beforehand), the user will be able to tell if the interrupt is generated
	 by MPU6050 or other devices if possible.

	 Return: None
	*/
		uint8_t output[2];
	/***************for matlab simulation****************************************************************/
		/*
		HAL_I2C_Mem_Read(&hi2c3,(uint16_t) MPU6050_ADDR, MPU6050_RA_GYRO_YOUT_H, 1 , output, 2,1000);
		HAL_UART_Transmit(&huart2,&output[0],1,100);
		HAL_UART_Transmit(&huart2,&output[1],1,100);
		*/
	/*output values from gyroscope or accelerometer */
		/* eg. MPU6050_Get_Val_Gyro(); 	*/

}
