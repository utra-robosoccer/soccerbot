/**
  *****************************************************************************
  * @file    MPU6050.cpp
  * @author  Izaak
  * @author  Tyler
  * @author  Jenny
  * @brief   All functions related to the MPU6060 IMU sensor.
  *
  * @defgroup MPU6050 MPU6050

  *****************************************************************************
  */

/********************************* Includes **********************************/
#include "MPU6050.h"
#include <math.h>
#include <stdio.h>


/********************************** Macros ***********************************/

/********************************** Anonymous Namespace **********************/
namespace {

/********************************** Local Helpers ****************************/

}

/********************************** Primary Namespace ************************/
using namespace MPU6050;


/********************************* Constants *********************************/

const float g = 9.81;


/********************************** Types ************************************/




/****************************** Public Variables *****************************/




/***************************** Private Variables *****************************/

    uint8_t                 _ID;                    /*!< Sensor identification (0-252)                  */
    uint32_t                _BaudRate;              /*!< UART communication baud rate*/
    uint8_t                 _Sample_Rate;
    UART_HandleTypeDef*     _UART_Handle;
    I2C_HandleTypeDef*      _I2C_Handle;
    float                   _X_GYRO;            /*!< x-axis angular velocity read from sensor*/
    float                   _Y_GYRO;            /*!< y-axis angular velocity read from sensor*/
    float                   _Z_GYRO;            /*!< z-axis angular velocity read from sensor*/
    float                   _X_ACCEL;           /*!< x-axis acceleration read from sensor*/
    float                   _Y_ACCEL;           /*!< y-axis acceleration read from sensor*/
    float                   _Z_ACCEL;           /*!< z-axis acceleration read from sensor*/

    //offsets:
    float                   _X_GYRO_OFFSET;
    float                   _Y_GYRO_OFFSET;
    float                   _Z_GYRO_OFFSET;
    float                   _X_ACCEL_OFFSET;
    float                   _Y_ACCEL_OFFSET;
    float                   _Z_ACCEL_OFFSET;

    //angles in degrees (calculated using _Z_ACCEL_OFFSET)
    //see page 10 of https://www.nxp.com/docs/en/application-note/AN3461.pdf

    float                   _ROLL;
    float                   _PITCH;


    uint8_t                 received_byte;
/************************ Private Function Prototypes ************************/




/******************************** Functions **********************************/

int MPU6050::Write_Reg(uint8_t reg_addr, uint8_t data){
    return HAL_I2C_Mem_Write(this -> _I2C_Handle, (uint16_t) MPU6050_ADDR, (uint16_t) reg_addr, 1, &data, 1, 10);
}

uint8_t MPU6050::Read_Reg(uint8_t reg_addr){
    uint8_t status = HAL_I2C_Mem_Read(this -> _I2C_Handle,(uint16_t) MPU6050_ADDR,(uint16_t) reg_addr, 1,  this &received_byte, 1,1000);
    return status;
}

BaseType_t MPU6050::Read_Data_IT(uint8_t Reg_addr, uint8_t* sensor_buffer){
	return HAL_I2C_Mem_Read_IT(this -> _I2C_Handle ,(uint16_t) MPU6050_ADDR,(uint16_t) Reg_addr, 1 , sensor_buffer, 6);
}

uint8_t MPU6050::Read_Data(uint8_t Reg_addr, uint8_t* sensor_buffer){
	uint8_t status = HAL_I2C_Mem_Read(this -> _I2C_Handle ,(uint16_t) MPU6050_ADDR,(uint16_t) Reg_addr, 1 , sensor_buffer, 6,1000);
	return status
}

void MPU6050::Read_Gyroscope_Withoffset(){
	uint8_t output_buffer[6];
	MPU6050::Read_Data(MPU6050_RA_GYRO_XOUT_H,output_buffer);
    int16_t X = ((int16_t)(output_buffer[0]<<8|output_buffer[1]));
    int16_t Y = ((int16_t)(output_buffer[2]<<8|output_buffer[3]));
    int16_t Z = ((int16_t)(output_buffer[4]<<8|output_buffer[5]));

    this ->_X_GYRO = (float)X/IMU_GY_RANGE-(this ->_X_GYRO_OFFSET);
    this ->_Y_GYRO = (float)Y/IMU_GY_RANGE-(this ->_Y_GYRO_OFFSET);
    this ->_Z_GYRO = (float)Z/IMU_GY_RANGE-(this ->_Z_GYRO_OFFSET);
}

void MPU6050::Read_Gyroscope_Withoffset_IT(){
    uint8_t output_buffer[6];
    uint32_t notification;
    BaseType_t status;

    if(MPU6050::Read_Data_IT(MPU6050_RA_GYRO_XOUT_H,output_buffer) != HAL_OK){
        // Try fix for flag bit silicon bug
        generateClocks(1, 1);
        return;
    }

    do{
        status = xTaskNotifyWait(0, NOTIFIED_FROM_RX_ISR, &notification, MAX_DELAY_TIME);
        if(status != pdTRUE){
            return;
        }
    }while((notification & NOTIFIED_FROM_RX_ISR) != NOTIFIED_FROM_RX_ISR);

    int16_t X = ((int16_t)(output_buffer[0]<<8|output_buffer[1]));
    int16_t Y = ((int16_t)(output_buffer[2]<<8|output_buffer[3]));
    int16_t Z = ((int16_t)(output_buffer[4]<<8|output_buffer[5]));


    this ->_X_GYRO = (float)X/IMU_GY_RANGE-(this ->_X_GYRO_OFFSET);
    this ->_Y_GYRO = (float)Y/IMU_GY_RANGE-(this ->_Y_GYRO_OFFSET);
    this ->_Z_GYRO = (float)Z/IMU_GY_RANGE-(this ->_Z_GYRO_OFFSET);
}

void MPU6050::Read_Accelerometer_Withoffset(){

    uint8_t output_buffer[6];
    MPU6050:Read_Data(MPU6050_RA_ACCEL_XOUT_H,output_buffer);
    int16_t X_A = (int16_t)(output_buffer[0]<<8|output_buffer[1]);
    int16_t Y_A = (int16_t)(output_buffer[2]<<8|output_buffer[3]);
    int16_t Z_A  = (int16_t)(output_buffer[4]<<8|output_buffer[5]);

    this ->_X_ACCEL = -( X_A * g / ACC_RANGE-(this ->_X_ACCEL_OFFSET));
    this ->_Y_ACCEL = -(Y_A * g / ACC_RANGE-(this ->_Y_ACCEL_OFFSET));
    this ->_Z_ACCEL = -(Z_A * g / ACC_RANGE-(this ->_Z_ACCEL_OFFSET));

    //Now find angles: consult pg 10 of https://www.nxp.com/docs/en/application-note/AN3461.pdf
    // for a sketch of what each angle means
    float X=this -> _X_ACCEL;
    float Y=this -> _Y_ACCEL;
    float Z=this -> _Z_ACCEL;
    float pitch, roll;
    pitch = atan2(Y, Z) * 180/M_PI;
    roll = atan2(-X, sqrt(Y*Y + Z*Z)) * 180/M_PI;
    this ->_ROLL = pitch;
    this ->_PITCH= roll;
}

void MPU6050::Read_Accelerometer_Withoffset_IT(){
    uint8_t output_buffer[6];
    uint32_t notification;
    BaseType_t status;

    if(MPU6050::Read_Data_IT(MPU6050_RA_ACCEL_XOUT_H,output_buffer) != HAL_OK){
        // Try fix for flag bit silicon bug
        generateClocks(1, 1);
        return;
    }

    do{
        status = xTaskNotifyWait(0, NOTIFIED_FROM_RX_ISR, &notification, MAX_DELAY_TIME);
        if(status != pdTRUE){
            return;
        }
    }while((notification & NOTIFIED_FROM_RX_ISR) != NOTIFIED_FROM_RX_ISR);

    int16_t X_A = (int16_t)(output_buffer[0]<<8|output_buffer[1]);
    int16_t Y_A = (int16_t)(output_buffer[2]<<8|output_buffer[3]);
    int16_t Z_A  = (int16_t)(output_buffer[4]<<8|output_buffer[5]);

    this ->_X_ACCEL = -(X_A * g / ACC_RANGE-(this ->_X_ACCEL_OFFSET));
    this ->_Y_ACCEL = -(Y_A * g / ACC_RANGE-(this ->_Y_ACCEL_OFFSET));
    this ->_Z_ACCEL = -(Z_A * g / ACC_RANGE-(this ->_Z_ACCEL_OFFSET));

    //Now find angles: consult pg 10 of https://www.nxp.com/docs/en/application-note/AN3461.pdf
    // for a sketch of what each angle means
    float X=this -> _X_ACCEL;
    float Y=this -> _Y_ACCEL;
    float Z=this -> _Z_ACCEL;
    this ->_ROLL = atan2(Y, Z) * 180/M_PI;
    this ->_PITCH= atan2(-X, sqrt(Y*Y + Z*Z)) * 180/M_PI;
}

int MPU6050::Set_LPF(uint8_t lpf){
    // the LPF reg is decimal 26
    uint8_t current_value;
    uint8_t bitmask=7; // 00000111

    MPU6050::Read_Reg(26); //read the current value of the LPF reg, and save it to received_byte
    current_value=this->received_byte;
    //note that this reg also contains unneeded fsync data which should be preserved

    current_value = (current_value & (~bitmask)) | lpf;
    int status=MPU6050::Write_Reg(26, current_value);
    return status;
}

void MPU6050::Manually_Set_Offsets(){

    this -> _X_ACCEL_OFFSET= (-714.25 * g / ACC_RANGE);
    this -> _Y_ACCEL_OFFSET= (-767.5 * g / ACC_RANGE);
    this -> _Z_ACCEL_OFFSET= ((16324 * g / ACC_RANGE) - 9.81);

    this -> _X_GYRO_OFFSET= (float)240/IMU_GY_RANGE;
    this -> _Y_GYRO_OFFSET= (float)-760/IMU_GY_RANGE;
    this -> _Z_GYRO_OFFSET= (float)-130/IMU_GY_RANGE;
}

void MPU6050::init(){
	MPU6050::Write_Reg(MPU6050_RA_I2C_MST_CTRL, 0b00001101); //0b00001101 is FAST MODE = 400 kHz
	MPU6050::Write_Reg(MPU6050_RA_ACCEL_CONFIG, 0);
	MPU6050::Write_Reg(MPU6050_RA_GYRO_CONFIG, 0);
	MPU6050::Write_Reg(MPU6050_RA_PWR_MGMT_1, 0);
	MPU6050::Write_Reg(MPU6050_RA_PWR_MGMT_2, 0);
	MPU6050::Write_Reg(MPU6050_RA_SMPLRT_DIV, MPU6050_CLOCK_DIV_296);
    this -> _Sample_Rate = 8000/ (1+ MPU6050_CLOCK_DIV_296);
    this -> _X_ACCEL_OFFSET=0;
    this -> _Y_ACCEL_OFFSET=0;
    this -> _Z_ACCEL_OFFSET=0;

    this -> _X_GYRO_OFFSET=0;
    this -> _Y_GYRO_OFFSET= 0;
    this -> _Z_GYRO_OFFSET= 0;
}
