/**
  *****************************************************************************
  * @file    MPU6050.c
  * @author  Izaak
  * @author  Tyler
  * @author  Jenny
  * @brief   This
  *
  * @defgroup MPU6050 MPU6050
  * @brief    All functions related to the MPU6060 IMU sensor.
  *****************************************************************************
  */

/********************************* Includes **********************************/
#include "MPU6050.h"
#include <math.h>
#include <stdio.h>

/********************************* Constants *********************************/
const float g = 9.81;

/******************************** Functions **********************************/
/**
 * @defgroup Init Init
 * @brief    These functions are for initialization of the MPU6050
 *
 * @ingroup  MPU6050
 */

/**
  * @brief   This function is used as the first step of initialization of an
  * 		 MPU6050_HandleTypeDef struct.
  * @param   *sMPU6050 Pointer to a struct of type MPU6050_HandleTypeDef
  * @ingroup Init
  * @return  None
  */
void MPU6050_init(MPU6050_HandleTypeDef *sMPU6050){
    MPU6050_WRITE_REG(sMPU6050, MPU6050_RA_I2C_MST_CTRL, 0b00001101); //0b00001101 is FAST MODE = 400 kHz
    MPU6050_WRITE_REG(sMPU6050, MPU6050_RA_ACCEL_CONFIG, 0);
    MPU6050_WRITE_REG(sMPU6050, MPU6050_RA_GYRO_CONFIG, 0);
    MPU6050_WRITE_REG(sMPU6050, MPU6050_RA_PWR_MGMT_1, 0);
    MPU6050_WRITE_REG(sMPU6050, MPU6050_RA_PWR_MGMT_2, 0);
    MPU6050_WRITE_REG(sMPU6050, MPU6050_RA_SMPLRT_DIV, MPU6050_CLOCK_DIV_296);
    sMPU6050 -> _Sample_Rate = 8000/ (1+ MPU6050_CLOCK_DIV_296);
    sMPU6050 -> _X_ACCEL_OFFSET=0;
    sMPU6050 -> _Y_ACCEL_OFFSET=0;
    sMPU6050 -> _Z_ACCEL_OFFSET=0;

    sMPU6050 -> _X_GYRO_OFFSET=0;
    sMPU6050 -> _Y_GYRO_OFFSET= 0;
    sMPU6050 -> _Z_GYRO_OFFSET= 0;
}

/**
 * @brief   This function is used to manually set the offsets of the
 *          accelerometer and gyroscope based on empirical measurements
 *          taken for this specific sensor. In the future, this could
 *          be updated to allow for various MPU6050 devices, each with
 *          their own specific offsets.
 * @param   *sMPU6050 Pointer to a struct of type MPU6050_HandleTypeDef
 * @ingroup Init
 * @return  None
 */
void MPU6050_manually_set_offsets(MPU6050_HandleTypeDef *sMPU6050){

    sMPU6050 -> _X_ACCEL_OFFSET= (-714.25 * g / ACC_RANGE);
    sMPU6050 -> _Y_ACCEL_OFFSET= (-767.5 * g / ACC_RANGE);
    sMPU6050 -> _Z_ACCEL_OFFSET= ((16324 * g / ACC_RANGE) - 9.81);

    sMPU6050 -> _X_GYRO_OFFSET= (float)240/IMU_GY_RANGE;
    sMPU6050 -> _Y_GYRO_OFFSET= (float)-760/IMU_GY_RANGE;
    sMPU6050 -> _Z_GYRO_OFFSET= (float)-130/IMU_GY_RANGE;
}

/**
 * @brief   This function allows you to set the value of the LPF manually.
 *          Please see https://cdn.sparkfun.com/datasheets/Sensors/Accelerometers/RM-MPU-6000A.pdf
 *          for descriptions of what each value does.
 * @param   *sMPU6050 Pointer to a struct of type MPU6050_HandleTypeDef
 * @param   lpf uint8_t between 0 and 7 inclusive
 * @ingroup Init
 * @return  None
 */
void MPU6050_set_LPF(MPU6050_HandleTypeDef *sMPU6050, uint8_t lpf){

    // the LPF reg is decimal 26

    uint8_t current_value;
    uint8_t bitmask=7; // 00000111

    current_value=MPU6050_READ_REG(sMPU6050, 26); //read the current value of the LPF reg
    //note that this reg also contains unneeded fsync data which should be preserved

    current_value = (current_value & (~bitmask)) | lpf;
    MPU6050_WRITE_REG(sMPU6050, 26, current_value);
}

/**
 * @defgroup Accelerometer Accelerometer
 * @brief    These functions are for the MPU6050 accelerometer
 *
 * @ingroup  MPU6050
 */

/**
  * @brief   This function reads the accelerometer using interrupts
  * @param   *sMPU6050 Pointer to a struct of type MPU6050_HandleTypeDef
  * @ingroup Accelerometer
  * @return  None
  */
void MPU6050_Read_Accelerometer_Withoffset_IT(MPU6050_HandleTypeDef *sMPU6050){
    uint8_t output_buffer[6];
    uint32_t notification;
    BaseType_t status;

    if(MPU6050_READ_DATA_IT(sMPU6050, MPU6050_RA_ACCEL_XOUT_H,output_buffer) != HAL_OK){
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

    sMPU6050 ->_X_ACCEL = -(X_A * g / ACC_RANGE-(sMPU6050 ->_X_ACCEL_OFFSET));
    sMPU6050 ->_Y_ACCEL = -(Y_A * g / ACC_RANGE-(sMPU6050 ->_Y_ACCEL_OFFSET));
    sMPU6050 ->_Z_ACCEL = -(Z_A * g / ACC_RANGE-(sMPU6050 ->_Z_ACCEL_OFFSET));

    //Now find angles: consult pg 10 of https://www.nxp.com/docs/en/application-note/AN3461.pdf
    // for a sketch of what each angle means
    float X=sMPU6050 -> _X_ACCEL;
    float Y=sMPU6050 -> _Y_ACCEL;
    float Z=sMPU6050 -> _Z_ACCEL;
    sMPU6050 ->_ROLL  = sMPU6050 ->_ROLL * 0.95 + (atan2(Y, Z) * 180/M_PI) * 0.05;
    sMPU6050 ->_PITCH = sMPU6050 ->_PITCH * 0.95 + (atan2(-X, sqrt(Y*Y + Z*Z)) * 180/M_PI) * 0.05;
}

/**
  * @brief   This function reads the accelerometer without using interrupts
  * @param   *sMPU6050 Pointer to a struct of type MPU6050_HandleTypeDef
  * @ingroup Accelerometer
  * @return  None
  */
void MPU6050_Read_Accelerometer_Withoffset(MPU6050_HandleTypeDef *sMPU6050){

    uint8_t output_buffer[6];
    MPU6050_READ_DATA(sMPU6050, MPU6050_RA_ACCEL_XOUT_H,output_buffer);
    int16_t X_A = (int16_t)(output_buffer[0]<<8|output_buffer[1]);
    int16_t Y_A = (int16_t)(output_buffer[2]<<8|output_buffer[3]);
    int16_t Z_A  = (int16_t)(output_buffer[4]<<8|output_buffer[5]);

    sMPU6050 ->_X_ACCEL = -( X_A * g / ACC_RANGE-(sMPU6050 ->_X_ACCEL_OFFSET));
    sMPU6050 ->_Y_ACCEL = -(Y_A * g / ACC_RANGE-(sMPU6050 ->_Y_ACCEL_OFFSET));
    sMPU6050 ->_Z_ACCEL = -(Z_A * g / ACC_RANGE-(sMPU6050 ->_Z_ACCEL_OFFSET));

    //Now find angles: consult pg 10 of https://www.nxp.com/docs/en/application-note/AN3461.pdf
    // for a sketch of what each angle means
    float X=sMPU6050 -> _X_ACCEL;
    float Y=sMPU6050 -> _Y_ACCEL;
    float Z=sMPU6050 -> _Z_ACCEL;
    float pitch, roll;
    pitch = atan2(Y, Z) * 180/M_PI;
    roll = atan2(-X, sqrt(Y*Y + Z*Z)) * 180/M_PI;
    sMPU6050 ->_ROLL = pitch;
    sMPU6050 ->_PITCH= roll;
}

/**
 * @defgroup Gyroscope Gyroscope
 * @brief    These functions are for the MPU6050 gyroscope
 *
 * @ingroup  MPU6050
 */

/**
  * @brief   This function reads the gyroscope using interrupts
  * @param   *sMPU6050 Pointer to a struct of type MPU6050_HandleTypeDef
  * @ingroup Gyroscope
  * @return  None
  */
void MPU6050_Read_Gyroscope_Withoffset_IT(MPU6050_HandleTypeDef *sMPU6050){
    uint8_t output_buffer[6];
    uint32_t notification;
    BaseType_t status;

    if(MPU6050_READ_DATA_IT(sMPU6050, MPU6050_RA_GYRO_XOUT_H,output_buffer) != HAL_OK){
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


    sMPU6050 ->_X_GYRO = (float)X/IMU_GY_RANGE-(sMPU6050 ->_X_GYRO_OFFSET);
    sMPU6050 ->_Y_GYRO = (float)Y/IMU_GY_RANGE-(sMPU6050 ->_Y_GYRO_OFFSET);
    sMPU6050 ->_Z_GYRO = (float)Z/IMU_GY_RANGE-(sMPU6050 ->_Z_GYRO_OFFSET);
}

/**
  * @brief   This function reads the gyroscope without using interrupts
  * @param   *sMPU6050 Pointer to a struct of type MPU6050_HandleTypeDef
  * @ingroup Gyroscope
  * @return  None
  */
void MPU6050_Read_Gyroscope_Withoffset(MPU6050_HandleTypeDef *sMPU6050){
    uint8_t output_buffer[6];
    MPU6050_READ_DATA(sMPU6050, MPU6050_RA_GYRO_XOUT_H,output_buffer);
    int16_t X = ((int16_t)(output_buffer[0]<<8|output_buffer[1]));
    int16_t Y = ((int16_t)(output_buffer[2]<<8|output_buffer[3]));
    int16_t Z = ((int16_t)(output_buffer[4]<<8|output_buffer[5]));


    sMPU6050 ->_X_GYRO = (float)X/IMU_GY_RANGE-(sMPU6050 ->_X_GYRO_OFFSET);
    sMPU6050 ->_Y_GYRO = (float)Y/IMU_GY_RANGE-(sMPU6050 ->_Y_GYRO_OFFSET);
    sMPU6050 ->_Z_GYRO = (float)Z/IMU_GY_RANGE-(sMPU6050 ->_Z_GYRO_OFFSET);
}
/**
 * @defgroup Register_RW Register_RW
 * @brief    These functions are for reading and writing to
 *           registers on the MPU6050
 *
 * @ingroup  MPU6050
 */

/**
  * @brief   This function reads a register from the MPU6050 without interrupts,
  *          and stores it into a buffer
  * @param   *sMPU6050 Pointer to a struct of type MPU6050_HandleTypeDef
  * @param   reg_addr uint8_t address of the register
  * @param   sensor_buffer uint8_t* pointer to output buffer
  * @ingroup Register_RW
  * @return  status
  */
void MPU6050_READ_DATA(MPU6050_HandleTypeDef *sMPU6050, uint8_t Reg_addr, uint8_t* sensor_buffer){
    /* Reads data stored in sensor output registers and stores data into a buffer

       Parameters: Reg_addr: address of register required to be read from
               sensor_buffer: an 8-bit array used to store sensor output data. Number of bytes aims to be stored in the buffer at a time is selected by
                              the user.
    */
    uint8_t status = HAL_I2C_Mem_Read(sMPU6050 -> _I2C_Handle ,(uint16_t) MPU6050_ADDR,(uint16_t) Reg_addr, 1 , sensor_buffer, 6,1000);
}

/**
  * @brief   This function reads a register from the MPU6050 with interrupts,
  *          and stores it into a buffer
  * @param   *sMPU6050 Pointer to a struct of type MPU6050_HandleTypeDef
  * @param   reg_addr uint8_t address of the register
  * @param   sensor_buffer uint8_t* pointer to output buffer
  * @ingroup Register_RW
  * @return  status
  */
BaseType_t MPU6050_READ_DATA_IT(MPU6050_HandleTypeDef *sMPU6050, uint8_t Reg_addr, uint8_t* sensor_buffer){
    /* Reads data stored in sensor output registers and stores data into a buffer using IT

       Parameters: Reg_addr: address of register required to be read from
               sensor_buffer: an 8-bit array used to store sensor output data. Number of bytes aims to be stored in the buffer at a time is selected by
                              the user.
    */

    return HAL_I2C_Mem_Read_IT(sMPU6050 -> _I2C_Handle ,(uint16_t) MPU6050_ADDR,(uint16_t) Reg_addr, 1 , sensor_buffer, 6);
}

/**
  * @brief   This function writes to a register from the MPU6050
  * @param   *sMPU6050 Pointer to a struct of type MPU6050_HandleTypeDef
  * @param   reg_addr uint8_t address of the register
  * @param   data uint8_t data to be written
  * @ingroup Register_RW
  * @return  status
  */
void MPU6050_WRITE_REG(MPU6050_HandleTypeDef *sMPU6050,uint8_t reg_addr, uint8_t data){
    HAL_I2C_Mem_Write(sMPU6050 -> _I2C_Handle, (uint16_t) MPU6050_ADDR, (uint16_t) reg_addr, 1, &data, 1, 10);
}

/**
  * @brief   This function reads from an MPU6050 register and returns it directly
  * @param   *sMPU6050 Pointer to a struct of type MPU6050_HandleTypeDef
  * @param   reg_addr uint8_t address of the register
  * @ingroup Register_RW
  * @return  Byte of data that was stored in the register
  */
uint8_t MPU6050_READ_REG(MPU6050_HandleTypeDef *sMPU6050, uint8_t reg_addr){
    uint8_t receivebyte;
    uint8_t status = HAL_I2C_Mem_Read(sMPU6050 -> _I2C_Handle,(uint16_t) MPU6050_ADDR,(uint16_t) reg_addr, 1,  &receivebyte, 1,1000);
    return receivebyte;
}



void MPU6050_RESET_SENSOR_REG(MPU6050_HandleTypeDef *sMPU6050){
    /* Resets the signal paths for all sensors (gyroscopes, accelerometers, and temperature sensor). This operation will also clear the sensor registers.
       This bit automatically clears to 0 after the reset has been triggered.

       register address: 6A
       Return: None */

    MPU6050_WRITE_REG(sMPU6050, MPU6050_RA_USER_CTRL, 1);
}


void MPU6050_Clear_Int(MPU6050_HandleTypeDef *sMPU6050){
    /* Disables all interrupts
       Register address 38
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
    int16_t X = ((int16_t)(output_buffer[0]<<8|output_buffer[1]));
    int16_t Y = ((int16_t)(output_buffer[2]<<8|output_buffer[3]));
    int16_t Z = ((int16_t)(output_buffer[4]<<8|output_buffer[5]));
    sMPU6050 ->_X_GYRO = X;
    sMPU6050 ->_Y_GYRO = Y;
    sMPU6050 ->_Z_GYRO = Z;

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
        /* eg. MPU6050_Get_Val_Gyro();  */

}


// Note: The following 2 functions are used as a workaround for an issue where the BUSY flag of the
// I2C module is erroneously asserted in the hardware (a silicon bug, essentially). This workaround has
// not been thoroughly tested.
//
// Overall, use these functions with EXTREME caution.
static uint8_t wait_for_gpio_state_timeout(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state, uint8_t timeout){
    /* Helper function for I2C_ClearBusyFlagErratum.
     *
     * Arguments: none
     *
     * Returns: none
     */

    uint32_t Tickstart = HAL_GetTick();
    uint8_t ret = 0;
    /* Wait until flag is set */
    while((state != HAL_GPIO_ReadPin(port, pin)) && (1 == ret)){
        /* Check for the timeout */
        if ((timeout == 0U) || (HAL_GetTick() - Tickstart >= timeout)){
            ret = 0;
        }
        asm("nop");
    }
    return ret;
}

void generateClocks(uint8_t numClocks, uint8_t sendStopBits){
    /* This function big-bangs the I2C master clock
     *
     * https://electronics.stackexchange.com/questions/267972/i2c-busy-flag-strange-behaviour/281046#281046
     * https://community.st.com/thread/35884-cant-reset-i2c-in-stm32f407-to-release-i2c-lines
     * https://electronics.stackexchange.com/questions/272427/stm32-busy-flag-is-set-after-i2c-initialization
     * http://www.st.com/content/ccc/resource/technical/document/errata_sheet/f5/50/c9/46/56/db/4a/f6/CD00197763.pdf/files/CD00197763.pdf/jcr:content/translations/en.CD00197763.pdf
     *
     *
     * Arguments: numClocks, the number of times to cycle the I2C master clock
     *            sendStopBits, 1 if stop bits are to be sent on SDA
     *
     * Returns: none
     */

    static struct I2C_Module{
        I2C_HandleTypeDef*   instance;
        uint16_t            sdaPin;
        GPIO_TypeDef*       sdaPort;
        uint16_t            sclPin;
        GPIO_TypeDef*       sclPort;
    }i2cmodule = {&hi2c1, GPIO_PIN_7, GPIOB, GPIO_PIN_6, GPIOB};
    static struct I2C_Module* i2c = &i2cmodule;
    static uint8_t timeout = 1;

    GPIO_InitTypeDef GPIO_InitStructure;

    I2C_HandleTypeDef* handler = NULL;

    handler = i2c->instance;

    // 1. Clear PE bit.
    CLEAR_BIT(handler->Instance->CR1, I2C_CR1_PE);

    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    GPIO_InitStructure.Pin = i2c->sclPin;
    HAL_GPIO_Init(i2c->sclPort, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = i2c->sdaPin;
    HAL_GPIO_Init(i2c->sdaPort, &GPIO_InitStructure);

    for(uint8_t i = 0; i < numClocks; i++){
        // 3. Check SCL and SDA High level in GPIOx_IDR.
        if(sendStopBits){HAL_GPIO_WritePin(i2c->sdaPort, i2c->sdaPin, GPIO_PIN_SET);}
        HAL_GPIO_WritePin(i2c->sclPort, i2c->sclPin, GPIO_PIN_SET);

        wait_for_gpio_state_timeout(i2c->sclPort, i2c->sclPin, GPIO_PIN_SET, timeout);
        if(sendStopBits){wait_for_gpio_state_timeout(i2c->sdaPort, i2c->sdaPin, GPIO_PIN_SET, timeout);}

        // 4. Configure the SDA I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
        if(sendStopBits){
            HAL_GPIO_WritePin(i2c->sdaPort, i2c->sdaPin, GPIO_PIN_RESET);
            wait_for_gpio_state_timeout(i2c->sdaPort, i2c->sdaPin, GPIO_PIN_RESET, timeout); // 5. Check SDA Low level in GPIOx_IDR
        }

        // 6. Configure the SCL I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
        HAL_GPIO_WritePin(i2c->sclPort, i2c->sclPin, GPIO_PIN_RESET);
        wait_for_gpio_state_timeout(i2c->sclPort, i2c->sclPin, GPIO_PIN_RESET, timeout); // 7. Check SCL Low level in GPIOx_IDR.

        // 8. Configure the SCL I/O as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
        HAL_GPIO_WritePin(i2c->sclPort, i2c->sclPin, GPIO_PIN_SET);
        wait_for_gpio_state_timeout(i2c->sclPort, i2c->sclPin, GPIO_PIN_SET, timeout); // 9. Check SCL High level in GPIOx_IDR.

        // 10. Configure the SDA I/O as General Purpose Output Open-Drain , High level (Write 1 to GPIOx_ODR).
        if(sendStopBits){
            HAL_GPIO_WritePin(i2c->sdaPort, i2c->sdaPin, GPIO_PIN_SET);
            wait_for_gpio_state_timeout(i2c->sdaPort, i2c->sdaPin, GPIO_PIN_SET, timeout); // 11. Check SDA High level in GPIOx_IDR.
        }
    }

    // 12. Configure the SCL and SDA I/Os as Alternate function Open-Drain.
    GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStructure.Alternate = GPIO_AF4_I2C1;

    GPIO_InitStructure.Pin = i2c->sclPin;
    HAL_GPIO_Init(i2c->sclPort, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = i2c->sdaPin;
    HAL_GPIO_Init(i2c->sdaPort, &GPIO_InitStructure);

    // 13. Set SWRST bit in I2Cx_CR1 register.
    SET_BIT(handler->Instance->CR1, I2C_CR1_SWRST);
    asm("nop");

    /* 14. Clear SWRST bit in I2Cx_CR1 register. */
    CLEAR_BIT(handler->Instance->CR1, I2C_CR1_SWRST);
    asm("nop");

    /* 15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register */
    SET_BIT(handler->Instance->CR1, I2C_CR1_PE);
    asm("nop");

    HAL_I2C_Init(handler);
}
