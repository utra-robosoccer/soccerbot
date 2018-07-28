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

int MPU6050::WriteReg(uint8_t reg_addr, uint8_t data){
    return HAL_I2C_Mem_Write(this _I2C_Handle, (uint16_t) MPU6050_ADDR, (uint16_t) reg_addr, 1, &data, 1, 10);
}

uint8_t MPU6050::MPU6050_READ_REG(uint8_t reg_addr){
    uint8_t status = HAL_I2C_Mem_Read(this _I2C_Handle,(uint16_t) MPU6050_ADDR,(uint16_t) reg_addr, 1,  this &received_byte, 1,1000);
    return status;
}




/*****************************************************************************/
/*  Submodule name                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/
/**
 * @defgroup TODO -- define the submodule (something like "Module_SubmoduleName Submodule name")
 * @brief    TODO -- describe submodule here
 *
 * # TODO -- Page title for submodule #
 *
 * TODO -- detailed description of submodule
 *
 * @ingroup TODO -- name of the parent module
 * @{
 */

// TODO: put functions here

/**
 * @}
 */
/* end TODO -- Module_SubmoduleName */

/*****************************************************************************/
/*  Submodule name                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/
/**
 * @defgroup Test Member Functions
 * @brief    TODO -- describe submodule here

 *
 * @ingroup Namespace
 * @{
 */

int Class::testMe(int a, const char *s) {
    return 1;
}

/**
 * @}
 */
/* end Test Member Functions */
