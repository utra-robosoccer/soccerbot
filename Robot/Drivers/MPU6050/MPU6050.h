/**
  ******************************************************************************
  * @file    MPU6050.h
  * @author  Izaak
  * @author  Jenny
  * @author  Tyler
  * @brief   Header code for the MPU6050 library, including the struct in which
  *          accelerometer and gyroscope data are stored.
  *
  *@defgroup  Header
  *@ingroup   MPU6050
  *@{
  ******************************************************************************
  */

/******************** Define to prevent recursive inclusion ******************/
#ifndef MPU6050_H_
#define MPU6050_H_

/********************************* Includes **********************************/
#include <stdint.h>
#include "i2c.h"
#include "cmsis_os.h"
#include "usart.h"
#include "gpio.h"

#include "sharedMacros.h"
#include "MPUFilter.h"
#include "UART_Handler.h"

/********************************** Namespace ********************************/

namespace IMUnamespace {

/********************************** Classes **********************************/
class MPU6050 {
    uint8_t                 _Sample_Rate;
    I2C_HandleTypeDef*      _I2C_Handle;
    float                   _x_Gyro;            /**< x-axis angular velocity read from sensor*/
    float                   _y_Gyro;            /**< y-axis angular velocity read from sensor*/
    float                   _z_Gyro;            /**< z-axis angular velocity read from sensor*/
    float                   _x_Accel;           /**< x-axis acceleration read from sensor*/
    float                   _y_Accel;           /**< y-axis acceleration read from sensor*/
    float                   _z_Accel;           /**< z-axis acceleration read from sensor*/

    //offsets:
    float                   _x_GyroOffset;
    float                   _y_GyroOffset;
    float                   _z_GyroOffset;
    float                   _x_AccelOffset;
    float                   _y_AccelOffset;
    float                   _z_AccelOffset;

    uint8_t                 received_byte;

    public:
    /**
     * @brief The constructor for the MPU6050 class, which initializes non-I/O members
     * @param  SensorNum The integer ID of the sensor being used
     * @param  I2CHandle A pointer to the I2C_HandleTypeDef being used
     * @return None
     */
    MPU6050(int SensorNum, I2C_HandleTypeDef* I2CHandle);

    /**
      * @brief   Reads the gyroscope with offsets without interrupts
      * @param   None
      * @return  None
      */
    void Read_Gyroscope_Withoffset();

    /**
      * @brief   Reads the gyroscope with interrupts and offsets
      * @param   None
      * @return  None
      */
    void Read_Gyroscope_Withoffset_IT();

    /**
      * @brief   Reads the accelerometer with offsets without interrupts
      * @param   None
      * @return  None
      */
    void Read_Accelerometer_Withoffset();

    /**
      * @brief   Reads the accelerometer with interrupts and offsets
      * @param   None
      * @return  None
      */
    void Read_Accelerometer_Withoffset_IT();

    /**
      * @brief   Fills an IMUStruct
      * @param   myStruct The pointer to the struct being filled
      * @return  None
      */
    void Fill_Struct(IMUStruct * myStruct);

    /**
      * @brief   This function is used to initialize all aspects of the IMU
      *          which require I/O
      * @param   lpf The uint8_t lpf setting being used
      * @return  None
      */
    void init(uint8_t lpf);

    /**
      * @brief   The MPU6050 desctructor
      * @param   None
      * @return  None
      */
    ~MPU6050() {}

    private:
    /**
      * @brief   Writes to a register from the MPU6050
      * @param   reg_addr uint8_t address of the register
      * @param   data uint8_t data to be written
      * @return  Status
      */
    int Write_Reg(uint8_t reg_addr, uint8_t data);

    /**
      * @brief   Sets the offsets of the sensor
      * @param   SensorNum The integer ID of the sensor being used
      * @return  None
      */
    void Manually_Set_Offsets(int SensorNum);

    /**
      * @brief   Sets the offsets of the sensor
      * @param   lpf The uint8_t setting being used for the built-in LPF
      * @return  None
      */
    int Set_LPF(uint8_t lpf);

    /**
      * @brief   Reads a byte from a register on the sensor, and stores it
      *          in the member variable received_byte
      * @param   reg_addr uint8_t address of the register
      * @return  Status
      */
    uint8_t Read_Reg(uint8_t reg_addr);

    /**
      * @brief   Reads 6 bytes from the sensor with interrupts, and stores them in
      *          the sensor_buffer
      * @param   reg_addr uint8_t address of the register
      * @param   sensor_buffer uint8_t pointer to output buffer
      * @return  Status
      */

    BaseType_t Read_Data_IT(uint8_t Reg_addr, uint8_t* sensor_buffer);
    /**
      * @brief   Reads 6 bytes from the sensor, and stores them in
      *          the sensor_buffer
      * @param   reg_addr uint8_t address of the register
      * @param   sensor_buffer uint8_t pointer to output buffer
      * @return  Status
      */
    uint8_t Read_Data(uint8_t Reg_addr, uint8_t* sensor_buffer);

}; //class MPU6050

} //namespace IMUnamespace

/**
 * @}
 */
/* end - Header */

#endif /* MPU6050_H_ */
