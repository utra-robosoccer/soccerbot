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

/********************************** Macros ***********************************/
#define INT_COEF 16384.0f
#define REM_COEF 16384

#define MPU6050_RA_WHO_AM_I         0x75
#define MPU6050_ADDR    	    0b11010000	// ID
#define MPU6050_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU6050_ADDR    	    0b11010000
#define MPU6050_RA_XG_OFFS_TC       0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_YG_OFFS_TC       0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_ZG_OFFS_TC       0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_X_FINE_GAIN      0x03 //[7:0] X_FINE_GAIN
#define MPU6050_RA_Y_FINE_GAIN      0x04 //[7:0] Y_FINE_GAIN
#define MPU6050_RA_Z_FINE_GAIN      0x05 //[7:0] Z_FINE_GAIN
#define MPU6050_RA_XA_OFFS_H        0x06 //[15:0] XA_OFFS
#define MPU6050_RA_XA_OFFS_L_TC     0x07
#define MPU6050_RA_YA_OFFS_H        0x08 //[15:0] YA_OFFS
#define MPU6050_RA_YA_OFFS_L_TC     0x09
#define MPU6050_RA_ZA_OFFS_H        0x0A //[15:0] ZA_OFFS
#define MPU6050_RA_ZA_OFFS_L_TC     0x0B
#define MPU6050_RA_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
#define MPU6050_RA_XG_OFFS_USRL     0x14
#define MPU6050_RA_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
#define MPU6050_RA_YG_OFFS_USRL     0x16
#define MPU6050_RA_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
#define MPU6050_RA_ZG_OFFS_USRL     0x18
#define MPU6050_RA_SMPLRT_DIV       0x19
#define MPU6050_RA_CONFIG           0x1A
#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_ACCEL_CONFIG     0x1C
#define MPU6050_RA_FIFO_EN          0x23
#define MPU6050_RA_I2C_MST_CTRL     0x24
#define MPU6050_RA_I2C_MST_STATUS   0x36
#define MPU6050_RA_INT_PIN_CFG      0x37
#define MPU6050_RA_INT_ENABLE       0x38
#define MPU6050_RA_DMP_INT_STATUS   0x39
#define MPU6050_RA_INT_STATUS       0x3A

//since we are using the 0 setting for MPU6050_RA_GYRO_CONFIG, we define :
#define IMU_GY_RANGE				131 // divide by this to get degrees per second
//since we are using the 0 setting for MPU6050_RA_ACCEL_CONFIG, we define :

#define ACC_RANGE 					16384 //divide to get in units of g

/*************Output****************************/
#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_ACCEL_XOUT_L     0x3C
#define MPU6050_RA_ACCEL_YOUT_H     0x3D
#define MPU6050_RA_ACCEL_YOUT_L     0x3E
#define MPU6050_RA_ACCEL_ZOUT_H     0x3F
#define MPU6050_RA_ACCEL_ZOUT_L     0x40
#define MPU6050_RA_TEMP_OUT_H       0x41
#define MPU6050_RA_TEMP_OUT_L       0x42
#define MPU6050_RA_GYRO_XOUT_H      0x43
#define MPU6050_RA_GYRO_XOUT_L      0x44
#define MPU6050_RA_GYRO_YOUT_H      0x45
#define MPU6050_RA_GYRO_YOUT_L      0x46
#define MPU6050_RA_GYRO_ZOUT_H      0x47
#define MPU6050_RA_GYRO_ZOUT_L      0x48

#define MPU6050_RA_USER_CTRL		0x6A

#define MPU6050_RA_PWR_MGMT_1		0x6B
#define MPU6050_RA_PWR_MGMT_2		0x6C

#define MPU6050_RA_FIFO_COUNTH      0x72
#define MPU6050_RA_FIFO_COUNTL      0x73
#define MPU6050_RA_FIFO_R_W         0x74
#define MPU6050_RA_WHO_AM_I         0x75

#define MPU6050_GCONFIG_FS_SEL_BIT      4
#define MPU6050_GCONFIG_FS_SEL_LENGTH   2

/***********Full scale selection*********************/
#define MPU6050_GYRO_FS_250         0x00
#define MPU6050_GYRO_FS_500         0x01
#define MPU6050_GYRO_FS_1000        0x02
#define MPU6050_GYRO_FS_2000        0x03

#define MPU6050_ACCEL_FS_2          0x00
#define MPU6050_ACCEL_FS_4          0x01
#define MPU6050_ACCEL_FS_8          0x02
#define MPU6050_ACCEL_FS_16         0x03

/* FIFO enable bit***********************************/
#define MPU6050_TEMP_FIFO_EN_BIT    7
#define MPU6050_XG_FIFO_EN_BIT      6
#define MPU6050_YG_FIFO_EN_BIT      5
#define MPU6050_ZG_FIFO_EN_BIT      4
#define MPU6050_ACCEL_FIFO_EN_BIT   3
#define MPU6050_SLV2_FIFO_EN_BIT    2
#define MPU6050_SLV1_FIFO_EN_BIT    1
#define MPU6050_SLV0_FIFO_EN_BIT    0

/*****************Sample Rate DIV*******************/
#define MPU6050_CLOCK_DIV_348       0x0
#define MPU6050_CLOCK_DIV_333       0x1
#define MPU6050_CLOCK_DIV_320       0x2
#define MPU6050_CLOCK_DIV_308       0x3
#define MPU6050_CLOCK_DIV_296       0x4
#define MPU6050_CLOCK_DIV_286       0x5
#define MPU6050_CLOCK_DIV_276       0x6
#define MPU6050_CLOCK_DIV_267       0x7
#define MPU6050_CLOCK_DIV_258       0x8
#define MPU6050_CLOCK_DIV_500       0x9
#define MPU6050_CLOCK_DIV_471       0xA
#define MPU6050_CLOCK_DIV_444       0xB
#define MPU6050_CLOCK_DIV_421       0xC
#define MPU6050_CLOCK_DIV_400       0xD
#define MPU6050_CLOCK_DIV_381       0xE
#define MPU6050_CLOCK_DIV_364       0xF

/***********interrupt*************************/
#define MPU6050_INTERRUPT_FF_BIT            7
#define MPU6050_INTERRUPT_MOT_BIT           6
#define MPU6050_INTERRUPT_ZMOT_BIT          5
#define MPU6050_INTERRUPT_FIFO_OFLOW_BIT    4
#define MPU6050_INTERRUPT_I2C_MST_INT_BIT   3
#define MPU6050_INTERRUPT_PLL_RDY_INT_BIT   2
#define MPU6050_INTERRUPT_DMP_INT_BIT       1
#define MPU6050_INTERRUPT_DATA_RDY_BIT      0

// TODO: figure out what these actually do
// UMPL source code is not very obivous
#define MPU6050_DMPINT_5_BIT            5
#define MPU6050_DMPINT_4_BIT            4
#define MPU6050_DMPINT_3_BIT            3
#define MPU6050_DMPINT_2_BIT            2
#define MPU6050_DMPINT_1_BIT            1
#define MPU6050_DMPINT_0_BIT            0

#define MPU6050_MOTION_MOT_XNEG_BIT     7
#define MPU6050_MOTION_MOT_XPOS_BIT     6
#define MPU6050_MOTION_MOT_YNEG_BIT     5
#define MPU6050_MOTION_MOT_YPOS_BIT     4
#define MPU6050_MOTION_MOT_ZNEG_BIT     3
#define MPU6050_MOTION_MOT_ZPOS_BIT     2
#define MPU6050_MOTION_MOT_ZRMOT_BIT    0

/****************Path Rest************************/
#define MPU6050_PATHRESET_GYRO_RESET_BIT    2
#define MPU6050_PATHRESET_ACCEL_RESET_BIT   1
#define MPU6050_PATHRESET_TEMP_RESET_BIT    0

#define MPU6050_USERCTRL_DMP_EN_BIT             7
#define MPU6050_USERCTRL_FIFO_EN_BIT            6
#define MPU6050_USERCTRL_I2C_MST_EN_BIT         5
#define MPU6050_USERCTRL_I2C_IF_DIS_BIT         4
#define MPU6050_USERCTRL_DMP_RESET_BIT          3
#define MPU6050_USERCTRL_FIFO_RESET_BIT         2
#define MPU6050_USERCTRL_I2C_MST_RESET_BIT      1
#define MPU6050_USERCTRL_SIG_COND_RESET_BIT     0

/************Power Management************************/
#define MPU6050_PWR1_DEVICE_RESET_BIT   7
#define MPU6050_PWR1_SLEEP_BIT          6
#define MPU6050_PWR1_CYCLE_BIT          5
#define MPU6050_PWR1_TEMP_DIS_BIT       3
#define MPU6050_PWR1_CLKSEL_BIT         2
#define MPU6050_PWR1_CLKSEL_LENGTH      3

#define MPU6050_PWR2_LP_WAKE_CTRL_BIT       7
#define MPU6050_PWR2_LP_WAKE_CTRL_LENGTH    2
#define MPU6050_PWR2_STBY_XA_BIT            5
#define MPU6050_PWR2_STBY_YA_BIT            4
#define MPU6050_PWR2_STBY_ZA_BIT            3
#define MPU6050_PWR2_STBY_XG_BIT            2
#define MPU6050_PWR2_STBY_YG_BIT            1
#define MPU6050_PWR2_STBY_ZG_BIT            0

#define MPU6050_WAKE_FREQ_1P25      0x0
#define MPU6050_WAKE_FREQ_2P5       0x1
#define MPU6050_WAKE_FREQ_5         0x2
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

    /**
      * @brief   This function big-bangs the I2C master clock
      *          https://electronics.stackexchange.com/questions/267972/i2c-busy-flag-strange-behaviour/281046#281046
      *          https://community.st.com/thread/35884-cant-reset-i2c-in-stm32f407-to-release-i2c-lines
      *          https://electronics.stackexchange.com/questions/272427/stm32-busy-flag-is-set-after-i2c-initialization
      *          http://www.st.com/content/ccc/resource/technical/document/errata_sheet/f5/50/c9/46/56/db/4a/f6/CD00197763.pdf/files/CD00197763.pdf/jcr:content/translations/en.CD00197763.pdf
      * @param   numClocks The number of times to cycle the I2C master clock
      * @param   sendStopBits 1 if stop bits are to be sent on SDA
      * @return  None
      */
    void generateClocks(uint8_t numClocks, uint8_t sendStopBits);
    /**
      * @brief   Helper function for I2C_ClearBusyFlagErratum.
      * @param   None
      * @return  None
      */
    static uint8_t wait_for_gpio_state_timeout(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state, uint8_t timeout);
}; //class MPU6050

} //namespace IMUnamespace

/**
 * @}
 */
/* end - Header */

#endif /* MPU6050_H_ */
