/**
  *****************************************************************************
  * @file    MPU6050.cpp
  * @author  Izaak
  * @author  Tyler
  * @author  Jenny
  * @brief   All functions related to the MPU6060 IMU sensor.
  *
  * @defgroup MPU6050
  * @{
  *****************************************************************************
  */

/********************************* Includes **********************************/
#include "MPU6050.h"
#include <math.h>


/********************************** Primary Namespace ************************/
using IMUnamespace::MPU6050;


/********************************* Constants *********************************/

const float g = 9.81;

constexpr uint8_t MPU6050_ADDR=            0b11010000;  // ID
constexpr uint8_t MPU6050_ADDRESS_AD0_LOW=     0x68; // address pin low (GND), default for InvenSense evaluation board
constexpr uint8_t MPU6050_ADDRESS_AD0_HIGH=    0x69; // address pin high (VCC)
constexpr uint8_t MPU6050_RA_XG_OFFS_TC=       0x00; //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
constexpr uint8_t MPU6050_RA_YG_OFFS_TC=       0x01; //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
constexpr uint8_t MPU6050_RA_ZG_OFFS_TC=       0x02; //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
constexpr uint8_t MPU6050_RA_X_FINE_GAIN=      0x03; //[7:0] X_FINE_GAIN
constexpr uint8_t MPU6050_RA_Y_FINE_GAIN=      0x04; //[7:0] Y_FINE_GAIN
constexpr uint8_t MPU6050_RA_Z_FINE_GAIN=      0x05; //[7:0] Z_FINE_GAIN
constexpr uint8_t MPU6050_RA_XA_OFFS_H=        0x06; //[15:0] XA_OFFS
constexpr uint8_t MPU6050_RA_XA_OFFS_L_TC=     0x07;
constexpr uint8_t MPU6050_RA_YA_OFFS_H=        0x08; //[15:0] YA_OFFS
constexpr uint8_t MPU6050_RA_YA_OFFS_L_TC=     0x09;
constexpr uint8_t MPU6050_RA_ZA_OFFS_H=        0x0A; //[15:0] ZA_OFFS
constexpr uint8_t MPU6050_RA_ZA_OFFS_L_TC=     0x0B;
constexpr uint8_t MPU6050_RA_XG_OFFS_USRH=     0x13; //[15:0] XG_OFFS_USR
constexpr uint8_t MPU6050_RA_XG_OFFS_USRL=     0x14;
constexpr uint8_t MPU6050_RA_YG_OFFS_USRH=     0x15; //[15:0] YG_OFFS_USR
constexpr uint8_t MPU6050_RA_YG_OFFS_USRL=     0x16;
constexpr uint8_t MPU6050_RA_ZG_OFFS_USRH=     0x17; //[15:0] ZG_OFFS_USR
constexpr uint8_t MPU6050_RA_ZG_OFFS_USRL=     0x18;
constexpr uint8_t MPU6050_RA_SMPLRT_DIV=       0x19;
constexpr uint8_t MPU6050_RA_CONFIG=           0x1A;
constexpr uint8_t MPU6050_RA_GYRO_CONFIG=      0x1B;
constexpr uint8_t MPU6050_RA_ACCEL_CONFIG=     0x1C;
constexpr uint8_t MPU6050_RA_FIFO_EN=          0x23;
constexpr uint8_t MPU6050_RA_I2C_MST_CTRL=     0x24;
constexpr uint8_t MPU6050_RA_I2C_MST_STATUS=   0x36;
constexpr uint8_t MPU6050_RA_INT_PIN_CFG=      0x37;
constexpr uint8_t MPU6050_RA_INT_ENABLE=       0x38;
constexpr uint8_t MPU6050_RA_DMP_INT_STATUS=   0x39;
constexpr uint8_t MPU6050_RA_INT_STATUS=       0x3A;

//since we are using the 0 setting for MPU6050_RA_GYRO_CONFIG, we define :
constexpr uint8_t IMU_GY_RANGE=                131; // divide by this to get degrees per second
//since we are using the 0 setting for MPU6050_RA_ACCEL_CONFIG, we define :
constexpr uint8_t ACC_RANGE=                   16384; //divide to get in units of g

/*************Output****************************/
constexpr uint8_t MPU6050_RA_ACCEL_XOUT_H=     0x3B;
constexpr uint8_t MPU6050_RA_ACCEL_XOUT_L=     0x3C;
constexpr uint8_t MPU6050_RA_ACCEL_YOUT_H=     0x3D;
constexpr uint8_t MPU6050_RA_ACCEL_YOUT_L=     0x3E;
constexpr uint8_t MPU6050_RA_ACCEL_ZOUT_H=     0x3F;
constexpr uint8_t MPU6050_RA_ACCEL_ZOUT_L=     0x40;
constexpr uint8_t MPU6050_RA_TEMP_OUT_H=       0x41;
constexpr uint8_t MPU6050_RA_TEMP_OUT_L=       0x42;
constexpr uint8_t MPU6050_RA_GYRO_XOUT_H=      0x43;
constexpr uint8_t MPU6050_RA_GYRO_XOUT_L=      0x44;
constexpr uint8_t MPU6050_RA_GYRO_YOUT_H=      0x45;
constexpr uint8_t MPU6050_RA_GYRO_YOUT_L=      0x46;
constexpr uint8_t MPU6050_RA_GYRO_ZOUT_H=      0x47;
constexpr uint8_t MPU6050_RA_GYRO_ZOUT_L=      0x48;

constexpr uint8_t MPU6050_RA_USER_CTRL=        0x6A;

constexpr uint8_t MPU6050_RA_PWR_MGMT_1=       0x6B;
constexpr uint8_t MPU6050_RA_PWR_MGMT_2=       0x6C;

constexpr uint8_t MPU6050_RA_FIFO_COUNTH=      0x72;
constexpr uint8_t MPU6050_RA_FIFO_COUNTL=      0x73;
constexpr uint8_t MPU6050_RA_FIFO_R_W=         0x74;
constexpr uint8_t MPU6050_RA_WHO_AM_I=         0x75;

constexpr uint8_t MPU6050_GCONFIG_FS_SEL_BIT=      4;
constexpr uint8_t MPU6050_GCONFIG_FS_SEL_LENGTH=   2;

/***********Full scale selection*********************/
constexpr uint8_t MPU6050_GYRO_FS_250=         0x00;
constexpr uint8_t MPU6050_GYRO_FS_500=         0x01;
constexpr uint8_t MPU6050_GYRO_FS_1000=        0x02;
constexpr uint8_t MPU6050_GYRO_FS_2000=        0x03;

constexpr uint8_t MPU6050_ACCEL_FS_2=          0x00;
constexpr uint8_t MPU6050_ACCEL_FS_4=          0x01;
constexpr uint8_t MPU6050_ACCEL_FS_8=          0x02;
constexpr uint8_t MPU6050_ACCEL_FS_16=         0x03;

/* FIFO enable bit***********************************/
constexpr uint8_t MPU6050_TEMP_FIFO_EN_BIT=    7;
constexpr uint8_t MPU6050_XG_FIFO_EN_BIT=      6;
constexpr uint8_t MPU6050_YG_FIFO_EN_BIT=      5;
constexpr uint8_t MPU6050_ZG_FIFO_EN_BIT=      4;
constexpr uint8_t MPU6050_ACCEL_FIFO_EN_BIT=   3;
constexpr uint8_t MPU6050_SLV2_FIFO_EN_BIT=    2;
constexpr uint8_t MPU6050_SLV1_FIFO_EN_BIT=    1;
constexpr uint8_t MPU6050_SLV0_FIFO_EN_BIT=    0;

/*****************Sample Rate DIV*******************/
constexpr uint8_t MPU6050_CLOCK_DIV_348=       0x0;
constexpr uint8_t MPU6050_CLOCK_DIV_333=       0x1;
constexpr uint8_t MPU6050_CLOCK_DIV_320=       0x2;
constexpr uint8_t MPU6050_CLOCK_DIV_308=       0x3;
constexpr uint8_t MPU6050_CLOCK_DIV_296=       0x4;
constexpr uint8_t MPU6050_CLOCK_DIV_286=       0x5;
constexpr uint8_t MPU6050_CLOCK_DIV_276=       0x6;
constexpr uint8_t MPU6050_CLOCK_DIV_267=       0x7;
constexpr uint8_t MPU6050_CLOCK_DIV_258=       0x8;
constexpr uint8_t MPU6050_CLOCK_DIV_500=       0x9;
constexpr uint8_t MPU6050_CLOCK_DIV_471=       0xA;
constexpr uint8_t MPU6050_CLOCK_DIV_444=       0xB;
constexpr uint8_t MPU6050_CLOCK_DIV_421=       0xC;
constexpr uint8_t MPU6050_CLOCK_DIV_400=       0xD;
constexpr uint8_t MPU6050_CLOCK_DIV_381=       0xE;
constexpr uint8_t MPU6050_CLOCK_DIV_364=       0xF;

/***********interrupt*************************/
constexpr uint8_t MPU6050_INTERRUPT_FF_BIT=            7;
constexpr uint8_t MPU6050_INTERRUPT_MOT_BIT=           6;
constexpr uint8_t MPU6050_INTERRUPT_ZMOT_BIT=          5;
constexpr uint8_t MPU6050_INTERRUPT_FIFO_OFLOW_BIT=    4;
constexpr uint8_t MPU6050_INTERRUPT_I2C_MST_INT_BIT=   3;
constexpr uint8_t MPU6050_INTERRUPT_PLL_RDY_INT_BIT=   2;
constexpr uint8_t MPU6050_INTERRUPT_DMP_INT_BIT=       1;
constexpr uint8_t MPU6050_INTERRUPT_DATA_RDY_BIT=      0;

// TODO: figure out what these actually do
// UMPL source code is not very obivous
constexpr uint8_t MPU6050_DMPINT_5_BIT=            5;
constexpr uint8_t MPU6050_DMPINT_4_BIT=            4;
constexpr uint8_t MPU6050_DMPINT_3_BIT=            3;
constexpr uint8_t MPU6050_DMPINT_2_BIT=            2;
constexpr uint8_t MPU6050_DMPINT_1_BIT=            1;
constexpr uint8_t MPU6050_DMPINT_0_BIT=            0;

constexpr uint8_t MPU6050_MOTION_MOT_XNEG_BIT=     7;
constexpr uint8_t MPU6050_MOTION_MOT_XPOS_BIT=     6;
constexpr uint8_t MPU6050_MOTION_MOT_YNEG_BIT=     5;
constexpr uint8_t MPU6050_MOTION_MOT_YPOS_BIT=     4;
constexpr uint8_t MPU6050_MOTION_MOT_ZNEG_BIT=     3;
constexpr uint8_t MPU6050_MOTION_MOT_ZPOS_BIT=     2;
constexpr uint8_t MPU6050_MOTION_MOT_ZRMOT_BIT=    0;

/****************Path Rest************************/
constexpr uint8_t MPU6050_PATHRESET_GYRO_RESET_BIT=    2;
constexpr uint8_t MPU6050_PATHRESET_ACCEL_RESET_BIT=   1;
constexpr uint8_t MPU6050_PATHRESET_TEMP_RESET_BIT=    0;

constexpr uint8_t MPU6050_USERCTRL_DMP_EN_BIT=             7;
constexpr uint8_t MPU6050_USERCTRL_FIFO_EN_BIT=            6;
constexpr uint8_t MPU6050_USERCTRL_I2C_MST_EN_BIT=         5;
constexpr uint8_t MPU6050_USERCTRL_I2C_IF_DIS_BIT=         4;
constexpr uint8_t MPU6050_USERCTRL_DMP_RESET_BIT=          3;
constexpr uint8_t MPU6050_USERCTRL_FIFO_RESET_BIT=         2;
constexpr uint8_t MPU6050_USERCTRL_I2C_MST_RESET_BIT=      1;
constexpr uint8_t MPU6050_USERCTRL_SIG_COND_RESET_BIT=     0;

/************Power Management************************/
constexpr uint8_t MPU6050_PWR1_DEVICE_RESET_BIT=   7;
constexpr uint8_t MPU6050_PWR1_SLEEP_BIT=          6;
constexpr uint8_t MPU6050_PWR1_CYCLE_BIT=          5;
constexpr uint8_t MPU6050_PWR1_TEMP_DIS_BIT=       3;
constexpr uint8_t MPU6050_PWR1_CLKSEL_BIT=         2;
constexpr uint8_t MPU6050_PWR1_CLKSEL_LENGTH=      3;

constexpr uint8_t MPU6050_PWR2_LP_WAKE_CTRL_BIT=       7;
constexpr uint8_t MPU6050_PWR2_LP_WAKE_CTRL_LENGTH=    2;
constexpr uint8_t MPU6050_PWR2_STBY_XA_BIT=            5;
constexpr uint8_t MPU6050_PWR2_STBY_YA_BIT=            4;
constexpr uint8_t MPU6050_PWR2_STBY_ZA_BIT=            3;
constexpr uint8_t MPU6050_PWR2_STBY_XG_BIT=            2;
constexpr uint8_t MPU6050_PWR2_STBY_YG_BIT=            1;
constexpr uint8_t MPU6050_PWR2_STBY_ZG_BIT=            0;

constexpr uint8_t MPU6050_WAKE_FREQ_1P25=      0x0;
constexpr uint8_t MPU6050_WAKE_FREQ_2P5=       0x1;
constexpr uint8_t MPU6050_WAKE_FREQ_5=         0x2;

/******************************** Functions **********************************/
MPU6050::MPU6050(int SensorNum, I2C_HandleTypeDef* I2CHandle){
    this->Manually_Set_Offsets(SensorNum);

    // Initialize all the variables
    this -> _Sample_Rate = 8000/ (1+ MPU6050_CLOCK_DIV_296);
    this -> _x_Accel = 0;
    this -> _y_Accel = 0;
    this -> _z_Accel = 0;
    this -> _x_Gyro = 0;
    this -> _y_Gyro = 0;
    this -> _z_Gyro = 0;
    this -> received_byte = 0;
    this -> _I2C_Handle = I2CHandle;
}
int MPU6050::Write_Reg(uint8_t reg_addr, uint8_t data){
    return HAL_I2C_Mem_Write(this -> _I2C_Handle,
            (uint16_t) MPU6050_ADDR,
            (uint16_t) reg_addr,
            1,
            &data,
            1,
            10
            );
}

uint8_t MPU6050::Read_Reg(uint8_t reg_addr){
    uint8_t status = HAL_I2C_Mem_Read(this -> _I2C_Handle,
            (uint16_t) MPU6050_ADDR,
            (uint16_t) reg_addr,
            1,
            &(this->received_byte),
            1,
            1000);
    return status;
}

void MPU6050::Fill_Struct(IMUStruct * myStruct){
    myStruct->_x_Accel = this->_x_Accel;
    myStruct->_y_Accel = this->_y_Accel;
    myStruct->_z_Accel = this->_z_Accel;

    myStruct->_x_Gyro = this->_x_Gyro;
    myStruct->_y_Gyro = this->_y_Gyro;
    myStruct->_z_Gyro = this->_z_Gyro;
}

BaseType_t MPU6050::Read_Data_IT(uint8_t Reg_addr, uint8_t* sensor_buffer){
    return HAL_I2C_Mem_Read_IT(this -> _I2C_Handle,
            (uint16_t) MPU6050_ADDR,
            (uint16_t) Reg_addr,
            1,
            sensor_buffer,
            6);
}

uint8_t MPU6050::Read_Data(uint8_t Reg_addr, uint8_t* sensor_buffer){
    uint8_t status = HAL_I2C_Mem_Read(this -> _I2C_Handle ,(uint16_t) MPU6050_ADDR,(uint16_t) Reg_addr, 1 , sensor_buffer, 6,1000);
    return status;
}

void MPU6050::Read_Gyroscope_Withoffset(){
    uint8_t output_buffer[6];
    MPU6050::Read_Data(MPU6050_RA_GYRO_XOUT_H,output_buffer);
    int16_t X = (int16_t)(output_buffer[0]<<8|output_buffer[1]);
    int16_t Y = (int16_t)(output_buffer[2]<<8|output_buffer[3]);
    int16_t Z = (int16_t)(output_buffer[4]<<8|output_buffer[5]);

    this ->_x_Gyro = (float)X/IMU_GY_RANGE-(this ->_x_GyroOffset);
    this ->_y_Gyro = (float)Y/IMU_GY_RANGE-(this ->_y_GyroOffset);
    this ->_z_Gyro = (float)Z/IMU_GY_RANGE-(this ->_z_GyroOffset);
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

    int16_t X = (int16_t)(output_buffer[0]<<8|output_buffer[1]);
    int16_t Y = (int16_t)(output_buffer[2]<<8|output_buffer[3]);
    int16_t Z = (int16_t)(output_buffer[4]<<8|output_buffer[5]);


    this ->_x_Gyro = (float)X/IMU_GY_RANGE-(this ->_x_GyroOffset);
    this ->_y_Gyro = (float)Y/IMU_GY_RANGE-(this ->_y_GyroOffset);
    this ->_z_Gyro = (float)Z/IMU_GY_RANGE-(this ->_z_GyroOffset);
}

void MPU6050::Read_Accelerometer_Withoffset(){
    uint8_t output_buffer[6];
    MPU6050:Read_Data(MPU6050_RA_ACCEL_XOUT_H,output_buffer);
    int16_t X_A = (int16_t)(output_buffer[0]<<8|output_buffer[1]);
    int16_t Y_A = (int16_t)(output_buffer[2]<<8|output_buffer[3]);
    int16_t Z_A  = (int16_t)(output_buffer[4]<<8|output_buffer[5]);

    this ->_x_Accel = -( X_A * g / ACC_RANGE-(this ->_x_AccelOffset));
    this ->_y_Accel = -(Y_A * g / ACC_RANGE-(this ->_y_AccelOffset));
    this ->_z_Accel = -(Z_A * g / ACC_RANGE-(this ->_z_AccelOffset));
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

    this ->_x_Accel = -(X_A * g / ACC_RANGE-(this ->_x_AccelOffset));
    this ->_y_Accel = -(Y_A * g / ACC_RANGE-(this ->_y_AccelOffset));
    this ->_z_Accel = -(Z_A * g / ACC_RANGE-(this ->_z_AccelOffset));
}

int MPU6050::Set_LPF(uint8_t lpf){
    // the LPF reg is decimal 26
    uint8_t current_value;
    uint8_t bitmask=7; // 00000111

    MPU6050::Read_Reg(26); //read the current value of the LPF reg, and save it to received_byte
    current_value=this->received_byte;

    // note that this reg also contains unneeded fsync data which should be preserved
    current_value = (current_value & (~bitmask)) | lpf;
    int status=MPU6050::Write_Reg(26, current_value);
    return status;
}

void MPU6050::Manually_Set_Offsets(int SensorNum){
    if (SensorNum==1){
        this -> _x_AccelOffset= (-714.25 * g / ACC_RANGE);
        this -> _y_AccelOffset= (-767.5 * g / ACC_RANGE);
        this -> _z_AccelOffset= ((16324 * g / ACC_RANGE) - 9.81);

        this -> _x_GyroOffset= (float)240/IMU_GY_RANGE;
        this -> _y_GyroOffset= (float)-760/IMU_GY_RANGE;
        this -> _z_GyroOffset= (float)-130/IMU_GY_RANGE;
    }
}

void MPU6050::init(uint8_t lpf){
    MPU6050::Write_Reg(MPU6050_RA_I2C_MST_CTRL, 0b00001101); //0b00001101 is FAST MODE = 400 kHz
    MPU6050::Write_Reg(MPU6050_RA_ACCEL_CONFIG, 0);
    MPU6050::Write_Reg(MPU6050_RA_GYRO_CONFIG, 0);
    MPU6050::Write_Reg(MPU6050_RA_PWR_MGMT_1, 0);
    MPU6050::Write_Reg(MPU6050_RA_PWR_MGMT_2, 0);
    MPU6050::Write_Reg(MPU6050_RA_SMPLRT_DIV, MPU6050_CLOCK_DIV_296);

    this->Set_LPF(lpf);
}

// Note: The following 2 functions are used as a workaround for an issue where the BUSY flag of the
// I2C module is erroneously asserted in the hardware (a silicon bug, essentially). This workaround has
// not been thoroughly tested.
//
// Overall, use these functions with EXTREME caution.
uint8_t MPU6050::wait_for_gpio_state_timeout(GPIO_TypeDef *port,
        uint16_t pin,
        GPIO_PinState state,
        uint8_t timeout){

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

void MPU6050::generateClocks(uint8_t numClocks, uint8_t sendStopBits){
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

/**
 * @}
 */
/* end - MPU6050 */
