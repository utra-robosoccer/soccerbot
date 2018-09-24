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

/********************************** Anonymous Namespace **********************/
namespace {

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

/********************************** Local Helpers ****************************/
// Note: The following 2 functions are used as a workaround for an issue where the BUSY flag of the
// I2C module is erroneously asserted in the hardware (a silicon bug, essentially). This workaround has
// not been thoroughly tested.
//
// Overall, use these functions with EXTREME caution.
uint8_t wait_for_gpio_state_timeout(GPIO_TypeDef *port,
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

void generateClocks(uint8_t numClocks, uint8_t sendStopBits){
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
}


/********************************** Primary Namespace ************************/
using IMUnamespace::MPU6050;


/********************************* Constants *********************************/

constexpr float g = 9.81;

constexpr uint8_t MPU6050_ADDR=            0b11010000;  // ID
constexpr uint8_t MPU6050_RA_GYRO_CONFIG=      0x1B;
constexpr uint8_t MPU6050_RA_ACCEL_CONFIG=     0x1C;
constexpr uint8_t MPU6050_RA_I2C_MST_CTRL=     0x24;
constexpr uint8_t MPU6050_RA_SMPLRT_DIV=       0x19;

//since we are using the 0 setting for MPU6050_RA_GYRO_CONFIG, we define :
constexpr uint8_t IMU_GY_RANGE=                131; // divide by this to get degrees per second
//since we are using the 0 setting for MPU6050_RA_ACCEL_CONFIG, we define :
constexpr float ACC_RANGE =                    16384.0; //divide to get in units of g

/**********************Output************************/
constexpr uint8_t MPU6050_RA_ACCEL_XOUT_H=     0x3B;
constexpr uint8_t MPU6050_RA_GYRO_XOUT_H=      0x43;
constexpr uint8_t MPU6050_RA_PWR_MGMT_1=       0x6B;
constexpr uint8_t MPU6050_RA_PWR_MGMT_2=       0x6C;

/*****************Sample Rate DIV*******************/
constexpr uint8_t MPU6050_CLOCK_DIV_296=       0x4;

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

/**
 * @}
 */
/* end - MPU6050 */
