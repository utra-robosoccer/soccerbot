/**
  *****************************************************************************
  * @file
  * @author  Izaak
  * @author  Tyler
  * @author  Jenny
  * @brief   Implements all functions in the MPU6060 class.
  *
  * @defgroup MPU6050
  * @brief    Module for the MPU6050 IMU sensor
  * @{
  *****************************************************************************
  */




/********************************* Includes **********************************/
#include "MPU6050.h"
#include <math.h>
#include "i2c.h"
#ifdef STM32F446xx
#include "gpio.h"
#include "SystemConf.h"
#endif




/******************************** File-local *********************************/
namespace {
// Constants
// ----------------------------------------------------------------------------
constexpr uint8_t MPU6050_ADDR=            0b11010000;  // ID
constexpr uint8_t MPU6050_RA_GYRO_CONFIG=      0x1B;
constexpr uint8_t MPU6050_RA_ACCEL_CONFIG=     0x1C;
constexpr uint8_t MPU6050_RA_I2C_MST_CTRL=     0x24;
constexpr uint8_t MPU6050_RA_SMPLRT_DIV=       0x19;

// Output
constexpr uint8_t MPU6050_RA_ACCEL_XOUT_H=     0x3B;
constexpr uint8_t MPU6050_RA_GYRO_XOUT_H=      0x43;
constexpr uint8_t MPU6050_RA_PWR_MGMT_1=       0x6B;
constexpr uint8_t MPU6050_RA_PWR_MGMT_2=       0x6C;

// Sample Rate DIV
constexpr uint8_t MPU6050_CLOCK_DIV_296=       0x4;




// Functions
// ----------------------------------------------------------------------------
// We only need these functions for a silicon issue that affects the F446RE and
// not the F767ZI
#if defined(USE_I2C_SILICON_BUG_FIX)
// Note: The following 2 functions are used as a workaround for an issue where the BUSY flag of the
// I2C module is erroneously asserted in the hardware (a silicon bug, essentially). This workaround has
// not been thoroughly tested, but we know it works
//
// Overall, use these functions with EXTREME caution.
/**
  * @brief   Helper function for I2C_ClearBusyFlagErratum.
  * @param   None
  * @return  None
  */
uint8_t wait_for_gpio_state_timeout(
    GPIO_TypeDef* port,
    uint16_t pin,
    GPIO_PinState state,
    uint8_t timeout
)
{

    uint32_t tick_start = HAL_GetTick();
    uint8_t ret = 0;
    /* Wait until flag is set */
    while((state != HAL_GPIO_ReadPin(port, pin)) && (1 == ret)){
        /* Check for the timeout */
        if ((timeout == 0U) || (HAL_GetTick() - tick_start >= timeout)){
            ret = 0;
        }
        asm("nop");
    }
    return ret;
}

/**
  * @brief   This function big-bangs the I2C master clock
  *          https://electronics.stackexchange.com/questions/267972/i2c-busy-flag-strange-behaviour/281046#281046
  *          https://community.st.com/thread/35884-cant-reset-i2c-in-stm32f407-to-release-i2c-lines
  *          https://electronics.stackexchange.com/questions/272427/stm32-busy-flag-is-set-after-i2c-initialization
  *          http://www.st.com/content/ccc/resource/technical/document/errata_sheet/f5/50/c9/46/56/db/4a/f6/CD00197763.pdf/files/CD00197763.pdf/jcr:content/translations/en.CD00197763.pdf
  * @param   num_clocks The number of times to cycle the I2C master clock
  * @param   send_stop_bits 1 if stop bits are to be sent on SDA
  * @return  None
  */
void generateClocks(uint8_t num_clocks, uint8_t send_stop_bits){
    static struct I2C_Module{
        I2C_HandleTypeDef* instance;
        uint16_t           sda_pin;
        GPIO_TypeDef*      sda_port;
        uint16_t           scl_pin;
        GPIO_TypeDef*      scl_port;
    }i2cmodule = {&hi2c1, GPIO_PIN_7, GPIOB, GPIO_PIN_6, GPIOB};
    static struct I2C_Module* i2c = &i2cmodule;
    static uint8_t timeout = 1;

    GPIO_InitTypeDef gpio_init_struct;

    I2C_HandleTypeDef* handler = NULL;

    handler = i2c->instance;

    // 1. Clear PE bit.
    CLEAR_BIT(handler->Instance->CR1, I2C_CR1_PE);

    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_OD;
    gpio_init_struct.Pull = GPIO_NOPULL;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    gpio_init_struct.Pin = i2c->scl_pin;
    HAL_GPIO_Init(i2c->scl_port, &gpio_init_struct);

    gpio_init_struct.Pin = i2c->sda_pin;
    HAL_GPIO_Init(i2c->sda_port, &gpio_init_struct);

    for(uint8_t i = 0; i < num_clocks; i++){
        // 3. Check SCL and SDA High level in GPIOx_IDR.
        if(send_stop_bits){
            HAL_GPIO_WritePin(i2c->sda_port, i2c->sda_pin, GPIO_PIN_SET);
        }
        HAL_GPIO_WritePin(i2c->scl_port, i2c->scl_pin, GPIO_PIN_SET);

        wait_for_gpio_state_timeout(i2c->scl_port, i2c->scl_pin, GPIO_PIN_SET, timeout);
        if(send_stop_bits){
            wait_for_gpio_state_timeout(i2c->sda_port, i2c->sda_pin, GPIO_PIN_SET, timeout);
        }

        // 4. Configure the SDA I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
        if(send_stop_bits){
            HAL_GPIO_WritePin(i2c->sda_port, i2c->sda_pin, GPIO_PIN_RESET);
            wait_for_gpio_state_timeout(i2c->sda_port, i2c->sda_pin, GPIO_PIN_RESET, timeout); // 5. Check SDA Low level in GPIOx_IDR
        }

        // 6. Configure the SCL I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
        HAL_GPIO_WritePin(i2c->scl_port, i2c->scl_pin, GPIO_PIN_RESET);
        wait_for_gpio_state_timeout(i2c->scl_port, i2c->scl_pin, GPIO_PIN_RESET, timeout); // 7. Check SCL Low level in GPIOx_IDR.

        // 8. Configure the SCL I/O as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
        HAL_GPIO_WritePin(i2c->scl_port, i2c->scl_pin, GPIO_PIN_SET);
        wait_for_gpio_state_timeout(i2c->scl_port, i2c->scl_pin, GPIO_PIN_SET, timeout); // 9. Check SCL High level in GPIOx_IDR.

        // 10. Configure the SDA I/O as General Purpose Output Open-Drain , High level (Write 1 to GPIOx_ODR).
        if(send_stop_bits){
            HAL_GPIO_WritePin(i2c->sda_port, i2c->sda_pin, GPIO_PIN_SET);
            wait_for_gpio_state_timeout(i2c->sda_port, i2c->sda_pin, GPIO_PIN_SET, timeout); // 11. Check SDA High level in GPIOx_IDR.
        }
    }

    // 12. Configure the SCL and SDA I/Os as Alternate function Open-Drain.
    gpio_init_struct.Mode = GPIO_MODE_AF_OD;
    gpio_init_struct.Alternate = GPIO_AF4_I2C1;

    gpio_init_struct.Pin = i2c->scl_pin;
    HAL_GPIO_Init(i2c->scl_port, &gpio_init_struct);

    gpio_init_struct.Pin = i2c->sda_pin;
    HAL_GPIO_Init(i2c->sda_port, &gpio_init_struct);

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
#endif

} // end anonymous namespace




/********************************* MPU6050 ***********************************/
using imu::MPU6050;
// Public
// ----------------------------------------------------------------------------
MPU6050::MPU6050(I2C_HandleTypeDef* m_i2c_handle)
    : m_i2c_handle(m_i2c_handle)
{
    // Initialize all the variables
    this -> m_ax = 0;
    this -> m_ay = 0;
    this -> m_az = 0;
    this -> m_vx = 0;
    this -> m_vy = 0;
    this -> m_vz = 0;
    this -> m_recv_byte = 0;
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

void MPU6050::Read_Gyroscope(){
    uint8_t output_buffer[6];
    MPU6050::Read_Data(MPU6050_RA_GYRO_XOUT_H,output_buffer);
    int16_t vx = (int16_t)(output_buffer[0]<<8|output_buffer[1]);
    int16_t vy = (int16_t)(output_buffer[2]<<8|output_buffer[3]);
    int16_t vz = (int16_t)(output_buffer[4]<<8|output_buffer[5]);

    this ->m_vx = (float)(vx) / IMU_GY_RANGE;
    this ->m_vy = (float)(vy) / IMU_GY_RANGE;
    this ->m_vz = (float)(vz) / IMU_GY_RANGE;
}

void MPU6050::Read_Gyroscope_IT(){
    uint8_t output_buffer[6];
    uint32_t notification;
    BaseType_t status;

    if(MPU6050::Read_Data_IT(MPU6050_RA_GYRO_XOUT_H,output_buffer) != HAL_OK){
#ifdef STM32F446xx
        // Try fix for flag bit silicon bug
        generateClocks(1, 1);
#endif
        return;
    }

    do{
        status = xTaskNotifyWait(0, NOTIFIED_FROM_RX_ISR, &notification, MAX_DELAY_TIME);
        if(status != pdTRUE){
            return;
        }
    }while((notification & NOTIFIED_FROM_RX_ISR) != NOTIFIED_FROM_RX_ISR);

    int16_t vx = (int16_t)(output_buffer[0]<<8|output_buffer[1]);
    int16_t vy = (int16_t)(output_buffer[2]<<8|output_buffer[3]);
    int16_t vz = (int16_t)(output_buffer[4]<<8|output_buffer[5]);


    this ->m_vx = (float)(vx) / IMU_GY_RANGE;
    this ->m_vy = (float)(vy) / IMU_GY_RANGE;
    this ->m_vz = (float)(vz) / IMU_GY_RANGE;
}

void MPU6050::Read_Accelerometer(){
    uint8_t output_buffer[6];
    MPU6050::Read_Data(MPU6050_RA_ACCEL_XOUT_H,output_buffer);
    int16_t ax = (int16_t)(output_buffer[0]<<8|output_buffer[1]);
    int16_t ay = (int16_t)(output_buffer[2]<<8|output_buffer[3]);
    int16_t az  = (int16_t)(output_buffer[4]<<8|output_buffer[5]);

    this ->m_ax = -(ax * g / ACC_RANGE);
    this ->m_ay = -(ay * g / ACC_RANGE);
    this ->m_az = -(az * g / ACC_RANGE);
}

void MPU6050::Read_Accelerometer_IT(){
    uint8_t output_buffer[6];
    uint32_t notification;
    BaseType_t status;

    if(MPU6050::Read_Data_IT(MPU6050_RA_ACCEL_XOUT_H,output_buffer)){
#ifdef STM32F446xx
        // Try fix for flag bit silicon bug
        generateClocks(1, 1);
#endif
    }

    do{
        status = xTaskNotifyWait(0, NOTIFIED_FROM_RX_ISR, &notification, MAX_DELAY_TIME);
        if(status != pdTRUE){
            return;
        }
    }while((notification & NOTIFIED_FROM_RX_ISR) != NOTIFIED_FROM_RX_ISR);

    int16_t ax = (int16_t)(output_buffer[0]<<8|output_buffer[1]);
    int16_t ay = (int16_t)(output_buffer[2]<<8|output_buffer[3]);
    int16_t az  = (int16_t)(output_buffer[4]<<8|output_buffer[5]);

    this ->m_ax = -(ax * g / ACC_RANGE);
    this ->m_ay = -(ay * g / ACC_RANGE);
    this ->m_az = -(az * g / ACC_RANGE);
}

void MPU6050::Fill_Struct(ImuStruct_t* p_data){
    p_data->x_Accel = this->m_ax;
    p_data->y_Accel = this->m_ay;
    p_data->z_Accel = this->m_az;

    p_data->x_Gyro = this->m_vx;
    p_data->y_Gyro = this->m_vy;
    p_data->z_Gyro = this->m_vz;
}

// Private
// ----------------------------------------------------------------------------
HAL_StatusTypeDef MPU6050::Write_Reg(uint8_t reg_addr, uint8_t data){
    return HAL_I2C_Mem_Write(
        const_cast<I2C_HandleTypeDef*>(this->m_i2c_handle),
        (uint16_t) MPU6050_ADDR,
        (uint16_t) reg_addr,
        1,
        &data,
        1,
        10
    );
}

HAL_StatusTypeDef MPU6050::Read_Reg(uint8_t reg_addr){
    return HAL_I2C_Mem_Read(
        const_cast<I2C_HandleTypeDef*>(this->m_i2c_handle),
        (uint16_t) MPU6050_ADDR,
        (uint16_t) reg_addr,
        1,
        &(this->m_recv_byte),
        1,
        1000
    );
}

HAL_StatusTypeDef MPU6050::Read_Data_IT(uint8_t reg_addr, uint8_t* sensor_buffer){
    return HAL_I2C_Mem_Read_IT(
        const_cast<I2C_HandleTypeDef*>(this->m_i2c_handle),
        (uint16_t) MPU6050_ADDR,
        (uint16_t) reg_addr,
        1,
        sensor_buffer,
        6
    );
}

HAL_StatusTypeDef MPU6050::Read_Data(uint8_t reg_addr, uint8_t* sensor_buffer){
    return HAL_I2C_Mem_Read(
        const_cast<I2C_HandleTypeDef*>(this->m_i2c_handle),
        (uint16_t) MPU6050_ADDR,
        (uint16_t) reg_addr,
        1,
        sensor_buffer,
        6,
        1000
    );
}

bool MPU6050::Set_LPF(uint8_t lpf){
    bool retval = false;
    if(lpf <= 6){
        // the LPF reg is decimal 26
        uint8_t current_value;
        uint8_t bitmask=7; // 00000111

        MPU6050::Read_Reg(26); //read the current value of the LPF reg, and save it to received_byte
        current_value=this->m_recv_byte;

        // note that this reg also contains unneeded fsync data which should be preserved
        current_value = (current_value & (~bitmask)) | lpf;
        if(MPU6050::Write_Reg(26, current_value) == HAL_OK){
            retval = true;
        }
    }
    return retval;
}

/**
 * @}
 */
/* end - MPU6050 */
