#include "MPU6050.h"

float g = 9.81;
uint8_t IMU_GY_RANGE = 250; /**< divide by this to get degrees per second */
float ACC_RANGE = 16384.0;

#include <stdbool.h>
//uint16_t devAddr = MPU6050_DEFAULT_ADDRESS; // default I2C address(?)
//I2C_HandleTypeDef hi2c1;

// Reference file from existing Robosoccer repo under soccer_embedded on firmware branch:
// https://github.com/utra-robosoccer/soccerbot/blob/firmware/soccer_embedded/Common/component/MPU6050/MPU6050.cpp

// --------------------------- WORKAROUND FUNCTIONS -------------------------------------------------
// We only need these functions for a silicon issue that affects the F446RE and
// not the F767ZI
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

// --------------------------- END OF WORKAROUND FUNCTIONS -------------------------------------------------


/********************************* MPU6050 ***********************************/
ImuSensor imuSensor;
ImuStruct_t p_data;

extern I2C_HandleTypeDef hi2c1;

extern lpf;

void MPU6050_init() {
	imuSensor = (ImuSensor){
		.m_i2c_handle = &hi2c1,
		.devAddr = MPU6050_ADDR,
		.m_ax = 0,
		.m_ay = 0,
		.m_az = 0,
		.m_vx = 0,
		.m_vy = 0,
		.m_vz = 0,
		.m_recv_byte = 0
	};

	// write the values to it's corresponding register
	Write_Reg(MPU6050_RA_I2C_MST_CTRL, 0b00001101); //0b00001101 is FAST MODE = 400 kHz
	Write_Reg(MPU6050_RA_ACCEL_CONFIG, 0);
	Write_Reg(MPU6050_RA_GYRO_CONFIG, 0);
	Write_Reg(MPU6050_RA_PWR_MGMT_1, 0);
	Write_Reg(MPU6050_RA_PWR_MGMT_2, 0);
	Write_Reg(MPU6050_RA_SMPLRT_DIV, MPU6050_CLOCK_DIV_296);

	const uint8_t IMU_DIGITAL_LOWPASS_FILTER_SETTING = 6;
	Set_LPF(IMU_DIGITAL_LOWPASS_FILTER_SETTING); // I don't know what lpf should be set to

}

HAL_StatusTypeDef Write_Reg(uint8_t reg_addr, uint8_t data){
    return HAL_I2C_Mem_Write(
    		imuSensor.m_i2c_handle,
        (uint16_t) MPU6050_ADDR,
        (uint16_t) reg_addr,
        1,
        &data,
        1,
        10
    );
}

HAL_StatusTypeDef Read_Reg(uint8_t reg_addr){
    return HAL_I2C_Mem_Read(
    	imuSensor.m_i2c_handle,
        (uint16_t) MPU6050_ADDR,
        (uint16_t) reg_addr,
        1,
        &(imuSensor.m_recv_byte),
        1,
        1000
    );
}

HAL_StatusTypeDef Read_Data_IT(uint8_t reg_addr, uint8_t* sensor_buffer){
    return HAL_I2C_Mem_Read_IT(
    	imuSensor.m_i2c_handle,
        (uint16_t) MPU6050_ADDR,
        (uint16_t) reg_addr,
        1,
        sensor_buffer,
        6
    );
}

HAL_StatusTypeDef Read_Data(uint8_t reg_addr, uint8_t* sensor_buffer){
    return HAL_I2C_Mem_Read(
    	imuSensor.m_i2c_handle,
        (uint16_t) MPU6050_ADDR,
        (uint16_t) reg_addr,
        1,
        sensor_buffer,
        6,
        1000
    );
}

bool Set_LPF(uint8_t lpf){
    bool retval = false;
    if(lpf <= 6){
        // the LPF reg is decimal 26
        uint8_t current_value;
        uint8_t bitmask=7; // 00000111

        Read_Reg(26); //read the current value of the LPF reg, and save it to received_byte
        current_value = imuSensor.m_recv_byte;

        // note that this reg also contains unneeded fsync data which should be preserved
        current_value = (current_value & (~bitmask)) | lpf;
        if(Write_Reg(26, current_value) == HAL_OK){
            retval = true;
        }
    }
    return retval;
}


void Read_Gyroscope(){
    uint8_t output_buffer[6];
    Read_Data_IT(MPU6050_RA_GYRO_XOUT_H,output_buffer);
    int16_t vx = (int16_t)(output_buffer[0]<<8|output_buffer[1]);
    int16_t vy = (int16_t)(output_buffer[2]<<8|output_buffer[3]);
    int16_t vz = (int16_t)(output_buffer[4]<<8|output_buffer[5]);

    imuSensor.m_vx = (float)(vx) / IMU_GY_RANGE;
    imuSensor.m_vy = (float)(vy) / IMU_GY_RANGE;
    imuSensor.m_vz = (float)(vz) / IMU_GY_RANGE;
}


void Read_Gyroscope_IT(uint8_t *buff){
    uint8_t output_buffer[6];

    if(Read_Data_IT(MPU6050_RA_GYRO_XOUT_H,output_buffer) != HAL_OK){
        // Try fix for flag bit silicon bug
        generateClocks(1, 1);
        return;
    }

    int16_t vx = (int16_t)(output_buffer[0]<<8|output_buffer[1]);
    int16_t vy = (int16_t)(output_buffer[2]<<8|output_buffer[3]);
    int16_t vz = (int16_t)(output_buffer[4]<<8|output_buffer[5]);

    float m_vx = (float)(vx) / IMU_GY_RANGE;
    float m_vy = (float)(vy) / IMU_GY_RANGE;
    float m_vz = (float)(vz) / IMU_GY_RANGE;

    for(uint8_t i = 0; i < 6; i++)
    {
      buff[i] = output_buffer[i];
    }
}


void Read_Accelerometer(){
    uint8_t output_buffer[6];
    Read_Data(MPU6050_RA_ACCEL_XOUT_H,output_buffer);
    int16_t ax = (int16_t)(output_buffer[0]<<8|output_buffer[1]);
    int16_t ay = (int16_t)(output_buffer[2]<<8|output_buffer[3]);
    int16_t az  = (int16_t)(output_buffer[4]<<8|output_buffer[5]);

    imuSensor.m_ax = -(ax * g / ACC_RANGE);
    imuSensor.m_ay = -(ay * g / ACC_RANGE);
    imuSensor.m_az = -(az * g / ACC_RANGE);
}

void Read_Accelerometer_IT(uint8_t *buff){
    uint8_t output_buffer[6];

    if(Read_Data(MPU6050_RA_ACCEL_XOUT_H,output_buffer)){
      // Try fix for flag bit silicon bug
      generateClocks(1, 1);
    }

    int16_t ax = (int16_t)(output_buffer[0]<<8|output_buffer[1]);
    int16_t ay = (int16_t)(output_buffer[2]<<8|output_buffer[3]);
    int16_t az  = (int16_t)(output_buffer[4]<<8|output_buffer[5]);

    float m_ax = -(ax * g / ACC_RANGE);
    float m_ay = -(ay * g / ACC_RANGE);
    float m_az = -(az * g / ACC_RANGE);

    for(uint8_t i = 0; i < 6; i++)
    {
      buff[i] = output_buffer[i];
    }
}

void Fill_Struct(ImuStruct_t* p_data){
    p_data->x_Accel = imuSensor.m_ax;
    p_data->y_Accel = imuSensor.m_ay;
    p_data->z_Accel = imuSensor.m_az;

    p_data->x_Gyro = imuSensor.m_vx;
    p_data->y_Gyro = imuSensor.m_vy;
    p_data->z_Gyro = imuSensor.m_vz;
}
/********************************* MPU6050 end ***********************************/
