#pragma once

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_gpio.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "main.h"


// -------------------------
// Reference file: https://github.com/utra-robosoccer/soccerbot/blob/firmware/soccer_embedded/Common/component/MPU6050/MPU6050.h


#define MPU6050_ADDR 0xD0
#define MPU6050_RA_GYRO_CONFIG 0x1B
#define MPU6050_RA_ACCEL_CONFIG 0x1C
#define MPU6050_RA_I2C_MST_CTRL 0x24
#define MPU6050_RA_SMPLRT_DIV 0x19

// Output
#define MPU6050_RA_ACCEL_XOUT_H 0x3B
#define MPU6050_RA_GYRO_XOUT_H 0x43
#define MPU6050_RA_PWR_MGMT_1 0x6B
#define MPU6050_RA_PWR_MGMT_2 0x6C

// Sample Rate DIV
#define MPU6050_CLOCK_DIV_296 0x4

//---------------  Constants -------------
extern float g;

// Unit coefficient constants
extern uint8_t IMU_GY_RANGE; /**< divide by this to get degrees per second */
extern float ACC_RANGE;  /**< divide to get in units of g */

//---------------  end Constants -------------


typedef struct {
	I2C_HandleTypeDef* m_i2c_handle; /**< I2C handle associated with sensor instance */
	uint8_t 				 devAddr;
	float                    m_vx;         /**< x-axis angular velocity read from sensor */
	float                    m_vy;         /**< y-axis angular velocity read from sensor */
	float                    m_vz;         /**< z-axis angular velocity read from sensor */
	float                    m_ax;         /**< x-axis acceleration read from sensor */
	float                    m_ay;         /**< y-axis acceleration read from sensor */
	float                    m_az;         /**< z-axis acceleration read from sensor */
	uint8_t                  m_recv_byte;
} ImuSensor;


/**
 * @brief The data structure which represents the data of the IMU, which is sent
 * in queues between tasks
 */
typedef struct{
    float x_Gyro;  /**< x-axis angular velocity read from sensor */
    float y_Gyro;  /**< y-axis angular velocity read from sensor */
    float z_Gyro;  /**< z-axis angular velocity read from sensor */
    float x_Accel; /**< x-axis acceleration read from sensor     */
    float y_Accel; /**< y-axis acceleration read from sensor     */
    float z_Accel; /**< z-axis acceleration read from sensor     */
}ImuStruct_t;

// -------- Function Prototypes ---------------

void MPU6050_init();

 /**
   * @brief   Writes to a register from the MPU6050
   * @param   reg_addr uint8_t address of the register
   * @param   data uint8_t data to be written
   * @return  Status
   */
HAL_StatusTypeDef Write_Reg(uint8_t reg_addr, uint8_t data);


/**
   * @brief   Reads a byte from a register on the sensor, and stores it
   *          in the member variable received_byte
   * @param   reg_addr uint8_t address of the register
   * @return  Status
   */
HAL_StatusTypeDef Read_Reg(uint8_t reg_addr);


/**
     * @brief   Reads 6 bytes from the sensor with interrupts, and stores them in
     *          the sensor_buffer
     * @param   reg_addr uint8_t address of the register
     * @param   sensor_buffer uint8_t pointer to output buffer
     * @return  Status
     */
HAL_StatusTypeDef Read_Data_IT(uint8_t reg_addr, uint8_t* sensor_buffer);


/**
      * @brief   Sets the offsets of the sensor. Note that a lower setting
      *          implies smaller bandwidth but higher signal delay
      * @param   lpf The setting being used for the built-in DLPF. Must be
      *          less than or equal to 6 (6 -> 5 Hz bandwidth, 0 -> about 260
      *          Hz bandwidth)
      * @return  Status
      */
bool Set_LPF(uint8_t lpf);

/**
  * @brief   Reads the gyroscope with offsets without interrupts
  */
void Read_Gyroscope();

void Read_Gyroscope_IT(uint8_t *buff);


/**
  * @brief   Reads the accelerometer with offsets without interrupts
  */
void Read_Accelerometer();

void Read_Accelerometer_IT(uint8_t *buff);

/**
  * @brief Fills an `ImuStruct_t` with the current sensor data
  * @param p_data Pointer to the struct being filled
  */
void Fill_Struct(ImuStruct_t* p_data);

void generateClocks(uint8_t num_clocks, uint8_t send_stop_bits);

uint8_t wait_for_gpio_state_timeout(
    GPIO_TypeDef* port,
    uint16_t pin,
    GPIO_PinState state,
    uint8_t timeout
);
