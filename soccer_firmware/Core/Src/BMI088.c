#include "BMI088.h"

/*
 *
 * INITIALISATION
 *
 */
uint8_t BMI088_Init(BMI088 *imu, I2C_HandleTypeDef *m_i2c_handle) {

  /* Store interface parameters in struct */
  imu->m_i2c_handle    = m_i2c_handle;
//
//  /* Clear DMA flags */
//  imu->readingAcc = 0;
//  imu->readingGyr = 0;

  uint8_t status = 0;

  /*
   *
   * ACCELEROMETER
   *
   */

  /* Accelerometer requires rising edge on CSB at start-up to activate SPI */
//  HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_RESET);
//  HAL_Delay(1);
//  HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_SET);
//  HAL_Delay(50);

  /* Perform accelerometer soft reset */
//  status += BMI088_WriteAccRegister(imu, BMI_ACC_SOFTRESET, 0xB6);
//  HAL_Delay(50);

  /* Check chip ID */
  uint8_t chipID = 0;
  status = BMI088_ReadAccRegister(imu, BMI_ACC_CHIP_ID, &chipID);

  if (chipID != 0x1E) {
//    while(true){
////      generateClocks2(1, 1);
//      status = BMI088_ReadAccRegister(imu, BMI_ACC_CHIP_ID, &chipID);
////      status = BMI088_ReadGyrRegister(imu, BMI_GYR_CHIP_ID, &chipID);
//    }
    return 0;

  }
  HAL_Delay(10);

  /* Configure accelerometer  */
  status += BMI088_WriteAccRegister(imu, BMI_ACC_CONF, 0xA8); /* (no oversampling, ODR = 100 Hz, BW = 40 Hz) */
  HAL_Delay(10);

  status += BMI088_WriteAccRegister(imu, BMI_ACC_RANGE, 0x00); /* +- 3g range */
  HAL_Delay(10);

  /* Enable accelerometer data ready interrupt */
  status += BMI088_WriteAccRegister(imu, BMI_INT1_IO_CONF, 0x0A); /* INT1 = push-pull output, active high */
  HAL_Delay(10);

  status += BMI088_WriteAccRegister(imu, BMI_INT1_INT2_MAP_DATA, 0x04);
  HAL_Delay(10);

  /* Put accelerometer into active mode */
  status += BMI088_WriteAccRegister(imu, BMI_ACC_PWR_CONF, 0x00);
  HAL_Delay(10);

  /* Turn accelerometer on */
  status += BMI088_WriteAccRegister(imu, BMI_ACC_PWR_CTRL, 0x04);
  HAL_Delay(10);

  /* Pre-compute accelerometer conversion constant (raw to m/s^2) */
  imu->accConversion = 9.81f / 32768.0f * 2.0f * 1.5f; /* Datasheet page 27 */

  /* Set accelerometer TX buffer for DMA */
  imu->accTxBuf[0] = BMI_ACC_DATA | 0x80;

  /*
   *
   * GYROSCOPE
   *
   */

//  HAL_GPIO_WritePin(imu->csGyrPinBank, imu->csGyrPin, GPIO_PIN_SET);

  /* Perform gyro soft reset */
  status += BMI088_WriteGyrRegister(imu, BMI_GYR_SOFTRESET, 0xB6);
  HAL_Delay(250);

  /* Check chip ID */
  status += BMI088_ReadGyrRegister(imu, BMI_GYR_CHIP_ID, &chipID);

  if (chipID != 0x0F) {

    //return 0;
    chipID++;

  }
  HAL_Delay(10);

  /* Configure gyroscope */
  status += BMI088_WriteGyrRegister(imu, BMI_GYR_RANGE, 0x01); /* +- 1000 deg/s */
  HAL_Delay(10);

  status += BMI088_WriteGyrRegister(imu, BMI_GYR_BANDWIDTH, 0x07); /* ODR = 100 Hz, Filter bandwidth = 32 Hz */
  HAL_Delay(10);

  /* Enable gyroscope data ready interrupt */
  status += BMI088_WriteGyrRegister(imu, BMI_GYR_INT_CTRL, 0x80); /* New data interrupt enabled */
  HAL_Delay(10);

  status += BMI088_WriteGyrRegister(imu, BMI_INT3_INT4_IO_CONF, 0x01); /* INT3 = push-pull, active high */
  HAL_Delay(10);

  status += BMI088_WriteGyrRegister(imu, BMI_INT3_INT4_IO_MAP, 0x01); /* Data ready interrupt mapped to INT3 pin */
  HAL_Delay(10);

  /* Pre-compute gyroscope conversion constant (raw to rad/s) */
  imu->gyrConversion = 0.01745329251f * 1000.0f / 32768.0f; /* Datasheet page 39 */

  /* Set gyroscope TX buffer for DMA */
  imu->gyrTxBuf[0] = BMI_GYR_DATA | 0x80;

  return status;

}

/*
 *
 * LOW-LEVEL REGISTER FUNCTIONS
 *
 */

/* ACCELEROMETER READS ARE DIFFERENT TO GYROSCOPE READS. SEND ONE BYTE ADDRESS, READ ONE DUMMY BYTE, READ TRUE DATA !!! */
uint8_t BMI088_ReadAccRegister(BMI088 *imu, uint8_t regAddr, uint8_t *data) {
  HAL_StatusTypeDef status;

  // Read data from the specified register address
  status = HAL_I2C_Mem_Read(imu->m_i2c_handle, BMI_ACC_I2C_ADDRESS, regAddr, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
  if (status != HAL_OK) {
      return status;
  }

  return HAL_OK;
}

uint8_t BMI088_ReadGyrRegister(BMI088 *imu, uint8_t regAddr, uint8_t *data) {
  HAL_StatusTypeDef status;

  // Read data from the specified register address
  status = HAL_I2C_Mem_Read(imu->m_i2c_handle, BMI_GYR_I2C_ADDRESS, regAddr, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
  if (status != HAL_OK) {
      return status;
  }

  return HAL_OK;
}

uint8_t BMI088_WriteAccRegister(BMI088 *imu, uint8_t regAddr, uint8_t data) {
  HAL_StatusTypeDef status;

  // Write data to the specified register address
  status = HAL_I2C_Mem_Write(imu->m_i2c_handle, BMI_ACC_I2C_ADDRESS, regAddr, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
  if (status != HAL_OK) {
      return status;
  }

  return HAL_OK;
}

uint8_t BMI088_WriteGyrRegister(BMI088 *imu, uint8_t regAddr, uint8_t data) {
  HAL_StatusTypeDef status;

  // Write data to the specified register address
  status = HAL_I2C_Mem_Write(imu->m_i2c_handle, BMI_GYR_I2C_ADDRESS, regAddr, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
  if (status != HAL_OK) {
      return status;
  }

  return HAL_OK;
}



/*
 *
 * POLLING
 *
 */
uint8_t BMI088_ReadAccelerometer(BMI088 *imu) {

//  /* Read raw accelerometer data */
//  uint8_t txBuf[8] = {(BMI_ACC_DATA | 0x80), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; /* Register addr, 1 byte dummy, 6 bytes data */
//  uint8_t rxBuf[8];
//
//  HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_RESET);
//  uint8_t status = (HAL_SPI_TransmitReceive(imu->spiHandle, txBuf, rxBuf, 8, HAL_MAX_DELAY) == HAL_OK);
//  HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_SET);
//
//  /* Form signed 16-bit integers */
//  int16_t accX = (int16_t) ((rxBuf[3] << 8) | rxBuf[2]);
//  int16_t accY = (int16_t) ((rxBuf[5] << 8) | rxBuf[4]);
//  int16_t accZ = (int16_t) ((rxBuf[7] << 8) | rxBuf[6]);
//
//  /* Convert to m/s^2 */
//  imu->acc_mps2[0] = imu->accConversion * accX;
//  imu->acc_mps2[1] = imu->accConversion * accY;
//  imu->acc_mps2[2] = imu->accConversion * accZ;
//
//  return status;
//  return 0;

}


// Function to read gyroscope values from BMI088 sensor
HAL_StatusTypeDef readGyroscopeFromBMI088(I2C_HandleTypeDef *hi2c, int16_t *gyroX, int16_t *gyroY, int16_t *gyroZ) {
    HAL_StatusTypeDef status;
    uint8_t data[6]; // Data buffer to store gyroscope data

    // Read gyroscope data starting from register 0x0C (Gyroscope data starting register)
    uint8_t regAddress = 0x02;
    status = HAL_I2C_Mem_Read(hi2c, BMI_GYR_I2C_ADDRESS, regAddress, I2C_MEMADD_SIZE_8BIT, data, 6, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return status;
    }

    // Parse gyroscope data (each value is 16 bits, little-endian)
    *gyroX = (int16_t)((data[1] << 8) | data[0]);
    *gyroY = (int16_t)((data[3] << 8) | data[2]);
    *gyroZ = (int16_t)((data[5] << 8) | data[4]);

    return HAL_OK;
}

/*
 *
 * DMA
 *
 */
uint8_t BMI088_ReadAccelerometerDMA(BMI088 *imu) {

//  HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_RESET);
//  if (HAL_SPI_TransmitReceive_DMA(imu->spiHandle, imu->accTxBuf, (uint8_t *) imu->accRxBuf, 8) == HAL_OK) {
//
//    imu->readingAcc = 1;
//    return 1;
//
//  } else {
//
//    HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_SET);
//    return 0;
//
//  }

}

void BMI088_ReadAccelerometerDMA_Complete(BMI088 *imu) {

//  HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_SET);
//  imu->readingAcc = 0;
//
//  /* Form signed 16-bit integers */
//  int16_t accX = (int16_t) ((imu->accRxBuf[3] << 8) | imu->accRxBuf[2]);
//  int16_t accY = (int16_t) ((imu->accRxBuf[5] << 8) | imu->accRxBuf[4]);
//  int16_t accZ = (int16_t) ((imu->accRxBuf[7] << 8) | imu->accRxBuf[6]);
//
//  /* Convert to m/s^2 */
//  imu->acc_mps2[0] = imu->accConversion * accX;
//  imu->acc_mps2[1] = imu->accConversion * accY;
//  imu->acc_mps2[2] = imu->accConversion * accZ;

}

uint8_t BMI088_ReadGyroscopeDMA(BMI088 *imu) {

//  HAL_GPIO_WritePin(imu->csGyrPinBank, imu->csGyrPin, GPIO_PIN_RESET);
//  if (HAL_SPI_TransmitReceive_DMA(imu->spiHandle, imu->gyrTxBuf, (uint8_t *) imu->gyrRxBuf, 7) == HAL_OK) {
//
//    imu->readingGyr = 1;
//    return 1;
//
//  } else {
//
//    HAL_GPIO_WritePin(imu->csGyrPinBank, imu->csGyrPin, GPIO_PIN_SET);
//    return 0;
//
//  }

}

void BMI088_ReadGyroscopeDMA_Complete(BMI088 *imu) {

//  HAL_GPIO_WritePin(imu->csGyrPinBank, imu->csGyrPin, GPIO_PIN_SET);
//  imu->readingGyr = 0;
//
//  /* Form signed 16-bit integers */
//  int16_t gyrX = (int16_t) ((imu->gyrRxBuf[2] << 8) | imu->gyrRxBuf[1]);
//  int16_t gyrY = (int16_t) ((imu->gyrRxBuf[4] << 8) | imu->gyrRxBuf[3]);
//  int16_t gyrZ = (int16_t) ((imu->gyrRxBuf[6] << 8) | imu->gyrRxBuf[5]);
//
//  /* Convert to deg/s */
//  imu->gyr_rps[0] = imu->gyrConversion * gyrX;
//  imu->gyr_rps[1] = imu->gyrConversion * gyrY;
//  imu->gyr_rps[2] = imu->gyrConversion * gyrZ;

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
void generateClocks2(uint8_t num_clocks, uint8_t send_stop_bits){
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
