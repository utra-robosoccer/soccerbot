/**
 *****************************************************************************
 * @file    HalI2CInterface.cpp
 * @author  Hannah
 * @brief   Performs HAL related functions for I2C Interface
 *
 * @defgroup HalI2CInterface
 * @ingroup  I2C
 * @brief    HAL related functions for I2C Interface
 * @{
 *****************************************************************************
 */

/********************************* Includes **********************************/
#include "HalI2CInterface.h"


namespace i2c{
/**************************** HAL I2C Interface ******************************/
// Public
HALI2CInterface::HALI2CInterface() {}
HALI2CInterface::~HALI2CInterface() {}

HAL_StatusTypeDef HALI2CInterface::memWrite(I2C_HandleTypeDef *i2cHandlePtr, uint16_t DevAddress,
        uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size,
        uint32_t Timeout) const { //HAL_I2C_Mem_Write

    return HAL_I2C_Mem_Write(i2cHandlePtr, DevAddress, MemAddress, MemAddSize,
            pData, Size, Timeout);
}

HAL_StatusTypeDef HALI2CInterface::memRead(I2C_HandleTypeDef *i2cHandlePtr, uint16_t DevAddress,
        uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size,
        uint32_t Timeout) const { //HAL_I2C_Mem_Read
    return HAL_I2C_Mem_Read(i2cHandlePtr, DevAddress, MemAddress, MemAddSize,
            pData, Size, Timeout);
}

HAL_StatusTypeDef HALI2CInterface::memWriteIT(I2C_HandleTypeDef *i2cHandlePtr, uint16_t DevAddress,
        uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData,
        uint16_t Size) const { //HAL_I2C_Mem_Write_IT
    return HAL_I2C_Mem_Write_IT(i2cHandlePtr, DevAddress, MemAddress,
            MemAddSize, pData, Size);
}

HAL_StatusTypeDef HALI2CInterface::memReadIT(I2C_HandleTypeDef *i2cHandlePtr, uint16_t DevAddress,
        uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData,
        uint16_t Size) const { //HAL_I2C_Mem_Read_IT
    return HAL_I2C_Mem_Read_IT(i2cHandlePtr, DevAddress, MemAddress, MemAddSize,
            pData, Size);
}
// ----------------------------------------------------------------------------

} /*end i2c namespace*/

/**
 * @}
 */
/* end - HalI2CInterface */

