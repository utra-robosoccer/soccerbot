/**
 *****************************************************************************
 * @file
 * @author  Hannah
 * @brief   Performs HAL related functions for I2C Interface
 *
 * @defgroup I2CInterfaceImpl
 * @ingroup  I2C
 * @brief    HAL related functions for I2C Interface
 * @{
 *****************************************************************************
 */

/********************************* Includes **********************************/
#include "I2CInterfaceImpl.h"


namespace hal{
/**************************** HAL I2C Interface ******************************/
// Public
I2CInterfaceImpl::I2CInterfaceImpl() {}
I2CInterfaceImpl::~I2CInterfaceImpl() {}

HAL_StatusTypeDef I2CInterfaceImpl::memWrite(I2C_HandleTypeDef *i2cHandlePtr, uint16_t DevAddress,
        uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size,
        uint32_t Timeout) const { //HAL_I2C_Mem_Write

    return HAL_I2C_Mem_Write(i2cHandlePtr, DevAddress, MemAddress, MemAddSize,
            pData, Size, Timeout);
}

HAL_StatusTypeDef I2CInterfaceImpl::memRead(I2C_HandleTypeDef *i2cHandlePtr, uint16_t DevAddress,
        uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size,
        uint32_t Timeout) const { //HAL_I2C_Mem_Read
    return HAL_I2C_Mem_Read(i2cHandlePtr, DevAddress, MemAddress, MemAddSize,
            pData, Size, Timeout);
}

HAL_StatusTypeDef I2CInterfaceImpl::memWriteIT(I2C_HandleTypeDef *i2cHandlePtr, uint16_t DevAddress,
        uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData,
        uint16_t Size) const { //HAL_I2C_Mem_Write_IT
    return HAL_I2C_Mem_Write_IT(i2cHandlePtr, DevAddress, MemAddress,
            MemAddSize, pData, Size);
}

HAL_StatusTypeDef I2CInterfaceImpl::memReadIT(I2C_HandleTypeDef *i2cHandlePtr, uint16_t DevAddress,
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
/* end - I2CInterfaceImpl */

