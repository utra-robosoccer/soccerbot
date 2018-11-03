/**
 *****************************************************************************
 * @file    HalI2CInterface.h
 * @author  Hannah L
 * @brief   Defines the HALI2CInterface class, which calls HAL functions related to I2C
 *
 * @defgroup HalI2CInterface
 * @ingroup  I2C
 * @{
 *****************************************************************************
 */

#ifndef COMMON_HALI2CINTERFACE_H_
#define COMMON_HALI2CINTERFACE_H_

/********************************* Includes **********************************/

#include "I2CInterface.h"
#include "SystemConf.h"

/****************************** HAL I2C Interface ****************************/
namespace i2c{
/**
 * @class Concrete HAL implementation of the abstract I2CInterface class, to be
 *        used in production builds
 */
class HALI2CInterface: public I2CInterface {
public:
    HALI2CInterface();
    ~HALI2CInterface();
    HAL_StatusTypeDef memWrite(I2C_HandleTypeDef *i2cHandlePtr, uint16_t DevAddress, uint16_t MemAddress,
            uint16_t MemAddSize, uint8_t *pData, uint16_t Size,
            uint32_t Timeout) const; //HAL_I2C_Mem_Write
    HAL_StatusTypeDef memRead(I2C_HandleTypeDef *i2cHandlePtr, uint16_t DevAddress, uint16_t MemAddress,
            uint16_t MemAddSize, uint8_t *pData, uint16_t Size,
            uint32_t Timeout) const; //HAL_I2C_Mem_Read
    HAL_StatusTypeDef memWriteIT(I2C_HandleTypeDef *i2cHandlePtr, uint16_t DevAddress, uint16_t MemAddress,
            uint16_t MemAddSize, uint8_t *pData, uint16_t Size) const; //HAL_I2C_Mem_Write_IT
    HAL_StatusTypeDef memReadIT(I2C_HandleTypeDef *i2cHandlePtr, uint16_t DevAddress, uint16_t MemAddress,
            uint16_t MemAddSize, uint8_t *pData, uint16_t Size) const; //HAL_I2C_Mem_Read_IT
};

} //end namespace i2c

/**
 * @}
 */
/* end - Header */

#endif /* COMMON_HALI2CINTERFACE_H_ */
