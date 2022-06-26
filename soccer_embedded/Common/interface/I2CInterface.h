/**
 *****************************************************************************
 * @file
 * @author  Hannah L
 *
 * @defgroup I2CInterface
 * @ingroup  I2C
 * @{
 *****************************************************************************
 */

#ifndef COMMON_INCLUDE_I2CINTERFACE_H
#define COMMON_INCLUDE_I2CINTERFACE_H

/********************************* Includes **********************************/
#include <stdint.h>
#include "i2c.h"

/******************************* I2C Interface *******************************/
namespace hal {

// Classes and Structs
/**
 * @class I2CInterface Abstract class defining the interface that a
 *        hardware-facing I2C object must have. This object takes care of
 *        directly interfacing with the hardware as it is instructed, without
 *        knowledge of the application logic
 */
class I2CInterface {
public:
    virtual ~I2CInterface();
    /**
     * @brief Associates the class with a particular i2c module
     * @param i2cHandlePtr Pointer to a structure that contains
     *        the configuration information for the desired i2c module
     */
    virtual void setI2CPointer(I2C_HandleTypeDef *i2cHandlePtr) const = 0;

    /**
     * @brief  Write an amount of data in blocking mode to a specific memory address
     * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
     *                the configuration information for the specified I2C.
     * @param  DevAddress Target device address
     * @param  MemAddress Internal memory address
     * @param  MemAddSize Size of internal memory address
     * @param  pData Pointer to data buffer
     * @param  Size Amount of data to be sent
     * @param  Timeout Timeout duration
     * @retval HAL status
     */
    virtual HAL_StatusTypeDef memWrite(I2C_HandleTypeDef *i2cHandlePtr,
            uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize,
            uint8_t *pData, uint16_t Size, uint32_t Timeout) const = 0;

    /**
     * @brief  Read an amount of data in blocking mode from a specific memory address
     * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
     *                the configuration information for the specified I2C.
     * @param  DevAddress Target device address
     * @param  MemAddress Internal memory address
     * @param  MemAddSize Size of internal memory address
     * @param  pData Pointer to data buffer
     * @param  Size Amount of data to be sent
     * @param  Timeout Timeout duration
     * @retval HAL status
     */
    virtual HAL_StatusTypeDef memRead(I2C_HandleTypeDef *i2cHandlePtr,
            uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize,
            uint8_t *pData, uint16_t Size, uint32_t Timeout) const = 0;

    /**
     * @brief  Write an amount of data in non-blocking mode with Interrupt to a specific memory address
     * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
     *                the configuration information for the specified I2C.
     * @param  DevAddress Target device address
     * @param  MemAddress Internal memory address
     * @param  MemAddSize Size of internal memory address
     * @param  pData Pointer to data buffer
     * @param  Size Amount of data to be sent
     * @retval HAL status
     */
    virtual HAL_StatusTypeDef memWriteIT(I2C_HandleTypeDef *i2cHandlePtr,
            uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize,
            uint8_t *pData, uint16_t Size) const = 0;

    /**
     * @brief  Read an amount of data in non-blocking mode with Interrupt from a specific memory address
     * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
     *                the configuration information for the specified I2C.
     * @param  DevAddress Target device address
     * @param  MemAddress Internal memory address
     * @param  MemAddSize Size of internal memory address
     * @param  pData Pointer to data buffer
     * @param  Size Amount of data to be sent
     * @retval HAL status
     */
    virtual HAL_StatusTypeDef memReadIT(I2C_HandleTypeDef *i2cHandlePtr,
            uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize,
            uint8_t *pData, uint16_t Size) const = 0;
};
// ----------------------------------------------------------------------------
}// end namespace hal

#endif /* COMMON_INCLUDE_I2CINTERFACE_H */
