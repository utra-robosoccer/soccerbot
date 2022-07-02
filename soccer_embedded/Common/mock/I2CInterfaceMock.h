/**
 *****************************************************************************
 * @file
 * @author  Hannah L
 * @brief   Defines the MockI2CInterface class using gmock
 *
 * @defgroup I2CInterfaceMock
 * @ingroup  I2C
 * @{
 *****************************************************************************
 */

#ifndef COMMON_INCLUDE_I2CINTERFACEMOCK_H_
#define COMMON_INCLUDE_I2CINTERFACEMOCK_H_

/********************************* Includes **********************************/
#include "I2CInterface.h"
#include <gtest/gtest.h>
#include <gmock/gmock.h>

/***************************** Mock I2C Interface ****************************/
using hal::I2CInterface;

namespace hal {
namespace gmock {

/**
 * @class MockI2CInterface Emulates I2CInterface for unit testing purposes
 */
class I2CInterfaceMock: public I2CInterface{
public:
	MOCK_CONST_METHOD1(assignHandle, void(I2C_HandleTypeDef *hi2c));
	MOCK_CONST_METHOD6(memWrite , HAL_StatusTypeDef(uint16_t DevAddress, uint16_t MemAddress,
				uint16_t MemAddSize, uint8_t *pData, uint16_t Size,
				uint32_t Timeout));
	MOCK_CONST_METHOD6(memRead , HAL_StatusTypeDef(uint16_t DevAddress, uint16_t MemAddress,
					uint16_t MemAddSize, uint8_t *pData, uint16_t Size,
					uint32_t Timeout));
	MOCK_CONST_METHOD5(memWrite_IT , HAL_StatusTypeDef(uint16_t DevAddress, uint16_t MemAddress,
					uint16_t MemAddSize, uint8_t *pData, uint16_t Size));
	MOCK_CONST_METHOD5(memRead_IT , HAL_StatusTypeDef(uint16_t DevAddress, uint16_t MemAddress,
						uint16_t MemAddSize, uint8_t *pData, uint16_t Size));
}

} // end namespace gmock
} // end namespace hal

/**
 * @}
 */
/* end I2CInterfaceMock */

#endif /* COMMON_INCLUDE_I2CINTERFACEMOCK_H_ */
