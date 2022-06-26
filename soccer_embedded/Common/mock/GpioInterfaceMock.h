/**
  *****************************************************************************
  * @file
  * @author  Tyler Gamvrelis
  * @brief   Mocks GPIO functions
  *
  * @defgroup GpioInterfaceMock
  * @ingroup  Mocks
  * @{
  *****************************************************************************
  */




#ifndef GPIO_INTERFACE_MOCK_H
#define GPIO_INTERFACE_MOCK_H




/********************************* Includes **********************************/
#include "GpioInterface.h"
#include <gtest/gtest.h>
#include <gmock/gmock.h>

using hal::GpioInterface;




/****************************** MockGpioInterface ****************************/
namespace hal {
namespace gmock {
// Classes and structs
// ----------------------------------------------------------------------------
class GpioInterfaceMock : public GpioInterface{
public:
    MOCK_CONST_METHOD2(
        readPin,
        GPIO_PinState(
            GPIO_TypeDef*,
            uint16_t
        )
    );

    MOCK_CONST_METHOD3(
        writePin,
        void(
            GPIO_TypeDef*,
            uint16_t,
            GPIO_PinState
        )
    );

    MOCK_CONST_METHOD2(
        togglePin,
        void(
            GPIO_TypeDef*,
            uint16_t
        )
    );
};

} // end namespace gmock
} // end namespace hal




/**
 * @}
 */
/* end - GpioInterfaceMock */

#endif /* GPIO_INTERFACE_MOCK_H */
