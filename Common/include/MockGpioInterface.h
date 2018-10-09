/**
  *****************************************************************************
  * @file    MockGpioInterface.h
  * @author  Tyler Gamvrelis
  * @brief   Mocks GPIO functions
  *
  * @defgroup MockGpioInterface
  * @ingroup  Mocks
  * @{
  *****************************************************************************
  */




#ifndef MOCK_GPIO_INTERFACE_H
#define MOCK_GPIO_INTERFACE_H




/********************************* Includes **********************************/
#include "GpioInterface.h"
#include <gtest/gtest.h>
#include <gmock/gmock.h>

using gpio::GpioInterface;




/****************************** MockGpioInterface ****************************/
namespace mocks{
// Classes and structs
// ----------------------------------------------------------------------------
class MockGpioInterface : public GpioInterface{
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

} // end namespace mocks




/**
 * @}
 */
/* end - MockGpioInterface */

#endif /* MOCK_GPIO_INTERFACE_H */
