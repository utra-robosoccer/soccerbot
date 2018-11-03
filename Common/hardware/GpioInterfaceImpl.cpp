/**
  *****************************************************************************
  * @file    GpioInterfaceImpl.cpp
  * @author  Tyler Gamvrelis
  *
  * @defgroup Implementation
  * @ingroup  gpio
  * @{
  *****************************************************************************
  */




/********************************* Includes **********************************/
#include "GpioInterfaceImpl.h"





/***************************** GpioInterfaceImpl *****************************/
namespace gpio{
// Classes and structs
// ----------------------------------------------------------------------------
GpioInterfaceImpl::GpioInterfaceImpl() {

}

GpioInterfaceImpl::~GpioInterfaceImpl() {

}

GPIO_PinState GpioInterfaceImpl::readPin(
     GPIO_TypeDef* GPIOx,
     uint16_t GPIO_Pin
 ) const
{
    return HAL_GPIO_ReadPin(
        GPIOx,
        GPIO_Pin
    );
}

void GpioInterfaceImpl::writePin(
    GPIO_TypeDef* GPIOx,
    uint16_t GPIO_Pin,
    GPIO_PinState PinState
) const
{
    HAL_GPIO_WritePin(
        GPIOx,
        GPIO_Pin,
        PinState
    );
}

void GpioInterfaceImpl::togglePin(
    GPIO_TypeDef* GPIOx,
    uint16_t GPIO_Pin
) const
{
    HAL_GPIO_TogglePin(
        GPIOx,
        GPIO_Pin
    );
}

} // end namespace gpio

/**
 * @}
 */
/* end - Implementation */
