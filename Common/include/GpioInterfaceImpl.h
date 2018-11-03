/**
  *****************************************************************************
  * @file    GpioInterfaceImpl.h
  * @author  Tyler Gamvrelis
  *
  * @defgroup Header
  * @ingroup  GPIO
  * @{
  *****************************************************************************
  */




#ifndef GPIO_INTERFACE_IMPL_H
#define GPIO_INTERFACE_IMPL_H




/********************************* Includes **********************************/
#include "GpioInterface.h"




/***************************** GpioInterfaceImpl *****************************/
namespace gpio{
// Classes and structs
// ----------------------------------------------------------------------------
class GpioInterfaceImpl : public GpioInterface{
public:
    GpioInterfaceImpl();
    ~GpioInterfaceImpl();

    GPIO_PinState readPin(
         GPIO_TypeDef* GPIOx,
         uint16_t GPIO_Pin
     ) const override final;

    void writePin(
        GPIO_TypeDef* GPIOx,
        uint16_t GPIO_Pin,
        GPIO_PinState PinState
    ) const override final;

    void togglePin(
        GPIO_TypeDef* GPIOx,
        uint16_t GPIO_Pin
    ) const override final;
};

} // end namespace gpio




/**
 * @}
 */
/* end - GPIO */

#endif /* GPIO_INTERFACE_IMPL_H */
