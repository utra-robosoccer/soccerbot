/**
  *****************************************************************************
  * @file    GpioInterface.h
  * @author  Tyler Gamvrelis
  * @brief   Abstract interface for GPIO functions
  *
  * @defgroup GpioInterface
  * @ingroup  GPIO
  * @{
  *****************************************************************************
  */




#ifndef GPIO_INTERFACE_H
#define GPIO_INTERFACE_H




/********************************* Includes **********************************/
#include <stdint.h>
#include "SystemConf.h"
#include "gpio.h"




/******************************* GpioInterface *******************************/
namespace gpio{
// Classes and structs
// ----------------------------------------------------------------------------
class GpioInterface{
public:
    virtual ~GpioInterface() {}

    virtual GPIO_PinState readPin(
         GPIO_TypeDef* GPIOx,
         uint16_t GPIO_Pin
    ) const = 0;

    virtual void writePin(
        GPIO_TypeDef* GPIOx,
        uint16_t GPIO_Pin,
        GPIO_PinState PinState
    ) const = 0;

    virtual void togglePin(
        GPIO_TypeDef* GPIOx,
        uint16_t GPIO_Pin
    ) const = 0;
};

} // end namespace gpio




/**
 * @}
 */
/* end - GPIO */

#endif /* GPIO_INTERFACE_H */
