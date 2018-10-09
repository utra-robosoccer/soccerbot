/**
  *****************************************************************************
  * @file    DaisyChain.h
  * @author  Tyler Gamvrelis
  *
  * @defgroup DaisyChain
  * @{
  *****************************************************************************
  */




#ifndef DAISY_CHAIN_H
#define DAISY_CHAIN_H




/********************************* Includes **********************************/
#include <cstdint>
#include "UartDriver.h"
#include "GpioInterface.h"

using uart::UartDriver;
using gpio::GpioInterface;




/********************************** Globals **********************************/
// Constants
// ----------------------------------------------------------------------------




/************************** insert module name here **************************/
namespace dynamixel{
// Constants
// ----------------------------------------------------------------------------




// Types & enums
// ----------------------------------------------------------------------------




// Classes and structs
// ----------------------------------------------------------------------------
struct DaisyChainParams{
    UartDriver* uartDriver;
    GpioInterface* gpioif;
    GPIO_TypeDef* dataDirPort;
    uint16_t dataDirPinNum;
};


class DaisyChain{
public:
    DaisyChain(DaisyChainParams& params);
    ~DaisyChain();
    bool requestTransmission(uint8_t* arr, size_t arrSize) const;
    bool requestReception(uint8_t* buf, size_t bufSize) const;

private:
    using Direction = enum class Direction{
        RX,
        TX
    };

    void changeBusDir(Direction dir) const;

    const UartDriver* uartDriver; /**< @see UartDriver */
    const GpioInterface* gpioif;  /**< @see GpioInterface */
    GPIO_TypeDef* dataDirPort; /**< Port data direction pin is on */
    uint16_t dataDirPinNum;    /**< Data direction pin number     */
};



// Functions
// ----------------------------------------------------------------------------




} // end namespace module_name




/***************************** Inline functions ******************************/




/**
 * @}
 */
/* end - Header */

#endif /* DAISY_CHAIN_H */
