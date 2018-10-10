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




/************************** insert module name here **************************/
// TODO: pick better namespace for this component
namespace dynamixel{
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

    const UartDriver* uartDriver;    /**< @see UartDriver               */
    const GpioInterface* gpioif;     /**< @see GpioInterface            */
    const GPIO_TypeDef* dataDirPort; /**< Port data direction pin is on */
    const uint16_t dataDirPinNum;    /**< Data direction pin number     */
};

} // end namespace dynamixel




/**
 * @}
 */
/* end - DaisyChain */

#endif /* DAISY_CHAIN_H */
