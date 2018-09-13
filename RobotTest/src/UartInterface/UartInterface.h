/**
  *****************************************************************************
  * @file    UartInterface.h
  * @author  Tyler Gamvrelis
  * @brief   Abstract interface for UART hardware, to be implemented by
  *          hardware-facing classes and test frameworks
  *
  * @defgroup Header
  * @ingroup  UartInterface
  * @{
  *****************************************************************************
  */




#ifndef __UART_INTERFACE_H__
#define __UART_INTERFACE_H__




/********************************* Includes **********************************/
#include <cstdint>




/******************************* uart_interface ******************************/
namespace uart_interface{
// Classes and structs
// ----------------------------------------------------------------------------
class UartInterface{
public:
    virtual ~UartInterface() {}
    virtual bool transmit(uint8_t* arrTransmit) = 0;
    virtual bool receive(uint8_t* arrReceive) = 0;
};




} // end namespace uart_interface




/**
 * @}
 */
/* end - Header */

#endif /* __UART_INTERFACE_H__ */
