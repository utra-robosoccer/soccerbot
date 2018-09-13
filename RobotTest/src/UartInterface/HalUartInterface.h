/**
  *****************************************************************************
  * @file    HalUartInterface.h
  * @author  Tyler Gamvrelis
  * @brief   HAL UART
  *
  * @defgroup Header
  * @ingroup  HalUartInterface
  * @{
  *****************************************************************************
  */




#ifndef __HAL_UART_INTERFACE_H__
#define __HAL_UART_INTERFACE_H__




/********************************* Includes **********************************/
#include "UartInterface.h"
#include <cstdint>




/******************************* uart_interface ******************************/
namespace hal_uart_interface{
// Classes and structs
// ----------------------------------------------------------------------------
enum class IO_Type{
    IO_POLL,
    IO_IT,
    IO_DMA
};

class HalUartInterface : public uart_interface::UartInterface{
public:
    HalUartInterface();
    void setIOType(IO_Type);
    virtual bool transmit(uint8_t* arrTransmit) final;
    virtual bool receive(uint8_t* arrReceive) final;

private:
    IO_Type io_type;
};




} // end namespace hal_uart_interface




/**
 * @}
 */
/* end - Header */

#endif /* __HAL_UART_INTERFACE_H__ */
