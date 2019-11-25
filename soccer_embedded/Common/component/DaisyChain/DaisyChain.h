/**
  *****************************************************************************
  * @file
  * @author  Tyler Gamvrelis
  *
  * @defgroup DaisyChain
  * @{
  *****************************************************************************
  */




#ifndef DAISY_CHAIN_H
#define DAISY_CHAIN_H




/********************************* Includes **********************************/
#include <stdint.h>
#include "UartDriver/UartDriver.h"
#include "GpioInterface.h"

using uart::UartDriver;
using hal::IO_Type;
using hal::GpioInterface;




/********************************* DaisyChain ********************************/
namespace dynamixel{
// Classes and structs
// ----------------------------------------------------------------------------
/** @brief Parameters for a DaisyChain */
struct DaisyChainParams{
    UartDriver* uartDriver;    /**< I/O Driver                        */
    GpioInterface* gpioif;     /**< GPIO interface                    */
    GPIO_TypeDef* dataDirPort; /**< Data direction control port       */
    uint16_t dataDirPinNum;    /**< Data direction control pin number */
};


class DaisyChain{
public:
    DaisyChain(const DaisyChainParams& params);
    ~DaisyChain();

    void setIOType(IO_Type io_type);
    IO_Type getIOType(void) const;

    /**
     * @brief Request a transmission on the daisy chain. This is guaranteed to
     *        send the requested information within a timely manner. However,
     *        the exact implementation details may vary (processors classes may
     *        be used to pack the data from several requests into 1 packet)
     * @param arr The array of bytes to be transmitted
     * @param arr_size The number of bytes to be transmitted
     * @return true if successful, otherwise false
     */
    bool requestTransmission(uint8_t* arr, size_t arr_size) const;

    /**
     * @brief Request a reception on the daisy chain. This is guaranteed to
     *        start the reception within a timely manner. However,
     *        the exact implementation details may vary
     * @param buf The array of bytes to buffer the received data
     * @param buf_size The number of bytes to be received
     * @return true if successful, otherwise false
     */
    bool requestReception(uint8_t* buf, size_t buf_size) const;


    // TODO(tyler): add method for "receive until condition...", where the
    // condition is possibly passed in as a function of some kind...

private:
    using Direction = enum class Direction{
        RX,
        TX
    };

    void changeBusDir(Direction dir) const;

    const UartDriver* m_uart_driver;     /**< @see UartDriver               */
    const GpioInterface* m_gpio_if;      /**< @see GpioInterface            */
    const GPIO_TypeDef* m_data_dir_port; /**< Port data direction pin is on */
    const uint16_t m_data_dir_pin_num;   /**< Data direction pin number     */
};

} // end namespace dynamixel




/**
 * @}
 */
/* end - DaisyChain */

#endif /* DAISY_CHAIN_H */
