/**
  *****************************************************************************
  * @file
  * @author  Tyler Gamvrelis
  *
  * @ingroup DaisyChain
  * @{
  *****************************************************************************
  */




/********************************* Includes **********************************/
#include "DaisyChain.h"




// TODO: choose better namespace for this
namespace dynamixel{
/******************************** DaisyChain *********************************/
// Public
// ----------------------------------------------------------------------------
DaisyChain::DaisyChain(const DaisyChainParams& params)
    :
        m_uart_driver(params.uartDriver),
        m_gpio_if(params.gpioif),
        m_data_dir_port(params.dataDirPort),
        m_data_dir_pin_num(params.dataDirPinNum)
{

}

DaisyChain::~DaisyChain(){

}

void DaisyChain::setIOType(IO_Type io_type){
    const_cast<UartDriver*>(m_uart_driver)->setIOType(io_type);
}

IO_Type DaisyChain::getIOType(void) const{
    return m_uart_driver->getIOType();
}

bool DaisyChain::requestTransmission(uint8_t* arr, size_t arrSize) const{
    changeBusDir(Direction::TX);
    return m_uart_driver->transmit(arr, arrSize);
}

bool DaisyChain::requestReception(uint8_t* buf, size_t bufSize) const{
    changeBusDir(Direction::RX);
    return m_uart_driver->receive(buf, bufSize);
}




// Private
// ----------------------------------------------------------------------------
void DaisyChain::changeBusDir(Direction dir) const{
    switch(dir){
        case Direction::RX:
            m_gpio_if->writePin(
                const_cast<GPIO_TypeDef*>(m_data_dir_port),
                m_data_dir_pin_num,
                GPIO_PIN_RESET
            );
            break;
        case Direction::TX:
            m_gpio_if->writePin(
                const_cast<GPIO_TypeDef*>(m_data_dir_port),
                m_data_dir_pin_num,
                GPIO_PIN_SET
            );
            break;
        default:
            break;
    }
}

} // end namespace dynamixel




/**
 * @}
 */
/* end - DaisyChain */
