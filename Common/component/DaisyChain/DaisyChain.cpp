/**
  *****************************************************************************
  * @file    DaisyChain.cpp
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
        uartDriver(params.uartDriver),
        gpioif(params.gpioif),
        dataDirPort(params.dataDirPort),
        dataDirPinNum(params.dataDirPinNum)
{

}

DaisyChain::~DaisyChain(){

}

void DaisyChain::setIOType(IO_Type io_type){
    const_cast<UartDriver*>(uartDriver)->setIOType(io_type);
}

IO_Type DaisyChain::getIOType(void) const{
    return uartDriver->getIOType();
}

bool DaisyChain::requestTransmission(uint8_t* arr, size_t arrSize) const{
    changeBusDir(Direction::TX);
    return uartDriver->transmit(arr, arrSize);
}

bool DaisyChain::requestReception(uint8_t* buf, size_t bufSize) const{
    changeBusDir(Direction::RX);
    return uartDriver->receive(buf, bufSize);
}




// Private
// ----------------------------------------------------------------------------
void DaisyChain::changeBusDir(Direction dir) const{
    switch(dir){
        case Direction::RX:
            gpioif->writePin(
                const_cast<GPIO_TypeDef*>(dataDirPort),
                dataDirPinNum,
                GPIO_PIN_RESET
            );
            break;
        case Direction::TX:
            gpioif->writePin(
                const_cast<GPIO_TypeDef*>(dataDirPort),
                dataDirPinNum,
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
