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




/********************************** Globals **********************************/
// Constants
// ----------------------------------------------------------------------------




/******************************** File-local *********************************/
namespace{
// Constants
// ----------------------------------------------------------------------------


// Types & enums
// ----------------------------------------------------------------------------




// Classes and structs
// ----------------------------------------------------------------------------




// Variables
// ----------------------------------------------------------------------------




// Functions
// ----------------------------------------------------------------------------

} // end anonymous namespace




namespace dynamixel{
/******************************** DaisyChain *********************************/
// Public
// ----------------------------------------------------------------------------
DaisyChain::DaisyChain(DaisyChainParams& params)
    :
        uartDriver(params.uartDriver),
        gpioif(params.gpioif),
        dataDirPort(params.dataDirPort),
        dataDirPinNum(params.dataDirPinNum)
{

}

DaisyChain::~DaisyChain(){

}

bool DaisyChain::requestTransmission(uint8_t* arr, size_t arrSize) const{
    changeBusDir(Direction::TX);
    return uartDriver->transmit(arr, arrSize);
}

bool DaisyChain::requestReception(uint8_t* buf, size_t bufSize) const{
    changeBusDir(Direction::RX);
    return uartDriver->receive(buf, bufSize);
}



// Protected
// ----------------------------------------------------------------------------



// Private
// ----------------------------------------------------------------------------
void DaisyChain::changeBusDir(Direction dir) const{
    switch(dir){
        case Direction::RX:
            gpioif->writePin(
                dataDirPort,
                dataDirPinNum,
                GPIO_PIN_RESET
            );
            break;
        case Direction::TX:
            gpioif->writePin(
                dataDirPort,
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
