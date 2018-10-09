/**
  *****************************************************************************
  * @file    Dynamixel.cpp
  * @author  Tyler Gamvrelis
  *
  * @ingroup Dynamixel
  * @{
  *****************************************************************************
  */




/********************************* Includes **********************************/
#include "Dynamixel.h"




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
/*********************************** Motor ***********************************/
// Public
// ----------------------------------------------------------------------------
Motor::Motor(
    uint8_t id,
    UartDriver* uartDriverPtr,
    GpioInterface* gpioIfPtr,
    PinConfig& pinConfig
)
    :
        id(id),
        uartDriver(uartDriverPtr),
        gpioIf(gpioIfPtr),
        pinConfig(pinConfig)
{
    lastReadIsValid = false;
}

Motor::~Motor(){

}



// Protected
// ----------------------------------------------------------------------------




// Private
// ----------------------------------------------------------------------------
//Motor::changeBusDir(Direction dir){
//    switch(dir){
//        case Direction::TX:
//            HAL_GPIO_WritePin(
//                pinConfig.dataDirPort,
//                pinConfig.dataDirPinNum,
//                1
//            );
//            break;
//        case Direction::RX:
//            HAL_GPIO_WritePin(
//                pinConfig.dataDirPort,
//                pinConfig.dataDirPinNum,
//                0
//            );
//            break;
//        default:
//            break;
//    }
//}

}




/**
 * @}
 */
/* end - Dynamixel */
