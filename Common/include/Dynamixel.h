/**
  *****************************************************************************
  * @file    Dynamixel.h
  * @author  Tyler
  * @brief   TODO -- briefly describe this file
  *
  * @defgroup Dynamixel
  * @{
  *****************************************************************************
  */




#ifndef DYNAMIXEL_H
#define DYNAMIXEL_H




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
/** @brief GPIO port & pin for half duplex UART direction control */
struct PinConfig{
    GPIO_TypeDef* dataDirPort; /**< Port data direction pin is on */
    uint16_t dataDirPinNum;    /**< Data direction pin number     */
};


class Motor{
public:
    // Setup functions
    Motor(
        uint8_t id,
        UartDriver* uartDriverPtr,
        GpioInterface* gpioIfPtr,
        PinConfig& pinConfig
     );

    ~Motor();
//    void reset();
//
    // Setters (use the WRITE DATA instruction)
//    // EEPROM
//    void setID(uint8_t ID);
//    void setBaudRate(uint32_t baud);
//    void setReturnDelayTime(uint16_t microSec);
//    void setCWAngleLimit(float minAngle);
//    void setCCWAngleLimit(float maxAngle);
//    void setHighestVoltageLimit(float highestVoltage);
//    void setLowestVoltageLimit(float lowestVoltage);
//    void setMaxTorque(float maxTorque);
//    void setStatusReturnLevel(uint8_t status_data);
//    void setAlarmLED(uint8_t alarm_LED_data);
//    void setAlarmShutdown(uint8_t alarm_shutdown_data);
    // RAM
//    void torqueEnable(uint8_t isEnabled);
//    void lEDEnable(uint8_t isEnabled);
    void setGoalPosition(float goalAngle);
//    void setGoalVelocity(float goalVelocity);
//    void setGoalTorque(float goalTorque);
//    void lockEEPROM();
//    void setPunch(float punch);
//
    // Getters (use READ DATA instruction)
//    void getPosition();
//    void getVelocity();
//    void getLoad();
//    float getVoltage();
//    uint8_t getTemperature();
//    bool isRegistered();
//    bool isMoving();
//    bool isJointMode();
//
//    // Other motor instruction functions
//    int8_t ping();
//
//    // Interfaces for previously-defined functions
//    void enterWheelMode(float goalVelocity);
//    void enterJointMode();

protected:
    void dataWriter(uint8_t* args, size_t numArgs);
//    void dataReader(uint8_t readAddr, size_t readLength);

private:
    using Direction = enum class Direction{
        RX,
        TX
    };

    void changeBusDir(Direction dir);

    uint8_t id;            /**< Motor identification (0-252, 0xFE) */
    bool lastReadIsValid;  /**< 1 if checksum verified for last read, 0 o.w. */
//    bool isJointMode;      /**< true if motor is in joint mode, false if in wheel mode */
    const UartDriver* uartDriver; /**< @see UartDriver */
    const GpioInterface* gpioIf;  /**< @see GpioInterface */
    const PinConfig pinConfig;    /**< @see PinConfig */
};



// Functions
// ----------------------------------------------------------------------------




} // end namespace Dynamixel




/***************************** Inline functions ******************************/




/**
 * @}
 */
/* end - Dynamixel */

#endif /* DYNAMIXEL_H */
