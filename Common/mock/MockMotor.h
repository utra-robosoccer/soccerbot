/**
  *****************************************************************************
  * @file    MockMotor.h
  * @author  Tyler Gamvrelis
  * @brief   Since this is an abstract class, it needs to be mocked to test the
  *          concrete methods
  *
  * @defgroup MockMotor
  * @ingroup Mocks
  * @{
  *****************************************************************************
  */




#ifndef MOCK_MOTOR_H
#define MOCK_MOTOR_H




/********************************* Includes **********************************/
#include "Dynamixel.h"
#include <gtest/gtest.h>
#include <gmock/gmock.h>

using dynamixel::Motor;
using dynamixel::DaisyChain;
using dynamixel::ResolutionDivider;




/****************************** MockOsInterface ******************************/
namespace mocks{
// Classes and structs
// ----------------------------------------------------------------------------
/**
 * @class MockMotor Mocks the abstract methods of Motor so that it can be
 *        unit tested
 */
class MockMotor : public Motor {
public:
    MockMotor(
        uint8_t id,
        DaisyChain* daisyChain,
        ResolutionDivider divider
    )
        : Motor(id, daisyChain, divider)
    {

    }

    MOCK_CONST_METHOD1(
        setBaudRate,
        bool(
            uint32_t baud
        )
    );

    MOCK_CONST_METHOD1(
        setGoalVelocity,
        bool(
            float goalVelocity
        )
    );

    MOCK_CONST_METHOD1(
        getVelocity,
        bool(
            float& retVal
        )
    );
};

} // end namespace mocks




/**
 * @}
 */
/* end MockMotor */

#endif /* MOCK_MOTOR_H */
