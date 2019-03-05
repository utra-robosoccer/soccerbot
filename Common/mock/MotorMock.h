/**
  *****************************************************************************
  * @file
  * @author  Tyler Gamvrelis
  * @brief   Since this is an abstract class, it needs to be mocked to test the
  *          concrete methods
  *
  * @defgroup MotorMock
  * @ingroup Mocks
  * @{
  *****************************************************************************
  */




#ifndef MOTOR_MOCK_H
#define MOTOR_MOCK_H




/********************************* Includes **********************************/
#include "Dynamixel/Dynamixel.h"
#include <gtest/gtest.h>
#include <gmock/gmock.h>

using dynamixel::Motor;
using dynamixel::DaisyChain;
using dynamixel::ResolutionDivider;




/****************************** MotorMockInterface ******************************/
namespace dynamixel {
namespace gmock {
// Classes and structs
// ----------------------------------------------------------------------------
/**
 * @class MotorMock Mocks the abstract methods of Motor so that it can be
 *        unit tested
 */
class MotorMock : public Motor {
public:
    MotorMock(
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

} // end namespace gmock
} // end namespace dynamixel




/**
 * @}
 */
/* end MotorMock */

#endif /* MOCK_MOTOR_H */
