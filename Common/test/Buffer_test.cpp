/**
  *****************************************************************************
  * @file    Buffer_test.cpp
  * @author  Gokul Dharan
  *
  * @defgroup Buffer_test
  * @ingroup Buffer
  * @brief Buffer unit tests
  * @{
  *****************************************************************************
  */




/********************************* Includes **********************************/
#include "BufferBase.h"
#include "MPU6050.h"
#include "UART_Handler.h"


#include <gtest/gtest.h>
#include <gmock/gmock.h>


using ::testing::DoAll;
using ::testing::SetArgPointee;
using ::testing::Return;
using ::testing::_;

using namespace buffer;




/******************************** File-local *********************************/
namespace{
// Functions
// ----------------------------------------------------------------------------
TEST(BufferTests, CanInitializeBuffer){
    BufferBase<int> intBuffer;
}

TEST(BufferTests, CanWriteToBuffer){
    BufferBase<int> intBuffer;
    intBuffer.write(10);
}

TEST(BufferTests, EmptyBoolCheck){
    BufferBase<int> intBuffer;

    ASSERT_TRUE(intBuffer.is_empty());
    intBuffer.write(10);
    ASSERT_FALSE(intBuffer.is_empty());
}

TEST(BufferTests, CanReadFromBuffer){
    BufferBase<int> intBuffer;
    intBuffer.write(10);
    int result = intBuffer.read();

    ASSERT_EQ(result, 10);
    ASSERT_TRUE(intBuffer.is_empty());
}

TEST(BufferTests, CanResetBuffer){
    BufferBase<int> intBuffer;
    intBuffer.write(10);

    ASSERT_FALSE(intBuffer.is_empty());
    intBuffer.reset();
    ASSERT_TRUE(intBuffer.is_empty());
}

TEST(BufferTests, CanInitializeBufferMaster){
    BufferMaster bufferMaster;
}

TEST(BufferTests, CanWriteToIMUBuffer){
    BufferMaster bufferMaster;
    imu::IMUStruct_t IMUdata;

    bufferMaster.IMUBuffer.write(IMUdata);
}

TEST(BufferTests, CanReadFromIMUBuffer){
    BufferMaster bufferMaster;
    imu::IMUStruct_t IMUdata;
    IMUdata.x_Accel = 1.0;
    IMUdata.x_Gyro = 2.0;
    IMUdata.y_Accel = 3.0;
    IMUdata.y_Gyro = 4.0;
    IMUdata.z_Accel = 5.0;
    IMUdata.z_Gyro = 6.0;

    bufferMaster.IMUBuffer.write(IMUdata);
    imu::IMUStruct_t readIMUdata = bufferMaster.IMUBuffer.read();

    ASSERT_EQ(readIMUdata.x_Accel, IMUdata.x_Accel);
    ASSERT_EQ(readIMUdata.y_Accel, IMUdata.y_Accel);
    ASSERT_EQ(readIMUdata.z_Accel, IMUdata.z_Accel);
    ASSERT_EQ(readIMUdata.x_Gyro, IMUdata.x_Gyro);
    ASSERT_EQ(readIMUdata.y_Gyro, IMUdata.y_Gyro);
    ASSERT_EQ(readIMUdata.z_Gyro, IMUdata.z_Gyro);
}

TEST(BufferTests, CanWriteToMotorBuffer){
    BufferMaster bufferMaster;
    Dynamixel_HandleTypeDef motorData[NUM_MOTORS];

    for(int i = 0; i < NUM_MOTORS; ++i)
    {
        bufferMaster.MotorBuffer[i].write(motorData[i]);
    }
}

TEST(BufferTests, CanReadMotorDataBuffer){
    BufferMaster bufferMaster;
    Dynamixel_HandleTypeDef motorData[12];
    Dynamixel_HandleTypeDef readMotorData[12];

    for(int i = 0; i < NUM_MOTORS; ++i)
    {
        motorData[i]._ID = i;
        bufferMaster.MotorBuffer[i].write(motorData[i]);
        readMotorData[i] = bufferMaster.MotorBuffer[i].read();
        ASSERT_EQ(readMotorData[i]._ID, i);
    }
}

} // end anonymous namespace

/**
 * @}
 */
/* end - Buffer_test */
