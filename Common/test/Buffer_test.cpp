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

TEST(BufferTests, NumReadsCheck){
    BufferBase<int> intBuffer;

    ASSERT_EQ(intBuffer.num_reads() , -1);
    intBuffer.write(10);
    ASSERT_EQ(intBuffer.num_reads() , 0);
}

TEST(BufferTests, CanReadFromBuffer){
    BufferBase<int> intBuffer;
    intBuffer.write(10);

    int result = intBuffer.read();
    ASSERT_EQ(result, 10);
    ASSERT_EQ(intBuffer.num_reads() , 1);

    result = intBuffer.read();
    ASSERT_EQ(result, 10);
    ASSERT_EQ(intBuffer.num_reads() , 2);
}

TEST(BufferTests, CanResetBuffer){
    BufferBase<int> intBuffer;
    intBuffer.write(10);

    ASSERT_EQ(intBuffer.num_reads() , 0);
    intBuffer.reset();
    ASSERT_EQ(intBuffer.num_reads() , -1);
}

TEST(BufferTests, CanInitializeBufferMaster){
    BufferMaster bufferMaster;
}

TEST(BufferTests, CanWriteToIMUBuffer){
    BufferMaster bufferMaster;
    imu::IMUStruct_t IMUdata;

    bufferMaster.IMUBufferPtr->write(IMUdata);
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

    bufferMaster.IMUBufferPtr->write(IMUdata);
    imu::IMUStruct_t readIMUdata = bufferMaster.IMUBufferPtr->read();

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
        bufferMaster.MotorBufferPtrs[i]->write(motorData[i]);
    }
}

TEST(BufferTests, CanReadMotorDataBuffer){
    BufferMaster bufferMaster;
    Dynamixel_HandleTypeDef motorData[NUM_MOTORS];
    Dynamixel_HandleTypeDef readMotorData[NUM_MOTORS];

    for(int i = 0; i < NUM_MOTORS; ++i)
    {
        motorData[i]._ID = i;
        bufferMaster.MotorBufferPtrs[i]->write(motorData[i]);
        readMotorData[i] = bufferMaster.MotorBufferPtrs[i]->read();
        ASSERT_EQ(readMotorData[i]._ID, i);
    }
}

TEST(BufferTests, CanConfirmAllDataReady){
    BufferMaster bufferMaster;
    Dynamixel_HandleTypeDef motorData[NUM_MOTORS];
    imu::IMUStruct_t IMUdata;

    ASSERT_FALSE(bufferMaster.all_data_ready());
    for(int i = 0; i < NUM_MOTORS; ++i)
    {
        bufferMaster.MotorBufferPtrs[i]->write(motorData[i]);
        ASSERT_FALSE(bufferMaster.all_data_ready());
    }

    bufferMaster.IMUBufferPtr->write(IMUdata);
    ASSERT_TRUE(bufferMaster.all_data_ready());

    imu::IMUStruct_t readIMUdata = bufferMaster.IMUBufferPtr->read();
    ASSERT_FALSE(bufferMaster.all_data_ready());

    bufferMaster.IMUBufferPtr->write(IMUdata);
    ASSERT_TRUE(bufferMaster.all_data_ready());
    Dynamixel_HandleTypeDef readMotorData = bufferMaster.MotorBufferPtrs[0]->read();
    ASSERT_FALSE(bufferMaster.all_data_ready());
}

} // end anonymous namespace

/**
 * @}
 */
/* end - Buffer_test */
