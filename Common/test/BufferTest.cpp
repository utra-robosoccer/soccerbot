/**
  *****************************************************************************
  * @file
  * @author  Gokul Dharan
  *
  * @defgroup Buffer_test
  * @ingroup Buffer
  * @brief Buffer unit tests
  * @{
  *****************************************************************************
  */

//TODO: Fix THREADED macro issues


/********************************* Includes **********************************/
#include "uart_handler.h"
#include "PeripheralInstances.h"
#include "BufferBase.h"

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "OsInterfaceMock.h"


using ::testing::DoAll;
using ::testing::SetArgPointee;
using ::testing::Return;
using ::testing::_;

using cmsis::gmock::OsInterfaceMock;
using namespace buffer;



/******************************** File-local *********************************/
namespace{
// Variables
// ----------------------------------------------------------------------------
osMutexId mutex = nullptr;

// Classes & structs
// ----------------------------------------------------------------------------
class BufferTest : public ::testing::Test {
protected:
    OsInterfaceMock os;
};

// Functions
// ----------------------------------------------------------------------------
TEST_F(BufferTest, CanInitializeBuffer){
    BufferBase<int> intBuffer;
    intBuffer.set_lock(mutex);
    intBuffer.set_osInterface(&os);
}

TEST_F(BufferTest, CanWriteToBuffer){
    BufferBase<int> intBuffer;
    intBuffer.set_lock(mutex);
    intBuffer.set_osInterface(&os);
    intBuffer.write(10);
}

TEST_F(BufferTest, NumReadsCheck){
    BufferBase<int> intBuffer;
    intBuffer.set_lock(mutex);
    intBuffer.set_osInterface(&os);

    ASSERT_EQ(intBuffer.num_reads() , -1);
    intBuffer.write(10);
    ASSERT_EQ(intBuffer.num_reads() , 0);
}

TEST_F(BufferTest, CanReadFromBuffer){
    BufferBase<int> intBuffer;
    intBuffer.set_lock(mutex);
    intBuffer.set_osInterface(&os);

    intBuffer.write(10);

    int result = intBuffer.read();
    ASSERT_EQ(result, 10);
    ASSERT_EQ(intBuffer.num_reads() , 1);

    result = intBuffer.read();
    ASSERT_EQ(result, 10);
    ASSERT_EQ(intBuffer.num_reads() , 2);
}

TEST_F(BufferTest, CanResetBuffer){
    BufferBase<int> intBuffer;
    intBuffer.set_lock(mutex);
    intBuffer.set_osInterface(&os);

    intBuffer.write(10);

    ASSERT_EQ(intBuffer.num_reads() , 0);
    intBuffer.reset();
    ASSERT_EQ(intBuffer.num_reads() , -1);
}

TEST_F(BufferTest, CanInitializeBufferMaster){
    BufferMaster bufferMaster;
    bufferMaster.setup_buffers(mutex,&os);
}

TEST_F(BufferTest, CanWriteToIMUBuffer){
    BufferMaster bufferMaster;
    bufferMaster.setup_buffers(mutex,&os);
    imu::ImuStruct_t IMUdata;

    bufferMaster.imu_buffer.write(IMUdata);
}

TEST_F(BufferTest, CanReadFromIMUBuffer){
    BufferMaster bufferMaster;
    bufferMaster.setup_buffers(mutex,&os);
    imu::ImuStruct_t IMUdata;
    IMUdata.x_Accel = 1.0;
    IMUdata.x_Gyro = 2.0;
    IMUdata.y_Accel = 3.0;
    IMUdata.y_Gyro = 4.0;
    IMUdata.z_Accel = 5.0;
    IMUdata.z_Gyro = 6.0;

    bufferMaster.imu_buffer.write(IMUdata);
    imu::ImuStruct_t readIMUdata = bufferMaster.imu_buffer.read();

    ASSERT_EQ(readIMUdata.x_Accel, IMUdata.x_Accel);
    ASSERT_EQ(readIMUdata.y_Accel, IMUdata.y_Accel);
    ASSERT_EQ(readIMUdata.z_Accel, IMUdata.z_Accel);
    ASSERT_EQ(readIMUdata.x_Gyro, IMUdata.x_Gyro);
    ASSERT_EQ(readIMUdata.y_Gyro, IMUdata.y_Gyro);
    ASSERT_EQ(readIMUdata.z_Gyro, IMUdata.z_Gyro);
}

TEST_F(BufferTest, CanWriteToMotorBuffer){
    //TODO: Use periph::NUM_MOTORS without defining THREADED
    BufferMaster bufferMaster;
    bufferMaster.setup_buffers(mutex,&os);
    MotorData_t motorData[periph::NUM_MOTORS];

    for(int i = 0; i < periph::NUM_MOTORS; ++i)
    {
        bufferMaster.motor_buffer_array[i].write(motorData[i]);
    }
}

TEST_F(BufferTest, CanReadMotorDataBuffer){
    BufferMaster bufferMaster;
    bufferMaster.setup_buffers(mutex,&os);
    MotorData_t motorData[periph::NUM_MOTORS];
    MotorData_t readMotorData[periph::NUM_MOTORS];

    for(int i = 0; i < periph::NUM_MOTORS; ++i)
    {
        motorData[i].id = i;
        bufferMaster.motor_buffer_array[i].write(motorData[i]);
        readMotorData[i] = bufferMaster.motor_buffer_array[i].read();
        ASSERT_EQ(readMotorData[i].id, i);
    }
}

TEST_F(BufferTest, CanConfirmAllDataReady){
    BufferMaster bufferMaster;
    bufferMaster.setup_buffers(mutex,&os);
    MotorData_t motorData[periph::NUM_MOTORS];
    imu::ImuStruct_t IMUdata;

    ASSERT_FALSE(bufferMaster.all_data_ready());
    for(int i = 0; i < periph::NUM_MOTORS; ++i)
    {
        bufferMaster.motor_buffer_array[i].write(motorData[i]);
        ASSERT_FALSE(bufferMaster.all_data_ready());
    }

    bufferMaster.imu_buffer.write(IMUdata);
    ASSERT_TRUE(bufferMaster.all_data_ready());

    imu::ImuStruct_t readIMUdata = bufferMaster.imu_buffer.read();
    ASSERT_FALSE(bufferMaster.all_data_ready());

    bufferMaster.imu_buffer.write(IMUdata);
    ASSERT_TRUE(bufferMaster.all_data_ready());
    MotorData_t readMotorData = bufferMaster.motor_buffer_array[0].read();
    ASSERT_FALSE(bufferMaster.all_data_ready());
}

} // end anonymous namespace

/**
 * @}
 */
/* end - Buffer_test */
