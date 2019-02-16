/**
  *****************************************************************************
  * @file    BufferBase.h
  * @author  Gokul Dharan
  * @brief   Defines an abstract buffer class to be implemented and extended as needed
  *
  * @defgroup Buffer
  * @{
  *****************************************************************************
  */

#ifndef BUFFER_MASTER_H
#define BUFFER_MASTER_H

/********************************** Includes **********************************/
#include "PeripheralInstances.h"

/**
 * @class BufferMaster Easily extendible thread-safe master class that holds
 *        all relevant buffers
 */

class BufferMaster
{
public:
    BufferMaster() {}
    ~BufferMaster() {}
    void setup_buffers(osMutexId lock, os::OsInterface *osInterface)
    {
        IMUBuffer.set_lock(lock);
        IMUBuffer.set_osInterface(osInterface);
        for(int i = 0; i < periph::NUM_MOTORS; ++i)
        {
            MotorBufferArray[i].set_lock(lock);
            MotorBufferArray[i].set_osInterface(osInterface);
        }
        m_lock = lock;
        m_osInterfacePtr = osInterface;
    }
    bool all_data_ready()
    {
        m_osInterfacePtr->OS_xSemaphoreTake(m_lock, osWaitForever);
        bool ready =  (IMUBuffer.num_reads() == 0);

        if(ready)
        {
            for(int i = 0; i < periph::NUM_MOTORS; ++i)
            {
                ready = (ready && MotorBufferArray[i].num_reads() == 0);
            }
        }
        m_osInterfacePtr->OS_xSemaphoreGive(m_lock);
        return ready;
    }
    BufferBase<imu::IMUStruct_t> IMUBuffer;
    BufferBase<MotorData_t> MotorBufferArray[periph::NUM_MOTORS];
    // Add buffer items here as necessary
private:
    osMutexId m_lock = nullptr;
    os::OsInterface* m_osInterfacePtr = nullptr;
};

#endif

