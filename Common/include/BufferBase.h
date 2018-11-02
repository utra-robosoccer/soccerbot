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

// NOTE: defgroup used above since there is no .cpp file associated with this class.

#ifndef __BUFFER_BASE_H__
#define __BUFFER_BASE_H__

/********************************** Includes **********************************/
#include <cstdio>
#include <memory>
#include "UART_Handler.h"
#include "MPU6050.h"

#if defined(THREADED)
#include "cmsis_os.h"
#endif

namespace buffer {


/*********************************** Buffer ***********************************/
/**
 * @class Generic templated thread-safe buffer class
 */

#if defined(THREADED)
template <class T>
class BufferBase
{
public:
    BufferBase(osMutexId lock)
    {
        m_data_buf = std::unique_ptr<T>(new T);
        m_lock = lock;
        m_read = -1;
    }
    ~BufferBase() {};
    void write(const T &item)
    {
        xSemaphoreTake(m_lock, portMAX_DELAY);
        *m_data_buf = item;
        m_read = 0;
        xSemaphoreGive(m_lock);
    }
    T read()
    {
        xSemaphoreTake(m_lock, portMAX_DELAY);
        m_read++;
        return *m_data_buf;
        xSemaphoreGive(m_lock);
    }
    void reset()
    {
        xSemaphoreTake(m_lock, portMAX_DELAY);
        m_read = -1;
        xSemaphoreGive(m_lock);
    }
    int8_t num_reads()
    {
        return m_read;
    }

    // Defining methods here in the declaration for ease of use as a templated class
private:
    std::unique_ptr<T> m_data_buf;
    //bool indicates whether data has been read, -1 if data not written yet
    int8_t m_read;
    osMutexId m_lock;
};


/**
 * @class Easily extendable thread-safe master class that holds all relevant buffers
 */

class BufferMaster
{
public:
    BufferMaster(osMutexId lock)
    {
        IMUBufferPtr = new BufferBase<imu::IMUStruct_t>(lock);
        for(int i = 0; i < NUM_MOTORS; ++i)
        {
            MotorBufferPtrs[i] = new BufferBase<Dynamixel_HandleTypeDef>(lock);
        }
        m_lock = lock;
    }
    ~BufferMaster()
    {
        delete IMUBufferPtr;
        for(int i = 0; i < NUM_MOTORS; ++i)
        {
            delete MotorBufferPtrs[i];
        }
    }
    bool all_data_ready()
    {
        xSemaphoreTake(m_lock, portMAX_DELAY);
        bool ready =  (IMUBufferPtr->num_reads() == 0);

        if(ready)
        {
            for(int i = 0; i < NUM_MOTORS; ++i)
            {
                ready = (ready && MotorBufferPtrs[i]->num_reads() == 0);
            }
        }
        xSemaphoreGive(m_lock);
        return ready;
    }
    BufferBase<imu::IMUStruct_t>* IMUBufferPtr;
    BufferBase<Dynamixel_HandleTypeDef>* MotorBufferPtrs[NUM_MOTORS];
    // Add buffer items here as necessary
private:
    osMutexId m_lock;
};

#else
/**
 * @class Generic templated buffer class (not thread-safe)
 */
template <class T>
class BufferBase
{
public:
    BufferBase()
    {
        m_data_buf = std::unique_ptr<T>(new T);
        m_read = -1;
    }
    ~BufferBase() {};
    void write(const T &item)
    {
        *m_data_buf = item;
        m_read = 0;
    }
    T read()
    {
        m_read++;
        return *m_data_buf;
    }
    void reset()
    {
        m_read = -1;
    }
    int8_t num_reads()
    {
        return m_read;
    }

    // Defining methods here in the declaration for ease of use as a templated class
private:
    std::unique_ptr<T> m_data_buf;
    int8_t m_read;
};


/**
 * @class Easily extendable master class that holds all relevant buffers (not thread-safe)
 */

class BufferMaster
{
public:
    BufferMaster()
    {
        IMUBufferPtr = new BufferBase<imu::IMUStruct_t>;
        for(int i = 0; i < NUM_MOTORS; ++i)
        {
            MotorBufferPtrs[i] = new BufferBase<Dynamixel_HandleTypeDef>;
        }
    }
    ~BufferMaster()
    {
        delete IMUBufferPtr;
        for(int i = 0; i < NUM_MOTORS; ++i)
        {
            delete MotorBufferPtrs[i];
        }
    }
    bool all_data_ready()
    {
        bool ready =  (IMUBufferPtr->num_reads() == 0);

        if(ready)
        {
            for(int i = 0; i < NUM_MOTORS; ++i)
            {
                ready = (ready && MotorBufferPtrs[i]->num_reads() == 0);
            }
        }
        return ready;
    }
    BufferBase<imu::IMUStruct_t>* IMUBufferPtr;
    BufferBase<Dynamixel_HandleTypeDef>* MotorBufferPtrs[NUM_MOTORS];
    // Add buffer items here as necessary
};
#endif
} // end namespace buffer

/**
 * @}
 */
/* end - BufferBase */

#endif /* __BUFFER_BASE_H__ */
