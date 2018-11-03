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
#include "PeripheralInstances.h"

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
    BufferBase()
    {
        m_data_buf = std::unique_ptr<T>(new T);
        m_read = -1;
    }
    ~BufferBase() {};
    void set_lock(osMutexId lock)
    {
        m_lock = lock;
    }
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
        xSemaphoreGive(m_lock);
        return *m_data_buf;
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
    //int indicates whether data has been read, -1 if data not written yet
    int8_t m_read;
    osMutexId m_lock = nullptr;
};


/**
 * @class Easily extendable thread-safe master class that holds all relevant buffers
 */

class BufferMaster
{
public:
    BufferMaster()  {}
    ~BufferMaster() {}
    void set_lock(osMutexId lock)
    {
        IMUBuffer.set_lock(lock);
        for(int i = 0; i < periph::NUM_MOTORS; ++i)
        {
            MotorBufferArray[i].set_lock(lock);
        }
        m_lock = lock;
    }
    bool all_data_ready()
    {
        xSemaphoreTake(m_lock, portMAX_DELAY);
        bool ready =  (IMUBuffer.num_reads() == 0);

        if(ready)
        {
            for(int i = 0; i < periph::NUM_MOTORS; ++i)
            {
                ready = (ready && MotorBufferArray[i].num_reads() == 0);
            }
        }
        xSemaphoreGive(m_lock);
        return ready;
    }
    BufferBase<imu::IMUStruct_t> IMUBuffer;
    BufferBase<MotorData_t> MotorBufferArray[periph::NUM_MOTORS];
    // Add buffer items here as necessary
private:
    osMutexId m_lock = nullptr;
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
    BufferMaster() {}
    ~BufferMaster() {}
    bool all_data_ready()
    {
        bool ready =  (IMUBuffer.num_reads() == 0);

        if(ready)
        {
            for(int i = 0; i < periph::NUM_MOTORS; ++i)
            {
                ready = (ready && MotorBufferArray[i].num_reads() == 0);
            }
        }
        return ready;
    }
    BufferBase<imu::IMUStruct_t> IMUBuffer;
    BufferBase<MotorData_t> MotorBufferArray[periph::NUM_MOTORS];
    // Add buffer items here as necessary
};
#endif
} // end namespace buffer

/**
 * @}
 */
/* end - BufferBase */

#endif /* __BUFFER_BASE_H__ */
