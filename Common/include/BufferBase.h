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
 * @class Generic templated buffer class
 */

template <class T>
class BufferBase
{
public:
    BufferBase()
    {
        m_data_buf = std::unique_ptr<T>(new T);
    }
    ~BufferBase() {};
    void write(const T &item)
    {
#if defined(THREADED)
        xSemaphoreTake(DATABUFFERHandle, portMAX_DELAY);
#endif
        *m_data_buf = item;
        m_empty = false;
#if defined(THREADED)
        xSemaphoreGive(DATABUFFERHandle);
#endif
    }
    T read()
    {
#if defined(THREADED)
        xSemaphoreTake(DATABUFFERHandle, portMAX_DELAY);
#endif
        if (m_empty)
        {
            return T();
        }
        m_empty = true;
        return *m_data_buf;
#if defined(THREADED)
        xSemaphoreGive(DATABUFFERHandle);
#endif
    }
    void reset()
    {
#if defined(THREADED)
        xSemaphoreTake(DATABUFFERHandle, portMAX_DELAY);
#endif
        m_empty = true;
#if defined(THREADED)
        xSemaphoreGive(DATABUFFERHandle);
#endif
    }
    bool is_empty()
    {
        return m_empty;
    }

    // Defining methods here in the declaration for ease of use as a templated class
private:
    std::unique_ptr<T> m_data_buf;
    bool m_empty = true;
};


/**
 * @class Easily extendable master class that holds all relevant buffers
 */

class BufferMaster
{
public:
    BufferMaster(){};
    ~BufferMaster(){};

    BufferBase<imu::IMUStruct_t> IMUBuffer;
    BufferBase<Dynamixel_HandleTypeDef> MotorBuffer[NUM_MOTORS];
    // Add buffer items here as necessary
};

} // end namespace buffer

/**
 * @}
 */
/* end - BufferBase */

#endif /* __BUFFER_BASE_H__ */
