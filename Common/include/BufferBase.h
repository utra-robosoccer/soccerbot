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
#include <stdint.h>
#include "UART_Handler.h"

namespace buffer {


/*********************************** Types ************************************/
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
    void write(T item)
    {
        *m_data_buf = item;
        m_empty = false;
    }
    T read()
    {
        if (m_empty)
        {
            return T();
        }
        m_empty = true;
        return *m_data_buf;
    }
    void reset()
    {
        m_empty = true;
    }
    bool is_empty()
    {
        return m_empty;
    }

    // Defining methods here in the declaration for ease of use as a templated class
private:
    //TODO: Thread-safety
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

    BufferBase<IMUStruct> IMUBuffer;
    BufferBase<Dynamixel_HandleTypeDef> MotorBuffer[NUM_MOTORS];
    // Add buffer items here as necessary
};

} // end namespace buffer

/**
 * @}
 */
/* end - BufferBase */

#endif /* __BUFFER_BASE_H__ */
