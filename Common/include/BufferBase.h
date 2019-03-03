/**
  *****************************************************************************
  * @file
  * @author  Gokul Dharan
  * @brief   Defines an abstract buffer class to be implemented and extended as needed
  *
  * @defgroup Buffer
  * @{
  *****************************************************************************
  */


#ifndef BUFFER_BASE_H
#define BUFFER_BASE_H

/********************************** Includes **********************************/
#include "PeripheralInstances.h"


using cmsis::OsInterface;


namespace buffer {


/*********************************** Buffer ***********************************/
/**
 * @class BufferBase Generic templated thread-safe buffer class
 */
template <class T>
class BufferBase
{
public:
    BufferBase() {}
    ~BufferBase() {}
    void set_osInterface(OsInterface *os_interface_ptr)
    {
        m_os_interface_ptr = os_interface_ptr;
    }
    void set_lock(osMutexId lock)
    {
        m_lock = lock;
    }
    void write(const T &item)
    {
        m_os_interface_ptr->OS_xSemaphoreTake(m_lock, osWaitForever);
        m_data_buf = item;
        m_read = 0;
        m_os_interface_ptr->OS_xSemaphoreGive(m_lock);
    }
    T read()
    {
        m_os_interface_ptr->OS_xSemaphoreTake(m_lock, osWaitForever);
        m_read++;
        m_os_interface_ptr->OS_xSemaphoreGive(m_lock);
        return m_data_buf;
    }
    void reset()
    {
        m_os_interface_ptr->OS_xSemaphoreTake(m_lock, osWaitForever);
        m_read = -1;
        m_os_interface_ptr->OS_xSemaphoreGive(m_lock);
    }
    int8_t num_reads()
    {
        return m_read;
    }

    // Defining methods here in the declaration for ease of use as a templated class
private:
    T m_data_buf;
    //int indicates whether data has been read, -1 if data not written yet
    int8_t m_read = -1;
    osMutexId m_lock = nullptr;
    OsInterface* m_os_interface_ptr = nullptr;
};


/**
 * @class BufferMaster Easily extendible thread-safe master class that holds
 *        all relevant buffers
 */

class BufferMaster
{
public:
    BufferMaster() {}
    ~BufferMaster() {}
    void setup_buffers(osMutexId lock, OsInterface *osInterface)
    {
        m_imu_buffer.set_lock(lock);
        m_imu_buffer.set_osInterface(osInterface);
        for(int i = 0; i < periph::NUM_MOTORS; ++i)
        {
            m_motor_buffer_array[i].set_lock(lock);
            m_motor_buffer_array[i].set_osInterface(osInterface);
        }
        m_lock = lock;
        m_os_interface_ptr = osInterface;
    }
    bool all_data_ready()
    {
        m_os_interface_ptr->OS_xSemaphoreTake(m_lock, osWaitForever);
        bool ready =  (m_imu_buffer.num_reads() == 0);

        if(ready)
        {
            for(int i = 0; i < periph::NUM_MOTORS; ++i)
            {
                ready = (ready && m_motor_buffer_array[i].num_reads() == 0);
            }
        }
        m_os_interface_ptr->OS_xSemaphoreGive(m_lock);
        return ready;
    }
    BufferBase<imu::ImuStruct_t> m_imu_buffer;
    BufferBase<MotorData_t> m_motor_buffer_array[periph::NUM_MOTORS];
    // Add buffer items here as necessary
private:
    osMutexId m_lock = nullptr;
    OsInterface* m_os_interface_ptr = nullptr;
};

} // end namespace buffer

/**
 * @}
 */
/* end - BufferBase */

#endif /* BUFFER_BASE_H */
