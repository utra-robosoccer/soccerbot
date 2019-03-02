/**
  *****************************************************************************
  * @file    UartDriver.cpp
  * @author  Tyler Gamvrelis
  *
  * @defgroup UartDriver
  * @ingroup UART
  * @brief Manages the usage of a particular UART, taking care of
  *        hardware-level calls, OS-level calls, and passing up statuses
  * @{
  *****************************************************************************
  */




/********************************* Includes **********************************/
#include "UartDriver.h"

#if defined(THREADED)
#include "Notification.h"
#endif




namespace uart{
/********************************* UartDriver ********************************/
// Public
// ----------------------------------------------------------------------------
UartDriver::UartDriver(){
    m_max_block_time = 2;
}

#if defined(THREADED)
UartDriver::UartDriver(
    OsInterface* m_os_if,
    UartInterface* m_hw_if,
    UART_HandleTypeDef* m_uart_handle_ptr
) :
    m_os_if(m_os_if),
    m_hw_if(m_hw_if),
    m_uart_handle_ptr(m_uart_handle_ptr)
{
    if(m_hw_if != nullptr && m_uart_handle_ptr != nullptr){
        m_hw_is_initialized = true;
    }

    m_max_block_time = pdMS_TO_TICKS(2);
}
#else
UartDriver::UartDriver(
    UartInterface* m_hw_if,
    UART_HandleTypeDef* m_uart_handle_ptr
) :
    m_hw_if(m_hw_if),
    m_uart_handle_ptr(m_uart_handle_ptr)

{
    if(m_hw_if != nullptr && m_uart_handle_ptr != nullptr){
        m_hw_is_initialized = true;
    }
}
#endif

void UartDriver::setMaxBlockTime(uint32_t timeout){
    m_max_block_time = timeout;
}

void UartDriver::setIOType(IO_Type io_type){
    this->m_io_type = io_type;
}

IO_Type UartDriver::getIOType(void) const{
    return this->m_io_type;
}

bool UartDriver::transmit(
    uint8_t* arrTransmit,
    size_t numBytes
) const
{
#if defined(THREADED)
    uint32_t notification = 0;
    BaseType_t status = pdFALSE;
#endif
    HAL_StatusTypeDef hal_status;
    bool retval = false;

    if(m_hw_is_initialized){
        switch(m_io_type) {
#if defined(THREADED)
            case IO_Type::DMA:
                if(m_os_if != nullptr){
                    if(m_hw_if->transmitDMA(m_uart_handle_ptr, arrTransmit, numBytes) == HAL_OK){
                        status = m_os_if->OS_xTaskNotifyWait(0, NOTIFIED_FROM_TX_ISR, &notification, m_max_block_time);

                        if((status == pdTRUE) && CHECK_NOTIFICATION(notification, NOTIFIED_FROM_TX_ISR)){
                            retval = true;
                        }
                    }
                }
                break;
            case IO_Type::IT:
                if(m_os_if != nullptr){
                    if(m_hw_if->transmitIT(m_uart_handle_ptr, arrTransmit, numBytes) == HAL_OK){
                        status = m_os_if->OS_xTaskNotifyWait(
                            0,
                            NOTIFIED_FROM_TX_ISR,
                            &notification,
                            m_max_block_time
                        );

                        if((status == pdTRUE) &&
                           CHECK_NOTIFICATION(notification, NOTIFIED_FROM_TX_ISR))
                        {
                            retval = true;
                        }
                    }
                }
                break;
#endif
            case IO_Type::POLL:
            default:
                hal_status = m_hw_if->transmitPoll(
                        m_uart_handle_ptr,
                    arrTransmit,
                    numBytes,
                    m_max_block_time
                );

                retval = (hal_status == HAL_OK);
                break;
        }

        if(retval != true){
            m_hw_if->abortTransmit(m_uart_handle_ptr);
        }
    }

    return retval;
}

bool UartDriver::receive(
    uint8_t* arrReceive,
    size_t numBytes
) const
{
#if defined(THREADED)
    uint32_t notification = 0;
    BaseType_t status = pdFALSE;
#endif
    HAL_StatusTypeDef hal_status;
    bool retval = false;

    if(m_hw_is_initialized){
        switch(m_io_type) {
#if defined(THREADED)
            case IO_Type::DMA:
                if(m_os_if != nullptr){
                    if(m_hw_if->receiveDMA(m_uart_handle_ptr, arrReceive, numBytes) == HAL_OK){
                        status = m_os_if->OS_xTaskNotifyWait(0, NOTIFIED_FROM_RX_ISR, &notification, m_max_block_time);

                        if((status == pdTRUE) && CHECK_NOTIFICATION(notification, NOTIFIED_FROM_RX_ISR)){
                            retval = true;
                        }
                    }
                }
                break;
            case IO_Type::IT:
                if(m_os_if != nullptr){
                    if(m_hw_if->receiveIT(m_uart_handle_ptr, arrReceive, numBytes) == HAL_OK){
                        status = m_os_if->OS_xTaskNotifyWait(
                            0,
                            NOTIFIED_FROM_RX_ISR,
                            &notification,
                            m_max_block_time
                        );

                        if((status == pdTRUE) &&
                           CHECK_NOTIFICATION(notification, NOTIFIED_FROM_RX_ISR))
                        {
                            retval = true;
                        }
                    }
                }
                break;
#endif
            case IO_Type::POLL:
            default:
                hal_status = m_hw_if->receivePoll(
                        m_uart_handle_ptr,
                    arrReceive,
                    numBytes,
                    m_max_block_time
                );

                retval = (hal_status == HAL_OK);
                break;
        }

        if(retval != true){
            m_hw_if->abortReceive(m_uart_handle_ptr);
        }
    }

    return retval;
}

} // end namespace uart




/**
 * @}
 */
/* end - UartDriver */
