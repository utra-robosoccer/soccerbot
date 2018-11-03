/**
  *****************************************************************************
  * @file    UartDriver.cpp
  * @author  Tyler Gamvrelis
  * @author  Robert Fairley
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
    /**
     * @brief Initializes the handle to the low-level hardware routines,
     *        associates a particular UART module on the board with this
     *        driver, and initializes the handle to the OS for system calls
     * @param os_if Pointer to the object handling the calls to the OS
     * @param hw_if Pointer to the hardware-facing object handling the
     *        low-level UART routines
     * @param uartHandlePtr Pointer to a structure that contains
     *        the configuration information for the desired UART module
     */
UartDriver::UartDriver(
    OsInterface* os_if,
    UartInterface* hw_if,
    UART_HandleTypeDef* uartHandlePtr
) :
    os_if(os_if),
    hw_if(hw_if),
    uartHandlePtr(uartHandlePtr)
{
    if(hw_if != nullptr && uartHandlePtr != nullptr){
        hw_is_initialized = true;
    }

    m_max_block_time = pdMS_TO_TICKS(2);
}
#else
    /**
     * @brief Initializes the handle to the low-level hardware routines, and
     *        associates a particular UART module on the board with this driver
     * @param hw_if Pointer to the hardware-facing object handling the
     *        low-level UART routines
     * @param uartHandlePtr Pointer to a structure that contains
     *        the configuration information for the desired UART module
     */
UartDriver::UartDriver(
    UartInterface* hw_if,
    UART_HandleTypeDef* uartHandlePtr
) :
    hw_if(hw_if),
    uartHandlePtr(uartHandlePtr)

{
    if(hw_if != nullptr && uartHandlePtr != nullptr){
        hw_is_initialized = true;
    }
}
#endif

void UartDriver::setMaxBlockTime(uint32_t timeout){
    m_max_block_time = timeout;
}

void UartDriver::setIOType(IO_Type io_type){
    this->io_type = io_type;
}

IO_Type UartDriver::getIOType(void) const{
    return this->io_type;
}

bool UartDriver::setup(void) {
    bool retval = false;

#if defined(THREADED)
    osMutexStaticDef(UartDriverUartResourceMutex, &uartResourceMutexControlBlock);
    uartResourceMutex = os_if->OS_osMutexCreate(osMutex(UartDriverUartResourceMutex));
#endif

    switch(io_type) {
#if defined(THREADED)
        case IO_Type::DMA:
        case IO_Type::IT:
#endif
        case IO_Type::POLL:
            retval = true;
        break;
        default:
            retval = false;
    }
    return retval;
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

    if(hw_is_initialized){
        switch(io_type) {
#if defined(THREADED)
            case IO_Type::DMA:
                if(os_if != nullptr){
                    if (os_if->OS_osMutexWait(uartResourceMutex, m_max_block_time) == osOK) {
                        if(hw_if->transmitDMA(uartHandlePtr, arrTransmit, numBytes) == HAL_OK){
                            os_if->OS_osMutexRelease(uartResourceMutex);
                            status = os_if->OS_xTaskNotifyWait(0, NOTIFIED_FROM_TX_ISR, &notification, m_max_block_time);

                            if((status == pdTRUE) && CHECK_NOTIFICATION(notification, NOTIFIED_FROM_TX_ISR)){
                                retval = true;
                            }
                        }
                        else {
                            os_if->OS_osMutexRelease(uartResourceMutex);
                        }

                    }
                }
                break;
            case IO_Type::IT:
                if(os_if != nullptr){
                    if(hw_if->transmitIT(uartHandlePtr, arrTransmit, numBytes) == HAL_OK){
                        status = os_if->OS_xTaskNotifyWait(
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
                hal_status = hw_if->transmitPoll(
                    uartHandlePtr,
                    arrTransmit,
                    numBytes,
                    m_max_block_time
                );

                retval = (hal_status == HAL_OK);
                break;
        }

        if(retval != true){
            hw_if->abortTransmit(uartHandlePtr);
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

    if(hw_is_initialized){
        switch(io_type) {
#if defined(THREADED)
            case IO_Type::DMA:
                if(os_if != nullptr){
                    if (os_if->OS_osMutexWait(uartResourceMutex, m_max_block_time) == osOK) {
                        if(hw_if->receiveDMA(uartHandlePtr, arrReceive, numBytes) == HAL_OK){
                            os_if->OS_osMutexRelease(uartResourceMutex);
                            status = os_if->OS_xTaskNotifyWait(0, NOTIFIED_FROM_RX_ISR, &notification, m_max_block_time);

                            if((status == pdTRUE) && CHECK_NOTIFICATION(notification, NOTIFIED_FROM_RX_ISR)){
                                retval = true;
                            }
                        }
                        else {
                            os_if->OS_osMutexRelease(uartResourceMutex);
                        }
                    }
                }
                break;
            case IO_Type::IT:
                if(os_if != nullptr){
                    if(hw_if->receiveIT(uartHandlePtr, arrReceive, numBytes) == HAL_OK){
                        status = os_if->OS_xTaskNotifyWait(
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
                hal_status = hw_if->receivePoll(
                    uartHandlePtr,
                    arrReceive,
                    numBytes,
                    m_max_block_time
                );

                retval = (hal_status == HAL_OK);
                break;
        }

        if(retval != true){
            hw_if->abortReceive(uartHandlePtr);
        }
    }

    return retval;
}

} // end namespace uart




/**
 * @}
 */
/* end - UartDriver */
