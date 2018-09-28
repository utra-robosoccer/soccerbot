/**
  *****************************************************************************
  * @file    UartDriver.cpp
  * @author  Tyler Gamvrelis
  *
  * @defgroup   UartDriver
  * @addtogroup UART
  * @brief      Manages the usage of a particular UART, taking care of
  *             hardware-level calls, OS-level calls, and passing up statuses
  * @{
  *****************************************************************************
  */




/********************************* Includes **********************************/
#include "UartDriver.h"




namespace UART{
/********************************* UartDriver ********************************/
// Public
// ----------------------------------------------------------------------------
UartDriver::UartDriver(){

}

void UartDriver::setUartInterface(
    UartInterface* hw_if,
    UART_HandleTypeDef* uartHandlePtr
)
{
    if(hw_if != nullptr){
        this->hw_if = hw_if;

        if(uartHandlePtr != nullptr){
            hw_if->setUartPtr(uartHandlePtr);
        }

        hw_is_initialized = true;
    }
}

#ifdef THREADED
void UartDriver::setOSInterface(FreeRTOSInterface* os_if){
    this->os_if = os_if;
}
#endif

void UartDriver::setIOType(IO_Type io_type){
    this->io_type = io_type;
}

IO_Type UartDriver::getIOType(void) const{
    return this->io_type;
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
    bool retval = false;

    if(hw_is_initialized){
        switch(io_type) {
#if defined(THREADED)
            case IO_Type::DMA:
                if(os_if != nullptr){
                    if(hw_if->transmitDMA(arrTransmit, numBytes) == HAL_OK){
                        status = os_if->OS_xTaskNotifyWait(0, NOTIFIED_FROM_TX_ISR, &notification, MAX_BLOCK_TIME);

                        if((status == pdTRUE) && CHECK_NOTIFICATION(notification, NOTIFIED_FROM_TX_ISR)){
                            retval = true;
                        }
                    }
                }
                break;
            case IO_Type::IT:
                if(os_if != nullptr){
                    if(hw_if->transmitIT(arrTransmit, numBytes) == HAL_OK){
                        status = os_if->OS_xTaskNotifyWait(0, NOTIFIED_FROM_TX_ISR, &notification, MAX_BLOCK_TIME);

                        if((status == pdTRUE) && CHECK_NOTIFICATION(notification, NOTIFIED_FROM_TX_ISR)){
                            retval = true;
                        }
                    }
                }
                break;
#endif
            case IO_Type::POLL:
            default:
                retval = (hw_if->transmitPoll(arrTransmit, numBytes, POLLED_TRANSFER_TIMEOUT) == HAL_OK);
                break;
        }

        if(retval != true){
            hw_if->abortTransmit();
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
    bool retval = false;

    if(hw_is_initialized){
        switch(io_type) {
#if defined(THREADED)
            case IO_Type::DMA:
                if(os_if != nullptr){
                    if(hw_if->receiveDMA(arrReceive, numBytes) == HAL_OK){
                        status = os_if->OS_xTaskNotifyWait(0, NOTIFIED_FROM_RX_ISR, &notification, MAX_BLOCK_TIME);

                        if((status == pdTRUE) && CHECK_NOTIFICATION(notification, NOTIFIED_FROM_RX_ISR)){
                            retval = true;
                        }
                    }
                }
                break;
            case IO_Type::IT:
                if(os_if != nullptr){
                    if(hw_if->receiveIT(arrReceive, numBytes) == HAL_OK){
                        status = os_if->OS_xTaskNotifyWait(0, NOTIFIED_FROM_RX_ISR, &notification, MAX_BLOCK_TIME);

                        if((status == pdTRUE) && CHECK_NOTIFICATION(notification, NOTIFIED_FROM_RX_ISR)){
                            retval = true;
                        }
                    }
                }
                break;
#endif
            case IO_Type::POLL:
            default:
                retval = (hw_if->receivePoll(arrReceive, numBytes, POLLED_TRANSFER_TIMEOUT) == HAL_OK);
                break;
        }

        if(retval != true){
            hw_if->abortReceive();
        }
    }

    return retval;
}

} // end namespace uart




/**
 * @}
 */
/* end - module name */
