/**
  *****************************************************************************
  * @file    FunctionalFreeRTOSInterface.h
  * @author  Izaak Niksan
  * @brief   An interface to various production FreeRTOS functions, which serves
  *          as a wrapper to the true functionality of the OS.
  *
  * @defgroup Header
  * @ingroup  Functional_FreeRTOS_Interface
  * @{
  *****************************************************************************
  */

#ifndef __FUNCTIONAL_FREERTOS_INTERFACE_H__
#define __FUNCTIONAL_FREERTOS_INTERFACE_H__

/********************************* Includes **********************************/
// Headers for FreeRTOS, such as task.h, are all included within cmsis_os.h
#include <cmsis_os.h>
#include <FreeRTOSInterface.h>

namespace Functional_FreeRTOS_Interface {

class FunctionalFreeRTOSInterface : public FreeRTOS_Interface::FreeRTOSInterface{
public:
    FunctionalFreeRTOSInterface() {}
    ~FunctionalFreeRTOSInterface() {}

    BaseType_t xTaskNotifyWait(
            uint32_t ulBitsToClearOnEntry,
            uint32_t ulBitsToClearOnExit,
            uint32_t *pulNotificationValue,
            TickType_t xTicksToWait
            );

    BaseType_t xQueueReceive(
            QueueHandle_t xQueue,
            void *pvBuffer,
            TickType_t xTicksToWait
            );

    BaseType_t xQueueSend(
            QueueHandle_t xQueue,
            const void * pvItemToQueue,
            TickType_t xTicksToWait
            );

    BaseType_t xSemaphoreTake(
            SemaphoreHandle_t xSemaphore,
            TickType_t xBlockTime
            );

    BaseType_t xSemaphoreGive(
            SemaphoreHandle_t xSemaphore
            );

    void vTaskDelayUntil(
            TickType_t * const pxPreviousWakeTime,
            const TickType_t xTimeIncrement
            );

    osStatus osDelay (
            uint32_t millisec
            );
};

} // end namespace Functional_FreeRTOS_Interface

/**
 * @}
 */
/* end - Header */

#endif /* __FUNCTIONAL_FREERTOS_INTERFACE_H__ */
