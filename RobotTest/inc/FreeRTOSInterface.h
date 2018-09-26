/**
  *****************************************************************************
  * @file    FreeRTOSInterface.h
  * @author  Izaak Niksan
  * @brief   Defines an abstract interface of FreeRTOS functions.
  *
  * @defgroup Header
  * @ingroup FreeRTOS_Interface
  * @{
  *****************************************************************************
  */


#ifndef FREERTOS_INTERFACE_H__
#define FREERTOS_INTERFACE_H__

#include <cstdint>

namespace FreeRTOS_Interface {

class FreeRTOSInterface {
public:

    virtual ~FreeRTOSInterface() {}

    virtual BaseType_t xTaskNotifyWait(
                uint32_t ulBitsToClearOnEntry,
                uint32_t ulBitsToClearOnExit,
                uint32_t *pulNotificationValue,
                TickType_t xTicksToWait
            ) = 0;

    virtual BaseType_t xQueueReceive(
            QueueHandle_t xQueue,
            void *pvBuffer,
            TickType_t xTicksToWait
            ) = 0;

    virtual BaseType_t xQueueSend(
            QueueHandle_t xQueue,
            const void * pvItemToQueue,
            TickType_t xTicksToWait
            ) = 0;

    virtual BaseType_t xSemaphoreTake(
            SemaphoreHandle_t xSemaphore,
            TickType_t xBlockTime
            ) = 0;

    virtual BaseType_t xSemaphoreGive(
            SemaphoreHandle_t xSemaphore
            ) = 0;

    virtual void vTaskDelayUntil(
            TickType_t * const pxPreviousWakeTime,
            const TickType_t xTimeIncrement
            ) = 0;

    virtual osStatus osDelay (
            uint32_t millisec
            ) = 0;
};

} // end namespace FreeRTOS_Interface

/**
 * @}
 */
/* end - Header */

#endif /* FREERTOS_INTERFACE_H__ */
