/**
  *****************************************************************************
  * @file    OsInterface.h
  * @author  Izaak Niksan
  * @brief   Defines an abstract interface of FreeRTOS functions.
  *
  * @defgroup Interface
  * @ingroup  OS
  * @{
  *****************************************************************************
  */




#ifndef OS_INTERFACE_H
#define OS_INTERFACE_H




/********************************* Includes **********************************/
#include <stdint.h>
#include "SystemConf.h"
#include "cmsis_os.h"




/******************************* OsInterface *********************************/
namespace os{
// Classes and structs
// ----------------------------------------------------------------------------
class OsInterface {
public:
    virtual ~OsInterface() {}

    virtual BaseType_t OS_xTaskNotifyWait(
        uint32_t ulBitsToClearOnEntry,
        uint32_t ulBitsToClearOnExit,
        uint32_t *pulNotificationValue,
        TickType_t xTicksToWait
    ) const = 0;

    virtual BaseType_t OS_xQueueReceive(
        QueueHandle_t xQueue,
        void *pvBuffer,
        TickType_t xTicksToWait
    ) const = 0;

    virtual BaseType_t OS_xQueueSend(
        QueueHandle_t xQueue,
        const void * pvItemToQueue,
        TickType_t xTicksToWait
    ) const = 0;

    virtual BaseType_t OS_xSemaphoreTake(
        SemaphoreHandle_t xSemaphore,
        TickType_t xBlockTime
    ) const = 0;

    virtual BaseType_t OS_xSemaphoreGive(
        SemaphoreHandle_t xSemaphore
    ) const = 0;

    virtual void OS_vTaskDelayUntil(
        TickType_t * const pxPreviousWakeTime,
        const TickType_t xTimeIncrement
    ) const = 0;

    virtual osStatus OS_osDelay (
        uint32_t millisec
    ) const = 0;

    virtual osMutexId OS_osMutexCreate (
      const osMutexDef_t *mutex_def
    ) const = 0;

    virtual osSemaphoreId OS_osSemaphoreCreate (
      const osSemaphoreDef_t *semaphore_def,
      int32_t count
    ) const = 0;
};

} // end namespace os

/**
 * @}
 */
/* end - Header */

#endif /* OS_INTERFACE_H */
