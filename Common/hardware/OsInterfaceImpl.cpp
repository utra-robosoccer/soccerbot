/**
  *****************************************************************************
  * @file    OsInterfaceImpl.cpp
  * @author  Izaak Niksan
  * @brief   Implements the FreeRTOS wrapper class, which makes direct calls
  *          to the true functions defined in the FreeRTOS library
  *
  * @defgroup Implementation
  * @ingroup  OS
  * @brief    The OsInterfaceImpl module provides an interface to true FreeRTOS
  *           functionality
  * @{
  *****************************************************************************
  */




/********************************* Includes **********************************/

#include "OsInterfaceImpl.h"
#include <stdint.h>
#include "cmsis_os.h"





/****************************** OsInterfaceImpl ******************************/
namespace os{
// Classes and structs
// ----------------------------------------------------------------------------
OsInterfaceImpl::OsInterfaceImpl() {

}

OsInterfaceImpl::~OsInterfaceImpl() {

}

BaseType_t OsInterfaceImpl::OS_xTaskNotifyWait(
    uint32_t ulBitsToClearOnEntry,
    uint32_t ulBitsToClearOnExit,
    uint32_t *pulNotificationValue,
    TickType_t xTicksToWait
) const
{
    return xTaskNotifyWait(
        ulBitsToClearOnEntry,
        ulBitsToClearOnExit,
        pulNotificationValue,
        xTicksToWait
    );
}

BaseType_t OsInterfaceImpl::OS_xQueueReceive(
    QueueHandle_t xQueue,
    void *pvBuffer,
    TickType_t xTicksToWait
) const
{
    return xQueueReceive(xQueue, pvBuffer, xTicksToWait);
}

BaseType_t OsInterfaceImpl::OS_xQueueSend(
    QueueHandle_t xQueue,
    const void * pvItemToQueue,
    TickType_t xTicksToWait
) const
{
    return xQueueSend(xQueue, pvItemToQueue, xTicksToWait);
}

BaseType_t OsInterfaceImpl::OS_xSemaphoreTake(
    SemaphoreHandle_t xSemaphore,
    TickType_t xBlockTime
) const
{
    return xSemaphoreTake(xSemaphore, xBlockTime);
}

BaseType_t OsInterfaceImpl::OS_xSemaphoreGive(
    SemaphoreHandle_t xSemaphore
) const
{
    return xSemaphoreGive(xSemaphore);
}

void OsInterfaceImpl::OS_vTaskDelayUntil(
    TickType_t * const pxPreviousWakeTime,
    const TickType_t xTimeIncrement
) const
{
    vTaskDelayUntil(pxPreviousWakeTime, xTimeIncrement);
}

osStatus OsInterfaceImpl::OS_osDelay (
    uint32_t millisec
) const
{
    return osDelay(millisec);
}

osMutexId OsInterfaceImpl::OS_osMutexCreate (
  const osMutexDef_t *mutex_def
) const
{
    return osMutexCreate(mutex_def);
}

osSemaphoreId OsInterfaceImpl::OS_osSemaphoreCreate (
  const osSemaphoreDef_t *semaphore_def,
  int32_t count
) const
{
    return osSemaphoreCreate(semaphore_def, count);
}

} // end namespace os

/**
 * @}
 */
/* end - Implementation */
