/**
  *****************************************************************************
  * @file    FunctionalFreeRTOSInterface.cpp
  * @author  Izaak Niksan
  * @brief   Implements the FreeRTOS wrapper class, which makes direct calls
  *          to the true functions defined in the FreeRTOS library
  *
  * @defgroup Functional_FreeRTOS_Interface
  * @brief    The FunctionalFreeRTOSInterface module provides an interface to
  *           true FreeRTOS functionality
  * @{
  *****************************************************************************
  */




/********************************* Includes **********************************/
#include <cstdint>
#include "cmsis_os.h"

#include "FunctionalFreeRTOSInterface.h"




/********************** Functional_FreeRTOS_Interface ************************/
namespace Functional_FreeRTOS_Interface{
// Classes and structs
// ----------------------------------------------------------------------------
FunctionalFreeRTOSInterface::FunctionalFreeRTOSInterface() {

}

FunctionalFreeRTOSInterface::~FunctionalFreeRTOSInterface() {

}

BaseType_t FunctionalFreeRTOSInterface::OS_xTaskNotifyWait(
    uint32_t ulBitsToClearOnEntry,
    uint32_t ulBitsToClearOnExit,
    uint32_t *pulNotificationValue,
    TickType_t xTicksToWait
)
{
    return xTaskNotifyWait(ulBitsToClearOnEntry, ulBitsToClearOnExit, pulNotificationValue, xTicksToWait);
}

BaseType_t FunctionalFreeRTOSInterface::OS_xQueueReceive(
    QueueHandle_t xQueue,
    void *pvBuffer,
    TickType_t xTicksToWait
)
{
    return xQueueReceive(xQueue, pvBuffer, xTicksToWait);
}

BaseType_t FunctionalFreeRTOSInterface::OS_xQueueSend(
    QueueHandle_t xQueue,
    const void * pvItemToQueue,
    TickType_t xTicksToWait
)
{
    return xQueueSend(xQueue, pvItemToQueue, xTicksToWait);
}

BaseType_t FunctionalFreeRTOSInterface::OS_xSemaphoreTake(
    SemaphoreHandle_t xSemaphore,
    TickType_t xBlockTime
)
{
    return xSemaphoreTake(xSemaphore, xBlockTime);
}

BaseType_t FunctionalFreeRTOSInterface::OS_xSemaphoreGive(
    SemaphoreHandle_t xSemaphore
)
{
    return xSemaphoreGive(xSemaphore);
}

void FunctionalFreeRTOSInterface::OS_vTaskDelayUntil(
    TickType_t * const pxPreviousWakeTime,
    const TickType_t xTimeIncrement
)
{
    vTaskDelayUntil(pxPreviousWakeTime, xTimeIncrement);
}

osStatus FunctionalFreeRTOSInterface::OS_osDelay (
    uint32_t millisec
)
{
    return osDelay(millisec);
}

} // end namespace Functional_FreeRTOS_Interface

/**
 * @}
 */
/* end - Functional_FreeRTOS_Interface */
