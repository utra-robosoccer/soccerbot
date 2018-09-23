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


#include <FunctionalFreeRTOSInterface.h>

Functional_FreeRTOS_Interface::FunctionalFreeRTOSInterface::FunctionalFreeRTOSInterface() {

}

Functional_FreeRTOS_Interface::FunctionalFreeRTOSInterface::~FunctionalFreeRTOSInterface() {

}

BaseType_t Functional_FreeRTOS_Interface::FunctionalFreeRTOSInterface::xTaskNotifyWait(
        uint32_t ulBitsToClearOnEntry,
        uint32_t ulBitsToClearOnExit,
        uint32_t *pulNotificationValue,
        TickType_t xTicksToWait
        ){

    return xTaskNotifyWait(ulBitsToClearOnEntry, ulBitsToClearOnExit, pulNotificationValue, xTicksToWait);
}

BaseType_t Functional_FreeRTOS_Interface::FunctionalFreeRTOSInterface::xQueueReceive(
        QueueHandle_t xQueue,
        void *pvBuffer,
        TickType_t xTicksToWait
        ){

    return xQueueReceive(xQueue, pvBuffer, xTicksToWait);
}

BaseType_t Functional_FreeRTOS_Interface::FunctionalFreeRTOSInterface::xQueueSend(
        QueueHandle_t xQueue,
        const void * pvItemToQueue,
        TickType_t xTicksToWait
        ){

    return xQueueSend(xQueue, pvItemToQueue, xTicksToWait);
}

BaseType_t Functional_FreeRTOS_Interface::FunctionalFreeRTOSInterface::xSemaphoreTake(
        SemaphoreHandle_t xSemaphore,
        TickType_t xBlockTime
        ){

    return xSemaphoreTake(xSemaphore, xBlockTime);
}

BaseType_t Functional_FreeRTOS_Interface::FunctionalFreeRTOSInterface::xSemaphoreGive(
        SemaphoreHandle_t xSemaphore
        ){

    return xSemaphoreGive(xSemaphore);
}

void Functional_FreeRTOS_Interface::FunctionalFreeRTOSInterface::vTaskDelayUntil(
        TickType_t * const pxPreviousWakeTime,
        const TickType_t xTimeIncrement
        ){

    vTaskDelayUntil(pxPreviousWakeTime, xTimeIncrement);
}

osStatus Functional_FreeRTOS_Interface::FunctionalFreeRTOSInterface::osDelay (
        uint32_t millisec
        ){

    return osDelay(millisec);
}

/**
 * @}
 */
/* end - Functional_FreeRTOS_Interface */
