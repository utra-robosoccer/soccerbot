/**
  *****************************************************************************
  * @file    OsInterfaceImpl.h
  * @author  Izaak Niksan
  * @brief   An interface to various production FreeRTOS functions, which serves
  *          as a wrapper to the true functionality of the OS.
  *
  * @defgroup Header
  * @ingroup  OS
  * @{
  *****************************************************************************
  */




#ifndef OS_INTERFACE_IMPL_H
#define OS_INTERFACE_IMPL_H




/********************************* Includes **********************************/
// Headers for FreeRTOS, such as task.h, are all included within cmsis_os.h
#include "OsInterface.h"




/****************************** OsInterfaceImpl ******************************/
namespace cmsis {
// Classes and structs
// ----------------------------------------------------------------------------
class OsInterfaceImpl : public OsInterface{
public:
    OsInterfaceImpl();
    ~OsInterfaceImpl();

    BaseType_t OS_xTaskNotifyWait(
        uint32_t ulBitsToClearOnEntry,
        uint32_t ulBitsToClearOnExit,
        uint32_t *pulNotificationValue,
        TickType_t xTicksToWait
    ) const override final;

    BaseType_t OS_xQueueReceive(
        QueueHandle_t xQueue,
        void *pvBuffer,
        TickType_t xTicksToWait
    ) const override final;

    BaseType_t OS_xQueueSend(
        QueueHandle_t xQueue,
        const void * pvItemToQueue,
        TickType_t xTicksToWait
    ) const override final;

    BaseType_t OS_xSemaphoreTake(
        SemaphoreHandle_t xSemaphore,
        TickType_t xBlockTime
    ) const override final;

    BaseType_t OS_xSemaphoreGive(
        SemaphoreHandle_t xSemaphore
    ) const override final;

    void OS_vTaskDelayUntil(
        TickType_t * const pxPreviousWakeTime,
        const TickType_t xTimeIncrement
    ) const override final;

    osStatus OS_osDelay (
        uint32_t millisec
    ) const override final;

    osMutexId OS_osMutexCreate (
        const osMutexDef_t *mutex_def
    ) const override final;
    
    osSemaphoreId OS_osSemaphoreCreate (
        const osSemaphoreDef_t *semaphore_def,
        int32_t count
    ) const override final;

    osStatus OS_osMutexWait (
        osMutexId mutex_id,
        uint32_t millisec
    ) const override final;

    osStatus OS_osMutexRelease (
        osMutexId mutex_id
    ) const override final;

    int32_t OS_osSemaphoreWait (
        osSemaphoreId semaphore_id,
        uint32_t millisec
    ) const override final;

     osStatus OS_osSemaphoreRelease (
        osSemaphoreId semaphore_id
    ) const override final;
};

} // end namespace os

/**
 * @}
 */
/* end - Header */

#endif /* OS_INTERFACE_IMPL_H */
