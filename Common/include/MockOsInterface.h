/**
  *****************************************************************************
  * @file    MockOsInterface.h
  * @author  Izaak Niksan
  * @author  Tyler Gamvrelis
  *
  * @defgroup MockOsInterface
  * @ingroup Mocks
  * @{
  *****************************************************************************
  */




#ifndef MOCK_OS_INTERFACE_H
#define MOCK_OS_INTERFACE_H




/********************************* Includes **********************************/
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "OsInterface.h"

using os::OsInterface;




/****************************** MockOsInterface ******************************/
namespace mocks{
// Classes and structs
// ----------------------------------------------------------------------------
/**
 * @class MockOsInterface Implements OsInterface for unit testing
 *        purposes
 */
class MockOsInterface : public OsInterface {
public:
    MOCK_CONST_METHOD4(
        OS_xTaskNotifyWait,
        BaseType_t(
            uint32_t ulBitsToClearOnEntry,
            uint32_t ulBitsToClearOnExit,
            uint32_t* pulNotificationValue,
            TickType_t xTicksToWait
        )
    );

    MOCK_CONST_METHOD3(
        OS_xQueueReceive,
        BaseType_t(
            QueueHandle_t xQueue,
            void* pvBuffer,
            TickType_t xTicksToWait
        )
    );

    MOCK_CONST_METHOD3(
        OS_xQueueSend,
        BaseType_t(
            QueueHandle_t xQueue,
            const void * pvItemToQueue,
            TickType_t xTicksToWait
        )
    );

    MOCK_CONST_METHOD2(
        OS_xSemaphoreTake,
        BaseType_t(
            SemaphoreHandle_t xSemaphore,
            TickType_t xBlockTime
        )
    );

    MOCK_CONST_METHOD1(
        OS_xSemaphoreGive,
        BaseType_t(
            SemaphoreHandle_t xSemaphore
        )
    );

    MOCK_CONST_METHOD2(
        OS_vTaskDelayUntil,
        void(
            TickType_t* const pxPreviousWakeTime,
            const TickType_t xTimeIncrement
        )
    );

    MOCK_CONST_METHOD1(
        OS_osDelay,
        osStatus(
            uint32_t millisec
        )
    );

    MOCK_CONST_METHOD1(
        OS_osMutexCreate,
        osMutexId(
            const osMutexDef_t *
        )
    );

    MOCK_CONST_METHOD2(
        OS_osSemaphoreCreate,
        osSemaphoreId(
            const osSemaphoreDef_t *,
            int32_t
        )
    );

    MOCK_CONST_METHOD2(
        OS_osMutexWait,
        osStatus(
            osMutexId,
            uint32_t
        )
    );

    MOCK_CONST_METHOD1(
        OS_osMutexRelease,
        osStatus(
                osMutexId
        )
    );

    MOCK_CONST_METHOD2(
        OS_osSemaphoreWait,
        int32_t(
            osSemaphoreId,
            uint32_t millisec
        )
    );

    MOCK_CONST_METHOD1(
            OS_osSemaphoreRelease,
            osStatus(
                osSemaphoreId semaphore_id
            )
    );
};

} // end namespace mocks




/**
 * @}
 */
/* end - MockOsInterface */

#endif /* MOCK_OS_INTERFACE_H */
