/**
  *****************************************************************************
  * @file    MockFreeRTOSInterface.h
  * @author  Izaak Niksan
  * @author  Tyler Gamvrelis
  *
  * @defgroup MockFreeRTOSInterface
  * @ingroup Mocks
  * @{
  *****************************************************************************
  */




#ifndef MOCK_FREERTOS_INTERFACE_H
#define MOCK_FREERTOS_INTERFACE_H




/********************************* Includes **********************************/
#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "FreeRTOSInterface.h"
using namespace FreeRTOS_Interface;




/************************** Test_FreeRTOS_Interface **************************/
namespace MOCKS{
// Classes and structs
// ----------------------------------------------------------------------------
/**
 * @class MockFreeRTOSInterface Implements FreeRTOSInterface for unit testing
 *        purposes
 */
class MockFreeRTOSInterface : public FreeRTOSInterface {
public:
    MOCK_METHOD4(
        OS_xTaskNotifyWait,
        BaseType_t(
            uint32_t ulBitsToClearOnEntry,
            uint32_t ulBitsToClearOnExit,
            uint32_t* pulNotificationValue,
            TickType_t xTicksToWait
        )
    );

    MOCK_METHOD3(
        OS_xQueueReceive,
        BaseType_t(
            QueueHandle_t xQueue,
            void* pvBuffer,
            TickType_t xTicksToWait
        )
    );

    MOCK_METHOD3(
        OS_xQueueSend,
        BaseType_t(
            QueueHandle_t xQueue,
            const void * pvItemToQueue,
            TickType_t xTicksToWait
        )
    );

    MOCK_METHOD2(
        OS_xSemaphoreTake,
        BaseType_t(
            SemaphoreHandle_t xSemaphore,
            TickType_t xBlockTime
        )
    );

    MOCK_METHOD1(
        OS_xSemaphoreGive,
        BaseType_t(
            SemaphoreHandle_t xSemaphore
        )
    );

    MOCK_METHOD2(
        OS_vTaskDelayUntil,
        void(
            TickType_t* const pxPreviousWakeTime,
            const TickType_t xTimeIncrement
        )
    );

    MOCK_METHOD1(
        OS_osDelay,
        osStatus(
            uint32_t millisec
        )
    );
};

} // end namespace Test_FreeRTOS_Interface




/**
 * @}
 */
/* end - MockFreeRTOSInterface */

#endif /* MOCK_FREERTOS_INTERFACE_H */
