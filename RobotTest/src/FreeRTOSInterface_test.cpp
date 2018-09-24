
/**
  *****************************************************************************
  * @file    FreeRTOSInterface_test.cpp
  * @author  Izaak Niksan
  * @brief   Source file for FreeRTOS testing and mocking
  *
  * @defgroup FreeRTOS_Interface_test
  * @brief    The FreeRTOS_Interface_test module is used to mock and test FreeRTOS functionality
  * @{
  *****************************************************************************
  */

#include <cstdint>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "FreeRTOSInterface.h"

using ::testing::Return;
using ::testing::_;


namespace {

class MockFreeRTOSInterface : public FreeRTOS_Interface::FreeRTOSInterface {
public:
    MOCK_METHOD4(xTaskNotifyWait, BaseType_t(
                                uint32_t ulBitsToClearOnEntry,
                                uint32_t ulBitsToClearOnExit,
                                uint32_t *pulNotificationValue,
                                TickType_t xTicksToWait
                                )
            );

    MOCK_METHOD3(xQueueReceive, BaseType_t(
                                QueueHandle_t xQueue,
                                void *pvBuffer,
                                TickType_t xTicksToWait
                                )
            );

    MOCK_METHOD3(xQueueSend, BaseType_t(
                                QueueHandle_t xQueue,
                                const void * pvItemToQueue,
                                TickType_t xTicksToWait
                                )
            );

    MOCK_METHOD2(xSemaphoreTake, BaseType_t(
                                SemaphoreHandle_t xSemaphore,
                                TickType_t xBlockTime
                                )
            );

    MOCK_METHOD1(xSemaphoreGive, BaseType_t(
                                SemaphoreHandle_t xSemaphore
                                )
            );

    MOCK_METHOD2(vTaskDelayUntil, void(
                                TickType_t * const pxPreviousWakeTime,
                                const TickType_t xTimeIncrement
                                )
            );

    MOCK_METHOD1(osDelay, osStatus(
                                uint32_t millisec
                                )
            );

};

}

/**
 * @}
 */
/* end - FreeRTOS_Interface_test*/
