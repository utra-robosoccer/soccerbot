/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
/**
 * @file    freertos.c
 * @brief   Code for freertos application
 * @author  Gokul
 * @author  Tyler
 * @author  Izaak
 * @author  Robert
 *
 * @defgroup FreeRTOS FreeRTOS
 * @brief    Everything related to FreeRTOS
 */

#include <math.h>

#include "uart_handler.h"
#include "Notification.h"
#include "SystemConf.h"
#include "BufferBase.h"
#include "PeripheralInstances.h"
#include "Communication.h"
#include "imu_helper.h"
#include "rx_helper.h"
#include "tx_helper.h"
#include "UartDriver.h"
#include "HalUartInterface.h"
#include "OsInterfaceImpl.h"
#include "CircularDmaBuffer.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;
osThreadId UART1TaskHandle;
uint32_t UART1TaskBuffer[ 128 ];
osStaticThreadDef_t UART1TaskControlBlock;
osThreadId UART2TaskHandle;
uint32_t UART2TaskBuffer[ 128 ];
osStaticThreadDef_t UART2TaskControlBlock;
osThreadId UART3TaskHandle;
uint32_t UART3TaskBuffer[ 128 ];
osStaticThreadDef_t UART3TaskControlBlock;
osThreadId UART4TaskHandle;
uint32_t UART4TaskBuffer[ 128 ];
osStaticThreadDef_t UART4TaskControlBlock;
osThreadId UART6TaskHandle;
uint32_t UART6TaskBuffer[ 128 ];
osStaticThreadDef_t UART6TaskControlBlock;
osThreadId IMUTaskHandle;
uint32_t IMUTaskBuffer[ 128 ];
osStaticThreadDef_t IMUTaskControlBlock;
osThreadId CommandTaskHandle;
uint32_t CommandTaskBuffer[ 128 ];
osStaticThreadDef_t CommandTaskControlBlock;
osThreadId RxTaskHandle;
uint32_t RxTaskBuffer[ 512 ];
osStaticThreadDef_t RxTaskControlBlock;
osThreadId TxTaskHandle;
uint32_t TxTaskBuffer[ 512 ];
osStaticThreadDef_t TxTaskControlBlock;
osThreadId BuffWriterTaskHandle;
uint32_t BuffWriterTaskBuffer[ 128 ];
osStaticThreadDef_t BuffWriterTaskControlBlock;
osThreadId MotorCmdGenTaskHandle;
uint32_t MotorCmdGenTaskBuffer[ 128 ];
osStaticThreadDef_t MotorCmdGenTaskControlBlock;
osMessageQId UART1_reqHandle;
uint8_t UART1_reqBuffer[ 16 * sizeof( UARTcmd_t ) ];
osStaticMessageQDef_t UART1_reqControlBlock;
osMessageQId UART2_reqHandle;
uint8_t UART2_reqBuffer[ 16 * sizeof( UARTcmd_t ) ];
osStaticMessageQDef_t UART2_reqControlBlock;
osMessageQId UART3_reqHandle;
uint8_t UART3_reqBuffer[ 16 * sizeof( UARTcmd_t ) ];
osStaticMessageQDef_t UART3_reqControlBlock;
osMessageQId UART4_reqHandle;
uint8_t UART4_reqBuffer[ 16 * sizeof( UARTcmd_t ) ];
osStaticMessageQDef_t UART4_reqControlBlock;
osMessageQId UART6_reqHandle;
uint8_t UART6_reqBuffer[ 16 * sizeof( UARTcmd_t ) ];
osStaticMessageQDef_t UART6_reqControlBlock;
osMessageQId BufferWriteQueueHandle;
uint8_t BufferWriteQueueBuffer[ 32 * sizeof( TXData_t ) ];
osStaticMessageQDef_t BufferWriteQueueControlBlock;
osMutexId PCUARTHandle;
osStaticMutexDef_t PCUARTControlBlock;
osMutexId DATABUFFERHandle;
osStaticMutexDef_t DATABUFFERControlBlock;

/* USER CODE BEGIN Variables */
namespace{


buffer::BufferMaster BufferMaster;
os::OsInterfaceImpl osInterfaceImpl;

bool setupIsDone = false;
static volatile uint32_t error;

uart::HalUartInterface uartInterface;
os::OsInterfaceImpl osInterface;
uart::UartDriver uartDriver(&osInterface, &uartInterface, &huart5);

uint8_t rxBuff[92] = { };
uint8_t txBuff[92] = { };

UARTcmd_t Motorcmd[18];

}

#define RX_BUFF_SIZE 92

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
extern void StartUART1Task(void const * argument);
extern void StartUART2Task(void const * argument);
extern void StartUART3Task(void const * argument);
extern void StartUART4Task(void const * argument);
extern void StartUART6Task(void const * argument);
extern void StartIMUTask(void const * argument);
extern void StartCommandTask(void const * argument);
extern void StartRxTask(void const * argument);
extern void StartTxTask(void const * argument);
extern void StartBuffWriterTask(void const * argument);
extern void StartMotorCmdGenTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
// For vApplicationGetIdleTaskMemory
#ifdef __cplusplus
extern "C" {
#endif
/* USER CODE END FunctionPrototypes */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* Hook prototypes */

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
// For vApplicationGetIdleTaskMemory
#ifdef __cplusplus
}
#endif

static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Create the mutex(es) */
  /* definition and creation of PCUART */
  osMutexStaticDef(PCUART, &PCUARTControlBlock);
  PCUARTHandle = osMutexCreate(osMutex(PCUART));

  /* definition and creation of DATABUFFER */
  osMutexStaticDef(DATABUFFER, &DATABUFFERControlBlock);
  DATABUFFERHandle = osMutexCreate(osMutex(DATABUFFER));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128, defaultTaskBuffer, &defaultTaskControlBlock);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of UART1Task */
  osThreadStaticDef(UART1Task, StartUART1Task, osPriorityBelowNormal, 0, 128, UART1TaskBuffer, &UART1TaskControlBlock);
  UART1TaskHandle = osThreadCreate(osThread(UART1Task), NULL);

  /* definition and creation of UART2Task */
  osThreadStaticDef(UART2Task, StartUART2Task, osPriorityBelowNormal, 0, 128, UART2TaskBuffer, &UART2TaskControlBlock);
  UART2TaskHandle = osThreadCreate(osThread(UART2Task), NULL);

  /* definition and creation of UART3Task */
  osThreadStaticDef(UART3Task, StartUART3Task, osPriorityBelowNormal, 0, 128, UART3TaskBuffer, &UART3TaskControlBlock);
  UART3TaskHandle = osThreadCreate(osThread(UART3Task), NULL);

  /* definition and creation of UART4Task */
  osThreadStaticDef(UART4Task, StartUART4Task, osPriorityBelowNormal, 0, 128, UART4TaskBuffer, &UART4TaskControlBlock);
  UART4TaskHandle = osThreadCreate(osThread(UART4Task), NULL);

  /* definition and creation of UART6Task */
  osThreadStaticDef(UART6Task, StartUART6Task, osPriorityBelowNormal, 0, 128, UART6TaskBuffer, &UART6TaskControlBlock);
  UART6TaskHandle = osThreadCreate(osThread(UART6Task), NULL);

  /* definition and creation of IMUTask */
  osThreadStaticDef(IMUTask, StartIMUTask, osPriorityNormal, 0, 128, IMUTaskBuffer, &IMUTaskControlBlock);
  IMUTaskHandle = osThreadCreate(osThread(IMUTask), NULL);

  /* definition and creation of CommandTask */
  osThreadStaticDef(CommandTask, StartCommandTask, osPriorityAboveNormal, 0, 128, CommandTaskBuffer, &CommandTaskControlBlock);
  CommandTaskHandle = osThreadCreate(osThread(CommandTask), NULL);

  /* definition and creation of RxTask */
  osThreadStaticDef(RxTask, StartRxTask, osPriorityRealtime, 0, 512, RxTaskBuffer, &RxTaskControlBlock);
  RxTaskHandle = osThreadCreate(osThread(RxTask), NULL);

  /* definition and creation of TxTask */
  osThreadStaticDef(TxTask, StartTxTask, osPriorityHigh, 0, 512, TxTaskBuffer, &TxTaskControlBlock);
  TxTaskHandle = osThreadCreate(osThread(TxTask), NULL);

  /* definition and creation of BuffWriterTask */
  osThreadStaticDef(BuffWriterTask, StartBuffWriterTask, osPriorityNormal, 0, 128, BuffWriterTaskBuffer, &BuffWriterTaskControlBlock);
  BuffWriterTaskHandle = osThreadCreate(osThread(BuffWriterTask), NULL);

  /* definition and creation of MotorCmdGenTask */
  osThreadStaticDef(MotorCmdGenTask, StartMotorCmdGenTask, osPriorityNormal, 0, 128, MotorCmdGenTaskBuffer, &MotorCmdGenTaskControlBlock);
  MotorCmdGenTaskHandle = osThreadCreate(osThread(MotorCmdGenTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of UART1_req */
  osMessageQStaticDef(UART1_req, 16, UARTcmd_t, UART1_reqBuffer, &UART1_reqControlBlock);
  UART1_reqHandle = osMessageCreate(osMessageQ(UART1_req), NULL);

  /* definition and creation of UART2_req */
  osMessageQStaticDef(UART2_req, 16, UARTcmd_t, UART2_reqBuffer, &UART2_reqControlBlock);
  UART2_reqHandle = osMessageCreate(osMessageQ(UART2_req), NULL);

  /* definition and creation of UART3_req */
  osMessageQStaticDef(UART3_req, 16, UARTcmd_t, UART3_reqBuffer, &UART3_reqControlBlock);
  UART3_reqHandle = osMessageCreate(osMessageQ(UART3_req), NULL);

  /* definition and creation of UART4_req */
  osMessageQStaticDef(UART4_req, 16, UARTcmd_t, UART4_reqBuffer, &UART4_reqControlBlock);
  UART4_reqHandle = osMessageCreate(osMessageQ(UART4_req), NULL);

  /* definition and creation of UART6_req */
  osMessageQStaticDef(UART6_req, 16, UARTcmd_t, UART6_reqBuffer, &UART6_reqControlBlock);
  UART6_reqHandle = osMessageCreate(osMessageQ(UART6_req), NULL);

  /* definition and creation of BufferWriteQueue */
  osMessageQStaticDef(BufferWriteQueue, 32, TXData_t, BufferWriteQueueBuffer, &BufferWriteQueueControlBlock);
  BufferWriteQueueHandle = osMessageCreate(osMessageQ(BufferWriteQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
	osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Application */
/**
 * @defgroup Threads Threads
 * @brief    These are functions run in the context of their own FreeRTOS
 *           threads
 *
 * @ingroup  FreeRTOS
 */

/**
  * @brief  This function is executed in the context of the commandTask
  *         thread. It initializes all data structures and peripheral
  *         devices associated with the application, and then assumes
  *         responsibility for distributing write commands to the
  *         actuators.
  *
  *         This function never returns.
  *
  * @ingroup Threads
  */
void StartCommandTask(void const * argument)
{
    // Wait for the motors to turn on
    osDelay(osKernelSysTickMicroSec(100000));

    // Use polled IO here for 2 reasons:
    //   1. Have to initialize the motors from this one thread, so using DMA
    //      doesn't gain us anything
    //   2. The UART callbacks are hardcoded to wake up their corresponding
    //      UART threads used later in the flow
    periph::initMotorIOType(IO_Type::POLL);

    // The return delay time is the time the motor waits before sending back
    // data for a read request. We found that a value of 100 us worked reliably
    // while values lower than this would cause packets to be dropped more
    // frequently
    constexpr uint16_t RETURN_DELAY_TIME = 100;
    for(uint8_t i = periph::MOTOR1; i <= periph::MOTOR18; ++i) {
        // Configure motor to return status packets only for read commands
        periph::motors[i]->setStatusReturnLevel(
            dynamixel::StatusReturnLevel::READS_ONLY
        );

        periph::motors[i]->setReturnDelayTime(RETURN_DELAY_TIME);
        periph::motors[i]->enableTorque(true);

        if(i >= periph::MOTOR13){
            // AX12A-only config for controls
            static_cast<dynamixel::AX12A*>(periph::motors[i])->setComplianceSlope(5);
            static_cast<dynamixel::AX12A*>(periph::motors[i])->setComplianceMargin(1);
        }
    }
 
    // The only other communication with the motors will occur in the UART
    // threads, so we can use DMA now.
    periph::initMotorIOType(IO_Type::DMA);

    // Configure the IMU to use the tightest filter bandwidth
    constexpr uint8_t IMU_DIGITAL_LOWPASS_FILTER_SETTING = 6;
    periph::imuData.init(IMU_DIGITAL_LOWPASS_FILTER_SETTING);

    // Set up the sensor data buffer
    BufferMaster.setup_buffers(DATABUFFERHandle, &osInterfaceImpl);

    // Unblock the other tasks now that initialization is done
    setupIsDone = true;

    osSignalSet(RxTaskHandle, NOTIFIED_FROM_TASK);
    osSignalSet(TxTaskHandle, NOTIFIED_FROM_TASK);
    osSignalSet(MotorCmdGenTaskHandle, NOTIFIED_FROM_TASK);
    osSignalSet(BuffWriterTaskHandle, NOTIFIED_FROM_TASK);
    osSignalSet(IMUTaskHandle, NOTIFIED_FROM_TASK);
    osSignalSet(UART1TaskHandle, NOTIFIED_FROM_TASK);
    osSignalSet(UART2TaskHandle, NOTIFIED_FROM_TASK);
    osSignalSet(UART3TaskHandle, NOTIFIED_FROM_TASK);
    osSignalSet(UART4TaskHandle, NOTIFIED_FROM_TASK);
    osSignalSet(UART6TaskHandle, NOTIFIED_FROM_TASK);

    UARTcmd_t cmd;
    cmd.type = cmdWritePosition;
    float positions[18];
    while(1){
        osSignalWait(NOTIFIED_FROM_TASK, osWaitForever);

        // Convert raw bytes from robotGoal received from PC into floats
        uint8_t* ptr = NULL;
        for(uint8_t i = 0; i < 18; i++){
            ptr = (uint8_t*)&positions[i];
            for(uint8_t j = 0; j < 4; j++){
                *ptr = robotGoal.msg[i * 4 + j];
                ptr++;
            }
        }

        // Send each goal position to the queue, where the UART handler
        // thread that's listening will receive it and send it to the motor
        for(uint8_t i = periph::MOTOR1; i < periph::NUM_MOTORS; ++i){
            cmd.motorHandle = periph::motors[i];
            cmd.value = positions[i];

            if(i <= periph::MOTOR3){
                cmd.qHandle = UART2_reqHandle;
            }
            else if(i <= periph::MOTOR6){
                cmd.qHandle = UART4_reqHandle;
            }
            else if(i <= periph::MOTOR9){
                cmd.qHandle = UART1_reqHandle;
            }
            else if(i <= periph::MOTOR12){
                cmd.qHandle = UART6_reqHandle;
            }
            else{
                cmd.qHandle = UART3_reqHandle;
            }

            xQueueSend(cmd.qHandle, &cmd, 0);
        }
    }
}

/**
  * @brief  This function is executed in the context of the UART1_
  *         thread. It processes all commands for the motors
  *         physically connected to UART1, and initiates the I/O
  *         calls to them. Whenever it processes read commands for
  *         a motor, it sends the data received to the
  *         multi-writer sensor queue, which is read only by the
  *         TX task.
  *
  *         This function never returns.
  *
  * @ingroup Threads
  */
void StartUART1Task(void const * argument)
{
    // Here, we use task notifications to block this task from running until a notification
    // is received. This allows one-time setup to complete in a low-priority task.
    osSignalWait(0, osWaitForever);

    UARTcmd_t cmdMessage;
    TXData_t dataToSend;
    dataToSend.eDataType = eMotorData;

    for(;;)
    {
        while(xQueueReceive(UART1_reqHandle, &cmdMessage, portMAX_DELAY) != pdTRUE);
        UART_ProcessEvent(&cmdMessage, &dataToSend);
    }
}

/**
  * @brief  This function is executed in the context of the UART2_
  *         thread. It processes all commands for the motors
  *         physically connected to UART2, and initiates the I/O
  *         calls to them. Whenever it processes read commands for
  *         a motor, it sends the data received to the
  *         multi-writer sensor queue, which is read only by the
  *         TX task.
  *
  *         This function never returns.
  *
  * @ingroup Threads
  */
void StartUART2Task(void const * argument)
{
    // Here, we use task notifications to block this task from running until a notification
    // is received. This allows one-time setup to complete in a low-priority task.
    osSignalWait(0, osWaitForever);

    UARTcmd_t cmdMessage;
    TXData_t dataToSend;
    dataToSend.eDataType = eMotorData;

    for(;;)
    {
        while(xQueueReceive(UART2_reqHandle, &cmdMessage, portMAX_DELAY) != pdTRUE);
        UART_ProcessEvent(&cmdMessage, &dataToSend);
    }
}

/**
  * @brief  This function is executed in the context of the UART3_
  *         thread. It processes all commands for the motors
  *         physically connected to UART3, and initiates the I/O
  *         calls to them. Whenever it processes read commands for
  *         a motor, it sends the data received to the
  *         multi-writer sensor queue, which is read only by the
  *         TX task.
  *
  *         This function never returns.
  *
  * @ingroup Threads
  */
void StartUART3Task(void const * argument)
{
    // Here, we use task notifications to block this task from running until a notification
    // is received. This allows one-time setup to complete in a low-priority task.
    osSignalWait(0, osWaitForever);

    UARTcmd_t cmdMessage;
    TXData_t dataToSend;
    dataToSend.eDataType = eMotorData;

    for(;;)
    {
        while(xQueueReceive(UART3_reqHandle, &cmdMessage, portMAX_DELAY) != pdTRUE);
        UART_ProcessEvent(&cmdMessage, &dataToSend);
    }
}

/**
  * @brief  This function is executed in the context of the UART4_
  *         thread. It processes all commands for the motors
  *         physically connected to UART4, and initiates the I/O
  *         calls to them. Whenever it processes read commands for
  *         a motor, it sends the data received to the
  *         multi-writer sensor queue, which is read only by the
  *         TX task.
  *
  *         This function never returns.
  *
  * @ingroup Threads
  */
void StartUART4Task(void const * argument)
{
    // Here, we use task notifications to block this task from running until a notification
    // is received. This allows one-time setup to complete in a low-priority task.
    osSignalWait(0, osWaitForever);

    UARTcmd_t cmdMessage;
    TXData_t dataToSend;
    dataToSend.eDataType = eMotorData;

    for(;;)
    {
        while(xQueueReceive(UART4_reqHandle, &cmdMessage, portMAX_DELAY) != pdTRUE);
        UART_ProcessEvent(&cmdMessage, &dataToSend);
    }
}

/**
  * @brief  This function is executed in the context of the UART6_
  *         thread. It processes all commands for the motors
  *         physically connected to UART6, and initiates the I/O
  *         calls to them. Whenever it processes read commands for
  *         a motor, it sends the data received to the
  *         multi-writer sensor queue, which is read only by the
  *         TX task.
  *
  *         This function never returns.
  *
  * @ingroup Threads
  */
void StartUART6Task(void const * argument)
{
    // Here, we use task notifications to block this task from running until a notification
    // is received. This allows one-time setup to complete in a low-priority task.
    osSignalWait(0, osWaitForever);

    /* Infinite loop */
    UARTcmd_t cmdMessage;
    TXData_t dataToSend;
    dataToSend.eDataType = eMotorData;

    for(;;)
    {
        while(xQueueReceive(UART6_reqHandle, &cmdMessage, portMAX_DELAY) != pdTRUE);
        UART_ProcessEvent(&cmdMessage, &dataToSend);
    }
}

/**
  * @brief  This function is executed in the context of the
  *         IMUTask thread. During each control cycle, this thread
  *         fetches accelerometer and gyroscope data, then sends
  *         this data to the multi-writer sensor queue, which is
  *         read only by the TX task.
  *
  *         This function never returns.
  *
  * @ingroup Threads
  */
void StartIMUTask(void const * argument)
{
    /* USER CODE BEGIN StartIMUTask */
    // Here, we use task notifications to block this task from running until a notification
    // is received. This allows one-time setup to complete in a low-priority task.
    osSignalWait(0, osWaitForever);

    constexpr TickType_t IMU_CYCLE_TIME_MS = 2;

    imu::IMUStruct_t myIMUStruct;
    TXData_t dataToSend = {eIMUData, &myIMUStruct};
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint8_t numSamples = 0;
    bool needsProcessing = false;

    app::initImuProcessor();

    for(;;)
    {
        // Service this thread every 2 ms for a 500 Hz sample rate
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(IMU_CYCLE_TIME_MS));

        needsProcessing = app::readFromSensor(periph::imuData, &numSamples);
        periph::imuData.Fill_Struct(&myIMUStruct);

        if(needsProcessing){
            app::processImuData(myIMUStruct);
        }

        xQueueSend(BufferWriteQueueHandle, &dataToSend, 0);
    }
    /* USER CODE END StartIMUTask */
}

void parse(uint8_t* bytes, uint8_t numBytes, int &parse_out){
    static uint state = 0;
    for(uint8_t i = 0; i < numBytes; ++i){
        switch(bytes[i]){
            default:
                if (state >= 92) {
                    state = 0;
                }
                else {
                    state++;
                }
                break;
        }

        switch(state){
            case 92:
                parse_out = 1;
                state = 0;
                break;
            default:
                parse_out = 0;
                break;
        }
    }
}

/**
 * @brief  This function is executed in the context of the RxTask
 *         thread. It initiates DMA-based receptions of RobotGoals
 *         from the PC via UART5. Upon successful reception of a
 *         RobotGoal, the UARTx_ and IMUTask threads are unblocked.
 *
 *         This function never returns.
 *
 * @ingroup Threads
 */
void StartRxTask(void const * argument) {
    int parse_out = 0;

    initializeVars();

    // RxTask waits for first time setup complete.
    osSignalWait(0, osWaitForever);

    uint32_t numIterations = 0;
    float positions[18];

    const uint32_t RX_CYCLE_TIME = osKernelSysTickMicroSec(1000);
    uint32_t xLastWakeTime = osKernelSysTick();

    uint8_t raw[RX_BUFF_SIZE];
    uint8_t processingBuff[RX_BUFF_SIZE];
    uart::CircularDmaBuffer rxBuffer = uart::CircularDmaBuffer(&huart5,
            &uartInterface, RX_BUFF_SIZE, RX_BUFF_SIZE, raw);

    rxBuffer.initiate();

    for (;;) {
        osDelayUntil(&xLastWakeTime, RX_CYCLE_TIME);

        rxBuffer.updateHead();
        if(rxBuffer.dataAvail()){
            uint8_t numBytesReceived = rxBuffer.readBuff(processingBuff);

            parse(processingBuff, numBytesReceived, parse_out);
        }
        rxBuffer.restartIfError();

        if (parse_out == 1) {
            parse_out = 0;

            // XXX: glue code to get rxBuff into buffRx.
            //pcInterface.getRxBuffer(rxBuff, sizeof(rxBuff));
            copyIntoBuffRx(rxBuff);

            // RxTask notifies CommandTask that a complete message has been received.
            receiveDataBuffer();
        }
    }
}

/**
 * @brief  This function is executed in the context of the TxTask
 *         thread. This thread is blocked until all sensor data
 *         has been received through the sensor queue. After this
 *         time, the UARTx_ and IMUTask will be blocked. Then, a
 *         DMA-based transmission of a RobotState is sent to the
 *         PC via UART5.
 *
 *         This function never returns.
 *
 * @ingroup Threads
 */
void StartTxTask(void const * argument) {

    // TxTask waits for first time setup complete.
    osSignalWait(0, osWaitForever);

    for (;;) {
        // Wait until woken up by Rx ("read command event")
        osSignalWait(0, osWaitForever);

        copySensorDataToSend(&BufferMaster);

        // XXX: Glue code
        const uint8_t *txArrayIn = (uint8_t*) &robotState;
        for (size_t iTxArray = 0; iTxArray < sizeof(RobotState); iTxArray++) {
            txBuff[iTxArray] = txArrayIn[iTxArray];
        }

        while(!uartDriver.transmit(txBuff, sizeof(RobotState))) {;}
    }
}


/**
  * @brief  This function is executed in the context of the BuffWriter
  *         thread. This thread creates the sensor data buffer and writes
  *         data from the BufferWrite queue into the appropriate buffer.
  *
  *         This function never returns.
  *
  * @ingroup Threads
  */
void StartBuffWriterTask(void const * argument)
{
    osSignalWait(0, osWaitForever);

    TXData_t dataToWrite;
    imu::IMUStruct_t* IMUDataPtr;
    MotorData_t* motorDataPtr;

    for(;;)
    {
        while(xQueueReceive(BufferWriteQueueHandle, &dataToWrite, osWaitForever) != pdTRUE);
        switch (dataToWrite.eDataType) {
            case eMotorData:
                motorDataPtr = static_cast<MotorData_t*>(dataToWrite.pData);

                if (motorDataPtr == NULL) { break; }

                // Validate data and store it in buffer (thread-safe)
                // Each type of buffer could have its own mutex but this will probably
                // only improve efficiency if there are multiple writer/reader threads
                // and BufferWrite queues.
                if (motorDataPtr->id <= periph::NUM_MOTORS) {
                    BufferMaster.MotorBufferArray[motorDataPtr->id - 1].write(*motorDataPtr);
                }
                break;
            case eIMUData:
                IMUDataPtr = (imu::IMUStruct_t*)dataToWrite.pData;

                if(IMUDataPtr == NULL){ break; }

                // Copy sensor data into the IMU Buffer (thread-safe)
                BufferMaster.IMUBuffer.write(*IMUDataPtr);
                break;
            default:
                break;
        }
    }
}

/**
  * @brief  This function is executed in the context of the MotorCmdGen
  *         thread. This thread generates read commands for the motors
  *         so that position data will be read from them on a
  *         time-triggered basis.
  *
  *         This function never returns.
  *
  * @ingroup Threads
  */
void StartMotorCmdGenTask(void const * argument){
    osSignalWait(0, osWaitForever);

    constexpr uint32_t CYCLE_TIME_MS = osKernelSysTickMicroSec(2000);
    TickType_t xLastWakeTime = osKernelSysTick();
    UARTcmd_t cmd;
    cmd.type = cmdReadPosition;

    for(;;)
    {
        vTaskDelayUntil(&xLastWakeTime, CYCLE_TIME_MS);

        for(uint8_t i = periph::MOTOR1; i < periph::NUM_MOTORS; ++i){
            cmd.motorHandle = periph::motors[i];

            if(i <= periph::MOTOR3){
                cmd.qHandle = UART2_reqHandle;
            }
            else if(i <= periph::MOTOR6){
                cmd.qHandle = UART4_reqHandle;
            }
            else if(i <= periph::MOTOR9){
                cmd.qHandle = UART1_reqHandle;
            }
            else if(i <= periph::MOTOR12){
                cmd.qHandle = UART6_reqHandle;
            }
            else{
                cmd.qHandle = UART3_reqHandle;
            }

            // Only read from legs
            if(i <= periph::MOTOR12){
                xQueueSend(cmd.qHandle, &cmd, 0);
            }
        }
    }
}

/**
 * @defgroup Callbacks Callbacks
 * @brief    Callback functions for unblocking FreeRTOS threads which perform
 *           non-blocking I/O
 *
 * @ingroup FreeRTOS
 */

/**
  * @brief  This function is called whenever a memory read from a I2C
  *         device is completed. For this program, the callback behaviour
  *         consists of unblocking the thread which initiated the I/O and
  *         yielding to a higher priority task from the ISR if there are
  *         any that can run.
  * @param  hi2c pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module corresponding to
  *         the callback
  * @return None
  *
  * @ingroup Callbacks
  */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	// This callback runs after the interrupt data transfer from the sensor to the mcu is finished
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xTaskNotifyFromISR(IMUTaskHandle, NOTIFIED_FROM_RX_ISR, eSetBits, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
  * @brief  This function is called whenever a transmission from a UART
  *         module is completed. For this program, the callback behaviour
  *         consists of unblocking the thread which initiated the I/O and
  *         yielding to a higher priority task from the ISR if there are
  *         any that can run.
  * @param  huart pointer to a UART_HandleTypeDef structure that contains
  *         the configuration information for UART module corresponding to
  *         the callback
  * @return None
  *
  * @ingroup Callbacks
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart){
    if(setupIsDone){
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if(huart == &huart2){
            xTaskNotifyFromISR(TxTaskHandle, NOTIFIED_FROM_TX_ISR, eSetBits, &xHigherPriorityTaskWoken);
        }
        if(huart == &huart1){
            xTaskNotifyFromISR(UART1TaskHandle, NOTIFIED_FROM_TX_ISR, eSetBits, &xHigherPriorityTaskWoken);
        }
        else if(huart == &huart5){
            xTaskNotifyFromISR(UART2TaskHandle, NOTIFIED_FROM_TX_ISR, eSetBits, &xHigherPriorityTaskWoken);
        }
        else if(huart == &huart3){
            xTaskNotifyFromISR(UART3TaskHandle, NOTIFIED_FROM_TX_ISR, eSetBits, &xHigherPriorityTaskWoken);
        }
        else if(huart == &huart4){
            xTaskNotifyFromISR(UART4TaskHandle, NOTIFIED_FROM_TX_ISR, eSetBits, &xHigherPriorityTaskWoken);
        }
        else if(huart == &huart6){
            xTaskNotifyFromISR(UART6TaskHandle, NOTIFIED_FROM_TX_ISR, eSetBits, &xHigherPriorityTaskWoken);
        }
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
  * @brief  This function is called whenever a reception from a UART
  *         module is completed. For this program, the callback behaviour
  *         consists of unblocking the thread which initiated the I/O and
  *         yielding to a higher priority task from the ISR if there are
  *         any that can run.
  * @param  huart pointer to a UART_HandleTypeDef structure that contains
  *         the configuration information for UART module corresponding to
  *         the callback
  * @return None
  *
  * @ingroup Callbacks
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (huart == &huart2) {
        xTaskNotifyFromISR(RxTaskHandle, NOTIFIED_FROM_RX_ISR, eSetBits, &xHigherPriorityTaskWoken);
    }
    if(huart == &huart1){
        xTaskNotifyFromISR(UART1TaskHandle, NOTIFIED_FROM_RX_ISR, eSetBits, &xHigherPriorityTaskWoken);
    }
    else if(huart == &huart5){
        xTaskNotifyFromISR(UART2TaskHandle, NOTIFIED_FROM_RX_ISR, eSetBits, &xHigherPriorityTaskWoken);
    }
    else if(huart == &huart3){
        xTaskNotifyFromISR(UART3TaskHandle, NOTIFIED_FROM_RX_ISR, eSetBits, &xHigherPriorityTaskWoken);
    }
    else if(huart == &huart4){
        xTaskNotifyFromISR(UART4TaskHandle, NOTIFIED_FROM_RX_ISR, eSetBits, &xHigherPriorityTaskWoken);
    }
    else if(huart == &huart6){
        xTaskNotifyFromISR(UART6TaskHandle, NOTIFIED_FROM_RX_ISR, eSetBits, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
  * @brief  This function is called whenever an error is encountered in
  *         association with a UART module. For this program, the callback
  *         behaviour consists of storing the error code in a local
  *         variable.
  * @param  huart pointer to a UART_HandleTypeDef structure that contains
  *         the configuration information for UART module corresponding to
  *         the callback
  * @return None
  *
  * @ingroup Callbacks
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    error = HAL_UART_GetError(huart);
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
