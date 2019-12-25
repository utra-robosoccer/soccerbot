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
 * @file
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
#include "UartDriver/UartDriver.h"
#include "OsInterfaceImpl.h"
#include "CircularDmaBuffer/CircularDmaBuffer.h"
#include "UartInterfaceImpl.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;
osThreadId UpperLeftLegHandle;
uint32_t UpperLeftLegBuffer[ 128 ];
osStaticThreadDef_t UpperLeftLegControlBlock;
osThreadId LowerRightLegHandle;
uint32_t LowerRightLegBuffer[ 128 ];
osStaticThreadDef_t LowerRightLegControlBlock;
osThreadId HeadAndArmsHandle;
uint32_t HeadAndArmsBuffer[ 128 ];
osStaticThreadDef_t HeadAndArmsControlBlock;
osThreadId UpperRightLegHandle;
uint32_t UpperRightLegBuffer[ 128 ];
osStaticThreadDef_t UpperRightLegControlBlock;
osThreadId LowerLeftLegHandle;
uint32_t LowerLeftLegBuffer[ 128 ];
osStaticThreadDef_t LowerLeftLegControlBlock;
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
osMessageQId UpperLeftLeg_reqHandle;
uint8_t UpperLeftLeg_reqBuffer[ 16 * sizeof( UartCmd_t ) ];
osStaticMessageQDef_t UpperLeftLeg_reqControlBlock;
osMessageQId LowerRightLeg_reqHandle;
uint8_t LowerRightLeg_reqBuffer[ 16 * sizeof( UartCmd_t ) ];
osStaticMessageQDef_t LowerRightLeg_reqControlBlock;
osMessageQId HeadAndArms_reqHandle;
uint8_t HeadAndArms_reqBuffer[ 16 * sizeof( UartCmd_t ) ];
osStaticMessageQDef_t HeadAndArms_reqControlBlock;
osMessageQId UpperRightLeg_reqHandle;
uint8_t UpperRightLeg_reqBuffer[ 16 * sizeof( UartCmd_t ) ];
osStaticMessageQDef_t UpperRightLeg_reqControlBlock;
osMessageQId LowerLeftLeg_reqHandle;
uint8_t LowerLeftLeg_reqBuffer[ 16 * sizeof( UartCmd_t ) ];
osStaticMessageQDef_t LowerLeftLeg_reqControlBlock;
osMessageQId BufferWriteQueueHandle;
uint8_t BufferWriteQueueBuffer[ 32 * sizeof( TxData_t ) ];
osStaticMessageQDef_t BufferWriteQueueControlBlock;
osMutexId PCUARTHandle;
osStaticMutexDef_t PCUARTControlBlock;
osMutexId DATABUFFERHandle;
osStaticMutexDef_t DATABUFFERControlBlock;

/* USER CODE BEGIN Variables */

constexpr size_t RX_BUFF_SIZE = 92;

namespace{

// TODO: these variables are to be moved out of freertos.cpp

buffer::BufferMaster buffer_master;
cmsis::OsInterfaceImpl os_if_impl;
hal::UartInterfaceImpl uart_if;
uart::UartDriver pc_uart_driver(&os_if_impl, &uart_if, UART_HANDLE_PC);

bool setup_is_done = false;
static volatile uint32_t error;

/* Set the period the TxThread waits before being timed out when waiting for DMA
 * transfer to complete. The theoretical minimum given a baud rate of 230400 and
 * message size of 92 bytes is 4ms. Give an extra millisecond to allow for any
 * scheduling delays, so this is set to 5ms. */
constexpr TickType_t TX_CYCLE_TIME_MS = 5;
}
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
extern void StartUpperLeftLeg(void const * argument);
extern void StartLowerRightLeg(void const * argument);
extern void StartHeadAndArms(void const * argument);
extern void StartUpperRightLeg(void const * argument);
extern void StartLowerLeftLeg(void const * argument);
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

using hal::IO_Type;
using namespace soccerbot; // TODO(tgamvrel) using namespace

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

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
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

  /* definition and creation of UpperLeftLeg */
  osThreadStaticDef(UpperLeftLeg, StartUpperLeftLeg, osPriorityBelowNormal, 0, 128, UpperLeftLegBuffer, &UpperLeftLegControlBlock);
  UpperLeftLegHandle = osThreadCreate(osThread(UpperLeftLeg), NULL);

  /* definition and creation of LowerRightLeg */
  osThreadStaticDef(LowerRightLeg, StartLowerRightLeg, osPriorityBelowNormal, 0, 128, LowerRightLegBuffer, &LowerRightLegControlBlock);
  LowerRightLegHandle = osThreadCreate(osThread(LowerRightLeg), NULL);

  /* definition and creation of HeadAndArms */
  osThreadStaticDef(HeadAndArms, StartHeadAndArms, osPriorityBelowNormal, 0, 128, HeadAndArmsBuffer, &HeadAndArmsControlBlock);
  HeadAndArmsHandle = osThreadCreate(osThread(HeadAndArms), NULL);

  /* definition and creation of UpperRightLeg */
  osThreadStaticDef(UpperRightLeg, StartUpperRightLeg, osPriorityBelowNormal, 0, 128, UpperRightLegBuffer, &UpperRightLegControlBlock);
  UpperRightLegHandle = osThreadCreate(osThread(UpperRightLeg), NULL);

  /* definition and creation of LowerLeftLeg */
  osThreadStaticDef(LowerLeftLeg, StartLowerLeftLeg, osPriorityBelowNormal, 0, 128, LowerLeftLegBuffer, &LowerLeftLegControlBlock);
  LowerLeftLegHandle = osThreadCreate(osThread(LowerLeftLeg), NULL);

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
  osMessageQStaticDef(UpperLeftLeg_req, 16, UartCmd_t, UpperLeftLeg_reqBuffer, &UpperLeftLeg_reqControlBlock);
  UpperLeftLeg_reqHandle = osMessageCreate(osMessageQ(UpperLeftLeg_req), NULL);

  /* definition and creation of LowerRightLeg_req */
  osMessageQStaticDef(LowerRightLeg_req, 16, UartCmd_t, LowerRightLeg_reqBuffer, &LowerRightLeg_reqControlBlock);
  LowerRightLeg_reqHandle = osMessageCreate(osMessageQ(LowerRightLeg_req), NULL);

  /* definition and creation of HeadAndArms_req */
  osMessageQStaticDef(HeadAndArms_req, 16, UartCmd_t, HeadAndArms_reqBuffer, &HeadAndArms_reqControlBlock);
  HeadAndArms_reqHandle = osMessageCreate(osMessageQ(HeadAndArms_req), NULL);

  /* definition and creation of UpperRightLeg_req */
  osMessageQStaticDef(UpperRightLeg_req, 16, UartCmd_t, UpperRightLeg_reqBuffer, &UpperRightLeg_reqControlBlock);
  UpperRightLeg_reqHandle = osMessageCreate(osMessageQ(UpperRightLeg_req), NULL);

  /* definition and creation of LowerLeftLeg_req */
  osMessageQStaticDef(LowerLeftLeg_req, 16, UartCmd_t, LowerLeftLeg_reqBuffer, &LowerLeftLeg_reqControlBlock);
  LowerLeftLeg_reqHandle = osMessageCreate(osMessageQ(LowerLeftLeg_req), NULL);

  /* definition and creation of BufferWriteQueue */
  osMessageQStaticDef(BufferWriteQueue, 32, TxData_t, BufferWriteQueueBuffer, &BufferWriteQueueControlBlock);
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

    pc_uart_driver.setIOType(IO_Type::DMA);
    pc_uart_driver.setMaxBlockTime(pdMS_TO_TICKS(TX_CYCLE_TIME_MS));

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

        if(i <= periph::MOTOR12){
            // MX-28-only config for controls
            auto pMotor = static_cast<dynamixel::MX28*>(periph::motors[i]);
            pMotor->setPGain(45);
            pMotor->setDGain(4);
        }
        else{
            // AX-12A-only config for controls
            auto pMotor = static_cast<dynamixel::AX12A*>(periph::motors[i]);
            pMotor->setComplianceSlope(5);
            pMotor->setComplianceMargin(1);
        }
    }

    // The only other communication with the motors will occur in the UART
    // threads, so we can use DMA now.
    periph::initMotorIOType(IO_Type::DMA);

    // Configure the IMU to use the tightest filter bandwidth
    constexpr uint8_t IMU_DIGITAL_LOWPASS_FILTER_SETTING = 6;
    periph::imu_data.init(IMU_DIGITAL_LOWPASS_FILTER_SETTING);

    // Set up the sensor data buffer
    buffer_master.setup_buffers(DATABUFFERHandle, &os_if_impl);

    // Unblock the other tasks now that initialization is done
    setup_is_done = true;

    osSignalSet(RxTaskHandle, NOTIFIED_FROM_TASK);
    osSignalSet(TxTaskHandle, NOTIFIED_FROM_TASK);
    osSignalSet(MotorCmdGenTaskHandle, NOTIFIED_FROM_TASK);
    osSignalSet(BuffWriterTaskHandle, NOTIFIED_FROM_TASK);
    osSignalSet(IMUTaskHandle, NOTIFIED_FROM_TASK);
    osSignalSet(UpperLeftLegHandle, NOTIFIED_FROM_TASK);
    osSignalSet(LowerRightLegHandle, NOTIFIED_FROM_TASK);
    osSignalSet(HeadAndArmsHandle, NOTIFIED_FROM_TASK);
    osSignalSet(UpperRightLegHandle, NOTIFIED_FROM_TASK);
    osSignalSet(LowerLeftLegHandle, NOTIFIED_FROM_TASK);

    UartCmd_t cmd;
    cmd.type = cmdWritePosition;
    float positions[18];
    while(1){
        osSignalWait(NOTIFIED_FROM_TASK, osWaitForever);

        // Convert raw bytes from robotGoal received from PC into floats
        uint8_t* ptr = nullptr;
        comm::RobotGoal_t& robot_goal = comm::getRobotGoal();
        for(uint8_t i = 0; i < 18; i++){
            ptr = (uint8_t*)&positions[i];
            for(uint8_t j = 0; j < 4; j++){
                *ptr = robot_goal.msg[i * 4 + j];
                ptr++;
            }
        }

        // Send each goal position to the queue, where the UART handler
        // thread that's listening will receive it and send it to the motor
        for(uint8_t i = periph::MOTOR1; i < periph::NUM_MOTORS; ++i){
            cmd.motorHandle = periph::motors[i];
            cmd.value = positions[i];

            if(i <= periph::MOTOR3){
                cmd.qHandle = LowerRightLeg_reqHandle;
            }
            else if(i <= periph::MOTOR6){
                cmd.qHandle = UpperRightLeg_reqHandle;
            }
            else if(i <= periph::MOTOR9){
                cmd.qHandle = UpperLeftLeg_reqHandle;
            }
            else if(i <= periph::MOTOR12){
                cmd.qHandle = LowerLeftLeg_reqHandle;
            }
            else{
                cmd.qHandle = HeadAndArms_reqHandle;
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
void StartUpperLeftLeg(void const * argument)
{
    // Here, we use task notifications to block this task from running until a notification
    // is received. This allows one-time setup to complete in a low-priority task.
    osSignalWait(0, osWaitForever);

    UartCmd_t cmd_msg;

    for(;;)
    {
        while(xQueueReceive(UpperLeftLeg_reqHandle, &cmd_msg, portMAX_DELAY) != pdTRUE);
        UART_ProcessEvent(&cmd_msg);
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
void StartLowerRightLeg(void const * argument)
{
    // Here, we use task notifications to block this task from running until a notification
    // is received. This allows one-time setup to complete in a low-priority task.
    osSignalWait(0, osWaitForever);

    UartCmd_t cmd_msg;

    for(;;)
    {
        while(xQueueReceive(LowerRightLeg_reqHandle, &cmd_msg, portMAX_DELAY) != pdTRUE);
        UART_ProcessEvent(&cmd_msg);
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
void StartHeadAndArms(void const * argument)
{
    // Here, we use task notifications to block this task from running until a notification
    // is received. This allows one-time setup to complete in a low-priority task.
    osSignalWait(0, osWaitForever);

    UartCmd_t cmd_msg;

    for(;;)
    {
        while(xQueueReceive(HeadAndArms_reqHandle, &cmd_msg, portMAX_DELAY) != pdTRUE);
        UART_ProcessEvent(&cmd_msg);
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
void StartUpperRightLeg(void const * argument)
{
    // Here, we use task notifications to block this task from running until a notification
    // is received. This allows one-time setup to complete in a low-priority task.
    osSignalWait(0, osWaitForever);

    UartCmd_t cmd_msg;

    for(;;)
    {
        while(xQueueReceive(UpperRightLeg_reqHandle, &cmd_msg, portMAX_DELAY) != pdTRUE);
        UART_ProcessEvent(&cmd_msg);
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
void StartLowerLeftLeg(void const * argument)
{
    // Here, we use task notifications to block this task from running until a notification
    // is received. This allows one-time setup to complete in a low-priority task.
    osSignalWait(0, osWaitForever);

    UartCmd_t cmd_msg;

    for(;;)
    {
        while(xQueueReceive(LowerLeftLeg_reqHandle, &cmd_msg, portMAX_DELAY) != pdTRUE);
        UART_ProcessEvent(&cmd_msg);
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

    imu::ImuStruct_t imu_data;
    TxData_t data_to_send = {eImuData, &imu_data};
    TickType_t last_wake_time = xTaskGetTickCount();
    uint8_t num_samples = 0;
    bool need_to_process = false;

    app::initImuProcessor();

    for(;;)
    {
        // Service this thread every 2 ms for a 500 Hz sample rate
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(IMU_CYCLE_TIME_MS));

        need_to_process = app::readFromSensor(periph::imu_data, &num_samples);
        periph::imu_data.Fill_Struct(&imu_data);

        if(need_to_process){
            app::processImuData(imu_data);
        }

        xQueueSend(BufferWriteQueueHandle, &data_to_send, 0);
    }
    /* USER CODE END StartIMUTask */
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
    // RxTask waits for first time setup complete.
    osSignalWait(0, osWaitForever);

    initializeVars();

    const uint32_t RX_CYCLE_TIME = osKernelSysTickMicroSec(1000);
    uint32_t last_wake_time = osKernelSysTick();

    bool parse_out = 0;
    uint8_t raw[RX_BUFF_SIZE];
    uint8_t processing_buff[RX_BUFF_SIZE];
    uart::CircularDmaBuffer rx_buffer = uart::CircularDmaBuffer(
        UART_HANDLE_PC,
        &uart_if,
        RX_BUFF_SIZE,
        RX_BUFF_SIZE,
        raw
    );

    rx_buffer.initiate();

    for (;;) {
        osDelayUntil(&last_wake_time, RX_CYCLE_TIME);

        rx_buffer.updateHead();
        if(rx_buffer.dataAvail()){
            uint8_t num_bytes_received = rx_buffer.readBuff(processing_buff);
            parseByteSequence(processing_buff, num_bytes_received, parse_out);
        }

        rx_buffer.reinitiateIfError();

        if (parse_out) {
            parse_out = false;

            copyParsedData();

            osSignalSet(TxTaskHandle, NOTIFIED_FROM_TASK);
            osSignalSet(CommandTaskHandle, NOTIFIED_FROM_TASK);
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

        copySensorDataToSend(&buffer_master);

        // TODO: should have a way to back out of a failed transmit and reinitiate
        // (e.g. timeout), number of attempts, ..., rather than infinitely loop.
        comm::RobotState_t& robot_state = comm::getRobotState();
        while(!pc_uart_driver.transmit(
                reinterpret_cast<uint8_t*>(&robot_state),
                sizeof(comm::RobotState_t)
            )
        )
        {
            continue;
        }
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

    TxData_t data_to_write;
    imu::ImuStruct_t* imu_data_ptr;
    MotorData_t* motor_data_ptr;

    for(;;)
    {
        while(xQueueReceive(BufferWriteQueueHandle, &data_to_write, osWaitForever) != pdTRUE);
        switch (data_to_write.eDataType) {
            case eMotorData:
                motor_data_ptr = static_cast<MotorData_t*>(data_to_write.pData);

                if (motor_data_ptr == nullptr) {
                    break;
                }

                // Validate data and store it in buffer (thread-safe)
                // Each type of buffer could have its own mutex but this will probably
                // only improve efficiency if there are multiple writer/reader threads
                // and BufferWrite queues.
                if (motor_data_ptr->id <= periph::NUM_MOTORS) {
                    buffer_master.motor_buffer_array[motor_data_ptr->id - 1].write(*motor_data_ptr);
                }
                break;
            case eImuData:
                imu_data_ptr = (imu::ImuStruct_t*)data_to_write.pData;

                if(imu_data_ptr == nullptr){
                    break;
                }

                // Copy sensor data into the IMU Buffer (thread-safe)
                buffer_master.imu_buffer.write(*imu_data_ptr);
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
    TickType_t last_wake_time = osKernelSysTick();
    UartCmd_t cmd;
    cmd.type = cmdReadPosition;

    for(;;)
    {
        vTaskDelayUntil(&last_wake_time, CYCLE_TIME_MS);

        for(uint8_t i = periph::MOTOR1; i < periph::NUM_MOTORS; ++i){
            cmd.motorHandle = periph::motors[i];

            if(i <= periph::MOTOR3){
                cmd.qHandle = LowerRightLeg_reqHandle;
            }
            else if(i <= periph::MOTOR6){
                cmd.qHandle = UpperRightLeg_reqHandle;
            }
            else if(i <= periph::MOTOR9){
                cmd.qHandle = UpperLeftLeg_reqHandle;
            }
            else if(i <= periph::MOTOR12){
                cmd.qHandle = LowerLeftLeg_reqHandle;
            }
            else{
                cmd.qHandle = HeadAndArms_reqHandle;
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
    if(setup_is_done){
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if(huart == UART_HANDLE_PC){
            xTaskNotifyFromISR(TxTaskHandle, NOTIFIED_FROM_TX_ISR, eSetBits, &xHigherPriorityTaskWoken);
        }
        if(huart == UART_HANDLE_UpperLeftLeg){
            xTaskNotifyFromISR(UpperLeftLegHandle, NOTIFIED_FROM_TX_ISR, eSetBits, &xHigherPriorityTaskWoken);
        }
        else if(huart == UART_HANDLE_LowerRightLeg){
            xTaskNotifyFromISR(LowerRightLegHandle, NOTIFIED_FROM_TX_ISR, eSetBits, &xHigherPriorityTaskWoken);
        }
        else if(huart == UART_HANDLE_HeadAndArms){
            xTaskNotifyFromISR(HeadAndArmsHandle, NOTIFIED_FROM_TX_ISR, eSetBits, &xHigherPriorityTaskWoken);
        }
        else if(huart == UART_HANDLE_UpperRightLeg){
            xTaskNotifyFromISR(UpperRightLegHandle, NOTIFIED_FROM_TX_ISR, eSetBits, &xHigherPriorityTaskWoken);
        }
        else if(huart == UART_HANDLE_LowerLeftLeg){
            xTaskNotifyFromISR(LowerLeftLegHandle, NOTIFIED_FROM_TX_ISR, eSetBits, &xHigherPriorityTaskWoken);
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
    if(huart == UART_HANDLE_UpperLeftLeg){
        xTaskNotifyFromISR(UpperLeftLegHandle, NOTIFIED_FROM_RX_ISR, eSetBits, &xHigherPriorityTaskWoken);
    }
    else if(huart == UART_HANDLE_LowerRightLeg){
        xTaskNotifyFromISR(LowerRightLegHandle, NOTIFIED_FROM_RX_ISR, eSetBits, &xHigherPriorityTaskWoken);
    }
    else if(huart == UART_HANDLE_HeadAndArms){
        xTaskNotifyFromISR(HeadAndArmsHandle, NOTIFIED_FROM_RX_ISR, eSetBits, &xHigherPriorityTaskWoken);
    }
    else if(huart == UART_HANDLE_UpperRightLeg){
        xTaskNotifyFromISR(UpperRightLegHandle, NOTIFIED_FROM_RX_ISR, eSetBits, &xHigherPriorityTaskWoken);
    }
    else if(huart == UART_HANDLE_LowerLeftLeg){
        xTaskNotifyFromISR(LowerLeftLegHandle, NOTIFIED_FROM_RX_ISR, eSetBits, &xHigherPriorityTaskWoken);
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
