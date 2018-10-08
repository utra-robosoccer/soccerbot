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
 *
 * @defgroup FreeRTOS FreeRTOS
 * @brief    Everything related to FreeRTOS
 */

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "Notification.h"
#include "SystemConf.h"
#include "usart.h"
#include "gpio.h"
#include "i2c.h"
#include "MPU6050.h"
#include "MPUFilter.h"
#include "UART_Handler.h"
#include "DynamixelProtocolV1.h"
#include "Communication.h"
#include "rx_helper.h"
#include "tx_helper.h"

#include "lwip.h"
#include "ethernetif.h"
#include "PcInterface.h"
#include "OsInterfaceImpl.h"
#include "LwipUdpinterface.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[128];
osStaticThreadDef_t defaultTaskControlBlock;
osThreadId UART1TaskHandle;
uint32_t UART1TaskBuffer[128];
osStaticThreadDef_t UART1TaskControlBlock;
osThreadId UART2TaskHandle;
uint32_t UART2TaskBuffer[128];
osStaticThreadDef_t UART2TaskControlBlock;
osThreadId UART3TaskHandle;
uint32_t UART3TaskBuffer[128];
osStaticThreadDef_t UART3TaskControlBlock;
osThreadId UART4TaskHandle;
uint32_t UART4TaskBuffer[128];
osStaticThreadDef_t UART4TaskControlBlock;
osThreadId UART6TaskHandle;
uint32_t UART6TaskBuffer[128];
osStaticThreadDef_t UART6TaskControlBlock;
osThreadId IMUTaskHandle;
uint32_t IMUTaskBuffer[128];
osStaticThreadDef_t IMUTaskControlBlock;
osThreadId CommandTaskHandle;
uint32_t CommandTaskBuffer[512];
osStaticThreadDef_t CommandTaskControlBlock;
osThreadId RxTaskHandle;
uint32_t RxTaskBuffer[512];
osStaticThreadDef_t RxTaskControlBlock;
osThreadId TxTaskHandle;
uint32_t TxTaskBuffer[512];
osStaticThreadDef_t TxTaskControlBlock;
osMessageQId UART1_reqHandle;
uint8_t UART1_reqBuffer[16 * sizeof(UARTcmd_t)];
osStaticMessageQDef_t UART1_reqControlBlock;
osMessageQId UART2_reqHandle;
uint8_t UART2_reqBuffer[16 * sizeof(UARTcmd_t)];
osStaticMessageQDef_t UART2_reqControlBlock;
osMessageQId UART3_reqHandle;
uint8_t UART3_reqBuffer[16 * sizeof(UARTcmd_t)];
osStaticMessageQDef_t UART3_reqControlBlock;
osMessageQId UART4_reqHandle;
uint8_t UART4_reqBuffer[16 * sizeof(UARTcmd_t)];
osStaticMessageQDef_t UART4_reqControlBlock;
osMessageQId UART6_reqHandle;
uint8_t UART6_reqBuffer[16 * sizeof(UARTcmd_t)];
osStaticMessageQDef_t UART6_reqControlBlock;
osMessageQId TXQueueHandle;
uint8_t TXQueueBuffer[32 * sizeof(TXData_t)];
osStaticMessageQDef_t TXQueueControlBlock;
osMutexId PCUARTHandle;
osStaticMutexDef_t PCUARTControlBlock;

/* USER CODE BEGIN Variables */
enum motorNames {
    MOTOR1,
    MOTOR2,
    MOTOR3,
    MOTOR4,
    MOTOR5,
    MOTOR6,
    MOTOR7,
    MOTOR8,
    MOTOR9,
    MOTOR10,
    MOTOR11,
    MOTOR12,
    MOTOR13,
    MOTOR14,
    MOTOR15,
    MOTOR16,
    MOTOR17,
    MOTOR18
};

Dynamixel_HandleTypeDef Motor1, Motor2, Motor3, Motor4, Motor5, Motor6, Motor7,
        Motor8, Motor9, Motor10, Motor11, Motor12, Motor13, Motor14, Motor15,
        Motor16, Motor17, Motor18;

IMUnamespace::MPU6050 IMUdata(1, &hi2c1);

bool setupIsDone = false;
static volatile uint32_t error;

ip_addr_t mcuIpAddr = {0xC0A8002B};
ip_addr_t pcIpAddr = {0xC0A80001};
os::OsInterfaceImpl osInterface;
lwip_udp_interface::LwipUdpInterface udpInterface;
udp_driver::UdpDriver udpDriver(mcuIpAddr, pcIpAddr, 7, 7, &udpInterface, &osInterface);
extern struct netif gnetif;
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

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
extern void ethernetif_input( void const * argument );

// For vApplicationGetIdleTaskMemory
#ifdef __cplusplus
extern "C" {
#endif
/* USER CODE END FunctionPrototypes */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
        StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize);

/* Hook prototypes */

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
// For vApplicationGetIdleTaskMemory
#ifdef __cplusplus
}
#endif

static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
        StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize) {
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
    osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128,
            defaultTaskBuffer, &defaultTaskControlBlock);
    defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

    /* definition and creation of UART1Task */
    osThreadStaticDef(UART1Task, StartUART1Task, osPriorityNormal, 0, 128,
            UART1TaskBuffer, &UART1TaskControlBlock);
    UART1TaskHandle = osThreadCreate(osThread(UART1Task), NULL);

    /* definition and creation of UART2Task */
    osThreadStaticDef(UART2Task, StartUART2Task, osPriorityNormal, 0, 128,
            UART2TaskBuffer, &UART2TaskControlBlock);
    UART2TaskHandle = osThreadCreate(osThread(UART2Task), NULL);

    /* definition and creation of UART3Task */
    osThreadStaticDef(UART3Task, StartUART3Task, osPriorityNormal, 0, 128,
            UART3TaskBuffer, &UART3TaskControlBlock);
    UART3TaskHandle = osThreadCreate(osThread(UART3Task), NULL);

    /* definition and creation of UART4Task */
    osThreadStaticDef(UART4Task, StartUART4Task, osPriorityNormal, 0, 128,
            UART4TaskBuffer, &UART4TaskControlBlock);
    UART4TaskHandle = osThreadCreate(osThread(UART4Task), NULL);

    /* definition and creation of UART6Task */
    osThreadStaticDef(UART6Task, StartUART6Task, osPriorityNormal, 0, 128,
            UART6TaskBuffer, &UART6TaskControlBlock);
    UART6TaskHandle = osThreadCreate(osThread(UART6Task), NULL);

    /* definition and creation of IMUTask */
    osThreadStaticDef(IMUTask, StartIMUTask, osPriorityNormal, 0, 128,
            IMUTaskBuffer, &IMUTaskControlBlock);
    IMUTaskHandle = osThreadCreate(osThread(IMUTask), NULL);

    /* definition and creation of CommandTask */
    osThreadStaticDef(CommandTask, StartCommandTask, osPriorityBelowNormal, 0,
            512, CommandTaskBuffer, &CommandTaskControlBlock);
    CommandTaskHandle = osThreadCreate(osThread(CommandTask), NULL);

    /* definition and creation of RxTask */
    osThreadStaticDef(RxTask, StartRxTask, osPriorityRealtime, 0, 512,
            RxTaskBuffer, &RxTaskControlBlock);
    RxTaskHandle = osThreadCreate(osThread(RxTask), NULL);

    /* definition and creation of TxTask */
    osThreadStaticDef(TxTask, StartTxTask, osPriorityHigh, 0, 512, TxTaskBuffer,
            &TxTaskControlBlock);
    TxTaskHandle = osThreadCreate(osThread(TxTask), NULL);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

    /* Create the queue(s) */
    /* definition and creation of UART1_req */
    osMessageQStaticDef(UART1_req, 16, UARTcmd_t, UART1_reqBuffer,
            &UART1_reqControlBlock);
    UART1_reqHandle = osMessageCreate(osMessageQ(UART1_req), NULL);

    /* definition and creation of UART2_req */
    osMessageQStaticDef(UART2_req, 16, UARTcmd_t, UART2_reqBuffer,
            &UART2_reqControlBlock);
    UART2_reqHandle = osMessageCreate(osMessageQ(UART2_req), NULL);

    /* definition and creation of UART3_req */
    osMessageQStaticDef(UART3_req, 16, UARTcmd_t, UART3_reqBuffer,
            &UART3_reqControlBlock);
    UART3_reqHandle = osMessageCreate(osMessageQ(UART3_req), NULL);

    /* definition and creation of UART4_req */
    osMessageQStaticDef(UART4_req, 16, UARTcmd_t, UART4_reqBuffer,
            &UART4_reqControlBlock);
    UART4_reqHandle = osMessageCreate(osMessageQ(UART4_req), NULL);

    /* definition and creation of UART6_req */
    osMessageQStaticDef(UART6_req, 16, UARTcmd_t, UART6_reqBuffer,
            &UART6_reqControlBlock);
    UART6_reqHandle = osMessageCreate(osMessageQ(UART6_req), NULL);

    /* definition and creation of TXQueue */
    osMessageQStaticDef(TXQueue, 32, TXData_t, TXQueueBuffer,
            &TXQueueControlBlock);
    TXQueueHandle = osMessageCreate(osMessageQ(TXQueue), NULL);

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument) {

    /* USER CODE BEGIN StartDefaultTask */
    /* Infinite loop */
    for (;;) {
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
 *         responsibility for distributing commands to the actuators
 *
 *         This function never returns.
 *
 * @ingroup Threads
 */
void StartCommandTask(void const * argument) {
//    Dynamixel_SetIOType(IO_POLL); // Configure IO
//
//    Dynamixel_Init(&Motor12, 12, &huart6, GPIOC, GPIO_PIN_8, MX28TYPE);
//    Dynamixel_Init(&Motor11, 11, &huart6, GPIOC, GPIO_PIN_8, MX28TYPE);
//    Dynamixel_Init(&Motor10, 10, &huart6, GPIOC, GPIO_PIN_8, MX28TYPE);
//    Dynamixel_Init(&Motor9, 9, &huart1, GPIOA, GPIO_PIN_8, MX28TYPE);
//    Dynamixel_Init(&Motor8, 8, &huart1, GPIOA, GPIO_PIN_8, MX28TYPE);
//    Dynamixel_Init(&Motor7, 7, &huart1, GPIOA, GPIO_PIN_8, MX28TYPE);
//    Dynamixel_Init(&Motor6, 6, &huart4, GPIOC, GPIO_PIN_3, MX28TYPE);
//    Dynamixel_Init(&Motor5, 5, &huart4, GPIOC, GPIO_PIN_3, MX28TYPE);
//    Dynamixel_Init(&Motor4, 4, &huart4, GPIOC, GPIO_PIN_3, MX28TYPE);
//    Dynamixel_Init(&Motor3, 3, &huart2, GPIOA, GPIO_PIN_4, MX28TYPE);
//    Dynamixel_Init(&Motor2, 2, &huart2, GPIOA, GPIO_PIN_4, MX28TYPE);
//    Dynamixel_Init(&Motor1, 1, &huart2, GPIOA, GPIO_PIN_4, MX28TYPE);
//    Dynamixel_Init(&Motor13, 13, &huart3, GPIOB, GPIO_PIN_2, AX12ATYPE);
//    Dynamixel_Init(&Motor14, 14, &huart3, GPIOB, GPIO_PIN_2, AX12ATYPE);
//    Dynamixel_Init(&Motor15, 15, &huart3, GPIOB, GPIO_PIN_2, AX12ATYPE);
//    Dynamixel_Init(&Motor16, 16, &huart3, GPIOB, GPIO_PIN_2, AX12ATYPE);
//    Dynamixel_Init(&Motor17, 17, &huart3, GPIOB, GPIO_PIN_2, AX12ATYPE);
//    Dynamixel_Init(&Motor18, 18, &huart3, GPIOB, GPIO_PIN_2, AX12ATYPE);
//
//    Dynamixel_HandleTypeDef* arrDynamixel[18] = { &Motor1, &Motor2, &Motor3,
//            &Motor4, &Motor5, &Motor6, &Motor7, &Motor8, &Motor9, &Motor10,
//            &Motor11, &Motor12, &Motor13, &Motor14, &Motor15, &Motor16,
//            &Motor17, &Motor18 };
//
//    UARTcmd_t Motorcmd[18];
//    for (uint8_t i = MOTOR1; i <= MOTOR18; i++) {
//        // Configure motor to return status packets only for read commands
//        Dynamixel_SetStatusReturnLevel(arrDynamixel[i], 1);
//
//        // Configure motor to return status packets with minimal latency
//        Dynamixel_SetReturnDelayTime(arrDynamixel[i], 100);
//
//        // Enable motor torque
//        Dynamixel_TorqueEnable(arrDynamixel[i], 1);
//
//        // Settings for torque near goal position, and acceptable error (AX12A only)
//        if (arrDynamixel[i]->_motorType == AX12ATYPE) {
//            AX12A_SetComplianceSlope(arrDynamixel[i], 5); // 4 vibrates; 7 is too loose
//            AX12A_SetComplianceMargin(arrDynamixel[i], 1);
//        }
//
//        (Motorcmd[i]).motorHandle = arrDynamixel[i];
//        (Motorcmd[i]).type = cmdWritePosition;
//    }
//
//    (Motorcmd[MOTOR1]).qHandle = UART2_reqHandle;
//    (Motorcmd[MOTOR2]).qHandle = UART2_reqHandle;
//    (Motorcmd[MOTOR3]).qHandle = UART2_reqHandle;
//    (Motorcmd[MOTOR4]).qHandle = UART4_reqHandle;
//    (Motorcmd[MOTOR5]).qHandle = UART4_reqHandle;
//    (Motorcmd[MOTOR6]).qHandle = UART4_reqHandle;
//    (Motorcmd[MOTOR7]).qHandle = UART1_reqHandle;
//    (Motorcmd[MOTOR8]).qHandle = UART1_reqHandle;
//    (Motorcmd[MOTOR9]).qHandle = UART1_reqHandle;
//    (Motorcmd[MOTOR10]).qHandle = UART6_reqHandle;
//    (Motorcmd[MOTOR11]).qHandle = UART6_reqHandle;
//    (Motorcmd[MOTOR12]).qHandle = UART6_reqHandle;
//    (Motorcmd[MOTOR13]).qHandle = UART3_reqHandle;
//    (Motorcmd[MOTOR14]).qHandle = UART3_reqHandle;
//    (Motorcmd[MOTOR15]).qHandle = UART3_reqHandle;
//    (Motorcmd[MOTOR16]).qHandle = UART3_reqHandle;
//    (Motorcmd[MOTOR17]).qHandle = UART3_reqHandle;
//    (Motorcmd[MOTOR18]).qHandle = UART3_reqHandle;
//
//    Dynamixel_SetIOType(IO_DMA); // Configure IO to use DMA
//
//    IMUdata.init(6); // 5 Hz bandwidth
//
//    // Set setupIsDone and unblock the higher-priority tasks
//    setupIsDone = true;
//    xTaskNotify(RxTaskHandle, 1UL, eNoAction);
//    xTaskNotify(TxTaskHandle, 1UL, eNoAction);
//    xTaskNotify(UART1TaskHandle, 1UL, eNoAction);
//    xTaskNotify(UART2TaskHandle, 1UL, eNoAction);
//    xTaskNotify(UART3TaskHandle, 1UL, eNoAction);
//    xTaskNotify(UART4TaskHandle, 1UL, eNoAction);
//    xTaskNotify(UART6TaskHandle, 1UL, eNoAction);
//    xTaskNotify(IMUTaskHandle, 1UL, eNoAction);
//
//    uint32_t numIterations = 0;
//    uint8_t i;
//    float positions[18];
    while (1) {
//        xTaskNotifyWait(0, NOTIFIED_FROM_TASK, NULL, portMAX_DELAY);
//
//        // Convert raw bytes from robotGoal received from PC into floats
//        for (uint8_t i = 0; i < 18; i++) {
//            uint8_t* ptr = (uint8_t*) &positions[i];
//            for (uint8_t j = 0; j < 4; j++) {
//                *ptr = robotGoal.msg[i * 4 + j];
//                ptr++;
//            }
//        }
//
//        if (numIterations % 100 == 0) {
//            // Every 100 iterations, assert torque enable
//            for (uint8_t i = MOTOR1; i <= MOTOR18; i++) {
//                Motorcmd[i].type = cmdWriteTorque;
//                Motorcmd[i].value = 1; // Enable
//                xQueueSend(Motorcmd[i].qHandle, &Motorcmd[i], 0);
//            }
//        }
//
//        // Send each goal position to the queue, where the UART handler
//        // thread that's listening will receive it and send it to the motor
//        for (i = MOTOR1; i <= MOTOR18; i++) { // NB: i begins at 0 (i.e. Motor1 corresponds to i = 0)
//            switch (i) {
//            case MOTOR1:
//                Motorcmd[i].value = positions[i] * 180 / M_PI + 150;
//                break;
//            case MOTOR2:
//                Motorcmd[i].value = positions[i] * 180 / M_PI + 150;
//                break;
//            case MOTOR3:
//                Motorcmd[i].value = positions[i] * 180 / M_PI + 150;
//                break;
//            case MOTOR4:
//                Motorcmd[i].value = -1 * positions[i] * 180 / M_PI + 150;
//                break;
//            case MOTOR5:
//                Motorcmd[i].value = -1 * positions[i] * 180 / M_PI + 150;
//                break;
//            case MOTOR6:
//                Motorcmd[i].value = -1 * positions[i] * 180 / M_PI + 150;
//                break;
//            case MOTOR7:
//                Motorcmd[i].value = -1 * positions[i] * 180 / M_PI + 150;
//                break;
//            case MOTOR8:
//                Motorcmd[i].value = -1 * positions[i] * 180 / M_PI + 150;
//                break;
//            case MOTOR9:
//                Motorcmd[i].value = positions[i] * 180 / M_PI + 150;
//                break;
//            case MOTOR10:
//                Motorcmd[i].value = -1 * positions[i] * 180 / M_PI + 150;
//                break;
//            case MOTOR11:
//                Motorcmd[i].value = -1 * positions[i] * 180 / M_PI + 150;
//                break;
//            case MOTOR12:
//                Motorcmd[i].value = positions[i] * 180 / M_PI + 150;
//                break;
//            case MOTOR13:
//                Motorcmd[i].value = positions[i] * 180 / M_PI + 150; // Left shoulder
//                break;
//            case MOTOR14:
//                Motorcmd[i].value = positions[i] * 180 / M_PI + 60; // Left elbow
//                break;
//            case MOTOR15:
//                Motorcmd[i].value = -1 * positions[i] * 180 / M_PI + 150; // Right shoulder
//                break;
//            case MOTOR16:
//                Motorcmd[i].value = -1 * positions[i] * 180 / M_PI + 240; // Right elbow
//                break;
//            case MOTOR17:
//                Motorcmd[i].value = -1 * positions[i] * 180 / M_PI + 150; // Neck pan
//                break;
//            case MOTOR18:
//                Motorcmd[i].value = -1 * positions[i] * 180 / M_PI + 150; // Neck tilt
//                break;
//            default:
//                break;
//            }
//
//            Motorcmd[i].type = cmdWritePosition;
//            xQueueSend(Motorcmd[i].qHandle, &Motorcmd[i], 0);
//
//            // Only read from legs
//            if (i <= MOTOR12) {
//                Motorcmd[i].type = cmdReadPosition;
//                xQueueSend(Motorcmd[i].qHandle, &Motorcmd[i], 0);
//            }
//        }
//
//        numIterations++;
        osDelay(1);
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
void StartUART1Task(void const * argument) {
//    // Here, we use task notifications to block this task from running until a notification
//    // is received. This allows one-time setup to complete in a low-priority task.
//    xTaskNotifyWait(UINT32_MAX, UINT32_MAX, NULL, portMAX_DELAY);
//
//    UARTcmd_t cmdMessage;
//    TXData_t dataToSend;
//    dataToSend.eDataType = eMotorData;

    for (;;) {
//        while (xQueueReceive(UART1_reqHandle, &cmdMessage, portMAX_DELAY)
//                != pdTRUE)
//            ;
//        UART_ProcessEvent(&cmdMessage, &dataToSend);

        osDelay(1);
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
void StartUART2Task(void const * argument) {
//    // Here, we use task notifications to block this task from running until a notification
//    // is received. This allows one-time setup to complete in a low-priority task.
//    xTaskNotifyWait(UINT32_MAX, UINT32_MAX, NULL, portMAX_DELAY);
//
//    UARTcmd_t cmdMessage;
//    TXData_t dataToSend;
//    dataToSend.eDataType = eMotorData;

    for (;;) {
//        while (xQueueReceive(UART2_reqHandle, &cmdMessage, portMAX_DELAY)
//                != pdTRUE)
//            ;
//        UART_ProcessEvent(&cmdMessage, &dataToSend);
        osDelay(1);
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
void StartUART3Task(void const * argument) {
//    // Here, we use task notifications to block this task from running until a notification
//    // is received. This allows one-time setup to complete in a low-priority task.
//    xTaskNotifyWait(UINT32_MAX, UINT32_MAX, NULL, portMAX_DELAY);
//
//    UARTcmd_t cmdMessage;
//    TXData_t dataToSend;
//    dataToSend.eDataType = eMotorData;

    for (;;) {
//        while (xQueueReceive(UART3_reqHandle, &cmdMessage, portMAX_DELAY)
//                != pdTRUE)
//            ;
//        UART_ProcessEvent(&cmdMessage, &dataToSend);
        osDelay(1);
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
void StartUART4Task(void const * argument) {
//    // Here, we use task notifications to block this task from running until a notification
//    // is received. This allows one-time setup to complete in a low-priority task.
//    xTaskNotifyWait(UINT32_MAX, UINT32_MAX, NULL, portMAX_DELAY);
//
//    UARTcmd_t cmdMessage;
//    TXData_t dataToSend;
//    dataToSend.eDataType = eMotorData;

    for (;;) {
//        while (xQueueReceive(UART4_reqHandle, &cmdMessage, portMAX_DELAY)
//                != pdTRUE)
//            ;
//        UART_ProcessEvent(&cmdMessage, &dataToSend);
        osDelay(1);
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
void StartUART6Task(void const * argument) {
//    // Here, we use task notifications to block this task from running until a notification
//    // is received. This allows one-time setup to complete in a low-priority task.
//    xTaskNotifyWait(UINT32_MAX, UINT32_MAX, NULL, portMAX_DELAY);
//
//    /* Infinite loop */
//    UARTcmd_t cmdMessage;
//    TXData_t dataToSend;
//    dataToSend.eDataType = eMotorData;

    for (;;) {
//        while (xQueueReceive(UART6_reqHandle, &cmdMessage, portMAX_DELAY)
//                != pdTRUE)
//            ;
//        UART_ProcessEvent(&cmdMessage, &dataToSend);
        osDelay(1);
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
void StartIMUTask(void const * argument) {
//    /* USER CODE BEGIN StartIMUTask */
//    // Here, we use task notifications to block this task from running until a notification
//    // is received. This allows one-time setup to complete in a low-priority task.
//    xTaskNotifyWait(UINT32_MAX, UINT32_MAX, NULL, portMAX_DELAY);
//
//    TXData_t dataToSend;
//    dataToSend.eDataType = eIMUData;
//
//    TickType_t xLastWakeTime;
//    xLastWakeTime = xTaskGetTickCount();
//
//    const TickType_t IMU_CYCLE_TIME_MS = 2;
//    uint8_t i = 0;
//    IMUStruct IMUStruct;
//
//    MPUFilter_InitAllFilters();

    for (;;) {
//        // Service this thread every 2 ms for a 500 Hz sample rate
//        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(IMU_CYCLE_TIME_MS));
//
//        IMUdata.Read_Accelerometer_Withoffset_IT(); // Also updates pitch and roll
//
//        // Gyroscope data is much more volatile/sensitive to changes than
//        // acceleration data. To compensate, we feed in samples to the filter
//        // slower. Good DSP practise? Not sure. To compensate for the high
//        // delays, we also use a filter with fewer taps than the acceleration
//        // filters. Ideally: we would sample faster to reduce aliasing, then
//        // use a filter with a smaller cutoff frequency. However, the filter
//        // designer we are using does not allow us to generate such filters in
//        // the free version, so this is the best we can do unless we use other
//        // software.
//        if (i % 16 == 0) {
//            IMUdata.Read_Gyroscope_Withoffset_IT();
//// TODO: convert the MPUFilter_FilterAngularVelocity function
//            //MPUFilter_FilterAngularVelocity();
//        }
//        i++;
//        IMUdata.Fill_Struct(&IMUStruct);
//        dataToSend.pData = &IMUStruct;
//        xQueueSend(TXQueueHandle, &dataToSend, 0);
        osDelay(1);
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
//    initializeVars();
//    xTaskNotifyWait(UINT32_MAX, UINT32_MAX, NULL, portMAX_DELAY);
//    initiateDMATransfer();

    pc_interface::PcInterface pcInterface(pc_interface::PcProtocol::UDP);
    pcInterface.setUdpDriver(&udpDriver);

    //uint8_t buffer[1024] = {};

    for (;;) {
//        waitForNotificationRX();
//        updateStatusToPC();
//        receiveDataBuffer();
        ethernetif_input(&gnetif);
        pcInterface.receive();
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
//    xTaskNotifyWait(UINT32_MAX, UINT32_MAX, NULL, portMAX_DELAY);
//    shiftNotificationMask();

    for (;;) {
//        copySensorDataToSend();
//        transmitStatusFromPC();
//        waitForNotificationTX();
       osDelay(1);
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
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    // This callback runs after the interrupt data transfer from the sensor to the mcu is finished
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(IMUTaskHandle, NOTIFIED_FROM_RX_ISR, eSetBits,
            &xHigherPriorityTaskWoken);
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
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
    if (setupIsDone) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (huart == &huart5) {
            xTaskNotifyFromISR(TxTaskHandle, NOTIFIED_FROM_TX_ISR, eSetBits,
                    &xHigherPriorityTaskWoken);
        }
        if (huart == &huart1) {
            xTaskNotifyFromISR(UART1TaskHandle, NOTIFIED_FROM_TX_ISR, eSetBits,
                    &xHigherPriorityTaskWoken);
        } else if (huart == &huart2) {
            xTaskNotifyFromISR(UART2TaskHandle, NOTIFIED_FROM_TX_ISR, eSetBits,
                    &xHigherPriorityTaskWoken);
        } else if (huart == &huart3) {
            xTaskNotifyFromISR(UART3TaskHandle, NOTIFIED_FROM_TX_ISR, eSetBits,
                    &xHigherPriorityTaskWoken);
        } else if (huart == &huart4) {
            xTaskNotifyFromISR(UART4TaskHandle, NOTIFIED_FROM_TX_ISR, eSetBits,
                    &xHigherPriorityTaskWoken);
        } else if (huart == &huart6) {
            xTaskNotifyFromISR(UART6TaskHandle, NOTIFIED_FROM_TX_ISR, eSetBits,
                    &xHigherPriorityTaskWoken);
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
    if (huart == &huart5) {
        xTaskNotifyFromISR(RxTaskHandle, NOTIFIED_FROM_RX_ISR, eSetBits,
                &xHigherPriorityTaskWoken);
    }
    if (huart == &huart1) {
        xTaskNotifyFromISR(UART1TaskHandle, NOTIFIED_FROM_RX_ISR, eSetBits,
                &xHigherPriorityTaskWoken);
    } else if (huart == &huart2) {
        xTaskNotifyFromISR(UART2TaskHandle, NOTIFIED_FROM_RX_ISR, eSetBits,
                &xHigherPriorityTaskWoken);
    } else if (huart == &huart3) {
        xTaskNotifyFromISR(UART3TaskHandle, NOTIFIED_FROM_RX_ISR, eSetBits,
                &xHigherPriorityTaskWoken);
    } else if (huart == &huart4) {
        xTaskNotifyFromISR(UART4TaskHandle, NOTIFIED_FROM_RX_ISR, eSetBits,
                &xHigherPriorityTaskWoken);
    } else if (huart == &huart6) {
        xTaskNotifyFromISR(UART6TaskHandle, NOTIFIED_FROM_RX_ISR, eSetBits,
                &xHigherPriorityTaskWoken);
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
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    error = HAL_UART_GetError(huart);
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
