This page builds on top of the [previous tutorial](https://github.com/utra-robosoccer/soccer-embedded/wiki/STM32:-Communication-Between-MCU-and-PC) by extending the program to run on top of FreeRTOS (free real-time operating system).

# Overview: Why an OS?
An operating system (OS) is a collection of algorithms which manage the microcontroller's resources—for example, scheduling the execution of _tasks_ ("**threads**") on the CPU. You are probably already familiar with the idea of tasks: reading from a sensor, executing a control algorithm, listening for commands, streaming data, generating control signals, etc. For simple systems, a state machine inside a super-loop is sometimes sufficient. However, this type of program does not scale well: adding in new tasks is laborious, and it can become difficult to satisfy timing requirements.

An RTOS is one solution which makes this situation easier to manage. RTOSes provide a means by which different tasks can be created, each with their own purpose, execution context, timing requirements, etc. Perhaps the most fundamental service provided by an RTOS is a _scheduler_ for the tasks. This is the algorithm which decides which task gets to run on the CPU, and it is hooked onto a timer interrupt, causing it to run at a fixed, deterministic frequency (e.g. 1000 times per second). When a task is scheduled, it gets full access to the CPU, memory, etc. If it finishes what it needs to do before the timer interrupt is triggered, it can tell the scheduler it is done early by _yielding_, thereby allowing a different task to run for the remainder of the period. Alternatively, if it does not yield within its execution period, then it may be forcibly stopped by the scheduler when the timer interrupt is generated. This would depend on several factors, such as whether there were any higher-priority tasks ready to run, and the scheduling algorithm used. By default, FreeRTOS uses a priority-based preemptive scheduler, which means the highest-priority task that is ready to run is the one that will be scheduled.

RTOSes also come with their own sets of challenges. For example, setting task priorities incorrectly may result in an unresponsive system or task starvation, and correct synchronization and communication between tasks is nontrivial.

# FreeRTOS Configuration (Cube)
In the Configuration tab in Cube, place a check in the box under "FREERTOS" to enable it. This will create a new clickable box for FreeRTOS in the Middlewares panel in the middle of the window.

![FreeRTOS Panel](https://raw.githubusercontent.com/utra-robosoccer/soccer-embedded/master/Diagrams/wiki/stm32/Getting-Started-with-FreeRTOS/1-FreeRTOS-enable.jpg)

Upon opening this window we are greeted with the main configuration parameters for the operating system. A few worth pointing out are:
- **USE_PREEMPTION**: if disabled, tasks will run until they block or yield (cooperative scheduling). If enabled, the scheduler will run the highest-priority task that is in the ready state (not blocked). We will use *preemptive*
- **TICK_RATE_HZ**: the number of times per second the system traps into the OS to run the scheduler, service software timers, etc.
- **Memory Allocation**: determines whether your system uses the heap, and consequently which FreeRTOS APIs are compiled (static, dynamic, or both). We will use *static*

**Note**: Many embedded systems strictly use static allocation schemes, and we will do the same. The main reason is that dynamic allocation is an non-deterministic action, i.e. it can fail. It is difficult to handle such failures gracefully, and even worse, such failures many not appear for hours, weeks, or even years if they are related to heap fragmentation.

![Config Parameters](https://raw.githubusercontent.com/utra-robosoccer/soccer-embedded/master/Diagrams/wiki/stm32/Getting-Started-with-FreeRTOS/2-FreeRTOS-config.jpg)

The next tab, Include parameters, allows us to enable/disable certain FreeRTOS functions (another compile-time thing). One that has good utility is `vTaskDelayUntil`. **Let us enable it**.

![vTaskDelayUntil](https://raw.githubusercontent.com/utra-robosoccer/soccer-embedded/master/Diagrams/wiki/stm32/Getting-Started-with-FreeRTOS/3-FreeRTOS-includes1.jpg)

Heading over to the Tasks and Queues tab, we can see that we're able to create new tasks to run on the RTOS directly through Cube. This can also be done in code, but for now we'll stick with Cube. Some main task parameters are:
- Task Name: the identifier for the task. Useful when referring to tasks in design documents
- Priority: how important the task is relative to others. This is a _very_ important parameter as it directly impacts when the task will get run
- Stack Size: the amount of memory to allocate for this task. Local variables and function calls will use the stack, so make sure this is sized appropriately for the application. Failing to do so will result in a stack overflow which is a critical system fault

![Task creation parameters](https://raw.githubusercontent.com/utra-robosoccer/soccer-embedded/master/Diagrams/wiki/stm32/Getting-Started-with-FreeRTOS/4-FreeRTOS-tasks.jpg)

Let us create 3 tasks as follows:
- LED: Will service the LED state
- Rx: Will receive commands from a PC
- Tx: Will send data to a PC

![The 3 tasks](https://raw.githubusercontent.com/utra-robosoccer/soccer-embedded/master/Diagrams/wiki/stm32/Getting-Started-with-FreeRTOS/5-Tasks.jpg)

As the last step before generating code, we need to change the HAL timebase source to be something other than SysTick. We will choose TIM1 for convenience, then generate code.

![Timebase](https://raw.githubusercontent.com/utra-robosoccer/soccer-embedded/master/Diagrams/wiki/stm32/Getting-Started-with-FreeRTOS/6-Timebase-source.jpg)

# Which Files Do I Write My Code in Now?
When we open up `main.c` now, we will see a call to `MX_FREERTOS_Init` and `osKernelStart`. After calling this latter function, things completely change as all code execution is now managed by the operating system. This means that the while loop in main.c will never be executed.

```C
int main(void)
{
...
  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
  }
}
```

Thus, we can head over to `freertos.c` where all the action is. The first thing to note is the threads (`osStaticThreadDef_t`) and buffers created for all the tasks (the buffers form the address space for the threads).

```C
/* USER CODE END Variables */
osThreadId LedHandle;
uint32_t LedBuffer[ 128 ];
osStaticThreadDef_t LedControlBlock;
osThreadId RxHandle;
uint32_t RxBuffer[ 128 ];
osStaticThreadDef_t RxControlBlock;
osThreadId TxHandle;
uint32_t TxBuffer[ 128 ];
osStaticThreadDef_t TxControlBlock;
```

Next we have `MX_FREERTOS_Init` which initializes the thread data structures with their entry functions, priorities, address spaces, stack size, etc. This is exactly what we configured in Cube.

```C
void MX_FREERTOS_Init(void) {
...
  /* Create the thread(s) */
  /* definition and creation of Led */
  osThreadStaticDef(Led, StartLedTask, osPriorityNormal, 0, 128, LedBuffer, &LedControlBlock);
  LedHandle = osThreadCreate(osThread(Led), NULL);

  /* definition and creation of Rx */
  osThreadStaticDef(Rx, StartRxTask, osPriorityRealtime, 0, 128, RxBuffer, &RxControlBlock);
  RxHandle = osThreadCreate(osThread(Rx), NULL);

  /* definition and creation of Tx */
  osThreadStaticDef(Tx, StartTxTask, osPriorityHigh, 0, 128, TxBuffer, &TxControlBlock);
  TxHandle = osThreadCreate(osThread(Tx), NULL);
...
}
```

Finally, we have the auto-generated `StartLedTask` function and the following user code section. For now, let's create empty functions for the Tx and Rx threads like so:

```C
void StartLedTask(void const * argument)
{

  /* USER CODE BEGIN StartLedTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartLedTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void StartRxTask(void const * argument){

}

void StartTxTask(void const * argument){

}
/* USER CODE END Application */
```

For the purposes of this tutorial, we will do all the coding inside these functions in `freertos.c`. However, real projects should use several files for better organization (one of the reasons we chose the "external" task code generation option in Cube is so that we could easily implement the threads in other files).

# Quick Note on APIs
Cortex Microcontroller Software Interface Standard (CMSIS) is a set of standardized APIs for various types of embedded software. CMSIS-RTOS is the set of RTOS APIs, and one of the files Cube provides is the FreeRTOS implementation of these APIs. These can be accessed transparently through `cmsis_os.h`. It is suggested that these standardized CMSIS-RTOS APIs are used to improve code portability. For details, please see http://arm-software.github.io/CMSIS_5/RTOS2/html/genRTOS2IF.html

# PC Communication
Let us first include the usart header file.

```C
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "usart.h"
/* USER CODE END Includes */
```

## Tx
We will begin by implementing the TX thread as it runs independently of the others. We wish for this thread to execute at a fixed frequency, namely 2 Hz, during which time the message "Hello world!" will be sent. We will use the `osDelayUntil` function ([FreeRTOS docs](https://www.freertos.org/vtaskdelayuntil.html)) to achieve this. Looking at the code below, we can see that after calling the DMA transmit function, the Tx thread will call `osDelayUntil`. This _delay_ function is not the same delay we usually think of (where the CPU wastes cycles comparing a counter to some value). Instead, _this_ delay causes the Tx thread to _yield_ ("block" itself) until the desired time of 500 ms has passed. As mentioned earlier, yielding causes the scheduler to run, thereby allowing a different thread to execute. This makes a lot of sense for a multitasking system.

```C
void StartRxTask(void const * argument){
    for(;;){
        // Need to block the Rx task since it is the highest priority. If we do
        // not do this, the Rx task will execute forever and nothing else will
        osDelay(1);
    }
}

void StartTxTask(void const * argument){
    const char msg[] = "Hello world!";
    const uint32_t TX_CYCLE_TIME = osKernelSysTickMicroSec(500000);
    uint32_t xLastWakeTime = osKernelSysTick();

    for(;;){
        // Wait for the next cycle
        osDelayUntil(&xLastWakeTime, TX_CYCLE_TIME);

        if(HAL_UART_Transmit_DMA(&huart2, (uint8_t*)msg, sizeof(msg)) != HAL_OK){
            HAL_UART_AbortTransmit_IT(&huart2);
            HAL_UART_Transmit_DMA(&huart2, (uint8_t*)msg, sizeof(msg));
        }
    }
}
```

**Note**: `osKernelSysTickMicroSec` is a macro which converts time in microseconds to OS ticks, which is what `osDelayUntil` takes in. This is good to use because it means the system will behave the same if for some reason the tick rate is changed

**Note**: we call `HAL_UART_AbortTransmit` if the `HAL_UART_Transmit_DMA` call fails to recover from whatever erroneous state the hardware entered. This abort function will clear error flags in the hardware, and allow things to continue smoothly as before. We will see something similar on the receiver side

## Rx
We recall from the previous tutorial that circular RX DMA is a solution to the communication problem that keeps us continuously subscribed to the transmitter sending us information. As we saw with TX above, since we set the RX thread to have the highest priority, we must periodically block it so other threads can run. During the time it is blocked, bytes will be written into the buffer by the DMA hardware, so we need to be careful we don't block for too long otherwise the buffer contents could start being overwritten.

Given our baud rate, our buffer size needs to be at least BAUD_RATE/(10*1000) bytes so that nothing gets overwritten if we check every millisecond. Using this formula, we can see that for a baud rate of 230400 and checks occurring every ms, a buffer of 32 bytes would be sufficient.

This idea is shown in code below; for now we take `copyCircBuffToFlatBuff` and `parse` as givens. The idea here is that we asynchronously read bytes from the UART line into the raw buffer using DMA, then periodically copy the contents to a _processing buffer_, which is then parsed to extract command information to distribute to the rest of the system. One reason for using 2 buffers is that the DMA could potentially start overwriting the contents of the buffer while we were processing it if we only used one.

```C
typedef struct{
    const uint8_t size;
    uint8_t iHead;
    uint8_t iTail;
    uint8_t* pBuff;
}CircBuff_t;

#define RX_BUFF_SIZE 32
void StartRxTask(void const * argument){
    const uint32_t RX_CYCLE_TIME = osKernelSysTickMicroSec(1000);
    uint32_t xLastWakeTime = osKernelSysTick();
    uint8_t raw[RX_BUFF_SIZE];
    uint8_t processingBuff[RX_BUFF_SIZE];
    CircBuff_t circBuff = {RX_BUFF_SIZE, 0, 0, raw};

    HAL_UART_Receive_DMA(&huart2, circBuff.pBuff, circBuff.size);
    for(;;){
        // Need to block the Rx task since it is the highest priority. If we do
        // not do this, the Rx task will execute forever and nothing else will
        osDelayUntil(&xLastWakeTime, RX_CYCLE_TIME);

        circBuff.iHead = circBuff.size - huart2.hdmarx->Instance->CNDTR;
        if(circBuff.iHead != circBuff.iTail){
            // Copy contents from raw circular buffer to non-circular
            // processing buffer (whose contents start at index 0)
            uint8_t numBytesReceived = copyCircBuffToFlatBuff(
                &circBuff,
                processingBuff
            );

            parse(processingBuff, numBytesReceived);
        }

        if(huart2.RxState == HAL_UART_STATE_ERROR){
            HAL_UART_AbortReceive_IT(&huart2);
            HAL_UART_Receive_DMA(&huart2, circBuff.pBuff, circBuff.size);
        }
    }
}
```

Now we understand what this Rx task is doing at a high level, so we can take a look at the `copyCircBuffToFlatBuff` and `parse` functions. As a quick recap, `copyCircBuffToFlatBuff` reads bytes out of a circular buffer and places them into a linear array, starting at element 0. It then returns the number of elements copied/received.

```C
uint8_t copyCircBuffToFlatBuff(
    CircBuff_t* src,
    uint8_t* dest
)
{
    const uint8_t BUFF_SIZE = src->size;
    const uint8_t* BUFF_PTR = src->pBuff;
    const uint8_t HEAD_IDX = src->iHead;
    uint8_t tailIdx = src->iTail;
    uint8_t* destPtr = dest;

    uint8_t numReceived;
    if(HEAD_IDX > tailIdx){
        numReceived = HEAD_IDX - tailIdx;
    }
    else{
        numReceived = BUFF_SIZE - tailIdx;
        numReceived += HEAD_IDX;
    }

    while(tailIdx != HEAD_IDX){
        *destPtr = BUFF_PTR[tailIdx];

        ++destPtr;
        ++tailIdx;

        if(tailIdx == BUFF_SIZE){
            tailIdx = 0;
        }
    }

    src->iTail = tailIdx;
    return numReceived;
}
```

The parse function interprets the raw bytes according to some communication protocol (which we will invent)—to state the obvious, the bytes could mean anything without this. In order to be able to use this program with a serial monitor, we will keep the communication protocol very high-level: simply sending "ON" or "OFF" to set the LED state. In a real protocol, we might want to use more efficient command codes, provide checks for data integrity, and add special words to delimit the start and end of a command frame.

The high-level idea for the `parse` function's implementation is to use a state machine to process the strings we receive. We have three different symbols that are permitted: O, N, F. The state machine can be used to process the received _sequence_ of symbols to see if there exists a valid string ("ON", or "OFF"). If the state machine enters a state indicating a valid string was received, we can respond to it accordingly. Getting to the programming specifics, we can use a static variable inside the `parse` function, as this will cause the variable `state` to retain its value between function calls. This is what we want, since we may only receive part of the command during one cycle of the Rx thread. **Note**: we have to be careful that _only_ the Rx thread uses this `parse` function, since functions using static variables are not thread-safe (an alternative is to have the caller manage the state variable and modify the `parse` function to take it as an argument).

```C
void parse(uint8_t* bytes, uint8_t numBytes){
    static enum {RST, O, ON, OFF_1, OFF_2} state;
    for(uint8_t i = 0; i < numBytes; ++i){
        switch(bytes[i]){
            case 'O':
                state = O;
                break;
            case 'N':
                if(state == O){
                    state = ON;
                }
                else{
                    state = RST;
                }
                break;
            case 'F':
                if(state == O){
                    state = OFF_1;
                }
                else if(state == OFF_1){
                    state = OFF_2;
                }
                else{
                    state = RST;
                }
                break;
            default:
                state = RST;
                break;
        }

        switch(state){
            case ON:
                // Send command to LED task to turn it ON
                break;
            case OFF_2:
                // Send command to LED task to turn it OFF
                break;
            default:
                break;
        }
    }
}
```

We have now figured out how to reliably receive data and process it to extract commands. The last step on our journey is figuring out how to send data safely between tasks, in our case, to change the state of the LED.

# Sharing Data Between Threads
Anytime shared data is accessed from multiple execution contexts (such as in a multithreaded program), there is the potential for a race condition. This often happens due to the inability of a CPU to make certain actions atomic—for example, i++ in C typically translates to 3 assembly instructions (read, modify, write). We can often solve these race conditions by enforcing _mutual exclusion_ for critical sections of code. By mutual exclusion, we mean only 1 thread can run that code at any given time.

There is much to be said about inter-task communication, but suffice to say that we will use mechanisms provided by the OS to transfer data between threads. For this tutorial, we will use OS-managed queues to send messages between tasks (in our case, sending a 1 or a 0 to the LED task). The OS is able to guarantee that actions with the queues are mutually exclusive, which makes them perfect for intertask communication. We can create queues through Cube from the FreeRTOS Tasks and Queues tab. Let us create a new queue then generate the code.

![Queue creation in Cube](https://raw.githubusercontent.com/utra-robosoccer/soccer-embedded/master/Diagrams/wiki/stm32/Getting-Started-with-FreeRTOS/7-cmdQ.jpg)

CMSIS provides the `osMessagePut` API to enqueue data, and the `osMessageGet` API to dequeue data. Note that for both of these, the last parameter is the number of milliseconds to wait on the action (sending or receiving). We can send the 1 or 0 signal into the queue by modifying the last switch statement in the `parse` function as follows:

```C
switch(state){
    case ON:
        // Send command to LED task to turn it ON
        osMessagePut(cmdQHandle, 1, 0);
        break;
    case OFF_2:
        // Send command to LED task to turn it OFF
        osMessagePut(cmdQHandle, 0, 0);
        break;
    default:
        break;
}
```

Heading over to the LED task now, we simply have to make it block on the queue until data is received, and then set the LED state according to the value of the data:

```C
void StartLedTask(void const * argument)
{

  /* USER CODE BEGIN StartLedTask */
    osEvent e;
    for(;;)
    {
        do{
            e = osMessageGet(cmdQHandle, osWaitForever);
        }while(e.status != osEventMessage);

        HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, (GPIO_PinState)e.value.v);
    }
  /* USER CODE END StartLedTask */
}
```

# Concluding Remarks
We have now seen how to create a microcontroller program on top of FreeRTOS which predictably, robustly, and efficiently sends and receives data via a PC link and forwards received command data to an application.

<details><summary>Click here to see the final freertos.c code listing</summary>
<p>


```C
/* USER CODE BEGIN Header */
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId LedHandle;
uint32_t LedBuffer[ 128 ];
osStaticThreadDef_t LedControlBlock;
osThreadId RxHandle;
uint32_t RxBuffer[ 128 ];
osStaticThreadDef_t RxControlBlock;
osThreadId TxHandle;
uint32_t TxBuffer[ 128 ];
osStaticThreadDef_t TxControlBlock;
osMessageQId cmdQHandle;
uint8_t cmdQBuffer[ 1 * sizeof( uint32_t ) ];
osStaticMessageQDef_t cmdQControlBlock;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartLedTask(void const * argument);
extern void StartRxTask(void const * argument);
extern void StartTxTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
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

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

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
  /* definition and creation of Led */
  osThreadStaticDef(Led, StartLedTask, osPriorityNormal, 0, 128, LedBuffer, &LedControlBlock);
  LedHandle = osThreadCreate(osThread(Led), NULL);

  /* definition and creation of Rx */
  osThreadStaticDef(Rx, StartRxTask, osPriorityRealtime, 0, 128, RxBuffer, &RxControlBlock);
  RxHandle = osThreadCreate(osThread(Rx), NULL);

  /* definition and creation of Tx */
  osThreadStaticDef(Tx, StartTxTask, osPriorityHigh, 0, 128, TxBuffer, &TxControlBlock);
  TxHandle = osThreadCreate(osThread(Tx), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of cmdQ */
  osMessageQStaticDef(cmdQ, 1, uint32_t, cmdQBuffer, &cmdQControlBlock);
  cmdQHandle = osMessageCreate(osMessageQ(cmdQ), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_StartLedTask */
/**
  * @brief  Function implementing the Led thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartLedTask */
void StartLedTask(void const * argument)
{

  /* USER CODE BEGIN StartLedTask */
    osEvent e;
    for(;;)
    {
        do{
            e = osMessageGet(cmdQHandle, osWaitForever);
        }while(e.status != osEventMessage);

        HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, (GPIO_PinState)e.value.v);
    }
  /* USER CODE END StartLedTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
typedef struct{
    const uint8_t size;
    uint8_t iHead;
    uint8_t iTail;
    uint8_t* pBuff;
}CircBuff_t;

uint8_t copyCircBuffToFlatBuff(
    CircBuff_t* src,
    uint8_t* dest
)
{
    const uint8_t BUFF_SIZE = src->size;
    const uint8_t* BUFF_PTR = src->pBuff;
    const uint8_t HEAD_IDX = src->iHead;
    uint8_t tailIdx = src->iTail;
    uint8_t* destPtr = dest;

    uint8_t numReceived;
    if(HEAD_IDX > tailIdx){
        numReceived = HEAD_IDX - tailIdx;
    }
    else{
        numReceived = BUFF_SIZE - tailIdx;
        numReceived += HEAD_IDX;
    }

    while(tailIdx != HEAD_IDX){
        *destPtr = BUFF_PTR[tailIdx];

        ++destPtr;
        ++tailIdx;

        if(tailIdx == BUFF_SIZE){
            tailIdx = 0;
        }
    }

    src->iTail = tailIdx;
    return numReceived;
}

void parse(uint8_t* bytes, uint8_t numBytes){
    static enum {RST, O, ON, OFF_1, OFF_2} state;
    for(uint8_t i = 0; i < numBytes; ++i){
        switch(bytes[i]){
            case 'O':
                state = O;
                break;
            case 'N':
                if(state == O){
                    state = ON;
                }
                else{
                    state = RST;
                }
                break;
            case 'F':
                if(state == O){
                    state = OFF_1;
                }
                else if(state == OFF_1){
                    state = OFF_2;
                }
                else{
                    state = RST;
                }
                break;
            default:
                state = RST;
                break;
        }

        switch(state){
            case ON:
                // Send command to LED task to turn it ON
                osMessagePut(cmdQHandle, 1, 0);
                break;
            case OFF_2:
                // Send command to LED task to turn it OFF
                osMessagePut(cmdQHandle, 0, 0);
                break;
            default:
                break;
        }
    }
}

#define RX_BUFF_SIZE 32
void StartRxTask(void const * argument){
    const uint32_t RX_CYCLE_TIME = osKernelSysTickMicroSec(1000);
    uint32_t xLastWakeTime = osKernelSysTick();
    uint8_t raw[RX_BUFF_SIZE];
    uint8_t processingBuff[RX_BUFF_SIZE];
    CircBuff_t circBuff = {RX_BUFF_SIZE, 0, 0, raw};

    HAL_UART_Receive_DMA(&huart2, circBuff.pBuff, circBuff.size);
    for(;;){
        // Need to block the Rx task since it is the highest priority. If we do
        // not do this, the Rx task will execute forever and nothing else will
        osDelayUntil(&xLastWakeTime, RX_CYCLE_TIME);

        circBuff.iHead = circBuff.size - huart2.hdmarx->Instance->CNDTR;
        if(circBuff.iHead != circBuff.iTail){
            // Copy contents from raw circular buffer to non-circular
            // processing buffer (whose contents start at index 0)
            uint8_t numBytesReceived = copyCircBuffToFlatBuff(
                &circBuff,
                processingBuff
            );

            parse(processingBuff, numBytesReceived);
        }

        if(huart2.RxState == HAL_UART_STATE_ERROR){
            HAL_UART_AbortReceive_IT(&huart2);
            HAL_UART_Receive_DMA(&huart2, circBuff.pBuff, circBuff.size);
        }
    }
}

void StartTxTask(void const * argument){
    const char msg[] = "Hello world!";
    const uint32_t TX_CYCLE_TIME = osKernelSysTickMicroSec(500000);
    uint32_t xLastWakeTime = osKernelSysTick();

    for(;;){
        // Wait for the next cycle
        osDelayUntil(&xLastWakeTime, TX_CYCLE_TIME);

        if(HAL_UART_Transmit_DMA(&huart2, (uint8_t*)msg, sizeof(msg)) != HAL_OK){
            HAL_UART_AbortTransmit_IT(&huart2);
            HAL_UART_Transmit_DMA(&huart2, (uint8_t*)msg, sizeof(msg));
        }
    }
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
```

</p>
</details>