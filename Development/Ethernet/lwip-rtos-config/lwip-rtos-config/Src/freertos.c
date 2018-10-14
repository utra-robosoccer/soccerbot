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
#include "gpio.h"
#include "ethernetif.h"
#include "lwip/netif.h"
#include "lwip/tcpip.h"
#include "lwip/udp.h"
//#include "lwip/timeouts.h"

/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId ethernetInputTaHandle;
osThreadId txTaskHandle;
osSemaphoreId recvSemHandle;

/* USER CODE BEGIN Variables */

extern struct netif gnetif;
struct udp_pcb * echo_server_pcb;

#define BUFFER_SIZE 1000
uint8_t buf[BUFFER_SIZE];

void *ARG = NULL;
struct udp_pcb *PCB = NULL;
struct pbuf *P = NULL;
ip_addr_t ADDR = {0};
u16_t PORT = 0;

int cmd_len = 20;

//ip_addr_t gateway;

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void StartEthernetInputTask(void const * argument);
void StartTxTask(void const * argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

void handleReceive (void *arg, struct udp_pcb *pcb, struct pbuf *p,
	    const ip_addr_t *addr, u16_t port);

/* USER CODE END FunctionPrototypes */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* Hook prototypes */

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

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of recvSem */
  osSemaphoreDef(recvSem);
  recvSemHandle = osSemaphoreCreate(osSemaphore(recvSem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 640);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of ethernetInputTa */
  osThreadDef(ethernetInputTa, StartEthernetInputTask, osPriorityNormal, 0, 640);
  ethernetInputTaHandle = osThreadCreate(osThread(ethernetInputTa), NULL);

  /* definition and creation of txTask */
  osThreadDef(txTask, StartTxTask, osPriorityNormal, 0, 512);
  txTaskHandle = osThreadCreate(osThread(txTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();

  /* USER CODE BEGIN StartDefaultTask */

  /* Initialize webserver demo */
  //http_server_netconn_init();

  /* Notify user about the network interface config */
  //User_notification(&gnetif);

  err_t err;

  echo_server_pcb = udp_new();

  if (echo_server_pcb) {
    //gateway.addr = (192 << 24) + (168 << 16) + 2;
    err = udp_bind(echo_server_pcb, IP_ADDR_ANY, 7);

    if (err == ERR_OK) {
      udp_recv(echo_server_pcb, handleReceive, 0);
    }
    else {
      udp_remove(echo_server_pcb);
    }
  }
  osSemaphoreWait(recvSemHandle, osWaitForever);
  xTaskNotify(ethernetInputTaHandle, 1UL, eNoAction);


  /* Infinite loop */
  for(;;)
  {
	osSemaphoreWait(recvSemHandle, osWaitForever);
	/* Connect to the remote client */

	pbuf_copy_partial(P, buf + 7, P->len, 0);

	xTaskNotify(txTaskHandle, 1UL, eNoAction);
	xTaskNotifyWait(UINT32_MAX, UINT32_MAX, NULL, portMAX_DELAY);
    osDelay(1);
    //sys_check_timeouts();
    //osThreadTerminate(NULL);
  }
  /* USER CODE END StartDefaultTask */
}

/* StartEthernetInputTask function */
void StartEthernetInputTask(void const * argument)
{
  /* USER CODE BEGIN StartEthernetInputTask */
  /* Infinite loop */
  for(;;)
  {
	  ethernetif_input(&gnetif);
  }
  /* USER CODE END StartEthernetInputTask */
}

/* StartTxTask function */
void StartTxTask(void const * argument)
{
  /* USER CODE BEGIN StartTxTask */
  /* Infinite loop */
  for(;;)
  {
	  xTaskNotifyWait(UINT32_MAX, UINT32_MAX, NULL, portMAX_DELAY);

	  struct pbuf *pret = pbuf_alloc(PBUF_TRANSPORT, cmd_len, PBUF_RAM);

	  buf[0] = '*';
	  buf[1] = '_';
	  buf[2] = 'M';
	  buf[3] = 'C';
	  buf[4] = 'U';
	  buf[5] = '_';
	  buf[6] = '*';

	  pbuf_take(pret, buf, cmd_len);

	udp_connect(echo_server_pcb, &ADDR, PORT);
	  /* Tell the client that we have accepted it */
	  udp_send(echo_server_pcb, pret);


	  /* free the UDP connection, so we can accept new clients */
	  udp_disconnect(echo_server_pcb);

	  /* Free the p buffer */
	  pbuf_free(pret);
	  pbuf_free(P);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	xTaskNotify(defaultTaskHandle, 1UL, eNoAction);

    osDelay(1);
  }
  /* USER CODE END StartTxTask */
}

/* USER CODE BEGIN Application */

void handleReceive (void *arg, struct udp_pcb *pcb, struct pbuf *p,
	    const ip_addr_t *addr, u16_t port) {

	ARG=arg;
	PCB=pcb;
	P=p;
	ADDR= *addr;
	PORT=port;

	osSemaphoreRelease(recvSemHandle);
}
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
