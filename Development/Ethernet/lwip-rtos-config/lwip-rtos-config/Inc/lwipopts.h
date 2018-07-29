
/**
  ******************************************************************************
  * File Name          : lwipopts.h
  * Description        : This file overrides LwIP stack default configuration
  *                      done in opt.h file.
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
 
/* Define to prevent recursive inclusion --------------------------------------*/
#ifndef __LWIPOPTS__H__
#define __LWIPOPTS__H__

#include "stm32f7xx_hal.h"

/*-----------------------------------------------------------------------------*/
/* Current version of LwIP supported by CubeMx: 2.0.3 -*/
/*-----------------------------------------------------------------------------*/

/* Within 'USER CODE' section, code will be kept by default at each generation */
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

#ifdef __cplusplus
 extern "C" {
#endif

/* STM32CubeMX Specific Parameters (not defined in opt.h) ---------------------*/
/* Parameters set in STM32CubeMX LwIP Configuration GUI -*/
/*----- WITH_RTOS enabled (Since FREERTOS is set) -----*/
#define WITH_RTOS 1
/*----- CHECKSUM_BY_HARDWARE enabled -----*/
#define CHECKSUM_BY_HARDWARE 1
/*-----------------------------------------------------------------------------*/

/* LwIP Stack Parameters (modified compared to initialization value in opt.h) -*/
/* Parameters set in STM32CubeMX LwIP Configuration GUI -*/
/*----- Value in opt.h for LWIP_DHCP: 0 -----*/
#define LWIP_DHCP 1
/*----- Default Value for MEMP_NUM_UDP_PCB: 4 ---*/
#define MEMP_NUM_UDP_PCB 6
/*----- Default Value for MEMP_NUM_TCP_PCB: 5 ---*/
#define MEMP_NUM_TCP_PCB 10
/*----- Value in opt.h for MEM_ALIGNMENT: 1 -----*/
#define MEM_ALIGNMENT 4
/*----- Default Value for MEM_SIZE: 1600 ---*/
#define MEM_SIZE 10240
/*----- Default Value for MEMP_NUM_PBUF: 16 ---*/
#define MEMP_NUM_PBUF 10
/*----- Default Value for MEMP_NUM_TCP_PCB_LISTEN: 8 ---*/
#define MEMP_NUM_TCP_PCB_LISTEN 5
/*----- Default Value for MEMP_NUM_SYS_TIMEOUT: 5 ---*/
#define MEMP_NUM_SYS_TIMEOUT 10
/*----- Default Value for PBUF_POOL_BUFSIZE: 592 ---*/
#define PBUF_POOL_BUFSIZE 1524
/*----- Value in opt.h for LWIP_ETHERNET: LWIP_ARP || PPPOE_SUPPORT -*/
#define LWIP_ETHERNET 1
/*----- Value in opt.h for LWIP_DNS_SECURE: (LWIP_DNS_SECURE_RAND_XID | LWIP_DNS_SECURE_NO_MULTIPLE_OUTSTANDING | LWIP_DNS_SECURE_RAND_SRC_PORT) -*/
#define LWIP_DNS_SECURE 7
/*----- Default Value for TCP_WND: 2144 ---*/
#define TCP_WND 2920
/*----- Default Value for TCP_QUEUE_OOSEQ: 1 ---*/
#define TCP_QUEUE_OOSEQ 0
/*----- Default Value for TCP_MSS: 536 ---*/
#define TCP_MSS 1460
/*----- Default Value for TCP_SND_BUF: 1072 ---*/
#define TCP_SND_BUF 5840
/*----- Default Value for TCP_SND_QUEUELEN: 9 ---*/
#define TCP_SND_QUEUELEN 8
/*----- Default Value for TCP_SNDLOWAT: 1071 ---*/
#define TCP_SNDLOWAT 2921
/*----- Value in opt.h for TCP_SNDQUEUELOWAT: LWIP_MAX(TCP_SND_QUEUELEN)/2, 5) -*/
#define TCP_SNDQUEUELOWAT 5
/*----- Default Value for TCP_WND_UPDATE_THRESHOLD: 536 ---*/
#define TCP_WND_UPDATE_THRESHOLD 730
/*----- Default Value for LWIP_NETIF_LINK_CALLBACK: 0 ---*/
#define LWIP_NETIF_LINK_CALLBACK 1
/*----- Value in opt.h for TCPIP_THREAD_STACKSIZE: 0 -----*/
#define TCPIP_THREAD_STACKSIZE 1024
/*----- Value in opt.h for TCPIP_THREAD_PRIO: 1 -----*/
#define TCPIP_THREAD_PRIO 3
/*----- Value in opt.h for TCPIP_MBOX_SIZE: 0 -----*/
#define TCPIP_MBOX_SIZE 6
/*----- Value in opt.h for SLIPIF_THREAD_STACKSIZE: 0 -----*/
#define SLIPIF_THREAD_STACKSIZE 1024
/*----- Value in opt.h for SLIPIF_THREAD_PRIO: 1 -----*/
#define SLIPIF_THREAD_PRIO 3
/*----- Value in opt.h for DEFAULT_THREAD_STACKSIZE: 0 -----*/
#define DEFAULT_THREAD_STACKSIZE 1024
/*----- Value in opt.h for DEFAULT_THREAD_PRIO: 1 -----*/
#define DEFAULT_THREAD_PRIO 3
/*----- Value in opt.h for DEFAULT_UDP_RECVMBOX_SIZE: 0 -----*/
#define DEFAULT_UDP_RECVMBOX_SIZE 6
/*----- Value in opt.h for DEFAULT_TCP_RECVMBOX_SIZE: 0 -----*/
#define DEFAULT_TCP_RECVMBOX_SIZE 6
/*----- Value in opt.h for DEFAULT_ACCEPTMBOX_SIZE: 0 -----*/
#define DEFAULT_ACCEPTMBOX_SIZE 6
/*----- Value in opt.h for LWIP_SOCKET: 1 -----*/
#define LWIP_SOCKET 0
/*----- Value in opt.h for RECV_BUFSIZE_DEFAULT: INT_MAX -----*/
#define RECV_BUFSIZE_DEFAULT 2000000000
/*----- Default Value for LWIP_HTTPD: 0 ---*/
#define LWIP_HTTPD 1
/*----- Default Value for LWIP_HTTPD_CGI: ---*/
#define LWIP_HTTPD_CGI 0
/*----- Default Value for LWIP_HTTPD_CGI_SSI: ---*/
#define LWIP_HTTPD_CGI_SSI 0
/*----- Default Value for LWIP_HTTPD_SSI: ---*/
#define LWIP_HTTPD_SSI 0
/*----- Default Value for LWIP_HTTPD_SSI_RAW: ---*/
#define LWIP_HTTPD_SSI_RAW 0
/*----- Default Value for LWIP_HTTPD_SUPPORT_POST: ---*/
#define LWIP_HTTPD_SUPPORT_POST 0
/*----- Default Value for LWIP_HTTPD_MAX_CGI_PARAMETERS: ---*/
#define LWIP_HTTPD_MAX_CGI_PARAMETERS 16
/*----- Default Value for LWIP_HTTPD_SSI_MULTIPART: ---*/
#define LWIP_HTTPD_SSI_MULTIPART 0
/*----- Default Value for LWIP_HTTPD_MAX_TAG_NAME_LEN: ---*/
#define LWIP_HTTPD_MAX_TAG_NAME_LEN 8
/*----- Default Value for LWIP_HTTPD_MAX_TAG_INSERT_LEN: ---*/
#define LWIP_HTTPD_MAX_TAG_INSERT_LEN 192
/*----- Default Value for LWIP_HTTPD_POST_MANUAL_WND: ---*/
#define LWIP_HTTPD_POST_MANUAL_WND 0
/*----- Default Value for HTTPD_SERVER_AGENT: ---*/
#define HTTPD_SERVER_AGENT "lwIP/2.0.0 (http://savannah.nongnu.org/projects/lwip)"
/*----- Default Value for LWIP_HTTPD_DYNAMIC_HEADERS: ---*/
#define LWIP_HTTPD_DYNAMIC_HEADERS 0
/*----- Default Value for HTTPD_USE_MEM_POOL: ---*/
#define HTTPD_USE_MEM_POOL 0
/*----- Default Value for HTTPD_SERVER_PORT: ---*/
#define HTTPD_SERVER_PORT 80
/*----- Default Value for HTTPD_MAX_RETRIES: ---*/
#define HTTPD_MAX_RETRIES 4
/*----- Default Value for HTTPD_POLL_INTERVAL: ---*/
#define HTTPD_POLL_INTERVAL 4
/*----- Default Value for HTTPD_TCP_PRIO: ---*/
#define HTTPD_TCP_PRIO 1
/*----- Default Value for LWIP_HTTPD_TIMING: ---*/
#define LWIP_HTTPD_TIMING 0
/*----- Default Value for LWIP_HTTPD_SUPPORT_EXTSTATUS: ---*/
#define LWIP_HTTPD_SUPPORT_EXTSTATUS 0
/*----- Default Value for LWIP_HTTPD_SUPPORT_V09: ---*/
#define LWIP_HTTPD_SUPPORT_V09 1
/*----- Default Value for LWIP_HTTPD_SUPPORT_11_KEEPALIVE: ---*/
#define LWIP_HTTPD_SUPPORT_11_KEEPALIVE 0
/*----- Default Value for LWIP_HTTPD_SUPPORT_REQUESTLIST: ---*/
#define LWIP_HTTPD_SUPPORT_REQUESTLIST 1
/*----- Default Value for LWIP_HTTPD_REQ_QUEUELEN: ---*/
#define LWIP_HTTPD_REQ_QUEUELEN 5
/*----- Default Value for LWIP_HTTPD_REQ_BUFSIZE: ---*/
#define LWIP_HTTPD_REQ_BUFSIZE 1023
/*----- Default Value for LWIP_HTTPD_MAX_REQ_LENGTH: ---*/
#define LWIP_HTTPD_MAX_REQ_LENGTH 1023
/*----- Default Value for LWIP_HTTPD_MAX_REQUEST_URI_LEN: ---*/
#define LWIP_HTTPD_MAX_REQUEST_URI_LEN 63
/*----- Default Value for LWIP_HTTPD_POST_MAX_RESPONSE_URI_LEN: ---*/
#define LWIP_HTTPD_POST_MAX_RESPONSE_URI_LEN 63
/*----- Default Value for LWIP_HTTPD_SSI_INCLUDE_TAG: ---*/
#define LWIP_HTTPD_SSI_INCLUDE_TAG 1
/*----- Default Value for LWIP_HTTPD_ABORT_ON_CLOSE_MEM_ERROR: ---*/
#define LWIP_HTTPD_ABORT_ON_CLOSE_MEM_ERROR 0
/*----- Default Value for LWIP_HTTPD_KILL_OLD_ON_CONNECTIONS_EXCEEDED: ---*/
#define LWIP_HTTPD_KILL_OLD_ON_CONNECTIONS_EXCEEDED 0
/*----- Default Value for LWIP_HTTPD_OMIT_HEADER_FOR_EXTENSIONLESS_URI: ---*/
#define LWIP_HTTPD_OMIT_HEADER_FOR_EXTENSIONLESS_URI 0
/*----- Default Value for HTTPD_LIMIT_SENDING_TO_2MSS: ---*/
#define HTTPD_LIMIT_SENDING_TO_2MSS 1
/*----- Default Value for LWIP_HTTPD_CUSTOM_FILES: ---*/
#define LWIP_HTTPD_CUSTOM_FILES 0
/*----- Default Value for LWIP_HTTPD_DYNAMIC_FILE_READ: ---*/
#define LWIP_HTTPD_DYNAMIC_FILE_READ 0
/*----- Default Value for LWIP_HTTPD_FILE_STATE: ---*/
#define LWIP_HTTPD_FILE_STATE 0
/*----- Default Value for HTTPD_PRECALCULATED_CHECKSUM: ---*/
#define HTTPD_PRECALCULATED_CHECKSUM 0
/*----- Default Value for LWIP_HTTPD_FS_ASYNC_READ: ---*/
#define LWIP_HTTPD_FS_ASYNC_READ 0
/*----- Default Value for HTTPD_USE_CUSTOM_FSDATA: ---*/
#define HTTPD_USE_CUSTOM_FSDATA 1
/*----- Value in opt.h for LWIP_STATS: 1 -----*/
#define LWIP_STATS 0
/*----- Value in opt.h for CHECKSUM_GEN_IP: 1 -----*/
#define CHECKSUM_GEN_IP 0
/*----- Value in opt.h for CHECKSUM_GEN_UDP: 1 -----*/
#define CHECKSUM_GEN_UDP 0
/*----- Value in opt.h for CHECKSUM_GEN_TCP: 1 -----*/
#define CHECKSUM_GEN_TCP 0
/*----- Value in opt.h for CHECKSUM_GEN_ICMP: 1 -----*/
#define CHECKSUM_GEN_ICMP 0
/*----- Value in opt.h for CHECKSUM_GEN_ICMP6: 1 -----*/
#define CHECKSUM_GEN_ICMP6 0
/*----- Value in opt.h for CHECKSUM_CHECK_IP: 1 -----*/
#define CHECKSUM_CHECK_IP 0
/*----- Value in opt.h for CHECKSUM_CHECK_UDP: 1 -----*/
#define CHECKSUM_CHECK_UDP 0
/*----- Value in opt.h for CHECKSUM_CHECK_TCP: 1 -----*/
#define CHECKSUM_CHECK_TCP 0
/*----- Value in opt.h for CHECKSUM_CHECK_ICMP: 1 -----*/
#define CHECKSUM_CHECK_ICMP 0
/*----- Value in opt.h for CHECKSUM_CHECK_ICMP6: 1 -----*/
#define CHECKSUM_CHECK_ICMP6 0
/*-----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

#ifdef __cplusplus
}
#endif
#endif /*__LWIPOPTS__H_H */

/************************* (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
