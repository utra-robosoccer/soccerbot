/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>

extern I2C_HandleTypeDef hi2c1;
extern ADC_HandleTypeDef hadc;

extern uint8_t usb_received;
extern uint32_t usbRxBufferCount;
extern uint32_t usbRxBufferSize;
extern uint8_t usbRxBuffer[100];

typedef struct {
  UART_HandleTypeDef *huart; // uart configuration
  DMA_HandleTypeDef *hdma_uart_tx; // uart dma configuration
  DMA_HandleTypeDef *hdma_uart_rx;
  GPIO_TypeDef *pinPort;
  uint16_t dirPinNum; // pin for setting buffer read/write direction
  //ADC_HandleTypeDef *hadc;
  uint8_t rxPacketLen; // received packet length should be smaller then max buffer size
  uint8_t rxBuffer[100]; // 100bytes is enough to store packets from Dynamixel
  bool dmaDoneReading;
  bool readRequestSent;
  bool motorServiced;
  uint16_t timeout;

  uint8_t currMotor; // for keeping track of which motor to write to
  uint8_t currReadMotor; // for keeping track of which motor to read from
  uint8_t numMotors; // how many motors connected to this uart port?
  uint8_t motorIds[10]; // support at most 10 motors
  uint8_t protocol[10];
  uint16_t currMotorPositions[10];
} MotorPort;

typedef struct {
	ADC_HandleTypeDef *hadc; // ADC configuration for pins
	uint16_t readValue;
	float v_read; //IN 8
	//float v_read;//IN 7
	float intensity; // intensity
}Voltage;

extern MotorPort* motorPorts[6];
extern MotorPort port1, port2, port3, port4, port5, port6;

extern Voltage* voltage[2];
extern Voltage adc_1,adc_2;


typedef enum
{
  BUFFER_READ = 0,
  BUFFER_WRITE
}Buffer_state;

void init_ports();
void init_motors();
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USART4_DIR_Pin GPIO_PIN_3
#define USART4_DIR_GPIO_Port GPIOC
#define USART2_DIR_Pin GPIO_PIN_4
#define USART2_DIR_GPIO_Port GPIOA
#define V_shunt_read_Pin GPIO_PIN_7
#define V_shunt_read_GPIO_Port GPIOA
#define V_bat_read_Pin GPIO_PIN_0
#define V_bat_read_GPIO_Port GPIOB
#define USART3_DIR_Pin GPIO_PIN_2
#define USART3_DIR_GPIO_Port GPIOB
#define USART6_DIR_Pin GPIO_PIN_15
#define USART6_DIR_GPIO_Port GPIOB
#define USART1_DIR_Pin GPIO_PIN_8
#define USART1_DIR_GPIO_Port GPIOA
#define GREEN_LED_Pin GPIO_PIN_15
#define GREEN_LED_GPIO_Port GPIOA
#define USART5_DIR_Pin GPIO_PIN_4
#define USART5_DIR_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
