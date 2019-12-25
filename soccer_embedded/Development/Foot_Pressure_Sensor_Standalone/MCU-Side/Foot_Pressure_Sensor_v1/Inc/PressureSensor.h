/*
 * PressureSensor.h
 *
 *  Created on: Jul 23, 2018
 *      Author: Hannah
 */

#ifndef PRESSURESENSOR_H_
#define PRESSURESENSOR_H_


/*************************Includes************************/
#ifdef stm32f4xx_hal
	#include "stm32f4xx_hal.h"
	#include "stm32f4xx_hal_conf.h"
#endif

#ifdef stm32h7xx_hal
	#include "stm32h7xx_hal.h"
	#include "stm32h7xx_hal_conf.h"
#endif

/* I/O */
#include "gpio.h"
#include "usart.h"

/* Types */
#include <stdint.h>

/* sensor containers */
#include "PressureSensor_HandleTypeDef.h"

/*********************************** Macros ***********************************/

/******************************* Public Variables *******************************/

/***************************** Function prototypes ****************************/
void FootPressureSensor_Init(FootPressureSensor_HandleTypeDef* fpsensor, uint16_t adcAverageVal, enum fpLocation_e fpLocation, ADC_HandleTypeDef* ADCHandle, DMA_HandleTypeDef* DMA_Handle);
void FootPressureSensor_Update(FootPressureSensor_HandleTypeDef* fpSensor);
void FootPressureSensor_Start(FootPressureSensor_HandleTypeDef* fpsensor);
void FootPressureSensor_Stop(FootPressureSensor_HandleTypeDef* fpsensor);


#endif /* PRESSURESENSOR_H_ */
