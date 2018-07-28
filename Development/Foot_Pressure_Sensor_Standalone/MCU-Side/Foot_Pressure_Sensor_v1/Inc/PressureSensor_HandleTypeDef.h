/*
 * PressureSensor_HandleTypeDef.h
 *
 *  Created on: Jul 23, 2018
 *      Author: Hannah
 */

#ifndef PRESSURESENSOR_HANDLETYPEDEF_H_
#define PRESSURESENSOR_HANDLETYPEDEF_H_

/********************************** Includes **********************************/
#include <stdint.h>
#include "gpio.h"
#include "main.h"

/*********************************** Types ************************************/
enum fpLocation_e{
	LFOOT_FL,
	LFOOT_FR,
	LFOOT_BL,
	LFOOT_BR,
	RFOOT_FL,
	RFOOT_FR,
	RFOOT_BL,
	RFOOT_BR
};

typedef struct{
	enum fpLocation_e		_fpLocation;			/*!< Identifies motor as AX12A, MX28, etc.			*/
	uint16_t 				_adcAverageVal;			/*!< Average of the most recent 16 ADC values 		*/
	ADC_HandleTypeDef*		_ADC_Handle;			/*!< ADC handle for sensor							*/
	//GPIO_TypeDef*     	_GPIO_Handle;			/*!< GPIO output pin for sensor						*/
	DMA_HandleTypeDef*		_DMA_Handle;			/*!< DMA handle for sensor							*/
}FootPressureSensor_HandleTypeDef;


#endif /* PRESSURESENSOR_HANDLETYPEDEF_H_ */
