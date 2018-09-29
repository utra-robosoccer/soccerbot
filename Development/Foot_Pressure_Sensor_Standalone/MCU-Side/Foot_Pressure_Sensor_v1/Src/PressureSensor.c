/*
 * PressureSensor.c
 *
 *  Created on: Jul 23, 2018
 *      Author: Hannah
 */

/************************************Includes************************************/
#include "PressureSensor.h"
#include "main.h"

/******************************* Public Variables *******************************/
uint16_t ADC_BUF[64];
uint16_t adc_conv;
/************************ Private Function Prototypes **************************/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	uint32_t avg=0;
	uint8_t len = sizeof(ADC_BUF)/sizeof(ADC_BUF[0]);
	if (hadc->Instance == ADC1) {
		for(uint8_t i=0 ;i<len;i++){
			avg = avg + ADC_BUF[i];
		}
		avg = avg >> 6;
		adc_conv = (uint16_t)avg;
	}
}

/******************************** Functions ************************************/

void FootPressureSensor_Init(FootPressureSensor_HandleTypeDef* fpSensor, uint16_t adcAverageVal, enum fpLocation_e fpLocation, ADC_HandleTypeDef* ADC_Handle, DMA_HandleTypeDef* DMA_Handle) {
    /* Initializes the motor handle.
     *
     * Arguments: hdynamixel, the motor handle to be initialized
     * 			  ID, the ID the motor has. Note that this function will not set
     * 			  	  the ID in case there are multiple actuators on the same bus
     * 			  UART_Handle, the handle to the UART that will be used to
     * 			      communicate with this motor
     * 			  DataDirPort, the pointer to the port that the data direction pin
     * 			  	  for the motor is on
     * 			  DataDirPinNum, the number corresponding to the pin that controls
     * 			      data direction (a power of two, e.g. 2^0 for pin 0, 2^15 for pin 15)
     * 			  motorType, indicates whether motor is AX12A or MX28
     *
     * Returns: none
     */

    /* Set fields in motor handle. */
    fpSensor->_fpLocation = fpLocation;	//Identifies the foot pressure location(left foot front left, etc.)
    fpSensor->_adcAverageVal = adcAverageVal;
    fpSensor->_ADC_Handle = ADC_Handle;
    fpSensor->_DMA_Handle = DMA_Handle;
}

void FootPressureSensor_Start(FootPressureSensor_HandleTypeDef* fpSensor){
	uint8_t len = sizeof(ADC_BUF)/sizeof(ADC_BUF[0]);
	HAL_ADC_Start_DMA(fpSensor->_ADC_Handle, (uint32_t*) ADC_BUF, len);
}

void FootPressureSensor_Update(FootPressureSensor_HandleTypeDef* fpSensor){
	fpSensor->_adcAverageVal = adc_conv;
}
void FootPressureSensor_Stop(FootPressureSensor_HandleTypeDef* fpSensor){
	HAL_ADC_Stop(fpSensor->_ADC_Handle);
}



