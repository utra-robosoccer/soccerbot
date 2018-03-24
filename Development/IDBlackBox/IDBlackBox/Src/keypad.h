
#ifndef KEYPAD_H
#define	KEYPAD_H

#include "gpio.h"

/************************ Private Function prototypes *************************/
uint8_t readKeyboard(void);
uint8_t keypadRoutine(void);


/*********************************** Types ************************************/
typedef struct{
	GPIO_TypeDef* pinPort;
	uint16_t pinNum;
}pin;

#endif	/* UART_PIC_H */
