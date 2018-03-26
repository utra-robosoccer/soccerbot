
#ifndef KEYPAD_H
#define	KEYPAD_H

#include "gpio.h"

extern const char keys[];

/************************ Private Function prototypes *************************/
uint8_t readKeypad(void);
int8_t keypadRoutine(void);

#endif	/* UART_PIC_H */
