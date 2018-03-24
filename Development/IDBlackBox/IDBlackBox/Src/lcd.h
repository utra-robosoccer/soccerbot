#ifndef LCD_H
#define	LCD_H

/********************************** Includes **********************************/
#include <stdio.h>
#include "gpio.h"

/*********************************** Macros ***********************************/
/* Sets cursor position to start of second line. */
#define __lcd_newline() lcdInst(0xC0);

/* Clears both LCD lines. */
#define __lcd_clear() lcdInst(0x01); HAL_Delay(15);

/* Sets cursor position to start of first line. */
#define __lcd_home() lcdInst(0x80); HAL_Delay(5);

/* D -> controls display on/off
 * C -> controls cursor on/off
 * B -> controls blinking on/off */
#define __lcd_display_control(D, C, B)  lcdInst(8 | (D << 2) | (C << 1)| B);

/*********************************** Types ************************************/
typedef struct{
	GPIO_TypeDef* pinPort;
	uint16_t pinNum;
}pin;

/****************************** Public Interfaces *****************************/
void lcdInst(char data);
void putch(char data);
void lcdNibble(char data);
void initLCD(void);

#endif	/* LCD_H */

