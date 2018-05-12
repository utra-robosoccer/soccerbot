/********************************* Includes ************************************/
#include "lcd.h"

/*********************************** Types ************************************/
typedef struct{
	GPIO_TypeDef* pinPort;
	uint16_t pinNum;
}pin;

/******************************** Variables ************************************/
const pin RS = {GPIOA, GPIO_PIN_1};
const pin E = {GPIOA, GPIO_PIN_3};
GPIO_TypeDef* const LCD_PORT = GPIOA;
const uint16_t DAT1 = GPIO_PIN_4; // least significant bit pin for data
const uint16_t DAT4 = GPIO_PIN_7; // most significant bit pin for data


/******************************** Functions ************************************/

void Delay_us(int us){
	/* Delays (empty CPU cycles) for the specified number of microseconds
	 *
	 * Arguments: us, the delay time in microseconds
	 *
	 * Returns: none
	 */

	// system clock ticks at 32 MHz, i.e. it has a 31 ns period
	// 33 * 31 ns ~= 1 us
	volatile int i = 0;
	while(i < 33 * us){
		i++;
	}
}

inline void pulseE(void){
	/* Used in lcdNibble; pulses the LCD register enable to latch data.
	 *
	 * Arguments: none
	 *
	 * Returns: none
	 */

	HAL_GPIO_WritePin(E.pinPort, E.pinNum, GPIO_PIN_SET);

	Delay_us(25); // only 1 ms in theory, but 25 selected to be safe

    HAL_GPIO_WritePin(E.pinPort, E.pinNum, GPIO_PIN_RESET);

    Delay_us(100);
}


void lcdInst(char data){
    /* Sends a command to a display control register.
     *
     * Arguments: data, the command byte for the Hitachi controller
     *
     * Returns: none
     */

    HAL_GPIO_WritePin(RS.pinPort, RS.pinNum, GPIO_PIN_RESET);
    lcdNibble(data);
    Delay_us(100);
}

void putch(char data){
    /* Sends a character to the display for printing.
     *
     * Arguments data, the character (byte) to be sent
     *
     * Returns: none
     */

	HAL_GPIO_WritePin(RS.pinPort, RS.pinNum, GPIO_PIN_SET);
    lcdNibble(data);
    Delay_us(100);
}

void lcdNibble(char data){
    /* Low-level byte-sending implementation.
     *
     * Arguments: data, the byte to be sent
     *
     * Returns: none
     */

	uint8_t tempData = (uint8_t) (data & 0xF0);
	LCD_PORT -> ODR = tempData << DAT1;

	pulseE();

    /* Send the 4 most significant bits (MSb). */
    tempData = (uint8_t) (data << 4) & 0xF0;
    LCD_PORT -> ODR = tempData << DAT1;

    pulseE();
}

void initLCD(void){
    /* Initializes the character LCD.
     *
     * Arguments: none
     *
     * Returns: none
     */

    HAL_Delay(15);
    lcdInst(0b00110011);
    lcdInst(0b00110010);
    lcdInst(0b00101000);
    lcdInst(0b00001111);
    lcdInst(0b00000110);
    lcdClear();

    /* Enforce on: display, cursor, and cursor blinking. */
    lcdControl(1, 1, 1);
}
