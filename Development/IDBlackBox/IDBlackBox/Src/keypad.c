
#include "keypad.h"

const char keys[] = {'1', '2', '3', 'A', '4', '5', '6', 'B', '7', '8', '9', 'D'};

typedef struct{
	pin row1;
	pin row2;
	pin row3;
	pin row4;
	pin col1;
	pin col2;
	pin col3;
	pin col4;
}keyPins_t;

const keyPins_t keyPins = {
	{GPIOA, GPIO_PIN_12}, 	// D2 (output)
	{GPIOB, GPIO_PIN_0}, 	// D3 (output)
	{GPIOB, GPIO_PIN_7}, 	// D4 (output)
	{GPIOB, GPIO_PIN_6}, 	// D5 (output)
	{GPIOB, GPIO_PIN_1}, 	// D6 (input)
	{GPIOA, GPIO_PIN_8}, 	// D9 (input)
	{GPIOA, GPIO_PIN_11}, 	// D10 (input)
	{GPIOB, GPIO_PIN_5} 	// D11 (input)
};

unsigned char readKeyboard(void){
    /* Iterates through the keypad for possible presses. For each row that is
     * set high, each column is scanned for a connection (by default, these
     * columns being scanned are pulled to ground). The first key found is
     * returned.
     *
     * Arguments: none
     *
     * Returns: byte corresponding to the key press, or 0xF0 if no key press
     */

    for(uint8_t i = 0; i < 4; i++){
        if(i == 0)
        	keyPins.row4.pinPort -> ODR = 1 << keyPins.row4.pinNum; // 0b00010000
        else if(i == 1)
        	keyPins.row3.pinPort -> ODR = 1 << keyPins.row3.pinNum; // 0b00100000
        else if(i == 2)
        	keyPins.row2.pinPort -> ODR = 1 << keyPins.row2.pinNum; // 0b01000000
        else if(i == 3)
        	keyPins.row1.pinPort -> ODR = 1 << keyPins.row1.pinNum; // 0b10000000

        if(HAL_GPIO_ReadPin(keyPins.col1.pinPort, keyPins.col1.pinNum))
            return (i * 4U);
        if(HAL_GPIO_ReadPin(keyPins.col2.pinPort, keyPins.col2.pinNum))
            return (i * 4U) + 1U;
        if(HAL_GPIO_ReadPin(keyPins.col3.pinPort, keyPins.col3.pinNum))
            return (i * 4U) + 2U;
        if(HAL_GPIO_ReadPin(keyPins.col4.pinPort, keyPins.col4.pinNum))
            return (i * 4U) + 3U;
    }
    return 0xF0;
}

uint8_t keypadRoutine(void){
    /* Main keypad routine.
     *
     * Arguments: none
     *
     * Returns: -1 if no key pressed, and a nonnegative number in range [0, 15] otherwise
     */

	uint8_t isValid = 0;

	/* Poll the keyboard for a key press. */
	uint8_t dataOut = readKeyboard();

	/* Check if a key push is detected. */
	if (dataOut != 0xF0){

		/* Debounce button press. If the same key is read a second time,
		 * after 20 milliseconds and it is the same as before, it is assumed
		 * valid. */
		HAL_Delay(20);
		if (dataOut == readKeyboard()){
			isValid = 1;

			/* Hold data avaliable pin high while key is pressed. */
			while (dataOut == readKeyboard()){
			}
		}
	}

	if (isValid){
		return keys[dataOut];
	}
	return -1;
}
