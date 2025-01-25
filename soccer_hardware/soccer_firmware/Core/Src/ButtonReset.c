/*
 * ButtonReset.c
 *
 *  Created on: Jan 25, 2025
 *      Author: Jingling Hou
 */

//Buttons for simple functionalities
//Specific functions it needs to perform:
/*
 * 1. reset all motor positions
 * 2. start motor walking trajectory
 */
//Pin needed: PC13 PC14
#include "ButtonReset.h"

/*
 * @brief Getting inputs from buttons
 * @param GPIOx -> Pin Port of GPIO Pin the button is connected to
 * @param GPIO_Pin -> Pin that the button is connected to
 * @retval GPIO_Pin_SET -> Button not pressed
 * @retval GPIO_Pin_RESET -> Button pressed
 */
GPIO_PinState ReadButtonState(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	GPIO_PinState buttonState = GPIO_PIN_SET;

	if (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_RESET){
		HAL_Delay(20);
		while (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_RESET);
		HAL_Delay(20);
		buttonState = GPIO_PIN_RESET;
	}

	return buttonState;
}

void MotorTest1(){

	motor_torque_en_p2(&port6, 19, 1); //Unsure about what the val is doing in this function
	HAL_Delay(5);

	write_goal_position_p2(&port6, 19, 0x400); //rotate to 90 degree
	HAL_Delay(500);
	HAL_GPIO_TogglePin(GPIOA, GREEN_LED_Pin);
	write_goal_position_p2(&port6, 19, 0x800); //rotate to 180 degree
	HAL_Delay(500);
	HAL_GPIO_TogglePin(GPIOA, GREEN_LED_Pin);
	write_goal_position_p2(&port6, 19, 0xC00); //rotate to 270 degree
	HAL_Delay(500);
	HAL_GPIO_TogglePin(GPIOA, GREEN_LED_Pin);
	write_goal_position_p2(&port6, 19, 0x800); //rotate to 180 degree
	HAL_Delay(500);
	HAL_GPIO_TogglePin(GPIOA, GREEN_LED_Pin);
}

void MotorTest2(){
	motor_torque_en_p2(&port6, 19, 1); //Unsure about what the val is doing in this function
	HAL_Delay(5);

	uint16_t angle = 90;
	uint8_t direction = 1;
	//rotate the motor from 90 degrees to 270 degrees(CCW),
	//and than rotate back to 90 degrees (CW)
	//each time the motor will rotate 30 degrees, and should take
	//a total of 180 / 30 = 6 * 0.5 = 3s * 2 = 6s to finish one such test
	while(angle >= 90){
		write_goal_position_p2(&port6, 19, angle / 0.088);

		HAL_Delay(400);
		HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);

		if (direction && angle >= 270){
			direction = 0; //reverse dir
		}


		if(direction){
			angle += 10; //increments 30 degrees per 0.5s
		}else{
			angle -= 10;
		}

	}
}

void MotorTest3(){
	//this is for testing controlling the speed of rotation
	motor_torque_en_p2(&port6, 19, 1); //Unsure about what the val is doing in this function
	HAL_Delay(5);

	uint16_t angle = 90;
	uint8_t direction = 1;
		//rotate the motor from 90 degrees to 270 degrees(CCW),
		//and than rotate back to 90 degrees (CW)
		//each time the motor will rotate 30 degrees, and should take
		//a total of 180 / 30 = 6 * 0.5 = 3s * 2 = 6s to finish one such test
	while(angle >= 90){
		write_goal_position_p2(&port6, 19, angle / 0.088);

		HAL_Delay(20); // by decreasing the time of delaying can increase the speed`

		if (direction && angle >= 270){
			direction = 0; //reverse dir
		}


		if(direction){
			angle += 10; //increments 10 degrees per 20ms
		}else{
			angle -= 5; //decrements 5 degrees per 20ms
		}

	}

	/*
	 * 0.05 degrees per 1 ms
	 *
	 */

}

void MotorTest4(uint16_t rotv){
	motor_torque_en_p2(&port6, 19, 1); //Unsure about what the val is doing in this function
	HAL_Delay(5);

	set_Rotational_Speed(&port6, 19, rotv);

	uint16_t angle = 90;
	uint8_t direction = 1;
	//rotate the motor from 90 degrees to 270 degrees(CCW),
	//and than rotate back to 90 degrees (CW)
	//each time the motor will rotate 30 degrees, and should take
	//a total of 180 / 30 = 6 * 0.5 = 3s * 2 = 6s to finish one such test
	write_goal_position_p2(&port6, 19, 270 / 0.088);
	HAL_Delay(1000);
	write_goal_position_p2(&port6, 19, 90 / 0.088);
	HAL_Delay(1000);
	write_goal_position_p2(&port6, 19, 180 / 0.088);
	HAL_Delay(1000);

}

/*
 *
 * Join Mode, Multi-Turn mode It is a moving speed to Goal Position.
0~1023 (0X3FF) can be used, and the unit is about 0.114rpm.
If it is set to 0, it means the maximum rpm of the motor is used without controlling the speed.
If it is 1023, it is about 116.62rpm.
For example, if it is set to 300, it is about 34.2 rpm.
However, the rpm will not exceed the No Load Speed.

Wheel Mode It is a moving speed to Goal direction.
0~2047 (0X7FF) can be used, and the unit is about 0.114rpm.
If a value in the range of 0~1023 is used, it is stopped by setting to 0 while rotating to CCW direction.
If a value in the range of 1024~2047 is used, it is stopped by setting to 1024 while rotating to CW direction.
That is, the 10th bit becomes the direction bit to control the direction.
 */

void  set_Rotational_Speed(MotorPort* port, uint8_t motor_ID, uint16_t rotationalSpeed)
{
	//convert input Rot_speed in RPM into Hex
	uint16_t RotSpeedHex;
	RotSpeedHex = rotationalSpeed / 0.114;


	uint16_t dataLen = 2;
	uint8_t data[dataLen];

	data[0] = rotationalSpeed & 0xff;
	data[1] = (rotationalSpeed >> 8) & 0xff;

	//_motor_write_p2(MotorPort *p, uint8_t id, uint16_t addr, uint8_t* data, uint16_t dataLen)
	_motor_write_p2(port, motor_ID, 104, data , dataLen); //address 32 controls the speed of the motor

}

// For the MX28 I have, the protocal is 2, and ID is 19
// angle -> 1024 -> 90 degree
// angle -> 2048 -> 180 degree
// angle -> 4096 -> 360 degrees
// angle unit is 0.088
//Subtask 1:
/* Default motor angle: 2048 for now?
 * set the default motor angle as an argument as not sure for now
 */



void set_Angle(MotorPort* port, uint8_t motorID, uint16_t defaultAngle){

	//Motor enable
	motor_torque_en_p2(port, motorID, 1); //Unsure about what the val is doing in this function
	HAL_Delay(5);

	write_goal_position_p2(port, motorID, defaultAngle);

}



//Press THE BUTTON and send a package to json
//or maybe create an array containing angles and run the array

//Deadline 2024/12/16
//Get everything ready









