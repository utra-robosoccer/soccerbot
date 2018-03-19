/*
 * Dynamixel.h
 *
 *  Created on: Mar 16, 2018
 *      Author: Tyler Gamvrelis
 */

/************************* Prevent recursive inclusion ************************/
#ifndef DYNAMIXEL_H_
#define DYNAMIXEL_H_

/********************************** Includes **********************************/
/* Hardware libraries */
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

/* c++ objects */
#include <stdint.h>

/*********************************** Macros ***********************************/
#define __DYNAMIXEL_TRANSMIT(port, pinNum) HAL_GPIO_WritePin(port, pinNum, GPIO_PIN_SET) // Set data direction pin high (TX)
#define __DYNAMIXEL_RECEIVE(port, pinNum) HAL_GPIO_WritePin(port, pinNum, GPIO_PIN_RESET) // Set data direction pin low (RX)


/********************************** Structs ***********************************/
struct MotorInitData{
	 int id;
	 UART_HandleTypeDef* uartHandle;
	 GPIO_TypeDef* dataDirPort;
	 uint16_t dataDirPinNum;
};

/********************************** Classes ***********************************/
// Abstract class
class Dynamixel {
	public:
		/********** Register Addresses **********/
		static uint8_t REG_GOAL_POSITION;

		/********** Properties **********/
		uint8_t					id;						/*!< Motor identification (0-252)					*/
		uint32_t				baudRate;				/*!< UART communication baud rate					*/
		uint16_t				lastPosition;			/*!< Position read from motor						*/
		float					lastVelocity;			/*!< Velocity read from motor						*/
		uint8_t					lastLoad;				/*!< Load read from motor							*/
		uint8_t					lastLoadDirection;		/*!< 1 -> CW | 0 -> CCW								*/
		float					lastVoltage;			/*!< Voltage read from motor						*/
		uint8_t					lastTemperature;		/*!< Temperature read from motor					*/
		UART_HandleTypeDef*		uartHandle;				/*!< UART handle for motor							*/
		GPIO_TypeDef*			dataDirPort;			/*!< Port data direction pin is on					*/
		uint16_t				dataDirPinNum;			/*!< Data direction pin number						*/




		/********** Methods **********/
		// Constructor and destructor
		Dynamixel(MotorInitData* motorInitData);
		virtual ~Dynamixel();


		// Initialization method
		int Init(); // TODO: While the constructor assigns values to object properties, this function should
					// perform the actual distribution of these parameters to the motors. That is, the
					// constructor should not perform any I/O with motors, while this initialization function should.
					// This ensures that objects creation will succeed, even when initialization of motors may fail


		// Low-level transmission and reception (pure virtual functions)
		virtual void dataWriter(uint8_t arrSize, uint8_t writeAddr, uint8_t param1, uint8_t param2) =0;
		virtual uint16_t dataReader(uint8_t readAddr, uint8_t readLength) =0;
		virtual void computeChecksum(uint8_t *arr, int length) =0;


		// Other low-level motor commands (seldom used)
		uint8_t ping();
		void regWrite(uint8_t arrSize, uint8_t writeAddr, uint8_t param1, uint8_t param2);
		void action();
		void reset();


		// Setters (use the WRITE DATA instruction)
		void setID(uint8_t ID); // (EEPROM)
		void setBaudRate(double baud); // (EEPROM)
		void setReturnDelayTime(double microSec); // (EEPROM)
		void setMinAngle(double minAngle); // (EEPROM) -- CW angle limit
		void setMaxAngle(double maxAngle); // (EEPROM) -- CCW angle limit
		void setTemperatureLimit(); // (EEPROM)
		void setHighestVoltageLimit(double highestVoltage); // (EEPROM)
		void setLowestVoltageLimit(double lowestVoltage); // (EEPROM)

		virtual void setStatusReturnLevel(uint8_t status_data); // (EEPROM in AX-12A, RAM in MX-28)

		void setTorqueEnable(uint8_t isEnabled); // (RAM)
		void setLEDEnable(uint8_t isEnabled); // (RAM)
		void setGoalPosition(double goalAngle); // (RAM)
		void setGoalVelocity(double goalVelocity); // (RAM)


		// Getters (use READ DATA instruction)
		void getPosition();
		void getVelocity();
		void getLoad();
		void getVoltage();
		void getTemperature();
		uint8_t isRegistered();
		bool isMoving();
};

#endif /* DYNAMIXEL_H_ */
