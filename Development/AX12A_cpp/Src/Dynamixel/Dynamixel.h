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



/********************************** Structs ***********************************/
struct MotorInitData{
	 int id;
	 UART_HandleTypeDef* uart;
	 GPIO_TypeDef* DataDirPort;
	 uint16_t DataDirPinNum;
};

/********************************** Classes ***********************************/
class Dynamixel {
	public:
		/********** Fields **********/
		uint8_t					_ID;					/*!< Motor identification (0-252)					*/
		uint32_t				_BaudRate;				/*!< UART communication baud rate					*/
		uint16_t				_lastPosition;			/*!< Position read from motor						*/
		float					_lastVelocity;			/*!< Velocity read from motor						*/
		uint8_t					_lastLoad;				/*!< Load read from motor							*/
		uint8_t					_lastLoadDirection;		/*!< 1 -> CW | 0 -> CCW								*/
		float					_lastVoltage;			/*!< Voltage read from motor						*/
		uint8_t					_lastTemperature;		/*!< Temperature read from motor					*/
		uint8_t					_isJointMode;			/*!< 1 if motor is joint mode, 0 if wheel mode		*/
		UART_HandleTypeDef*		_UART_Handle;			/*!< UART handle for motor							*/
		GPIO_TypeDef*			_dataDirPort;			/*!< Port data direction pin is on					*/
		uint16_t				_dataDirPinNum;			/*!< Data direction pin number						*/



		/********** Methods **********/
		// Constructor and destructor
		Dynamixel(MotorInitData motorInitData);
		virtual ~Dynamixel();


		// Low-level transmission and reception
		virtual void dataWriter(uint8_t arrSize, uint8_t writeAddr, uint8_t param1, uint8_t param2);
		virtual uint16_t dataReader(uint8_t readAddr, uint8_t readLength);
		virtual void computeChecksum(uint8_t *arr, int length)


		// Other low-level motor commands (seldom used)
		uint8_t ping();
		void regWrite(uint8_t arrSize, uint8_t writeAddr, uint8_t param1, uint8_t param2);
		void action();
		void reset();


		// Setters (use the WRITE DATA instruction)
		void setID(uint8_t ID); // (EEPROM)
		void setBaudRate(double baud); // (EEPROM)
		void setReturnDelayTime(double microSec); // (EEPROM)
		void setCWAngleLimit(double minAngle); // (EEPROM)
		void setCCWAngleLimit(double maxAngle); // (EEPROM)
		void setHighestVoltageLimit(double highestVoltage); // (EEPROM)
		void setLowestVoltageLimit(double lowestVoltage); // (EEPROM)
		void setMaxTorque(double maxTorque); // (EEPROM)
		void setStatusReturnLevel(uint8_t status_data); // (EEPROM)
		void setAlarmLED(uint8_t alarm_LED_data); // (EEPROM)
		void setAlarmShutdown(uint8_t alarm_shutdown_data); // (EEPROM)
		void setTorqueEnable(uint8_t isEnabled); // (RAM)
		void setLEDEnable(uint8_t isEnabled); // (RAM)
		void setCWComplianceMargin(uint8_t CWcomplianceMargin); // (RAM)
		void setCCWComplianceMargin(uint8_t CCWcomplianceMargin); // (RAM)
		void setCWComplianceSlope(uint8_t CWcomplianceSlope); // (RAM)
		void setCCWComplianceSlope(uint8_t CCWcomplianceSlope); // (RAM)
		void setGoalPosition(double goalAngle); // (RAM)
		void setGoalVelocity(double goalVelocity); // (RAM)
		void setGoalTorque(double goalTorque); // (RAM)
		void setEEPROMLock(); // (RAM)
		void setPunch(double punch); // (RAM)


		// Getters (use READ DATA instruction)
		void getPosition();
		void getVelocity();
		void getLoad();
		void getVoltage();
		void getTemperature();
		uint8_t isRegistered();
		uint8_t isMoving();
		uint8_t isJointMode();


		// Wrappers
		void setComplianceSlope(uint8_t complianceSlope);
		void setComplianceMargin(uint8_t complianceMargin);
};

#endif /* DYNAMIXEL_H_ */
