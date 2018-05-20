/****************** Drivers for Dynamixel MX28 Servo Motor ******************/

/******************** Define to prevent recursive inclusion *******************/
#ifndef __MX28_H__
#define __MX28_H__

/********************************** Includes **********************************/
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

/* Types */
#include <stdint.h>
/*********************************** Macros ***********************************/
#define __DYNAMIXEL_TRANSMIT(port, pinNum) HAL_GPIO_WritePin(port, pinNum, 1) // Set data direction pin high (TX)
#define __DYNAMIXEL_RECEIVE(port, pinNum) HAL_GPIO_WritePin(port, pinNum, 0) // Set data direction pin low (RX)

#define UNUSEDARGUMENT 0

/* Communications. */
#define NUM_MOTORS				18		// Used to determine buffer sizes
#define BUFF_SIZE_RX			8		// Receive buffer size for UART receptions (number of bytes)
#define TX_PACKET_SIZE			9		// Maximum packet size for regular motor commands (exclusion: sync write)
#define NUM_UARTS				6		// Number of UARTs available for motor communication
extern const uint8_t TRANSMIT_TIMEOUT; 	// Timeout for blocking UART transmissions, in milliseconds
extern const uint8_t RECEIVE_TIMEOUT;	// Timeout for blocking UART receptions, in milliseconds

/* Value limit definitions. */
#define MAX_VELOCITY          114	// Maximum angular velocity (RPM)
#define MIN_VELOCITY          1		// Minimum angular velocity (RPM)
#define MAX_ANGLE             300	// Maximum angular position (joint mode)
#define MIN_ANGLE             0		// Minimum angular position (joint mode)
#define MAX_TORQUE            100	// Maximum torque (percent of maximum)
#define MIN_TORQUE            0		// Minimum torque (percent of maximum)
#define MAX_VOLTAGE           14	// Maximum operating voltage
#define MIN_VOLTAGE           6		// Minimum operating voltage
#define MAX_PUNCH             1023	// Maximum punch (proportional to minimum current)
#define MIN_PUNCH             0		// Minimum punch (proportional to minimum current)

/* Instruction set definitions. */
// The intersection of protocol version 1.0 and version 2.0:
// (Note that the AX12A is only compatible with version 1.0, while
// the MX-28 is compatible with both protocols. Also, note that
// protocol version 1.0 is a proper subset of protocol version 2.0.)
#define INST_PING				0x01	    // Gets a status packet
#define INST_READ_DATA			0x02	    // Reads data from a motor register
#define INST_WRITE_DATA			0x03	    // Writes data for immediate execution
#define INST_REG_WRITE			0x04	    // Registers an instruction to be executed at a later time
#define INST_ACTION				0x05	    // Triggers instructions registered by INST_REG_WRITE
#define INST_RESET				0x06	    // Resets the control tables of the Dynamixel actuator(s) specified
#define INST_SYNC_WRITE			0x83	    // Writes on a specified address with a specified data length on multiple devices

// Instructions that are only in protocol version 2.0
#define V2_INST_REBOOT 			0x07		// Reboots the actuators
#define V2_INST_RETURN 			0x55		// Return instruction for the instruction packet
#define V2_INST_SYNC_READ 		0x82		// Reads from a specified address with a specified data length on multiple devices
#define V2_INST_BULK_READ 		0x92		// Reads from various addresses with various data lengths on multiple devices
#define V2_INST_BULK_WRITE 		0x93		// Writes on various addresses with various data lengths on multiple devices


/* Register definitions. */
// TODO
#define MX28_REG_ID 					    		7		// Motor ID register
#define MX28_REG_BAUD_RATE							8		// Baud rate register
#define MX28_REG_RETURN_DELAY_TIME					9		// Status packet return delay time register
#define MX28_REG_PROTOCOL_VERSION					13		// Version of communication protocol to be used
//#define MX28_REG_CW_ANGLE_LIMIT		    		0x06	// Clockwise angle limit register (0x06 = low byte, 0x07 = high byte)
//#define MX28_REG_CCW_ANGLE_LIMIT		    		0x08	// Counter-clockwise angle limit register (0x08 = low byte, 0x09 = high byte)
//#define MX28_REG_HIGH_VOLTAGE_LIMIT	    		0x0C	// Maximum voltage limit register
//#define MX28_REG_LOW_VOLTAGE_LIMIT				0x0D	// Minimum voltage limit register
//#define MX28_REG_MAX_TORQUE			    		0x0E	// Maximum torque limit register (0x0E = low byte, 0x0F = high byte)
//#define MX28_REG_STATUS_RETURN_LEVEL	    		0x10	// Status packet return condition(s) register
//#define MX28_REG_ALARM_LED						0xxxx	// Alarm LED condition(s) register
//#define MX28_REG_ALARM_SHUTDOWN		    		0x12	// Alarm shutdown condition(s) register
#define MX28_REG_TORQUE_ENABLE 		    			64		// Motor power control register
#define MX28_REG_LED_ENABLE			    			65		// LED control register
//#define MX28_REG_CW_COMPLIANCE_MARGIN				0x1A	// Clockwise compliance margin register
//#define MX28_REG_CCW_COMPLIANCE_MARGIN			0x1B	// Counter-clockwise compliance margin register
//#define MX28_REG_CW_COMPLIANCE_SLOPE	    		0x1C	// Clockwise compliance slope register
//#define MX28_REG_CCW_COMPLIANCE_SLOPE    			0x1D	// Counter-clockwise compliance slope register
#define MX28_REG_GOAL_POSITION		    			116		// Goal position register (0x1E = low byte, 0x1F = high byte)
#define MX28_REG_GOAL_VELOCITY		    			104		// Goal velocity register (0x20 = low byte, 0x21 = high byte)
//#define MX28_REG_GOAL_TORQUE			    		0x22	// Goal torque register (0x22 = low byte, 0x23 = high byte)
//#define MX28_REG_LOCK_EEPROM 	 	    			0x2F	// EEPROM lock register
//#define MX28_REG_PUNCH 	 			    		0x30	// Punch (0x30 = low register, 0x31 = high register)
//#define MX28_REG_CURRENT_POSITION 	    		0x24	// Current position register (0x24 = low byte, 0x25 = high byte)
//#define MX28_REG_CURRENT_VELOCITY 	    		0x26	// Current velocity register (0x26 = low byte, 0x27 = high byte)
//#define MX28_REG_CURRENT_LOAD 		    		0x28	// Current load register (0x28 = low byte, 0x29 = high byte)
//#define MX28_REG_CURRENT_VOLTAGE 	    			0x2A	// Current voltage register
//#define MX28_REG_CURRENT_TEMPERATURE     			0x2B	// Current temperature register
//#define MX28_REG_REGISTERED 			    		0x2C	// Command execution status register
//#define MX28_REG_MOVING 				    		0x2E	// Motor motion register

/* Default register value definitions. */
#define BROADCAST_ID				 0xFE	    // Motor broadcast ID (i.e. messages sent to this ID will be sent to all motors on the bus)
#define DEFAULT_ID					 0x01	    // Default motor ID
#define DEFAULT_BAUD_RATE			 0x01	    // Default baud rate
#define DEFAULT_RETURN_DELAY		 0xFA	    // Default time motor waits before returning status packet (microseconds)
#define DEFAULT_TORQUE_ENABLE		 0x00	    // Default motor power state
#define DEFAULT_LED_ENABLE			 0x00	    // Default LED state
#define DEFAULT_STATUS_RETURN_LEVEL	 0x02	    // Default condition(s) under which a status packet will be returned (all)


/******************************* Public Variables *******************************/
/* Buffer for data received from motors. */
extern uint8_t arrReceive[NUM_MOTORS][BUFF_SIZE_RX];

/* Bytes to be transmitted to motors are written to this array. */
extern uint8_t arrTransmit[NUM_MOTORS + 1][TX_PACKET_SIZE];

/*********************************** Types ************************************/
// TODO: Add a field here that is like "type of motor" and then an enum with AX12A and MX28
// this way we can use a generic handle for all motors
// TODO: Add a field for lastPWM
enum motorTypes_e{
	AX12ATYPE,
	MX28TYPE
};

typedef struct{
	enum motorTypes_e		_motorType;				/*!< Identifies motor as AX12A, MX28, etc.			*/
	uint8_t					_protocolVersion;
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
}Dynamixel_HandleTypeDef;

/***************************** Function prototypes ****************************/
// Setters (use the WRITE DATA instruction)
void setID(Dynamixel_HandleTypeDef* hdynamixel, uint8_t ID);
void setBaudRate(Dynamixel_HandleTypeDef* hdynamixel, long baud); // (EEPROM)

void setGoalPosition(Dynamixel_HandleTypeDef* hdynamixel, double goalAngle);
void Dynamixel_TorqueEnable(Dynamixel_HandleTypeDef* hdynamixel, uint8_t isEnabled);
void torqueEnable(Dynamixel_HandleTypeDef* hdynamixel, uint8_t isEnabled);
void Dynamixel_LEDEnable(Dynamixel_HandleTypeDef* hdynamixel, uint8_t isEnabled);
void LEDEnable(Dynamixel_HandleTypeDef* hdynamixel, uint8_t isEnabled);
//void setDriveMode(); // (EEPROM)
//void setOperatingMode(); // (EEPROM)
//void setShadowID(); // (EEPROM)
//void setProtocolVersion(); // (EEPROM)
//void setHomingOffset(); // (EEPROM)
//void setMovingThreshold(); // (EEPROM)
//void setPWMLimit(); // (EEPROM)
//void setAccelerationLimit(); // (EEPROM)
//void setVelocityLimit(); // (EEPROM)
//void setShutdownConditions(); // (EEPROM)
//
//void setStatusReturnLevel(uint8_t status_data); // (RAM in MX-28)
//
//void setHardwareErrorStatus(); // (RAM)
//void setVelocityIGain(); // (RAM)
//void setVelocityPGain(); // (RAM)
//void setPositionDGain(); // (RAM)
//void setPositionIGain(); // (RAM)
//void setPositionPGain(); // (RAM)
//void setFeedforwardGain2(); // (RAM)
//void setFeedforwardGain1(); // (RAM)
//void setBusWatchdog(); // (RAM)
//void setGoalPWM(); // (RAM)
//void setAccelerationprofile(); // (RAM)
//void setVelocityProfile(); // (RAM)


// Getters (use the READ DATA instruction)
//uint16_t getTick();
//uint8_t getMovingStatus();
//void getPresentPWM();
//void getVelocityTrajectory();
//void getPositionTrajectory();


// Transmission & Reception
//uint8_t Dynamixel_Ping(Dynamixel_HandleTypeDef* hdynamixel);
void MX28_DataWriter(Dynamixel_HandleTypeDef* hdynamixel, uint8_t* args, uint8_t numArgs);
void Dynamixel_DataWriter(Dynamixel_HandleTypeDef* hdynamixel, uint8_t arrSize, uint8_t writeAddr, uint8_t param1, uint8_t param2);
//uint16_t Dynamixel_DataReader(Dynamixel_HandleTypeDef* hdynamixel, uint8_t readAddr, uint8_t readLength);
void setProtocolTo1(Dynamixel_HandleTypeDef* hdynamixel);
uint8_t Dynamixel_Ping(Dynamixel_HandleTypeDef* hdynamixel);
uint8_t Ping(Dynamixel_HandleTypeDef* hdynamixel);
uint16_t Dynamixel_DataReader(Dynamixel_HandleTypeDef* hdynamixel, uint8_t readAddr, uint8_t readLength);

// Initialization
void Dynamixel_Init(Dynamixel_HandleTypeDef* hdynamixel, uint8_t ID, UART_HandleTypeDef* UART_Handle,\
		GPIO_TypeDef* DataDirPort, uint16_t DataDirPinNum, enum motorTypes_e motorType);
void Dynamixel_Reset(Dynamixel_HandleTypeDef* hdynamixel);
void Reset(Dynamixel_HandleTypeDef* hdynamixel, uint8_t arg);
void Reboot(Dynamixel_HandleTypeDef* hdynamixel);

#endif /* __MX28_H__ */
