/*
 * MotorManager.h
 *
 *  Created on: Mar 17, 2018
 *      Author: Admin
 */

#ifndef MOTORMANAGER_H_
#define MOTORMANAGER_H_


#include "Dynamixel.h"
#include <vector>


/* Communications. */
#define TRANSMIT_IT				0		// 1 if using interrupts for transmit, otherwise 0 (polling)
#define NUM_MOTORS				18		// Used to determine buffer sizes
#define BUFF_SIZE_RX			8		// Receive buffer size for UART receptions (number of bytes)
#define TX_PACKET_SIZE			9		// Maximum packet size for regular motor commands (exclusion: sync write)
#define BUFF_SIZE_SYNC_WRITE	64		// Maximum packet size for sync write
#define NUM_UARTS				6		// Number of UARTs available for motor communication
const uint8_t TRANSMIT_TIMEOUT = 10; 	// Timeout for blocking UART transmissions, in milliseconds
const uint8_t RECEIVE_TIMEOUT = 10;		// Timeout for blocking UART receptions, in milliseconds


#define INST_WRITE_DATA			0x03	// Writes data for immediate execution


/******************************* Public Variables *******************************/
/* Buffer for data received from motors. */
extern uint8_t arrReceive[NUM_MOTORS][BUFF_SIZE_RX];

/* Bytes to be transmitted to motors are written to this array. */
extern uint8_t arrTransmit[NUM_MOTORS + 1][TX_PACKET_SIZE];

/* Sync write buffer. */
extern uint8_t arrSyncWrite[NUM_UARTS][BUFF_SIZE_SYNC_WRITE];


/*********************************** Enums ***********************************/
enum motorNames{
	MOTOR1,
	MOTOR2,
	MOTOR3,
	MOTOR4,
	MOTOR5,
	MOTOR6,
	MOTOR7,
	MOTOR8,
	MOTOR9,
	MOTOR10,
	MOTOR11,
	MOTOR12,
	MOTOR13,
	MOTOR14,
	MOTOR15,
	MOTOR16,
	MOTOR17,
	MOTOR18
};

/********************************** Classes ***********************************/
class MotorManager {
	public:
		std::vector<Dynamixel*> motorTable;

		MotorManager();
		virtual ~MotorManager();
};

#endif /* MOTORMANAGER_H_ */
