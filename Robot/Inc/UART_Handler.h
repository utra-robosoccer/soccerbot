/******************** Define to prevent recursive inclusion *******************/
#ifndef __UART_HANDLER_H
#define __UART_HANDLER_H

/********************************** Includes **********************************/
#include "../Drivers/Dynamixel/DynamixelProtocolV1.h"
#include "cmsis_os.h"

/*********************************** Types ************************************/
// Used for sending data from the control task to the UART tasks
typedef enum{
	cmdReadPosition,
	cmdWritePosition,
	cmdWriteTorque
}eUARTcmd_t;

typedef struct {
	eUARTcmd_t 					type;
	Dynamixel_HandleTypeDef*	motorHandle;
	float 						value;
	QueueHandle_t				qHandle;
}UARTcmd_t;

// Used for sending data to the PC TX task
typedef enum{
	eMotorData,
	eIMUData
}eTXData_t;

typedef struct {
	eTXData_t eDataType; // Tells the receiving task what the data is
	void* pData; // Points to the container for the data
}TXData_t;

/***************************** Function prototypes ****************************/
void UART_ProcessEvent(UARTcmd_t* cmdPtr);

#endif /* __UART_HANDLER_H */
