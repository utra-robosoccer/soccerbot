/**
  ******************************************************************************
  * @file    UART_Handler.h
  * @author  Gokul
  * @author  Tyler
  * @brief   Header file for the UART event processor, which is called whenever
  *          there are commands that need to be executed by a motor. This header
  *          also defined various types which are used to encapsulate command
  *          data and sensor data to be transmitted via queue
  *
  * @defgroup UART_HandlerHeader UART Handler (header)
  * @brief    Header for UART event processor, showing the public content
  * @ingroup  UART_Handler
  * @{
  ******************************************************************************
  */




#ifndef UART_HANDLER_H
#define UART_HANDLER_H




/********************************** Includes **********************************/
#include "Dynamixel/Dynamixel.h"
#include "SystemConf.h"
#if defined(THREADED)
#include "cmsis_os.h"
#endif



// TODO(rfairley): move this file into Common/app
// (after removing dependencies on this file in component)

/*********************************** Types ************************************/
/**
 * @brief Enumerates the types of motor commands that can be sent to the UART
 *        handlers
 */
typedef enum{
    cmdReadPosition,  /**< Command to read motor position */
    cmdWritePosition, /**< Command to set new motor goal position */
    cmdWriteTorque    /**< Command to refresh the motor torque enable */
}eUARTcmd_t;

/**
 * @brief The container type for motor commands. The control thread sends these
 *        to the various UART handlers through the UART queues. The container
 *        provides all the information needed to generate the appropriate motor
 *        action (reading from or writing to motor command registers)
 */
typedef struct {
    eUARTcmd_t        type;          /**< Indicates the type of motor
	                                      command                        */
    dynamixel::Motor* motorHandle;   /**< Pointer to the motor container */
    float             value;         /**< The value to be written in the
                                          case of a write instruction    */
    QueueHandle_t    qHandle;       /**< Pointer to the queue for this
                                          motor's commands               */
}UartCmd_t;

/**
 * @brief Enumerates the types of data that can be sent to the sensor queue.
 *        This is used so that the reader of the queue will know how to
 *        cast the void pointer so that the data can be interpreted
 *        correctly. See TXData_t for more details
 */
typedef enum{
    eMotorData, /**< Indicates that the pointer is a Motor   */
    eIMUData    /**< Indicates that the pointer is a MPU6050 */
}eTXData_t;

/** @brief Motor data sent from the UART threads to the TX thread */
typedef struct{
    uint8_t id;
    float payload;
    enum{
        T_FLOAT
    }type;
}MotorData_t;

/**
 * @brief   This is the data structure copied into the sensor queue, and read
 *          by the thread that sends data to the PC. It includes 2 fields: one
 *          that indicates the type of data it encapsulates, and the other
 *          pointing to the data
 * @details The fact that the pointer to the data is of type void* allows
 *          pointers to different types of data to be sent through the same
 *          struct, and then typecasted into the appropriate type (based on
 *          eDataType) on the receiving end. Thus, this is flexible as it
 *          allows any type of sensor data to be sent through the queue
 */
typedef struct {
    eTXData_t eDataType; /**< Tells the receiving task what the data is */
    void* pData;         /**< Points to the container for the data      */
}TXData_t;




/***************************** Function prototypes ****************************/
void UART_ProcessEvent(UartCmd_t* cmdPtr);

/**
 * @}
 */
/* end UART_HandlerHeader */


#endif /* UART_HANDLER_H*/
