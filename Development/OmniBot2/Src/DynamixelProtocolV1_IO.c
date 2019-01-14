/**
 * @file DynamixelProtocolV1_IO.c
 * @author Tyler
 * @brief Implements low-level IO functions
 *
 * @defgroup DynamixelProtocolV1_IO IO
 * @ingroup DynamixelProtocolV1
 * @brief Implements low-level IO functions
 * @details
 *     Provides a set of functions which provide flexible
 *     interfaces for transferring packets between motors and the MCU.
 *
 *     This driver uses polled I/O, interrupt-based I/O, or DMA-based I/O
 *     depending on the value of IOType. The user is responsible for
 *     ensuring their system configuration is appropriate when using
 *     interrupt-based or DMA-based I/O. For example, the interrupt-based
 *     mode assumes that the user has enabled interrupts for the UART
 *     module corresponding to this motor, and similarly, the DMA-based
 *     mode assumes that a DMA channel has been allocated. Both of these
 *     non-blocking modes also assumes the user is calling from within the
 *     context of a FreeRTOS thread, and that the callback function has
 *     been implemented to unblock the thread using task notifications.
 * @{
 */

/********************************* Includes **********************************/
#include "DynamixelProtocolV1_IO.h"
#include "Dynamixel_Data.h"

//#include "SystemConf.h"

#if defined(THREADED)
#include "Notification.h"
#include "cmsis_os.h"
#endif




/********************************** Macros ***********************************/
// Communications
#define BUFF_SIZE_RX            8    /**< Receive buffer size for UART receptions (number of bytes) */
#define TX_PACKET_SIZE          9    /**< Maximum packet size for regular motor commands (exclusion: sync write) */

// Instruction set definitions
#define INST_PING               0x01 /**< Gets a status packet  */
#define INST_READ_DATA          0x02 /**< Reads data from a motor register */
#define INST_WRITE_DATA         0x03 /**< Writes data for immediate execution */
#define INST_REG_WRITE          0x04 /**< Registers an instruction to be executed at a later time */
#define INST_ACTION             0x05 /**< Triggers instructions registered by INST_REG_WRITE */
#define INST_RESET              0x06 /**< Resets the control tables of the Dynamixel actuator(s) specified */
#define INST_SYNC_WRITE         0x83 /**< Writes on a specified address with a specified data length on multiple devices */




/********************************* Constants *********************************/
/** Timeout for blocking UART transmissions, in milliseconds */
static const uint32_t TRANSMIT_TIMEOUT = 1;

/** Timeout for blocking UART receptions, in milliseconds */
static const uint32_t RECEIVE_TIMEOUT = 1;




/******************************* Public Variables ****************************/
/**
 * @brief Configures the low-level I/O mode used by the library
 *        Default: polled I/O
 */
ioFlags_t IOType = IO_POLL;




/***************************** Private Variables *****************************/
/** @brief Pre-allocated buffer for reading in packets from motors */
static uint8_t arrReceive[NUM_MOTORS][BUFF_SIZE_RX] = {{0}};

/** @brief Pre-allocated buffer for transmitting packets to motors */
static uint8_t arrTransmit[NUM_MOTORS + 1][TX_PACKET_SIZE] = {
    {0xFF, 0xFF, 0xFE, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
    {0xFF, 0xFF, 1, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
    {0xFF, 0xFF, 2, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
    {0xFF, 0xFF, 3, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
    {0xFF, 0xFF, 4, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
    {0xFF, 0xFF, 5, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
    {0xFF, 0xFF, 6, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
    {0xFF, 0xFF, 7, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
    {0xFF, 0xFF, 8, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
    {0xFF, 0xFF, 9, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
    {0xFF, 0xFF, 10, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
    {0xFF, 0xFF, 11, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
    {0xFF, 0xFF, 12, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
    {0xFF, 0xFF, 13, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
    {0xFF, 0xFF, 14, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
    {0xFF, 0xFF, 15, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
    {0xFF, 0xFF, 16, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
    {0xFF, 0xFF, 17, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
    {0xFF, 0xFF, 18, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00}
};




/***************************** Private Functions *****************************/
/**
 * @defgroup DynamixelProtocolV1_IO_Private_Functions Private Functions
 * @brief    Functions only accessible from within the IO library
 * @ingroup  DynamixelProtocolV1_IO
 * @{
 */

/**
 * @brief  Set data direction pin high (TX)
 * @param  port the port the data direction pin is on
 * @param  pinNum the pin number of the data direction pin on the specified port
 * @return None
 */
static inline void Dynamixel_BusDirTX(GPIO_TypeDef* port, uint16_t pinNum){
    HAL_GPIO_WritePin(port, pinNum, 1);
}

/**
 * @brief  Set data direction pin low (RX)
 * @param  port the port the data direction pin is on
 * @param  pinNum the pin number of the data direction pin on the specified port
 * @return None
 */
static inline void Dynamixel_BusDirRX(GPIO_TypeDef* port, uint16_t pinNum){
    HAL_GPIO_WritePin(port, pinNum, 0);
}

/**
 * @brief  Compute the checksum for data passes in, according to a modular
 *         checksum algorithm employed by the Dynamixel V1.0 protocol
 * @param  arr the array to be ran through the checksum function
 * @param  length the total length of the array arr
 * @return The 1-byte number that is the checksum
 */
static inline uint8_t Dynamixel_ComputeChecksum(uint8_t *arr, int length){
    uint8_t accumulate = 0;

    /* Loop through the array starting from the 2nd element of the array and
     * finishing before the last since the last is where the checksum will
     * be stored */
    for(uint8_t i = 2; i < length - 1; i++){
        accumulate += arr[i];
    }

    return (~accumulate) & 0xFF; // Lower 8 bits of the logical NOT of the sum
}

// These functions are not defined in the F767xx's HAL drivers, so we drag them
// over from the F446xx's HAL drivers
#ifdef STM32F767xx
/**
  * @brief  Abort ongoing Transmit transfer (blocking mode).
  * @param  huart UART handle.
  * @note   This procedure could be used for aborting any ongoing transfer started in Interrupt or DMA mode.
  *         This procedure performs following operations :
  *           - Disable PPP Interrupts
  *           - Disable the DMA transfer in the peripheral register (if enabled)
  *           - Abort DMA transfer by calling HAL_DMA_Abort (in case of transfer in DMA mode)
  *           - Set handle State to READY
  * @note   This procedure is executed in blocking mode : when exiting function, Abort is considered as completed.
  * @retval HAL status
*/
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *huart)
{
  /* Disable TXEIE and TCIE interrupts */
  CLEAR_BIT(huart->Instance->CR1, (USART_CR1_TXEIE | USART_CR1_TCIE));

  /* Disable the UART DMA Tx request if enabled */
  if(HAL_IS_BIT_SET(huart->Instance->CR3, USART_CR3_DMAT))
  {
    CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAT);

    /* Abort the UART DMA Tx channel : use blocking DMA Abort API (no callback) */
    if(huart->hdmatx != NULL)
    {
      /* Set the UART DMA Abort callback to Null.
         No call back execution at end of DMA abort procedure */
      huart->hdmatx->XferAbortCallback = NULL;

      HAL_DMA_Abort(huart->hdmatx);
    }
  }

  /* Reset Tx transfer counter */
  huart->TxXferCount = 0x00U;

  /* Restore huart->gState to Ready */
  huart->gState = HAL_UART_STATE_READY;

  return HAL_OK;
}

/**
  * @brief  Abort ongoing Receive transfer (blocking mode).
  * @param  huart UART handle.
  * @note   This procedure could be used for aborting any ongoing transfer started in Interrupt or DMA mode.
  *         This procedure performs following operations :
  *           - Disable PPP Interrupts
  *           - Disable the DMA transfer in the peripheral register (if enabled)
  *           - Abort DMA transfer by calling HAL_DMA_Abort (in case of transfer in DMA mode)
  *           - Set handle State to READY
  * @note   This procedure is executed in blocking mode : when exiting function, Abort is considered as completed.
  * @retval HAL status
*/
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *huart)
{
  /* Disable RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts */
  CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
  CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);

  /* Disable the UART DMA Rx request if enabled */
  if(HAL_IS_BIT_SET(huart->Instance->CR3, USART_CR3_DMAR))
  {
    CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);

    /* Abort the UART DMA Rx channel : use blocking DMA Abort API (no callback) */
    if(huart->hdmarx != NULL)
    {
      /* Set the UART DMA Abort callback to Null.
         No call back execution at end of DMA abort procedure */
      huart->hdmarx->XferAbortCallback = NULL;

      HAL_DMA_Abort(huart->hdmarx);
    }
  }

  /* Reset Rx transfer counter */
  huart->RxXferCount = 0x00U;

  /* Restore huart->RxState to Ready */
  huart->RxState = HAL_UART_STATE_READY;

  return HAL_OK;
}
#endif

/**
 * @brief   Generic function for receiving data. Supports all IO modes
 * @details When using non-blocking IO, this function will cancel the data
 *          transfer upon timeout
 * @param   hdynamixel pointer to a Dynamixel_HandleTypeDef structure that
 *          contains the configuration information for the motor
 * @param   arr pointer to the receive buffer
 * @param   arrSize the number of bytes to be received
 * @return  true if no issues, false otherwise
 */
static inline bool Dynamixel_GenericReceive(
    Dynamixel_HandleTypeDef* hdynamixel,
    uint8_t* arr,
    uint8_t arrSize
)
{
#if defined(THREADED)
    uint32_t notification;
    BaseType_t status;
#endif
    bool retval = true;

    // Set data direction for receive
    Dynamixel_BusDirRX(hdynamixel -> _dataDirPort, hdynamixel -> _dataDirPinNum);

    switch(IOType) {
#if defined(THREADED)
        case IO_DMA:
            HAL_UART_Receive_DMA(hdynamixel -> _UART_Handle, arr, arrSize);

            status = xTaskNotifyWait(0, NOTIFIED_FROM_RX_ISR, &notification, MAX_DELAY_TIME);

            if(status != pdTRUE || !CHECK_NOTIFICATION(notification, NOTIFIED_FROM_RX_ISR)){
                retval = false;
            }
            break;
        case IO_IT:
            HAL_UART_Receive_IT(hdynamixel -> _UART_Handle, arr, arrSize);

            status = xTaskNotifyWait(0, NOTIFIED_FROM_RX_ISR, &notification, MAX_DELAY_TIME);

            if(status != pdTRUE || !CHECK_NOTIFICATION(notification, NOTIFIED_FROM_RX_ISR)){
                retval = false;
            }
            break;
#endif
        case IO_POLL:
        default:
            HAL_UART_Receive(hdynamixel -> _UART_Handle, arr, arrSize, RECEIVE_TIMEOUT);
            break;
    }

    if(!retval){
        HAL_UART_AbortReceive(hdynamixel -> _UART_Handle);
    }

    return retval;
}

/**
 * @brief   Generic function for transmitting data. Supports all IO modes
 * @details When using non-blocking IO, this function will cancel the data
 *          transfer upon timeout
 * @param   hdynamixel pointer to a Dynamixel_HandleTypeDef structure that
 *          contains the configuration information for the motor
 * @param   arr pointer to the packet to be transmitted
 * @param   arrSize the number of bytes to be transmitted
 * @return  true if no issues, false otherwise
 */
static inline bool Dynamixel_GenericTransmit(
    Dynamixel_HandleTypeDef* hdynamixel,
    uint8_t* arr,
    uint8_t arrSize
)
{
#if defined(THREADED)
    uint32_t notification;
    BaseType_t status;
#endif
    bool retval = true;

    // Set data direction for transmit
    Dynamixel_BusDirTX(hdynamixel -> _dataDirPort, hdynamixel -> _dataDirPinNum);

    switch(IOType) {
#if defined(THREADED)
        case IO_DMA:
            HAL_UART_Transmit_DMA(hdynamixel -> _UART_Handle, arr, arrSize);

            status = xTaskNotifyWait(0, NOTIFIED_FROM_TX_ISR, &notification, MAX_DELAY_TIME);

            if(status != pdTRUE || !CHECK_NOTIFICATION(notification, NOTIFIED_FROM_TX_ISR)){
                retval = false;
            }
            break;
        case IO_IT:
            HAL_UART_Transmit_IT(hdynamixel -> _UART_Handle, arr, arrSize);

            status = xTaskNotifyWait(0, NOTIFIED_FROM_TX_ISR, &notification, MAX_DELAY_TIME);

            if(status != pdTRUE || !CHECK_NOTIFICATION(notification, NOTIFIED_FROM_TX_ISR)){
                retval = false;
            }
            break;
#endif
        case IO_POLL:
        default:
            HAL_UART_Transmit(hdynamixel -> _UART_Handle, arr, arrSize, TRANSMIT_TIMEOUT);
            break;
    }

    if(!retval){
        HAL_UART_AbortTransmit(hdynamixel -> _UART_Handle);
    }

    return retval;
}

/**
 * @}
 */
/* DynamixelProtocolV1_IO_Private_Functions */




/****************************** Public Functions *****************************/
/**
 * @defgroup DynamixelProtocolV1_IO_Public_Functions Public functions
 * @brief Functions accessible from outside this file
 * @ingroup DynamixelProtocolV1_IO
 * @{
 */

/**
 * @defgroup DynamixelProtocolV1_IO_Public_Functions_Lib_Config  \
 *           Library Configuration
 * @brief    Library configuration functions
 *
 * # Library configuration functions #
 *
 * This subsection provides a set of functions which provide interfaces for
 * configuring certain parts of the library. Right now, the only thing that
 * can be changed is the I/O mode (polling, interrupt-based, or DMA-based).
 *
 * @ingroup DynamixelProtocolV1_IO_Public_Functions
 * @{
 */

/**
 * @brief   Sets the I/O type used by the library
 * @details Sets the IO protocol to one of three options:
 *              -# Blocking (Polling)
 *              -# Non-Blocking (Interrupt)
 *              -# DMA
 * @param   type one of IO_POLL, IO_IT, or IO_DMA
 * @return  None
 */
void Dynamixel_SetIOType(enum IO_FLAGS type) {
    IOType = type;
}

/**
 * @brief   Gets the IO protocol setting for the library
 * @return  One of IO_POLL, IO_IT, or IO_DMA
 */
enum IO_FLAGS Dynamixel_GetIOType(){
    return IOType;
}

/**
 * @}
 */
/* DynamixelProtocolV1_IO_Public_Functions_Lib_Config */

/**
 * @defgroup DynamixelProtocolV1_IO_Public_Functions_Primitives IO Primitives
 * @brief Implements the primitive IO instructions supported in V1 of the
 *        protocol
 * @{
 */


/**
 * @brief   Sends an array of data to a motor as per its configuration details
 * @details Uses the WRITE DATA instruction, 0x03, in the motor instruction set.
 * @param   hdynamixel pointer to a Dynamixel_HandleTypeDef structure that
 *          contains the configuration information for the motor
 * @param   args an array of arguments of the form `{ADDR, PARAM_1, ... ,
 *          PARAM_N}`
 * @param   numArgs this must be equal to `sizeof(args)`, and must be either 2
 *          or 3
 * @return  None
 */
void Dynamixel_DataWriter(
    Dynamixel_HandleTypeDef* hdynamixel,
    uint8_t* args,
    uint8_t numArgs
)
{
    // Check validity so that we don't accidentally write something invalid
    if(numArgs <= 3){
        // Do assignments and computations
        uint8_t ID = hdynamixel -> _ID;
        if(ID == BROADCAST_ID){
            ID = 0;
        }
        arrTransmit[ID][3] = 2 + numArgs;
        arrTransmit[ID][4] = INST_WRITE_DATA;
        for(uint8_t i = 0; i < numArgs; i ++){
            arrTransmit[ID][5 + i] = args[i];
        }

        // Checksum
        arrTransmit[ID][4 + numArgs + 1] = Dynamixel_ComputeChecksum(arrTransmit[ID], 4 + numArgs + 2);

        // Transmit
        Dynamixel_GenericTransmit(hdynamixel, arrTransmit[ID], 4 + numArgs + 2);
    }
}

/**
 * @brief   Reads data back from the motor passed in by reference
 * @details Uses the READ DATA instruction, 0x02, in the motor instruction set.
 *          The status packet returned will be of the following form
 *
 *          @code{.c}
 *          {0xFF, 0xFF, ID, LENGTH, ERR, PARAM_1,...,PARAM_N, CHECKSUM}
 *          @endcode
 *
 *          Where N = readLength. Also, this function computes the checksum of
 *          data using the same algorithm as the motors, and it sets
 *          `hdynamixel -> _lastReadIsValid' if the computations match, and
 *          clears this field otherwise. This is a basic data integrity check
 *          that reduces the probability of passing invalid data to the
 *          application
 * @param   hdynamixel pointer to a Dynamixel_HandleTypeDef structure that
 *          contains the configuration information for the motor
 * @param   readAddr the address inside the motor memory table where reading
 *          is to begin
 * @param   readLength the number of bytes to be read. Must be either 1 or 2
 * @return  A 16-bit value containing 1 or both bytes received, as applicable.
 *          The 1st byte received will be the LSB and the 2nd byte received
 *          will be the MSB
 */
uint16_t Dynamixel_DataReader(
    Dynamixel_HandleTypeDef* hdynamixel,
    uint8_t readAddr,
    uint8_t readLength
)
{
    uint16_t retval;
    uint8_t rxPacketSize;
    uint8_t recvChecksum;
    uint8_t computedChecksum;
    uint8_t ID = hdynamixel -> _ID;

    if(ID == BROADCAST_ID){
        ID = 0;
    }

    // Do assignments and computations
    arrTransmit[ID][3] = 4; // Length of message minus the obligatory bytes
    arrTransmit[ID][4] = INST_READ_DATA; // READ DATA instruction
    arrTransmit[ID][5] = readAddr; // Write address for register
    arrTransmit[ID][6] = readLength; // Number of bytes to be read from motor
    arrTransmit[ID][7] = Dynamixel_ComputeChecksum(arrTransmit[ID], 8);

    // Are 1 or 2 bytes to be read?
    rxPacketSize = (readLength == 1) ? 7 : 8;

    // Set data direction for transmit
    Dynamixel_BusDirTX(hdynamixel -> _dataDirPort, hdynamixel -> _dataDirPinNum);

    // Transmit read request
    if(!Dynamixel_GenericTransmit(hdynamixel, arrTransmit[ID], 8)){
        hdynamixel -> _lastReadIsValid = false;
        return -1;
    }

    // Receive requested data
    if(!Dynamixel_GenericReceive(hdynamixel, arrReceive[ID], rxPacketSize)){
        hdynamixel -> _lastReadIsValid = false;
        return -1;
    }

    // Check data integrity and place this flag in a field the application can read
    recvChecksum = arrReceive[ID][rxPacketSize - 1];
    computedChecksum = Dynamixel_ComputeChecksum(arrReceive[ID], rxPacketSize);
    hdynamixel -> _lastReadIsValid = (computedChecksum == recvChecksum);

    retval = (uint16_t)arrReceive[ID][5];
    if(readLength == 2){
        retval |= arrReceive[ID][6] << 8;
    }

    return retval;
}

/**
 * @brief   Implementation of the REG WRITE instruction with 2 parameters
 * @param   hdynamixel pointer to a Dynamixel_HandleTypeDef structure that
 *          contains the configuration information for the motor
 * @param   arrSize the size of the array to be written (either 1 or 2)
 * @param   writeAddr the starting address for where the data is to be written
 * @param   param1 the first parameter
 * @param   param2 the second parameter
 * @return  None
 */
void Dynamixel_RegWrite(
    Dynamixel_HandleTypeDef* hdynamixel,
    uint8_t arrSize,
    uint8_t writeAddr,
    uint8_t param1,
    uint8_t param2
)
{
    uint8_t arrTransmitLocal[arrSize];

    // Do assignments and computations
    arrTransmitLocal[0] = 0xFF;
    arrTransmitLocal[1] = 0xFF;
    arrTransmitLocal[2] = hdynamixel -> _ID == BROADCAST_ID ? 0 : hdynamixel -> _ID;
    arrTransmitLocal[3] = arrSize - 4;
    arrTransmitLocal[4] = INST_REG_WRITE;
    arrTransmitLocal[5] = writeAddr;
    arrTransmitLocal[7] = (
            (arrSize == 8) ?
            Dynamixel_ComputeChecksum(arrTransmitLocal, arrSize) :
            param2
    );
    if(arrSize == 9){
        arrTransmitLocal[8] = Dynamixel_ComputeChecksum(arrTransmitLocal, arrSize);
    }

    // Transmit
    Dynamixel_GenericTransmit(hdynamixel, arrTransmitLocal, arrSize);
}

/**
 * @brief   Implementation of the ACTION instruction
 * @details This triggers the instruction registered by the REG WRITE
 *          instruction. This way, time delays can be reduced for the
 *          concurrent motion of several motors
 * @param   hdynamixel pointer to a Dynamixel_HandleTypeDef structure that
 *          contains the configuration information for the motor
 * @return  None
 */
void Dynamixel_Action(Dynamixel_HandleTypeDef* hdynamixel){
    uint8_t arrTransmitLocal[6];

    // Do assignments and computations
    arrTransmitLocal[0] = 0xFF;
    arrTransmitLocal[1] = 0xFF;
    arrTransmitLocal[2] = hdynamixel -> _ID == BROADCAST_ID ? 0 : hdynamixel -> _ID;
    arrTransmitLocal[3] = 2;
    arrTransmitLocal[4] = INST_ACTION;
    arrTransmitLocal[5] = Dynamixel_ComputeChecksum(arrTransmitLocal, 6);

    // Transmit
    Dynamixel_GenericTransmit(hdynamixel, arrTransmitLocal, sizeof(arrTransmitLocal));
}

/**
 * @brief   Implementation of the PING instruction
 * @details Used only for returning a status packet or checking the existence
 *          of a motor with a specified ID. Does not command any operations
 * @param   hdynamixel pointer to a Dynamixel_HandleTypeDef structure that
 *          contains the configuration information for the motor
 * @return  The motor ID seen in status packet if received a valid
 *          status packet, otherwise the max uint8_t value
 */
int8_t Dynamixel_Ping(Dynamixel_HandleTypeDef* hdynamixel){
    uint8_t arr[6];

    // Do assignments and computations
    arr[0] = 0xff;
    arr[1] = 0xff;
    arr[2] = hdynamixel -> _ID;
    arr[3] = 2;
    arr[4] = INST_PING;
    arr[5] = Dynamixel_ComputeChecksum(arr, 6);

    // Transmit read request
    if(!Dynamixel_GenericTransmit(hdynamixel, arr, sizeof(arr))){
        hdynamixel -> _lastReadIsValid = false;
        return -1;
    }

    // Receive requested data
    if(!Dynamixel_GenericReceive(hdynamixel, arr, sizeof(arr))){
        hdynamixel -> _lastReadIsValid = false;
        return -1;
    }

    return arr[2];
}

/**
 * @brief   Initializes a motor handle
 * @param   hdynamixel pointer to a Dynamixel_HandleTypeDef structure that
 *          contains the configuration information for the motor
 * @param   ID the ID the motor has. Note that this function will not set the
 *          ID in case there are multiple actuators on the same bus
 * @param   UART_Handle the handle to the UART that will be used to communicate
 *          with this motor
 * @param   DataDirPort the pointer to the port that the data direction pin for
 *          the motor is on
 * @param   DataDirPinNum the number corresponding to the pin that controls
 *          data direction (a power of two, e.g. 2^0 for pin 0, 2^15 for pin 15)
 * @param   motorType indicates whether motor is AX12A or MX28
 * @return None
 */
void Dynamixel_Init(Dynamixel_HandleTypeDef* hdynamixel, uint8_t ID, UART_HandleTypeDef *UART_Handle,\
        GPIO_TypeDef* DataDirPort, uint16_t DataDirPinNum, enum motorTypes_e motorType){
    /* Set fields in motor handle. */
    hdynamixel -> _motorType = motorType;       // Identifies the type of actuator; used in certain functions
    hdynamixel -> _ID = ID;                     // Motor ID (unique or global)
    hdynamixel -> _lastReadIsValid = false;     // By default, don't trust the "_last*" fields below until their integrity is vouched for by the DataReader
    hdynamixel -> _lastPosition = -1;           // In future, could initialize this accurately
    hdynamixel -> _lastVelocity = -1;           // In future, could initialize this accurately
    hdynamixel -> _lastLoad = -1;               // In future, could initialize this accurately
    hdynamixel -> _isJointMode = 1;             // In future, could initialize this accurately
    hdynamixel -> _UART_Handle = UART_Handle;   // For UART TX and RX
    hdynamixel -> _dataDirPort = DataDirPort;
    hdynamixel -> _dataDirPinNum = DataDirPinNum;
}

/**
 * @brief   Resets motor control table
 * @details Resets the control table values of the motor to the Factory Default
 *          Value settings. Note that post-reset, motor ID will be 1. Thus, if
 *          several motors with ID 1 are connected on the same bus, there will
 *          not be a way to assign them unique IDs without first disconnecting
 *          them. Need to wait around 500 ms before motor becomes valid again
 * @param   hdynamixel pointer to a Dynamixel_HandleTypeDef structure that
 *          contains the configuration information for the motor
 * @return  None
 */
void Dynamixel_Reset(Dynamixel_HandleTypeDef* hdynamixel){
    uint8_t arrTransmitLocal[6];

    /* Do assignments and computations. */
    arrTransmitLocal[0] = 0xff; // Obligatory bytes for starting communication
    arrTransmitLocal[1] = 0xff; // Obligatory bytes for starting communication
    arrTransmitLocal[2] = hdynamixel -> _ID; // Motor ID
    arrTransmitLocal[3] = 2; // Length of message minus the obligatory bytes
    arrTransmitLocal[4] = INST_RESET; // Reset instruction
    arrTransmitLocal[5] = Dynamixel_ComputeChecksum(arrTransmitLocal, 6);

    // Transmit
    if(Dynamixel_GenericTransmit(hdynamixel, arrTransmitLocal, sizeof(arrTransmitLocal))){
        hdynamixel -> _ID = DEFAULT_ID;
    }
}

/**
 * @}
 */
/* end - DynamixelProtocolV1_IO_Public_Functions_Primitives */

/**
 * @}
 */
/* DynamixelProtocolV1_IO_Public_Functions */

/**
 * @}
 */
/* end - DynamixelProtocolV1_IO */
