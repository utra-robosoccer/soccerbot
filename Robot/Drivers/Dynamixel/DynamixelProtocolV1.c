/**
  *****************************************************************************
  * @file    DynamixelProtocolV1.c
  * @author  Tyler
  * @author  Gokul
  * @author  Hannah
  * @brief   Top-level module for Dynamixel library (protocol V1.0). Contains
  *          common functional code for the AX12A library and MX28 library.
  * 		 It is generic in that any Dynamixel actuator using protocol version
  * 		 1.0 should be able to be integrated with little effort, as the
  * 		 instructions and register addresses are very similar for all
  * 		 actuators using this protocol version
  *
  * @defgroup Dynamixel Dynamixel
  * @brief    Everything related to Dynamixel actuators
  *
  * @defgroup DynamixelProtocolV1 Dynamixel Protocol V1.0
  * @brief    Everything related to implementing the Dynamixel communication
  *           protocol, version 1.0
  * @ingroup  Dynamixel
  *****************************************************************************
  */

/********************************* Includes **********************************/
#include "DynamixelProtocolV1.h"




/********************************** Macros ***********************************/
/* Communications */
#define BUFF_SIZE_RX            8       /**< Receive buffer size for UART receptions (number of bytes) */
#define TX_PACKET_SIZE          9       /**< Maximum packet size for regular motor commands (exclusion: sync write) */

/* Instruction set definitions */
#define INST_PING               0x01        /**< Gets a status packet  */
#define INST_READ_DATA          0x02        /**< Reads data from a motor register */
#define INST_WRITE_DATA         0x03        /**< Writes data for immediate execution */
#define INST_REG_WRITE          0x04        /**< Registers an instruction to be executed at a later time */
#define INST_ACTION             0x05        /**< Triggers instructions registered by INST_REG_WRITE */
#define INST_RESET              0x06        /**< Resets the control tables of the Dynamixel actuator(s) specified */
#define INST_SYNC_WRITE         0x83        /**< Writes on a specified address with a specified data length on multiple devices */

/* Register addresses */
#define REG_ID                              0x03    /**< Motor ID register */
#define REG_BAUD_RATE                       0x04    /**< Baud rate register */
#define REG_RETURN_DELAY_TIME               0x05    /**< Status packet return delay time register */
#define REG_CW_ANGLE_LIMIT                  0x06    /**< Clockwise angle limit register (0x06 = low byte, 0x07 = high byte) */
#define REG_CCW_ANGLE_LIMIT                 0x08    /**< Counter-clockwise angle limit register (0x08 = low byte, 0x09 = high byte) */
#define REG_HIGH_VOLTAGE_LIMIT              0x0C    /**< Maximum voltage limit register */
#define REG_LOW_VOLTAGE_LIMIT               0x0D    /**< Minimum voltage limit register */
#define REG_MAX_TORQUE                      0x0E    /**< Maximum torque limit register (0x0E = low byte, 0x0F = high byte) */
#define REG_STATUS_RETURN_LEVEL             0x10    /**< Status packet return condition(s) register */
#define REG_ALARM_LED                       0x11    /**< Alarm LED condition(s) register */
#define REG_ALARM_SHUTDOWN                  0x12    /**< Alarm shutdown condition(s) register */
#define REG_TORQUE_ENABLE                   0x18    /**< Motor power control register */
#define REG_LED_ENABLE                      0x19    /**< LED control register */
#define REG_GOAL_POSITION                   0x1E    /**< Goal position register (0x1E = low byte, 0x1F = high byte) */
#define REG_GOAL_VELOCITY                   0x20    /**< Goal velocity register (0x20 = low byte, 0x21 = high byte) */
#define REG_GOAL_TORQUE                     0x22    /**< Goal torque register (0x22 = low byte, 0x23 = high byte) */
#define REG_LOCK_EEPROM                     0x2F    /**< EEPROM lock register */
#define REG_PUNCH                           0x30    /**< Punch (0x30 = low register, 0x31 = high register) */
#define REG_CURRENT_POSITION                0x24    /**< Current position register (0x24 = low byte, 0x25 = high byte) */
#define REG_CURRENT_VELOCITY                0x26    /**< Current velocity register (0x26 = low byte, 0x27 = high byte) */
#define REG_CURRENT_LOAD                    0x28    /**< Current load register (0x28 = low byte, 0x29 = high byte) */
#define REG_CURRENT_VOLTAGE                 0x2A    /**< Current voltage register */
#define REG_CURRENT_TEMPERATURE             0x2B    /**< Current temperature register */
#define REG_REGISTERED                      0x2C    /**< Command execution status register */
#define REG_MOVING                          0x2E    /**< Motor motion register */




/********************************* Constants *********************************/
// Communications
static const uint32_t TRANSMIT_TIMEOUT    = 1;          /**< Timeout for blocking UART transmissions, in milliseconds */
static const uint32_t RECEIVE_TIMEOUT     = 1;          /**< Timeout for blocking UART receptions, in milliseconds */

// Default register values
const uint8_t BROADCAST_ID                = 0xFE;       /**< Motor broadcast ID (i.e. messages sent to this ID will be sent to all motors on the bus) */
const uint8_t DEFAULT_ID                  = 0x01;       /**< Default motor ID */
const uint8_t DEFAULT_RETURN_DELAY        = 0xFA;       /**< Default time motor waits before returning status packet (microseconds) */
const uint16_t DEFAULT_CW_ANGLE_LIMIT     = 0x0000;     /**< Default clockwise angle limit */
const uint8_t DEFAULT_LOW_VOLTAGE_LIMIT   = 0x3C;       /**< Default permitted minimum voltage (0x3C = 60 -> 6.0 V) */
const uint16_t DEFAULT_MAXIMUM_TORQUE     = 0x03FF;     /**< Default maximum torque limit (10-bit resolution percentage) */
const uint8_t DEFAULT_STATUS_RETURN_LEVEL = 0x02;       /**< Default condition(s) under which a status packet will be returned (all) */
const uint8_t DEFAULT_ALARM_LED           = 0x24;       /**< Default condition(s) under which the alarm LED will be set */
const uint8_t DEFAULT_ALARM_SHUTDOWN      = 0x24;       /**< Default condition(s) under which the motor will shut down due to an alarm */
const uint8_t DEFAULT_TORQUE_ENABLE       = 0x00;       /**< Default motor power state */
const uint8_t DEFAULT_LED_ENABLE          = 0x00;       /**< Default LED state */
const uint8_t DEFAULT_EEPROM_LOCK         = 0x00;       /**< Default value for the EEPROM lock */

// Value limit definitions
const float MIN_VELOCITY                  = 1.0;        /**< Minimum angular velocity (RPM) */
const float MAX_ANGLE                     = 300.0;      /**< Maximum angular position (joint mode) */
const float MIN_ANGLE                     = 0.0;        /**< Minimum angular position (joint mode) */
const float MAX_TORQUE                    = 100.0;      /**< Maximum torque (percent of maximum) */
const float MIN_TORQUE                    = 0.0;        /**< Minimum torque (percent of maximum) */
const float MAX_VOLTAGE                   = 14.0;       /**< Maximum operating voltage */
const float MIN_VOLTAGE                   = 6.0;        /**< Minimum operating voltage */
const uint16_t MAX_PUNCH                  = 1023;       /**< Maximum punch (proportional to minimum current) */
const uint16_t MIN_PUNCH                  = 0;          /**< Minimum punch (proportional to minimum current) */




/******************************* Public Variables ****************************/
// IO Type - initialized to blocking IO
enum IO_FLAGS IOType = IO_POLL; /**< Configures the low-level I/O mode used by
                                     the library. Default: polled I/O        */




/***************************** Private Variables *****************************/
/** Pre-allocated buffer for reading in packets from motors */
static uint8_t arrReceive[NUM_MOTORS][BUFF_SIZE_RX] = {{0}};

/** Pre-allocated buffer for transmitting packets to motors */
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




/************************ Private Function Prototypes ************************/
static inline uint8_t Dynamixel_ComputeChecksum(uint8_t *arr, int length);
static inline bool Dynamixel_GenericReceive(
    Dynamixel_HandleTypeDef* hdynamixel,
    uint8_t* arr,
    uint8_t arrSize
);
static inline bool Dynamixel_GenericTransmit(
    Dynamixel_HandleTypeDef* hdynamixel,
    uint8_t* arr,
    uint8_t arrSize
);




/******************************** Functions **********************************/
/**
 * @defgroup DynamixelProtocolV1_Public_Functions Public Functions
 * @brief    Globally-accessible functions for interfacing with Dynamixel
 *           actuators and controlling the settings of this library
 * @ingroup  DynamixelProtocolV1
 */




/*****************************************************************************/
/*  Setter functions                                                         */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/
/**
 * @defgroup DynamixelProtocolV1_Public_Functions_Setters Setters
 * @brief    Register-setting functions
 *
 * # Setter functions #
 *
 * This subsection provides a set of functions which provide interfaces for
 * setting motor register values.
 *
 * @ingroup DynamixelProtocolV1_Public_Functions
 * @{
 */

/**
 * @brief   Sets the ID (identification number) for the current motor
 * @details Note that the instruction will be broadcasted using the current ID.
 * 	        As such, if the ID is not known, the motor ID should be initialized
 *          to the broadcast ID (0xFE) in the Dynamixel_Init function
 *
 *          Instruction register address: 0x03 (EEPROM)
 *          Default value: 1
 * @param   hdynamixel pointer to a Dynamixel_HandleTypeDef structure that
 *          contains the configuration information for the motor
 * @param   ID the number between 0 and 252 or equal to 254 to identify the
 *          motor. If 0xFE (254), any messages broadcasted to that ID will be
 *          broadcasted to all motors
 * @return  None
 */
void Dynamixel_SetID(Dynamixel_HandleTypeDef* hdynamixel, uint8_t ID){
    /* Compute validity. */
    if((ID == 253) || (ID == 255)){
        ID = DEFAULT_ID;
    }

    /* Write data to motor. */
    uint8_t args[2] = {REG_ID, ID};
    Dynamixel_DataWriter(hdynamixel, args, sizeof(args));

    /* Save ID in handle */
    hdynamixel -> _ID = ID;
}

/**
 * @brief   Sets the baud rate of a particular motor
 * @details Register address is 0x04 in motor EEPROM
 *
 *          Instruction register address: 0x04 (EEPROM)
 *
 *          Default value: 0x01
 * @param   hdynamixel pointer to a Dynamixel_HandleTypeDef structure that
 *          contains the configuration information for the motor
 * @param   baud the baud rate. Arguments in range [7844, 1000000] are valid
 * @return  None
 */
void Dynamixel_SetBaudRate(Dynamixel_HandleTypeDef* hdynamixel, uint32_t baud){
    uint8_t baudArg = 0x01; // Default to 1 Mbps

    if(hdynamixel -> _motorType == AX12ATYPE){
        /* Set _baud equal to the hex code corresponding to baud. Default to 1
         * Mbps. */
        if(baud > 0){
            /* Valid for baud in range [7844, 1000000]. Will be converted to
             * 8-bit resolution. */
            baudArg = (uint8_t)((2000000 / baud) - 1);
        }
        else{
            /* Default to 1 Mbps. */
            baudArg = AX12A_DEFAULT_BAUD_RATE;
        }
    }
    else if(hdynamixel -> _motorType == MX28TYPE){
        /* Set _baud equal to the hex code corresponding to baud. Default to 1
         * Mbps. */
        if(baud >= 9600 && baud <= 3500000){
            if(baud >= 2250000){
                if(baud < 2500000){
                    baudArg = 250;
                }
                else if(baud < 3000000){
                    baudArg = 251;
                }
                else{
                    baudArg = 252;
                }
            }
            else{
                baudArg = (uint8_t)((2000000 / baud) - 1);
            }
        }
        else{
            /* Default to 1000000 symbols/s (MX28_DEFAULT_BAUD_RATE is not to
             * be used for our application) */
            baudArg = 0x01;
        }
    }

    /* Write data to motor. */
    uint8_t args[2] = {REG_BAUD_RATE, baudArg};
    Dynamixel_DataWriter(hdynamixel, args, sizeof(args));
}

/**
 * @brief   Sets the time, in microseconds, that the motor should wait before
 *          returning a status packet
 * @details Instruction register address: 0x05(EEPROM)
 *
 *          Default value: 250 (0xFA)
 * @param   hdynamixel pointer to a Dynamixel_HandleTypeDef structure that
 *          contains the configuration information for the motor
 * @param   microSec the time in microseconds to delay. Arguments in range
 *          [2, 508] are valid. Default: 500
 * @return  None
 */
void Dynamixel_SetReturnDelayTime(Dynamixel_HandleTypeDef* hdynamixel, uint16_t microSec){
    /* Compute validity. */
    uint8_t motor_data;
    if((microSec < 2) || (microSec > 508)){
        motor_data = DEFAULT_RETURN_DELAY;
    }
    else{
        motor_data = (uint8_t)(microSec / 2);
    }

    /* Write data to motor. */
    uint8_t args[2] = {REG_RETURN_DELAY_TIME, motor_data};
    Dynamixel_DataWriter(hdynamixel, args, sizeof(args));
}

/**
 * @brief   Sets the clockwise angle limit for the current motor
 * @details If maxAngle for CCW angle limit is 0 AND minAngle for CW angle
 *          limit is 0, then motor is in wheel mode where it can continuously
 *          rotate. Otherwise, motor is in joint mode where its motion is
 *          constrained between the set bounds.
 *
 *          Register 0x06 in EEPROM for low byte, 0x07 in EEPROM for high byte
 *
 *          Instruction register address: 0x06 (EEPROM)
 *
 *          Default value: 0x0000
 * @param   hdynamixel pointer to a Dynamixel_HandleTypeDef structure that
 *          contains the configuration information for the motor
 * @param   minAngle the minimum angle for all motor operations. Arguments
 *          between 0 and 300 are valid
 * @return  None
 */
void Dynamixel_SetCWAngleLimit(Dynamixel_HandleTypeDef* hdynamixel, float minAngle){
    /* Initialize local variable to default value. */
    uint16_t normalized_value = DEFAULT_CW_ANGLE_LIMIT;

    /* Evaluate argument validity. Optimize for edge case minAngle == MIN_ANGLE. */
    if((minAngle > MIN_ANGLE) && (minAngle <= MAX_ANGLE)){

        /* Translate the angle from degrees into a 10-bit number. */
        if(hdynamixel -> _motorType == AX12ATYPE){
            normalized_value = (uint16_t)(minAngle / MAX_ANGLE * 1023);
        }
        else if(hdynamixel -> _motorType == MX28TYPE){
            normalized_value = (uint16_t)(minAngle / MAX_ANGLE * 4095);
        }
    }

    uint8_t lowByte = (uint8_t)(normalized_value & 0xFF); // Low byte of CW angle limit
    uint8_t highByte = (uint8_t)((normalized_value >> 8) & 0xFF); // High byte of CW angle limit

    /* Write data to motor. */
    uint8_t args[3] = {REG_CW_ANGLE_LIMIT, lowByte, highByte};
    Dynamixel_DataWriter(hdynamixel, args, sizeof(args));
}

/**
 * @brief   Sets the counter-clockwise angle limit for the current motor
 * @details If maxAngle for CCW angle limit is 0 AND minAngle for CW angle
 *          limit is 0, then motor is in wheel mode where it can continuously
 *          rotate. Otherwise, motor is in joint mode where its motion is
 *          constrained between the set bounds.
 *
 *          Register 0x08 in EEPROM for low byte, 0x09 in EEPROM for high byte
 *
 *          Instruction register address: 0x08 (EEPROM)
 *
 *          Default value: 0x03FF
 * @param   hdynamixel pointer to a Dynamixel_HandleTypeDef structure that
 *          contains the configuration information for the motor
 * @param   maxAngle the maximum angle for all motor operations. Arguments
 *          between 0 and 300 are valid
 * @return  None
 */
void Dynamixel_SetCCWAngleLimit(Dynamixel_HandleTypeDef* hdynamixel, float maxAngle){
    /* Initialize local variable to default value. */
    uint16_t normalized_value = AX12A_DEFAULT_CCW_ANGLE_LIMIT; // default
    if(hdynamixel -> _motorType == AX12ATYPE){
        normalized_value = AX12A_DEFAULT_CCW_ANGLE_LIMIT;
    }
    else if(hdynamixel -> _motorType == MX28TYPE){
        normalized_value = MX28_DEFAULT_CCW_ANGLE_LIMIT;
    }

    /* Evaluate argument validity. Optimize for edge case maxAngle = MAX_ANGLE. */
    if((maxAngle >= MIN_ANGLE) && (maxAngle < MAX_ANGLE)){

        /* Translate the angle from degrees into a 10-bit number. */
        if(hdynamixel -> _motorType == AX12ATYPE){
            normalized_value = (uint16_t)(maxAngle / MAX_ANGLE * 1023);
        }
        else if(hdynamixel -> _motorType == MX28TYPE){
            normalized_value = (uint16_t)(maxAngle / MAX_ANGLE * 4095);
        }

    }

    uint8_t lowByte = (uint8_t)(normalized_value & 0xFF); // Low byte of CCW angle limit
    uint8_t highByte = (uint8_t)((normalized_value >> 8) & 0xFF); // High byte of CCW angle limit

    /* Write data to motor. */
    uint8_t args[3] = {REG_CCW_ANGLE_LIMIT, lowByte, highByte};
    Dynamixel_DataWriter(hdynamixel, args, sizeof(args));
}

// TODO: Test
/**
 * @brief   Sets the highest operating voltage limit for the current motor
 * @details Instruction register address: 0x0C (EEPROM)
 *
 *          Default value: 140 (0xBE)
 * @param   hdynamixel pointer to a Dynamixel_HandleTypeDef structure that
 *          contains the configuration information for the motor
 * @param   highestVoltage the highest operating voltage in volts
 * @return  None
 */
void Dynamixel_SetHighestVoltageLimit(Dynamixel_HandleTypeDef* hdynamixel, float highestVoltage){
    /* Declare local variable. Initialize to default value. */
    uint8_t high_voltage_data = AX12A_DEFAULT_HIGHEST_VOLTAGE_LIMIT; // minimum is safer to default to

    if(hdynamixel -> _motorType == AX12ATYPE){
        high_voltage_data = AX12A_DEFAULT_HIGHEST_VOLTAGE_LIMIT;
    }
    else if(hdynamixel -> _motorType == MX28TYPE){
        high_voltage_data = MX28_DEFAULT_HIGHEST_VOLTAGE_LIMIT;
    }

    /* Evaluate argument validity and translate into motor data. Optimize for highestVoltage = MAX_VOLTAGE. */
    if((highestVoltage >= MIN_VOLTAGE) && (highestVoltage < MAX_VOLTAGE)){
        high_voltage_data = (uint8_t)(highestVoltage * 10);
    }

    /* Write data to motor. */
    uint8_t args[2] = {REG_HIGH_VOLTAGE_LIMIT, high_voltage_data};
    Dynamixel_DataWriter(hdynamixel, args, sizeof(args));
}

// TODO: Test
/**
 * @brief   Sets the lowest operating voltage limit for the current motor
 * @details Instruction register address: 0x0D (EEPROM)
 *
 *          Default value: 60 (0x3C)
 * @param   hdynamixel pointer to a Dynamixel_HandleTypeDef structure that
 *          contains the configuration information for the motor
 * @param   lowestVoltage the lowest operating voltage in volts
 * @return  None
 */
void Dynamixel_SetLowestVoltageLimit(Dynamixel_HandleTypeDef* hdynamixel, float lowestVoltage){
    /* Declare local variable. Initialize to default value. */
    uint8_t low_voltage_data = DEFAULT_LOW_VOLTAGE_LIMIT;

    /* Evaluate argument validity and translate into motor data. Optimize for lowestVoltage = MIN_VOLTAGE. */
    if((lowestVoltage > MIN_VOLTAGE) && (lowestVoltage <= MAX_VOLTAGE)){

        /* Translate into format that motor can understand. */
        low_voltage_data = (uint8_t)(lowestVoltage * 10);
    }

    /* Write data to motor. */
    uint8_t args[2] = {REG_LOW_VOLTAGE_LIMIT, low_voltage_data};
    Dynamixel_DataWriter(hdynamixel, args, sizeof(args));
}

// TODO: Test
/**
 * @brief   Sets the maximum torque limit for all motor operations
 * @details Low byte is addr 0x0E in motor RAM, high byte is addr 0x0F in motor
 *          RAM
 *
 *          Register address: 0x0E (EEPROM)
 *
 *          Default value: 0x3FF
 * @param   hdynamixel pointer to a Dynamixel_HandleTypeDef structure that
 *          contains the configuration information for the motor
 * @param   maxTorque the maximum torque as a percentage (max: 100). Gets
 *          converted to 10-bit number
 * @return  None
 */
void Dynamixel_SetMaxTorque(Dynamixel_HandleTypeDef* hdynamixel, float maxTorque){
    /* Initialize to default value. */
    uint16_t normalized_value = DEFAULT_MAXIMUM_TORQUE;

    /* Evaluate argument validity and optimize for edge case maxTorque = MAX_TORQUE. */
    if((maxTorque >= MIN_TORQUE) && (maxTorque < MAX_TORQUE)){

        /* Translate the input from percentage into a 10-bit number. */
        normalized_value = (uint16_t)(maxTorque / 100 * 1023);
    }

    uint8_t lowByte = normalized_value & 0xFF; // Low byte of max torque
    uint8_t highByte = (normalized_value >> 8) & 0xFF; // High byte of max torque

    /* Write data to motor. */
    uint8_t args[3] = {REG_MAX_TORQUE, lowByte, highByte};
    Dynamixel_DataWriter(hdynamixel, args, sizeof(args));
}

/**
 * @brief   Sets the conditions under which a status packet will be returned
 * @details Register address: 0x10 (EEPROM)
 *
 *          Default value: 0x02
 * @param   hdynamixel pointer to a Dynamixel_HandleTypeDef structure that
 *          contains the configuration information for the motor
 * @param   status_data
 *              - 0 to return only on ping
 *              - 1 to return only for reads
 *              - 2 to return for all commands
 * @return  None
 */
void Dynamixel_SetStatusReturnLevel(Dynamixel_HandleTypeDef* hdynamixel, uint8_t status_data){
    /* Evaluate argument validity. */
    if((status_data < 0) || (status_data > 2)){
        status_data = DEFAULT_STATUS_RETURN_LEVEL;
    }

    /* Write data to motor. */
    uint8_t args[2] = {REG_STATUS_RETURN_LEVEL, status_data};
    Dynamixel_DataWriter(hdynamixel, args, sizeof(args));
}

/**
 * @brief   Sets the conditions under which the motor LED will light up
 * @details Register bits may be set simultaneously.
 *
 *          Register address: 0x11 (EEPROM)
 *
 *          Default value: 0x24 (0b00100100)
 * @param   hdynamixel pointer to a Dynamixel_HandleTypeDef structure that
 *          contains the configuration information for the motor
 * @param   alarm_LED_data the bits indicate the following:
 *              - bit 7: no function
 *              - bit 6: flash LED when an instruction error occurs
 *              - bit 5: flash LED when current load cannot be controlled with
 *                       the specified maximum torque
 *              - bit 4: flash LED when the checksum of the transmitted packet
 *                       is invalid
 *              - bit 3: flash LED when the command is given beyond the range
 *                       of usage
 *              - bit 2: flash LED when the internal temperature exceeds the
 *                       operating range
 *              - bit 1: flash LED when goal position exceeds the CW angle
 *                       limit or CCW angle limit
 *              - bit 0: flash LED when applied voltage is out of operating
 *                       range
 * @return  None
 */
void Dynamixel_SetAlarmLED(Dynamixel_HandleTypeDef* hdynamixel, uint8_t alarm_LED_data){
    /* Write data to motor. */
    uint8_t args[2] = {REG_ALARM_LED, alarm_LED_data};
    Dynamixel_DataWriter(hdynamixel, args, sizeof(args));
}

/**
 * @brief   Sets the conditions under which the motor will turn off its torque
 * @details Register bits may be set simultaneously
 *
 *          Register address: 0x12 (EEPROM)
 *
 *          Default value: 0x24 (0b00100100)
 * @param   hdynamixel pointer to a Dynamixel_HandleTypeDef structure that
 *          contains the configuration information for the motor
 * @param   alarm_shutdown_data the bits indicate the following:
 *              - bit 7: no function
 *              - bit 6: torque off when an instruction error occurs
 *              - bit 5: torque off when current load cannot be controlled with
 *                       the specified maximum torque
 *              - bit 4: torque off when the checksum of the transmitted packet
 *                       is invalid
 *              - bit 3: torque off when the command is given beyond the range
 *                       of usage
 *              - bit 2: torque off when the internal temperature exceeds the
 *                       operating range
 *              - bit 1: torque off when goal position exceeds the CW angle
 *                       limit or CCW angle limit
 *              - bit 0: torque off when applied voltage is out of operating
 *                       range
 * @return  None
 */
void Dynamixel_SetAlarmShutdown(Dynamixel_HandleTypeDef* hdynamixel, uint8_t alarm_shutdown_data){
    /* Write data to motor. */
    uint8_t args[2] = {REG_ALARM_SHUTDOWN, alarm_shutdown_data};
    Dynamixel_DataWriter(hdynamixel, args, sizeof(args));
}

/**
 * @brief   Enables or disables torque for current motor
 * @details Instruction register address: 0x18 (RAM)
 *
 *          Default value: 0x00
 * @param   hdynamixel pointer to a Dynamixel_HandleTypeDef structure that
 *          contains the configuration information for the motor
 * @param   isEnabled
 *              - if 1, then generates torque by impressing power to the motor
 *              - if 0, then interrupts power to the motor to prevent it from
 *                generating torque
 * @return  None
 */
void Dynamixel_TorqueEnable(Dynamixel_HandleTypeDef* hdynamixel, uint8_t isEnabled){
    /* Evaluate argument validity. */
    if((isEnabled != 1) && (isEnabled != 0)){
        isEnabled = DEFAULT_TORQUE_ENABLE;
    }

    /* Write data to motor. */
    uint8_t args[2] = {REG_TORQUE_ENABLE, isEnabled};
    Dynamixel_DataWriter(hdynamixel, args, sizeof(args));
}

/**
 * @brief   Toggles the motor LED
 * @details Instruction register address: 0x19 (RAM)
 *
 *          Default value: 0x00
 * @param   hdynamixel pointer to a Dynamixel_HandleTypeDef structure that
 *          contains the configuration information for the motor
 * @param   isEnabled
 *              - if 1, LED is on
 *              - if 0, LED is off
 * @return  None
 */
void Dynamixel_LEDEnable(Dynamixel_HandleTypeDef* hdynamixel, uint8_t isEnabled){
    /* Evaluate argument validity. */
    if((isEnabled != 1) && (isEnabled != 0)){
        isEnabled = DEFAULT_LED_ENABLE;
    }

    /* Write data to motor. */
    uint8_t args[2] = {REG_LED_ENABLE, isEnabled};
    Dynamixel_DataWriter(hdynamixel, args, sizeof(args));
}

/**
 * @brief   Sets the goal position of the motor in RAM
 * @details Takes a double between 0 and 300, encodes this position in an
 *          upper and low hex byte pair (with a maximum of 1023 as defined in
 *          the AX-12 user manual), and sends this information (along with
 *          requisites) over UART. Low byte is 0x1E in motor RAM, high byte is
 *          0x1F in motor RAM.
 *
 *          Instruction register address: 0x1E (RAM)
 *
 *          Default value: None
 * @param   hdynamixel pointer to a Dynamixel_HandleTypeDef structure that
 *          contains the configuration information for the motor
 * @param   goalAngle the desired angular position. Arguments between 0 and 300 are
 *          valid. Note that 150 corresponds to the middle position
 * @return  None
 */
void Dynamixel_SetGoalPosition(Dynamixel_HandleTypeDef* hdynamixel, float goalAngle){
    /* Check for input validity. If input not valid, replace goalAngle with closest
     * valid value to ensure code won't halt. */
    if((goalAngle < MIN_ANGLE) || (goalAngle > MAX_ANGLE)){
        if(goalAngle > MIN_ANGLE){
            goalAngle = MAX_ANGLE;
        }
        else{
            goalAngle = MIN_ANGLE;
        }
    }

    /* Translate the angle from degrees into a 10- or 12-bit number. */
    uint16_t normalized_value = 0;
    if(hdynamixel -> _motorType == AX12ATYPE){
        normalized_value = (uint16_t)(goalAngle / MAX_ANGLE * 1023);
    }
    else if(hdynamixel -> _motorType == MX28TYPE){
        normalized_value = (uint16_t)(goalAngle / MAX_ANGLE * 4095);
    }
    else{
        // Should NEVER reach here!
    }

    uint8_t lowByte = (uint8_t)(normalized_value & 0xFF); // Low byte of goal position
    uint8_t highByte = (uint8_t)((normalized_value >> 8) & 0xFF); // High byte of goal position

    /* Write data to motor. */
    uint8_t args[3] = {REG_GOAL_POSITION, lowByte, highByte};
    Dynamixel_DataWriter(hdynamixel, args, sizeof(args));
}

/**
 * @brief   Sets the goal velocity of the motor in RAM
 * @details Low byte is 0x20 in motor RAM, high byte is 0x21 in motor RAM
 *
 *          Instruction register address: 0x20 (RAM)
 *
 *          Default value: None
 * @param   hdynamixel pointer to a Dynamixel_HandleTypeDef structure that
 *          contains the configuration information for the motor
 * @param   goalVelocity the goal velocity in RPM. Arguments of 0-114 are valid
 *          when in joint mode. 0 corresponds to MAX motion in joint mode,
 *          and minimum motion in wheel mode. In wheel mode, negative
 *          arguments correspond to CW rotation
 * @return  None
 */
void Dynamixel_SetGoalVelocity(Dynamixel_HandleTypeDef* hdynamixel, float goalVelocity){
    /* Translate the position from RPM into a 10-bit number. */
    uint16_t normalized_value = 0;
    if(hdynamixel -> _isJointMode){

        /* Check for input validity. If input not valid, replace goalAngle with closest
         * valid value to ensure code won't halt. */
        if(goalVelocity != 0){
            if(hdynamixel -> _motorType == AX12ATYPE){
                if((goalVelocity < MIN_VELOCITY) || (goalVelocity > AX12A_MAX_VELOCITY)){
                    if(goalVelocity > MIN_VELOCITY){
                        goalVelocity = AX12A_MAX_VELOCITY;
                    }
                    else{
                        goalVelocity = MIN_VELOCITY;
                    }
                }
            }
            else if(hdynamixel -> _motorType == MX28TYPE){
                if((goalVelocity < MIN_VELOCITY) || (goalVelocity > MX28_MAX_VELOCITY)){
                    if(goalVelocity > MIN_VELOCITY){
                        goalVelocity = MX28_MAX_VELOCITY;
                    }
                    else{
                        goalVelocity = MIN_VELOCITY;
                    }
                }
            }
        }
    }

    if(hdynamixel -> _motorType == AX12ATYPE){
        normalized_value = (uint16_t)(goalVelocity / AX12A_MAX_VELOCITY * 1023);
        if(goalVelocity < 0){
            normalized_value = ((uint16_t)((goalVelocity * -1) / AX12A_MAX_VELOCITY * 1023)) | 0b000010000000000;
        }
    }
    else if(hdynamixel -> _motorType == MX28TYPE){
        normalized_value = (uint16_t)(goalVelocity / MX28_MAX_VELOCITY * 1023);
        if(goalVelocity < 0){
            normalized_value = ((uint16_t)((goalVelocity * -1) / MX28_MAX_VELOCITY * 1023)) | 0b000010000000000;
        }
    }

    uint8_t lowByte = (uint8_t)(normalized_value & 0xFF); // Low byte of goal velocity
    uint8_t highByte = (uint8_t)((normalized_value >> 8) & 0xFF); // High byte of goal velocity

    /* Write data to motor. */
    uint8_t args[3] = {REG_GOAL_VELOCITY, lowByte, highByte};
    Dynamixel_DataWriter(hdynamixel, args, sizeof(args));
}

/**
 * @brief   Sets the torque limit for the motor in RAM
 * @details The initial value is taken from 0x0E and 0x0F (max torque in
 *          EEPROM)
 *
 *          Low byte is 0x22 in motor RAM, high byte is 0x23 in motor RAM.
 *
 *          Instruction register address: 0x22 (RAM)
 *
 *          Default value: ADDR14 (low byte) and ADDR15 (high byte)
 * @param   hdynamixel pointer to a Dynamixel_HandleTypeDef structure that
 *          contains the configuration information for the motor
 * @param   goalTorque the percentage of the maximum possible torque (max:
 *          100). Gets converted into a 10-bit number
 * @return  None
 */
void Dynamixel_SetGoalTorque(Dynamixel_HandleTypeDef* hdynamixel, float goalTorque){
    /* Translate the input from percentage into and 10-bit number. */
    uint16_t normalized_value = (uint16_t)(goalTorque / 100 * 1023);

    uint8_t lowByte = (uint8_t)(normalized_value & 0xFF); // Low byte of goal torque
    uint8_t highByte = (uint8_t)((normalized_value >> 8) & 0xFF); // High byte of goal torque

    /* Write data to motor. */
    uint8_t args[3] = {REG_GOAL_TORQUE, lowByte, highByte};
    Dynamixel_DataWriter(hdynamixel, args, sizeof(args));
}

/**
 * @brief   Locks the EEPROM of the current motor until the next power cycle
 * @details Instruction register address: 0x2F (RAM)
 *
 *          Default value: 0x00
 * @param   hdynamixel pointer to a Dynamixel_HandleTypeDef structure that
 *          contains the configuration information for the motor
 * @return  None
 */
void Dynamixel_LockEEPROM(Dynamixel_HandleTypeDef* hdynamixel){
    /* Write data to motor. */
    uint8_t args[2] = {REG_LOCK_EEPROM, 1};
    Dynamixel_DataWriter(hdynamixel, args, sizeof(args));
}

/**
 * @brief   Sets a quantity proportional to the minimum current supplied to the
 *          motor during operation
 * @details Units are not specified in the datasheet, and therefore this
 *          function is not entirely useful without sufficient testing
 *
 *          Low byte at address 0x30 and high byte at address 0x31
 *
 *          Instruction register address: 0x30
 *
 *          Default value: 0x0020 (maximum: 0x3FF)
 * @param   hdynamixel pointer to a Dynamixel_HandleTypeDef structure that
 *          contains the configuration information for the motor
 * @param   punch for now, arguments in range [0, 1023] are valid
 * @return  None
 */
void Dynamixel_SetPunch(Dynamixel_HandleTypeDef* hdynamixel, float punch){
    /* Evaluate argument validity. */
    if((punch < MIN_PUNCH) || (punch > MAX_PUNCH)){
        if(punch < MIN_PUNCH){
            punch = MIN_PUNCH;
        }
        else{
            punch = MAX_PUNCH;
        }
    }

    /* Translate the punch into a 10-bit number. */
    uint16_t normalized_value = punch;

    uint8_t lowByte = (uint8_t)(normalized_value & 0xFF); // Low byte of punch
    uint8_t highByte = (uint8_t)((normalized_value >> 8) & 0xFF); // High byte of punch

    /* Write data to motor. */
    uint8_t args[3] = {REG_PUNCH, lowByte, highByte};
    Dynamixel_DataWriter(hdynamixel, args, sizeof(args));
}

/**
 * @}
 */
/* end DynamixelProtocolV1_Public_Functions_Setters */




/*****************************************************************************/
/*  Getter functions                                                         */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/
/** @defgroup DynamixelProtocolV1_Public_Functions_Getters Getters
 *  @brief    Register-reading functions
 *
 * # Getter functions #
 *
 * This subsection provides a set of functions which provide interfaces for
 * reading motor register values.
 *
 * @ingroup DynamixelProtocolV1_Public_Functions
 * @{
 */

/**
 * @brief   Reads the angular position of the motor in degrees
 * @details Reads addresses 0x24 and 0x25 in the motors RAM to see what the
 *          current position of the motor is. The results are written to
 *          `hdynamixel -> _lastPosition`, and
 *          `hdynamixel -> _lastReadIsValid` is set if the
 *          checksum is verified, and is cleared otherwise
 * @param   hdynamixel pointer to a Dynamixel_HandleTypeDef structure that
 *          contains the configuration information for the motor
 * @return  None
 */
void Dynamixel_GetPosition(Dynamixel_HandleTypeDef* hdynamixel){
    /* Read data from motor. */
    uint16_t retVal = Dynamixel_DataReader(hdynamixel, REG_CURRENT_POSITION, 2);

    /* Parse data and write it into motor handle. */
    if(hdynamixel->_lastReadIsValid){
        if(hdynamixel -> _motorType == AX12ATYPE){
            hdynamixel -> _lastPosition = (float)(retVal * 300 / 1023.0);
        }
        else if(hdynamixel -> _motorType == MX28TYPE){
            hdynamixel -> _lastPosition = (float)(retVal * 300 / 4095.0);
        }
    }
    else{
        hdynamixel -> _lastPosition = INFINITY;
    }
}

/**
 * @brief   Reads the angular velocity of the motor in RPM
 * @details Reads addresses 0x26 and 0x27 in the motor RAM to see what the
 *          current velocity of the motor is. The results are written to
 *          `hdynamixel -> _lastVelocity`, and
 *          `hdynamixel -> _lastReadIsValid` is set if the
 *          checksum is verified, and is cleared otherwise
 * @param   hdynamixel pointer to a Dynamixel_HandleTypeDef structure that
 *          contains the configuration information for the motor
 * @return  None
 */
void Dynamixel_GetVelocity(Dynamixel_HandleTypeDef* hdynamixel){
    /* Read data from motor. */
    uint16_t retVal = Dynamixel_DataReader(hdynamixel, REG_CURRENT_VELOCITY, 2);

    /* Parse data and write it into motor handle. */
    uint16_t modifier;
    if(hdynamixel->_lastReadIsValid){
        if(hdynamixel -> _isJointMode){
            modifier = 1023;
        }
        else{
            modifier = 2047;
        }

        if(hdynamixel -> _motorType == AX12ATYPE){
            hdynamixel -> _lastVelocity = (float)(retVal / modifier * AX12A_MAX_VELOCITY);
        }
        else if(hdynamixel -> _motorType == MX28TYPE){
            hdynamixel -> _lastVelocity = (float)(retVal / modifier * MX28_MAX_VELOCITY);
        }
    }
    else{
        hdynamixel -> _lastVelocity = INFINITY;
    }
}

/**
 * @brief   Reads the "load", the percentage of the maximum torque the motor is
 *          exerting
 * @details Reads addresses 0x28 and 0x29 in the motor RAM to see what the
 *          current load is. Load is a percentage of the maximum torque. A
 *          value  of 0-1023 gets translated into a counterclockwise load
 *          (positive), and a value of 1024-2047 gets translated into a
 *          clockwise load (negative). The results are written to
 *          `hdynamixel -> _lastLoad`, and
 *          `hdynamixel -> _lastReadIsValid` is set if the
 *          checksum is verified, and is cleared otherwise
 * @param   hdynamixel pointer to a Dynamixel_HandleTypeDef structure that
 *          contains the configuration information for the motor
 * @return  None
 */
void Dynamixel_GetLoad(Dynamixel_HandleTypeDef* hdynamixel){
    /* Read data from motor. */
    uint16_t retVal = Dynamixel_DataReader(hdynamixel, REG_CURRENT_LOAD, 2);

    /* Parse data and write it into motor handle. */
    if(hdynamixel->_lastReadIsValid){
        uint8_t isNegative = (retVal >> 9) & 0x1;
        if(retVal > 1023){
            retVal = retVal - 1023;
        }

        float retValf = (float)(retVal / 1023.0 * 100.0);
        if(isNegative){
            retValf *= -1;
        }
        hdynamixel -> _lastLoad = retValf;
    }
    else{
        hdynamixel -> _lastVelocity = INFINITY;
    }
}

/**
 * @brief   Reads the motor supply voltage
 * @details Reads address 0x2A in the motor RAM to see what the current voltage
 *          is. The value retrieved from motor is 10 times the actual voltage.
 *          The results are written to `hdynamixel -> _lastLoad`,
 *          and `hdynamixel -> _lastReadIsValid` is set if the
 *          checksum is verified, and is cleared otherwise
 * @param   hdynamixel pointer to a Dynamixel_HandleTypeDef structure that
 *          contains the configuration information for the motor
 * @return  The voltage in volts if the last read is valid, otherwise
 *          INFINITY
 */
float Dynamixel_GetVoltage(Dynamixel_HandleTypeDef* hdynamixel){
    /* Read data from motor. */
    uint8_t retVal = (uint8_t)Dynamixel_DataReader(hdynamixel, REG_CURRENT_VOLTAGE, 1);

    if(hdynamixel->_lastReadIsValid){
        return((float)(retVal / 10.0));
    }
    return INFINITY;
}

/**
 * @brief   Reads the internal motor temperature
 * @details Reads address 0x2B in the motor RAM to see what the current
 *          temperature is inside the motor. Results are in degrees Celsius
 * @param   hdynamixel pointer to a Dynamixel_HandleTypeDef structure that
 *          contains the configuration information for the motor
 * @return  The internal motor temperature in degrees Celsius
 */
uint8_t Dynamixel_GetTemperature(Dynamixel_HandleTypeDef* hdynamixel){
    /* Read data from motor straight into the motor handle. */
    return((uint8_t)Dynamixel_DataReader(hdynamixel, REG_CURRENT_TEMPERATURE, 1));
}

/**
 * @brief   Used to tell if a command sent was written to motor registers
 * @details Can also be used to see if the instruction in the motor register
 *          has been executed. If the ACTION command is executed, the value
 *          read is changed to 0
 *
 *          Read address: 0x2C
 *
 *          Default value: 0x00
 * @param   hdynamixel pointer to a Dynamixel_HandleTypeDef structure that
 *          contains the configuration information for the motor
 * @return  1 if there are commands transmitted by REG_WRITE, 0 otherwise
 */
bool Dynamixel_IsRegistered(Dynamixel_HandleTypeDef* hdynamixel){
    /* Return the data read from the motor. */
    return((bool)Dynamixel_DataReader(hdynamixel, REG_REGISTERED, 1));
}

/**
 * @brief   Indicates whether the motor is in motion
 * @details Reads the 0x2E address in motor RAM to see if motor is moving.
 * @param   hdynamixel pointer to a Dynamixel_HandleTypeDef structure that
 *          contains the configuration information for the motor
 * @return  1 if moving, otherwise 0
 */
bool Dynamixel_IsMoving(Dynamixel_HandleTypeDef* hdynamixel){
    /* Return the data from the motor. */
    return((uint8_t)Dynamixel_DataReader(hdynamixel, REG_MOVING, 1));
}

/**
 * @brief   Indicates whether the motor is operating in joint mode or wheel
 *          mode
 * @details Reads the CW (addr: 0x06) and CCW (addr: 0x08) angle limits. If
 *          both are 0, motor is in wheel mode and can spin indefinitely.
 *          Otherwise, motor is in joint mode and has angle limits set
 * @param   hdynamixel pointer to a Dynamixel_HandleTypeDef structure that
 *          contains the configuration information for the motor
 *
 * @return  1 if in joint mode, 0 if in wheel mode
 */
bool Dynamixel_IsJointMode(Dynamixel_HandleTypeDef* hdynamixel){
    /* Read data from motor. */
    uint16_t retValCW = Dynamixel_DataReader(hdynamixel, REG_CW_ANGLE_LIMIT, 2);
    uint16_t retValCCW = Dynamixel_DataReader(hdynamixel, REG_CCW_ANGLE_LIMIT, 2);

    /* Parse data and write it into motor handle. */
    hdynamixel -> _isJointMode = (bool)((retValCW | retValCCW) != 0);

    /* Return. */
    return(hdynamixel -> _isJointMode);
}

/**
  * @}
  */
/* end DynamixelProtocolV1_Public_Functions_Getters */



/*****************************************************************************/
/*  Library configuration functions                                          */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/
/**
 * @defgroup DynamixelProtocolV1_Public_Functions_Library_Configuration  \
 *           Library Configuration
 * @brief    Library configuration functions
 *
 * # Library configuration functions #
 *
 * This subsection provides a set of functions which provide interfaces for
 * configuring certain parts of the library. Right now, the only thing that
 * can be changed is the I/O mode (polling, interrupt-based, or DMA-based).
 *
 * @ingroup DynamixelProtocolV1_Public_Functions
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
/* DynamixelProtocolV1_Public_Functions_Library_Configuration */




/*****************************************************************************/
/*  Low-level transmission and reception functions                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/
/**
 * @defgroup DynamixelProtocolV1_Public_Functions_LL_TX_RX Transmission and \
 *           reception
 * @brief    Low-level transmission and reception functions
 *
 * # Low-level transmission and reception functions  #
 *
 * This subsection provides a set of functions which provide flexible
 * interfaces for transferring packets between motors and the MCU.
 *
 * This driver uses polled I/O, interrupt-based I/O, or DMA-based I/O
 * depending on the value of IOType. The user is responsible for
 * ensuring their system configuration is appropriate when using
 * interrupt-based or DMA-based I/O. For example, the interrupt-based
 * mode assumes that the user has enabled interrupts for the UART
 * module corresponding to this motor, and similarly, the DMA-based
 * mode assumes that a DMA channel has been allocated. Both of these
 * non-blocking modes also assumes the user is calling from within the
 * context of a FreeRTOS thread, and that the callback function has
 * been implemented to unblock the thread using task notifications.
 *
 * @ingroup DynamixelProtocolV1_Public_Functions
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
  * @}
  */
/* DynamixelProtocolV1_Public_Functions_LL_TX_RX */


/*****************************************************************************/
/*  Other motor instruction functions                                        */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/
/**
 * @defgroup DynamixelProtocolV1_Public_Functions_Other Other motor \
 *           instruction functions
 * @brief    Other motor instruction functions
 *
 * # Other motor instruction functions  #
 *
 * This subsection provides a set of functions which implement certain
 * instructions (motor commands), but their uses are rather niche compared
 * to the regular reading and writing functions.
 *
 * @ingroup DynamixelProtocolV1_Public_Functions
 * @{
 */

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

    // Set data direction
    Dynamixel_BusDirTX(hdynamixel -> _dataDirPort, hdynamixel -> _dataDirPinNum);

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

    // Set data direction for transmit
    Dynamixel_BusDirTX(hdynamixel -> _dataDirPort, hdynamixel -> _dataDirPinNum);

    // Transmit
    Dynamixel_GenericTransmit(hdynamixel, arr, sizeof(arr));

    // Receive
    Dynamixel_GenericReceive(hdynamixel, arr, sizeof(arr));

    /* Set data direction for receive. */
    Dynamixel_BusDirRX(hdynamixel -> _dataDirPort, hdynamixel -> _dataDirPinNum);

    /* Receive. */
    if(HAL_UART_Receive(hdynamixel -> _UART_Handle, arr, 6, RECEIVE_TIMEOUT)){
        return(arr[2]);
    }
    else{
        return -1;
    }
}

/**
  * @}
  */
/* DynamixelProtocolV1_Public_Functions_Other */




/*****************************************************************************/
/*  Setup functions                                                          */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/
/**
 * @defgroup DynamixelProtocolV1_Public_Functions_Setup Setup functions
 * @brief    Setup functions
 *
 * # Setup functions  #
 *
 * This subsection provides a set of functions which implement functions
 * related to setup (data structure initialization, etc) and resetting.
 *
 * @ingroup DynamixelProtocolV1_Public_Functions
 * @{
 */

/**
 * @brief   Initializes a motor handle
 * @param 	hdynamixel pointer to a Dynamixel_HandleTypeDef structure that
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
    hdynamixel -> _motorType = motorType;		// Identifies the type of actuator; used in certain functions
    hdynamixel -> _ID = ID; 					// Motor ID (unique or global)
    hdynamixel -> _lastReadIsValid = false;		// By default, don't trust the "_last*" fields below until their integrity is vouched for by the DataReader
    hdynamixel -> _lastPosition = -1; 			// In future, could initialize this accurately
    hdynamixel -> _lastVelocity = -1; 			// In future, could initialize this accurately
    hdynamixel -> _lastLoad = -1; 				// In future, could initialize this accurately
    hdynamixel -> _isJointMode = 1; 			// In future, could initialize this accurately
    hdynamixel -> _UART_Handle = UART_Handle; 	// For UART TX and RX
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
 * @param 	hdynamixel pointer to a Dynamixel_HandleTypeDef structure that
 *          contains the configuration information for the motor
 * @return  None
 */
void Dynamixel_Reset(Dynamixel_HandleTypeDef* hdynamixel){
    uint8_t arrTransmit[6];

    /* Do assignments and computations. */
    arrTransmit[0] = 0xff; // Obligatory bytes for starting communication
    arrTransmit[1] = 0xff; // Obligatory bytes for starting communication
    arrTransmit[2] = hdynamixel -> _ID; // Motor ID
    arrTransmit[3] = 2; // Length of message minus the obligatory bytes
    arrTransmit[4] = INST_RESET; // Reset instruction
    arrTransmit[5] = Dynamixel_ComputeChecksum(arrTransmit, 6);

    /* Set data direction. */
    Dynamixel_BusDirTX(hdynamixel -> _dataDirPort, hdynamixel -> _dataDirPinNum);

    /* Transmit. */
    HAL_UART_Transmit(hdynamixel -> _UART_Handle, arrTransmit, 6, TRANSMIT_TIMEOUT);
    hdynamixel -> _ID = DEFAULT_ID;
}

/**
  * @}
  */
/* DynamixelProtocolV1_Public_Functions_Setup */




/*****************************************************************************/
/*  Interfaces for previously-defined functions                              */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/
/**
 * @defgroup DynamixelProtocolV1_Public_Functions_Interfaces Interfaces for \
 *           previously-defined functions
 * @brief    Interfaces for previously-defined functions
 *
 * # Interfaces for previously-defined functions #
 *
 * This subsection provides a set of functions which implement functions
 * which call previously-defined functions in order to accomplish specific
 * tasks.
 *
 * @ingroup DynamixelProtocolV1_Public_Functions
 * @{
 */

/**
 * @brief   Sets the control registers such that the rotational angle of the
 *          motor is not bounded
 * @details When the angle limits are both set to 0, then motor will attempt to
 *          rotate with maximum velocity. To prevent undesired behaviour, the
 *          goal velocity should be set right after calling this function
 * @param 	hdynamixel pointer to a Dynamixel_HandleTypeDef structure that
 *          contains the configuration information for the motor
 * @param   goalVelocity the desired velocity in RPM to use when entering wheel
 *          mode
 * @return  None
 */
void Dynamixel_EnterWheelMode(Dynamixel_HandleTypeDef* hdynamixel, float goalVelocity){
    Dynamixel_SetCWAngleLimit(hdynamixel, 0);
    Dynamixel_SetCCWAngleLimit(hdynamixel, 0);
    hdynamixel -> _isJointMode = 0;
    Dynamixel_SetGoalVelocity(hdynamixel, goalVelocity);
}

/**
 * @brief  Sets the control registers such that the rotational angle of the
 *         motor is constrained between the default values
 * @param  hdynamixel pointer to a Dynamixel_HandleTypeDef structure that
 *         contains the configuration information for the motor
 * @return None
 */
void Dynamixel_EnterJointMode(Dynamixel_HandleTypeDef* hdynamixel){
    Dynamixel_SetCWAngleLimit(hdynamixel, MIN_ANGLE);
    Dynamixel_SetCCWAngleLimit(hdynamixel, MAX_ANGLE);
    hdynamixel -> _isJointMode = 1;
}

/**
 * @}
 */
/* DynamixelProtocolV1_Public_Functions_Interfaces */




/**
 * @defgroup DynamixelProtocolV1_Private_Functions Dynamixel Private Functions
 * @brief    Functions only accessible from within the library
 * @ingroup  DynamixelProtocolV1
 * @{
 */

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
    uint32_t notification;
    BaseType_t status;
    bool retval = true;

    // Set data direction for receive
    Dynamixel_BusDirRX(hdynamixel -> _dataDirPort, hdynamixel -> _dataDirPinNum);

    switch(IOType) {
        case IO_DMA:
            HAL_UART_Receive_DMA(hdynamixel -> _UART_Handle, arr, arrSize);

            status = xTaskNotifyWait(0, NOTIFIED_FROM_RX_ISR, &notification, MAX_DELAY_TIME);

            if(status != pdTRUE || !CHECK_NOTIFICATION(notification, NOTIFIED_FROM_RX_ISR)){
                retval = false;
            }
            break;
        case IO_POLL:
            HAL_UART_Receive(hdynamixel -> _UART_Handle, arr, arrSize, RECEIVE_TIMEOUT);
            break;
        case IO_IT:
            HAL_UART_Receive_IT(hdynamixel -> _UART_Handle, arr, arrSize);

            status = xTaskNotifyWait(0, NOTIFIED_FROM_RX_ISR, &notification, MAX_DELAY_TIME);

            if(status != pdTRUE || !CHECK_NOTIFICATION(notification, NOTIFIED_FROM_RX_ISR)){
                retval = false;
            }
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
    uint32_t notification;
    BaseType_t status;
    bool retval = true;

    // Set data direction for transmit
    Dynamixel_BusDirTX(hdynamixel -> _dataDirPort, hdynamixel -> _dataDirPinNum);

    switch(IOType) {
        case IO_DMA:
            HAL_UART_Transmit_DMA(hdynamixel -> _UART_Handle, arr, arrSize);

            status = xTaskNotifyWait(0, NOTIFIED_FROM_TX_ISR, &notification, MAX_DELAY_TIME);

            if(status != pdTRUE || !CHECK_NOTIFICATION(notification, NOTIFIED_FROM_TX_ISR)){
                retval = false;
            }
            break;
        case IO_POLL:
            HAL_UART_Transmit(hdynamixel -> _UART_Handle, arr, arrSize, TRANSMIT_TIMEOUT);
            break;
        case IO_IT:
            HAL_UART_Transmit_IT(hdynamixel -> _UART_Handle, arr, arrSize);

            status = xTaskNotifyWait(0, NOTIFIED_FROM_TX_ISR, &notification, MAX_DELAY_TIME);

            if(status != pdTRUE || !CHECK_NOTIFICATION(notification, NOTIFIED_FROM_TX_ISR)){
                retval = false;
            }

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
/* DynamixelProtocolV1_Private_Functions */
