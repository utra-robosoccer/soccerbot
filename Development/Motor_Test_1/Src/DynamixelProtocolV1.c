/* This file holds common functional code for the AX12A library and MX28 library. It is
 * somewhat generic in that any Dynamixel actuator using protocol version 1.0 should
 * be able to be integrated with little effort, as the instructions and register
 * addresses are very similar for all actuators using this protocol version.
 *
 * Author: Tyler
 */

/********************************* Includes ************************************/
#include "DynamixelProtocolV1.h"
#include "main.h"

/******************************* Public Variables *******************************/
/* Buffer for data received from motors. */
uint8_t arrReceive[NUM_MOTORS][BUFF_SIZE_RX] = { { 0 } };

/* Bytes to be transmitted to motors are written to this array. */
uint8_t arrTransmit[NUM_MOTORS + 1][TX_PACKET_SIZE] = { { 0xFF, 0xFF, 0xFE,
        0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00 }, { 0xFF, 0xFF, 1, 0x00,
INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00 }, { 0xFF, 0xFF, 2, 0x00,
INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00 }, { 0xFF, 0xFF, 3, 0x00,
INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00 }, { 0xFF, 0xFF, 4, 0x00,
INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00 }, { 0xFF, 0xFF, 5, 0x00,
INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00 }, { 0xFF, 0xFF, 6, 0x00,
INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00 }, { 0xFF, 0xFF, 7, 0x00,
INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00 }, { 0xFF, 0xFF, 8, 0x00,
INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00 }, { 0xFF, 0xFF, 9, 0x00,
INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00 }, { 0xFF, 0xFF, 10, 0x00,
INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00 }, { 0xFF, 0xFF, 11, 0x00,
INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00 }, { 0xFF, 0xFF, 12, 0x00,
INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00 }, { 0xFF, 0xFF, 13, 0x00,
INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00 }, { 0xFF, 0xFF, 14, 0x00,
INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00 }, { 0xFF, 0xFF, 15, 0x00,
INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00 }, { 0xFF, 0xFF, 16, 0x00,
INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00 }, { 0xFF, 0xFF, 17, 0x00,
INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00 }, { 0xFF, 0xFF, 18, 0x00,
INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00 } };

/* Sync-writing position uses this array. */
uint8_t arrSyncWritePosition[NUM_UARTS][23] = { { 0xFF, 0xFF, 0xFE, 0x00,
INST_SYNC_WRITE, REG_GOAL_POSITION, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, { 0xFF,
        0xFF, 0xFE, 0x00, INST_SYNC_WRITE, REG_GOAL_POSITION, 0x02, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00 }, { 0xFF, 0xFF, 0xFE, 0x00, INST_SYNC_WRITE,
REG_GOAL_POSITION, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, { 0xFF, 0xFF, 0xFE, 0x00,
INST_SYNC_WRITE, REG_GOAL_POSITION, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, { 0xFF,
        0xFF, 0xFE, 0x00,
        INST_SYNC_WRITE, REG_GOAL_POSITION, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } };

/************************ Private Function Prototypes **************************/
inline uint8_t Dynamixel_ComputeChecksum(uint8_t *arr, int length);

/******************************** Functions ************************************/
/*******************************************************************************/
/*	Setter helper functions													   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*******************************************************************************/
void Dynamixel_SetID(Dynamixel_HandleTypeDef* hdynamixel, uint8_t ID) {
    /* Sets the ID (identification number) for the current motor.
     * Note that the instruction will be broadcasted using the current ID.
     * As such, if the ID is not known, the motor ID should be initialized
     * to the broadcast ID (0xFE) in the Dynamixel_Init function.
     *
     * Instruction register address: 0x03 (EEPROM)
     * Default value: 1
     *
     * Arguments: hdynamixel, the motor handle
     * 			  ID, the number between 0 and 252 or equal to 254 to identify the motor.
     * 			  	If 0xFE (254), any messages broadcasted to that ID will be broadcasted
     * 			  	to all motors.
     *
     * Returns: none
     */

    /* Compute validity. */
    if ((ID == 253) || (ID == 255)) {
        ID = DEFAULT_ID;
    }

    /* Write data to motor. */
    uint8_t args[2] = { REG_ID, ID }; //REG_ID is motor id register, 0x03
    Dynamixel_DataWriter(hdynamixel, args, sizeof(args));

    /* Save ID in handle */
    hdynamixel->_ID = ID;
}

void Dynamixel_SetBaudRate(Dynamixel_HandleTypeDef* hdynamixel, long baud) {
    /* Sets the baud rate of a particular motor. Register address is 0x04 in motor EEPROM.
     *
     * Instruction register address: 0x04 (EEPROM)
     * Default value: 0x01
     *
     * Arguments: hdynamixel, the motor handle
     * 			  baud, the baud rate. Arguments in range [7844, 1000000] are valid
     *
     * Returns: none
     */

    uint8_t baudArg = 0x01; // Default to 1 Mbps

    if (hdynamixel->_motorType == AX12ATYPE) {
        /* Set _baud equal to the hex code corresponding to baud. Default to 1 Mbps. */
        if (baud > 0) {
            /* Valid for baud in range [7844, 1000000]. Will be converted to 8-bit resolution. */
            baudArg = (uint8_t) ((2000000 / baud) - 1);
        } else {
            /* Default to 1 Mbps. */
            baudArg = AX12A_DEFAULT_BAUD_RATE;
        }
    } else if (hdynamixel->_motorType == MX28TYPE) {
        /* Set _baud equal to the hex code corresponding to baud. Default to 1 Mbps. */
        if (baud >= 9600 && baud <= 3500000) {
            if (baud >= 2250000) {
                if (baud < 2500000) {
                    baudArg = 250;
                } else if (baud < 3000000) {
                    baudArg = 251;
                } else {
                    baudArg = 252;
                }
            } else {
                baudArg = (uint8_t) ((2000000 / baud) - 1);
            }
        } else {
            /* Default to 1000000 symbols/s (MX28_DEFAULT_BAUD_RATE is not to be used for our application) */
            baudArg = 0x01;
        }
    }

    /* Write data to motor. */
    uint8_t args[2] = { REG_BAUD_RATE, baudArg };
    Dynamixel_DataWriter(hdynamixel, args, sizeof(args));
}

void Dynamixel_SetReturnDelayTime(Dynamixel_HandleTypeDef* hdynamixel,
        uint16_t microSec) {
    /* Sets the time, in microseconds, that the motor should wait before returning a status packet.
     *
     * Instruction register address: 0x05(EEPROM)
     * Default value: 250 (0xFA)
     *
     * Arguments: hdynamixel, the motor handle
     * 			  microSec, the time in microseconds to delay. Arguments in range [2, 508] are valid. Default: 500
     *
     * Returns: none
     */

    /* Compute validity. */
    uint8_t motor_data;
    if ((microSec < 2) || (microSec > 508)) {
        motor_data = DEFAULT_RETURN_DELAY;
    } else {
        motor_data = (uint8_t) (microSec / 2);
    }

    /* Write data to motor. */
    uint8_t args[2] = { REG_RETURN_DELAY_TIME, motor_data };
    Dynamixel_DataWriter(hdynamixel, args, sizeof(args));
}

void Dynamixel_SetCWAngleLimit(Dynamixel_HandleTypeDef* hdynamixel,
        float minAngle) {
    /* Sets the clockwise angle limit for the current motor.
     * Register 0x06 in EEPROM for low byte, 0x07 in EEPROM for high byte
     *
     * Instruction register address: 0x06 (EEPROM)
     * Default value: 0x0000
     *
     * Arguments: hdynamixel, the motor handle
     * 			  minAngle, the minimum angle for all motor operations. Arguments between 0 and 300 are valid
     *
     * If maxAngle for CCW angle limit is 0 AND minAngle for CW angle limit is 0,
     * then motor is in wheel mode where it can continuously rotate. Otherwise, motor is in joint mode where its
     * motion is constrained between the set bounds.
     *
     * Returns: none
     */

    /* Initialize local variable to default value. */
    uint16_t normalized_value = DEFAULT_CW_ANGLE_LIMIT;

    /* Evaluate argument validity. Optimize for edge case minAngle == MIN_ANGLE. */
    if ((minAngle > MIN_ANGLE) && (minAngle <= MAX_ANGLE)) {

        /* Translate the angle from degrees into a 10-bit number. */
        if (hdynamixel->_motorType == AX12ATYPE) {
            normalized_value = (uint16_t) (minAngle / MAX_ANGLE * 1023);
        } else if (hdynamixel->_motorType == MX28TYPE) {
            normalized_value = (uint16_t) (minAngle / MAX_ANGLE * 4095);
        }
    }

    uint8_t lowByte = (uint8_t) (normalized_value & 0xFF); // Low byte of CW angle limit
    uint8_t highByte = (uint8_t) ((normalized_value >> 8) & 0xFF); // High byte of CW angle limit

    /* Write data to motor. */
    uint8_t args[3] = { REG_CW_ANGLE_LIMIT, lowByte, highByte };
    Dynamixel_DataWriter(hdynamixel, args, sizeof(args));
}

void Dynamixel_SetCCWAngleLimit(Dynamixel_HandleTypeDef* hdynamixel,
        float maxAngle) {
    /* Sets the counter-clockwise angle limit for the current motor.
     * Register 0x08 in EEPROM for low byte, 0x09 in EEPROM for high byte
     *
     * Instruction register address: 0x08 (EEPROM)
     * Default value: 0x03FF
     *
     * Arguments: hdynamixel, the motor handle
     * 			  maxAngle, the maximum angle for all motor operations. Arguments between 0 and 300 are valid.
     *
     * If maxAngle for CCW angle limit is 0 AND minAngle for CW angle limit is 0,
     * then motor is in wheel mode where it can continuously rotate. Otherwise, motor is in joint mode where its
     * motion is constrained between the set bounds.
     *
     * Returns: none
     */

    /* Initialize local variable to default value. */
    uint16_t normalized_value = AX12A_DEFAULT_CCW_ANGLE_LIMIT; // default
    if (hdynamixel->_motorType == AX12ATYPE) {
        normalized_value = AX12A_DEFAULT_CCW_ANGLE_LIMIT;
    } else if (hdynamixel->_motorType == MX28TYPE) {
        normalized_value = MX28_DEFAULT_CCW_ANGLE_LIMIT;
    }

    /* Evaluate argument validity. Optimize for edge case maxAngle = MAX_ANGLE. */
    if ((maxAngle >= MIN_ANGLE) && (maxAngle < MAX_ANGLE)) {

        /* Translate the angle from degrees into a 10-bit number. */
        if (hdynamixel->_motorType == AX12ATYPE) {
            normalized_value = (uint16_t) (maxAngle / MAX_ANGLE * 1023);
        } else if (hdynamixel->_motorType == MX28TYPE) {
            normalized_value = (uint16_t) (maxAngle / MAX_ANGLE * 4095);
        }

    }

    uint8_t lowByte = (uint8_t) (normalized_value & 0xFF); // Low byte of CCW angle limit
    uint8_t highByte = (uint8_t) ((normalized_value >> 8) & 0xFF); // High byte of CCW angle limit

    /* Write data to motor. */
    uint8_t args[3] = { REG_CCW_ANGLE_LIMIT, lowByte, highByte };
    Dynamixel_DataWriter(hdynamixel, args, sizeof(args));
}

// TODO: Test
void Dynamixel_SetHighestVoltageLimit(Dynamixel_HandleTypeDef* hdynamixel,
        float highestVoltage) {
    /* Sets the highest operating voltage limit for the current motor.
     *
     * Instruction register address: 0x0C (EEPROM)
     * Default value: 140 (0xBE)
     *
     * Arguments: hdynamixel, the motor handle
     * 			  highestVoltage, the highest operating voltage in volts
     *
     * Returns: none
     */

    /* Declare local variable. Initialize to default value. */
    uint8_t high_voltage_data = AX12A_DEFAULT_HIGHEST_VOLTAGE_LIMIT; // minimum is safer to default to

    if (hdynamixel->_motorType == AX12ATYPE) {
        high_voltage_data = AX12A_DEFAULT_HIGHEST_VOLTAGE_LIMIT;
    } else if (hdynamixel->_motorType == MX28TYPE) {
        high_voltage_data = MX28_DEFAULT_HIGHEST_VOLTAGE_LIMIT;
    }

    /* Evaluate argument validity and translate into motor data. Optimize for highestVoltage = MAX_VOLTAGE. */
    if ((highestVoltage >= MIN_VOLTAGE) && (highestVoltage < MAX_VOLTAGE)) {
        high_voltage_data = (uint8_t) (highestVoltage * 10);
    }

    /* Write data to motor. */
    uint8_t args[2] = { REG_HIGH_VOLTAGE_LIMIT, high_voltage_data };
    Dynamixel_DataWriter(hdynamixel, args, sizeof(args));
}

// TODO: Test
void Dynamixel_SetLowestVoltageLimit(Dynamixel_HandleTypeDef* hdynamixel,
        float lowestVoltage) {
    /* Sets the lowest operating voltage limit for the current motor.
     *
     * Instruction register address: 0x0D (EEPROM)
     * Default value: 60 (0x3C)
     *
     * Arguments: hdynamixel, the motor handle
     * 			  lowestVoltage, the lowest operating voltage in volts
     *
     * Returns: none
     */

    /* Declare local variable. Initialize to default value. */
    uint8_t low_voltage_data = DEFAULT_LOW_VOLTAGE_LIMIT;

    /* Evaluate argument validity and translate into motor data. Optimize for lowestVoltage = MIN_VOLTAGE. */
    if ((lowestVoltage > MIN_VOLTAGE) && (lowestVoltage <= MAX_VOLTAGE)) {

        /* Translate into format that motor can understand. */
        low_voltage_data = (uint8_t) (lowestVoltage * 10);
    }

    /* Write data to motor. */
    uint8_t args[2] = { REG_LOW_VOLTAGE_LIMIT, low_voltage_data };
    Dynamixel_DataWriter(hdynamixel, args, sizeof(args));
}

// TODO: Test
void Dynamixel_SetMaxTorque(Dynamixel_HandleTypeDef* hdynamixel,
        float maxTorque) {
    /* Sets the maximum torque limit for all motor operations.
     * Low byte is addr 0x0E in motor RAM, high byte is addr 0x0F in motor RAM.
     *
     * Register address: 0x0E (EEPROM)
     * Default value: 0x3FF
     *
     * Arguments: hdynamixel, the motor handle
     *			  maxTorque, the maximum torque as a percentage (max: 100). Gets converted to 10-bit number
     *
     * Returns: none
     */

    /* Initialize to default value. */
    uint16_t normalized_value = DEFAULT_MAXIMUM_TORQUE;

    /* Evaluate argument validity and optimize for edge case maxTorque = MAX_TORQUE. */
    if ((maxTorque >= MIN_TORQUE) && (maxTorque < MAX_TORQUE)) {

        /* Translate the input from percentage into a 10-bit number. */
        normalized_value = (uint16_t) (maxTorque / 100 * 1023);
    }

    uint8_t lowByte = normalized_value & 0xFF; // Low byte of max torque
    uint8_t highByte = (normalized_value >> 8) & 0xFF; // High byte of max torque

    /* Write data to motor. */
    uint8_t args[3] = { REG_MAX_TORQUE, lowByte, highByte };
    Dynamixel_DataWriter(hdynamixel, args, sizeof(args));
}

// TODO: Test
void Dynamixel_SetStatusReturnLevel(Dynamixel_HandleTypeDef* hdynamixel,
        uint8_t status_data) {
    /* Sets the conditions under which a status packet will be returned.
     *
     * Register address: 0x10 (EEPROM)
     * Default value: 0x02
     *
     * Arguments: hdynamixel, the motor handle
     * 			  status, 0 to return only on ping
     * 			  		  1 to return only for reads
     * 			  		  2 to return for all commands
     *
     * Returns: none
     */

    /* Evaluate argument validity. */
    if ((status_data < 0) || (status_data > 2)) {
        status_data = DEFAULT_STATUS_RETURN_LEVEL;
    }

    /* Write data to motor. */
    uint8_t args[2] = { REG_STATUS_RETURN_LEVEL, status_data };
    Dynamixel_DataWriter(hdynamixel, args, sizeof(args));
}

// TODO: Test
void Dynamixel_SetAlarmLED(Dynamixel_HandleTypeDef* hdynamixel,
        uint8_t alarm_LED_data) {
    /* Sets the conditions under which the motor LED will light up.
     *
     * Register address: 0x11 (EEPROM)
     * Default value: 0x24 (0x00100100 -> b7b6b5b4b3b2b1b0)
     *
     * Arguments: hdynamixel, the motor handle
     * 			  alarm_data, bit 7: no function
     * 			  			  bit 6: flash LED when an instruction error occurs
     * 			  			  bit 5: flash LED when current load cannot be controlled with the specified maximum torque
     * 			  			  bit 4: flash LED when the checksum of the transmitted packet is invalid
     * 			  			  bit 3: flash LED when the command is given beyond the range of usage
     * 			  			  bit 2: flash LED when the internal temperature exceeds the operating range
     * 			  			  bit 1: flash LED when goal position exceeds the CW angle limit or CCW angle limit
     * 			  			  bit 0: flash LED when applied voltage is out of operating range
     *
     * Several of these bits may be set simultaneously.
     *
     * Returns: none
     */

    /* Write data to motor. */
    uint8_t args[2] = { REG_ALARM_LED, alarm_LED_data };
    Dynamixel_DataWriter(hdynamixel, args, sizeof(args));
}

// TODO: Test
void Dynamixel_SetAlarmShutdown(Dynamixel_HandleTypeDef* hdynamixel,
        uint8_t alarm_shutdown_data) {
    /* Sets the conditions under which the motor will turn off its torque.
     *
     * Register address: 0x12 (EEPROM)
     * Default value: 0x24
     *
     * Arguments: hdynamixel, the motor handle
     * 			  alarm_data, bit 7: no function
     * 			  			  bit 6: torque off when an instruction error occurs
     * 			  			  bit 5: torque off when current load cannot be controlled with the specified maximum torque
     * 			  			  bit 4: torque off when the checksum of the transmitted packet is invalid
     * 			  			  bit 3: torque off when the command is given beyond the range of usage
     * 			  			  bit 2: torque off when the internal temperature exceeds the operating range
     * 			  			  bit 1: torque off when goal position exceeds the CW angle limit or CCW angle limit
     * 			  			  bit 0: torque off when applied voltage is out of operating range
     *
     * Several of these bits may be set simultaneously.
     *
     * Returns: none
     */

    /* Write data to motor. */
    uint8_t args[2] = { REG_ALARM_SHUTDOWN, alarm_shutdown_data };
    Dynamixel_DataWriter(hdynamixel, args, sizeof(args));
}

void Dynamixel_TorqueEnable(Dynamixel_HandleTypeDef* hdynamixel,
        uint8_t isEnabled) {
    /* Enables or disables torque for current motor.
     *
     * Instruction register address: 0x18 (RAM)
     * Default value: 0x00
     *
     * Arguments: hdynamixel, the motor handle
     * 			  isEnabled, if 1, then generates torque by impressing power to the motor
     * 			  			 if 0, then interrupts power to the motor to prevent it from generating torque
     *
     * Returns: none
     */

    /* Evaluate argument validity. */
    if ((isEnabled != 1) && (isEnabled != 0)) {
        isEnabled = DEFAULT_TORQUE_ENABLE;
    }

    /* Write data to motor. */
    uint8_t args[2] = { REG_TORQUE_ENABLE, isEnabled };
    Dynamixel_DataWriter(hdynamixel, args, sizeof(args));
}

void Dynamixel_LEDEnable(Dynamixel_HandleTypeDef* hdynamixel, uint8_t isEnabled) {
    /* Toggles the motor LED.
     *
     * Register address: 0x19
     * Default value: 0x00
     *
     * Arguments: hdynamixel, the motor handle
     * 			  isEnabled, 0 if LED off
     * 			  			 1 if LED on
     *
     * Returns: none
     */

    /* Evaluate argument validity. */
    if ((isEnabled != 1) && (isEnabled != 0)) {
        isEnabled = DEFAULT_LED_ENABLE;
    }

    /* Write data to motor. */
    uint8_t args[2] = { REG_LED_ENABLE, isEnabled };
    Dynamixel_DataWriter(hdynamixel, args, sizeof(args));
}

void Dynamixel_SetGoalPosition(Dynamixel_HandleTypeDef* hdynamixel,
        float goalAngle) {
    /* Takes a double between 0 and 300, encodes this position in an
     * upper and low hex byte pair (with a maximum of 1023 as defined in the AX-12
     * user manual), and sends this information (along with requisites) over UART.
     * Low byte is 0x1E in motor RAM, high byte is 0x1F in motor RAM.
     *
     * Instruction register address: 0x1E (RAM)
     * Default value: none
     *
     * Arguments: hdynamixel, the motor handle
     * 			  angle, the desired angular position. Arguments between 0 and 300 are valid
     *
     * Returns: none
     */

    /* Check for input validity. If input not valid, replace goalAngle with closest
     * valid value to ensure code won't halt. */
    if ((goalAngle < MIN_ANGLE) || (goalAngle > MAX_ANGLE)) {
        if (goalAngle > MIN_ANGLE) {
            goalAngle = MAX_ANGLE;
        } else {
            goalAngle = MIN_ANGLE;
        }
    }

    /* Translate the angle from degrees into a 10- or 12-bit number. */
    uint16_t normalized_value = 0;
    if (hdynamixel->_motorType == AX12ATYPE) {
        normalized_value = (uint16_t) (goalAngle / MAX_ANGLE * 1023);
    } else if (hdynamixel->_motorType == MX28TYPE) {
        normalized_value = (uint16_t) (goalAngle / MAX_ANGLE * 4095);
    } else {
        // Should NEVER reach here!
    }

    uint8_t lowByte = (uint8_t) (normalized_value & 0xFF); // Low byte of goal position
    uint8_t highByte = (uint8_t) ((normalized_value >> 8) & 0xFF); // High byte of goal position

    /* Write data to motor. */
    uint8_t args[3] = { REG_GOAL_POSITION, lowByte, highByte };
    Dynamixel_DataWriter(hdynamixel, args, sizeof(args));
}

void Dynamixel_SetGoalVelocity(Dynamixel_HandleTypeDef* hdynamixel,
        float goalVelocity) {
    /* Sets the goal velocity of the motor in RAM.
     * Low byte is 0x20 in motor RAM, high byte is 0x21 in motor RAM.
     *
     * Instruction register address: 0x20 (RAM)
     * Default value: none
     *
     * Arguments: hdynamixel, the motor handle
     * 			  velocity, the goal velocity in RPM. Arguments of 0-114 are valid when in joint mode.
     * 			  0 corresponds to MAX motion in joint mode, and minimum motion in wheel mode
     * 			  In wheel mode, negative arguments correspond to CW rotation
     *
     * Returns: none
     */

    /* Translate the position from RPM into a 10-bit number. */
    uint16_t normalized_value = 0;
    if (hdynamixel->_isJointMode) {

        /* Check for input validity. If input not valid, replace goalAngle with closest
         * valid value to ensure code won't halt. */
        if (goalVelocity != 0) {
            if (hdynamixel->_motorType == AX12ATYPE) {
                if ((goalVelocity < MIN_VELOCITY)
                        || (goalVelocity > AX12A_MAX_VELOCITY)) {
                    if (goalVelocity > MIN_VELOCITY) {
                        goalVelocity = AX12A_MAX_VELOCITY;
                    } else {
                        goalVelocity = MIN_VELOCITY;
                    }
                }
            } else if (hdynamixel->_motorType == MX28TYPE) {
                if ((goalVelocity < MIN_VELOCITY)
                        || (goalVelocity > MX28_MAX_VELOCITY)) {
                    if (goalVelocity > MIN_VELOCITY) {
                        goalVelocity = MX28_MAX_VELOCITY;
                    } else {
                        goalVelocity = MIN_VELOCITY;
                    }
                }
            }
        }
    }

    if (hdynamixel->_motorType == AX12ATYPE) {
        normalized_value =
                (uint16_t) (goalVelocity / AX12A_MAX_VELOCITY * 1023);
        if (goalVelocity < 0) {
            normalized_value = ((uint16_t) ((goalVelocity * -1)
                    / AX12A_MAX_VELOCITY * 1023)) | 0b000010000000000;
        }
    } else if (hdynamixel->_motorType == MX28TYPE) {
        normalized_value = (uint16_t) (goalVelocity / MX28_MAX_VELOCITY * 1023);
        if (goalVelocity < 0) {
            normalized_value = ((uint16_t) ((goalVelocity * -1)
                    / MX28_MAX_VELOCITY * 1023)) | 0b000010000000000;
        }
    }

    uint8_t lowByte = (uint8_t) (normalized_value & 0xFF); // Low byte of goal velocity
    uint8_t highByte = (uint8_t) ((normalized_value >> 8) & 0xFF); // High byte of goal velocity

    /* Write data to motor. */
    uint8_t args[3] = { REG_GOAL_VELOCITY, lowByte, highByte };
    Dynamixel_DataWriter(hdynamixel, args, sizeof(args));
}

void Dynamixel_SetGoalTorque(Dynamixel_HandleTypeDef* hdynamixel,
        float goalTorque) {
    /* Sets the goal torque for the current motor. Initial value is taken from 0x0E and 0x0F (max torque in EEPROM)
     * Low byte is 0x22 in motor RAM, high byte is 0x23 in motor RAM.
     *
     * Instruction register address: 0x22 (RAM)
     * Default value: ADDR14 (low byte) and ADDR15 (high byte)
     *
     * Arguments: hdynamixel, the motor handle
     * 			  goalTorque, the percentage of the maximum possible torque (max: 100). Gets converted into 10-bit number
     *
     * Returns: none
     */

    /* Translate the input from percentage into and 10-bit number. */
    uint16_t normalized_value = (uint16_t) (goalTorque / 100 * 1023);

    uint8_t lowByte = (uint8_t) (normalized_value & 0xFF); // Low byte of goal torque
    uint8_t highByte = (uint8_t) ((normalized_value >> 8) & 0xFF); // High byte of goal torque

    /* Write data to motor. */
    uint8_t args[3] = { REG_GOAL_TORQUE, lowByte, highByte };
    Dynamixel_DataWriter(hdynamixel, args, sizeof(args));
}

void Dynamixel_LockEEPROM(Dynamixel_HandleTypeDef* hdynamixel) {
    /* Locks the EEPROM of the current motor until the next power cycle.
     *
     * Instruction register address: 0x2F (RAM)
     * Default value: 0x00
     *
     * Arguments: hdynamixel, the motor handle
     *
     * Returns: none
     */

    /* Write data to motor. */
    uint8_t args[2] = { REG_LOCK_EEPROM, 1 };
    Dynamixel_DataWriter(hdynamixel, args, sizeof(args));
}

void Dynamixel_SetPunch(Dynamixel_HandleTypeDef* hdynamixel, float punch) {
    /* Sets a quantity proportional to the minimum current supplied to the motor during operation.
     * Units are not specified in datasheet, and therefore this function is not entirely useful without
     * sufficient testing.
     *
     * Low byte at address 0x30 and high byte at address 0x31
     *
     * Instruction register address: 0x30
     * Default value: 0x0020 (maximum: 0x3FF)
     *
     * Arguments: hdynamixel, the motor handle
     * 			  punch, for now, arguments in range [0, 1023] are valid
     *
     * Returns: none
     */

    /* Evaluate argument validity. */
    if ((punch < MIN_PUNCH) || (punch > MAX_PUNCH)) {
        if (punch < MIN_PUNCH) {
            punch = MIN_PUNCH;
        } else {
            punch = MAX_PUNCH;
        }
    }

    /* Translate the punch into a 10-bit number. */
    uint16_t normalized_value = punch;

    uint8_t lowByte = (uint8_t) (normalized_value & 0xFF); // Low byte of punch
    uint8_t highByte = (uint8_t) ((normalized_value >> 8) & 0xFF); // High byte of punch

    /* Write data to motor. */
    uint8_t args[3] = { REG_PUNCH, lowByte, highByte };
    Dynamixel_DataWriter(hdynamixel, args, sizeof(args));
}

/*******************************************************************************/
/*	Getter helper functions											  		   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*******************************************************************************/
void Dynamixel_GetPosition(Dynamixel_HandleTypeDef* hdynamixel) {
    /* Reads addresses 0x24 and 0x25 in the motors RAM to see what the current position
     * of the motor is.
     * Writes the results to hdynamixel -> _lastPosition
     *
     * Arguments: hdynamixel, the motor handle
     *
     * Returns: none
     */

    /* Read data from motor. */
    uint16_t retVal = Dynamixel_DataReader(hdynamixel, REG_CURRENT_POSITION, 2);

    /* Parse data and write it into motor handle. */
    if (hdynamixel->_motorType == AX12ATYPE) {
        hdynamixel->_lastPosition = (float) (retVal * 300 / 1023.0);
    } else if (hdynamixel->_motorType == MX28TYPE) {
        hdynamixel->_lastPosition = (float) (retVal * 300 / 4095.0);
    }
}

void Dynamixel_GetVelocity(Dynamixel_HandleTypeDef* hdynamixel) {
    /* Reads addresses 0x26 and 0x27 in the motor RAM to see what the current velocity
     * of the motor is.
     * Writes the results to hdynamixel -> _lastVelocity
     *
     * Arguments: hdynamixel, the motor handle
     *
     * Returns: none
     */

    /* Read data from motor. */
    uint16_t retVal = Dynamixel_DataReader(hdynamixel, REG_CURRENT_VELOCITY, 2);

    /* Parse data and write it into motor handle. */
    uint16_t modifier;
    if (hdynamixel->_isJointMode) {
        modifier = 1023;
    } else {
        modifier = 2047;
    }

    if (hdynamixel->_motorType == AX12ATYPE) {
        hdynamixel->_lastVelocity = (float) (retVal / modifier
                * AX12A_MAX_VELOCITY);
    } else if (hdynamixel->_motorType == MX28TYPE) {
        hdynamixel->_lastVelocity = (float) (retVal / modifier
                * MX28_MAX_VELOCITY);
    }
}

void Dynamixel_GetLoad(Dynamixel_HandleTypeDef* hdynamixel) {
    /* Reads addresses 0x28 and 0x29 in the motor RAM to see what the current
     * load is. Load is a percentage of the maximum torque. Value of 0-1023
     * gets translated into a counterclockwise load (positive), and value of 1024-2047
     * gets translated into a clockwise load (negative).
     *
     * Writes the torque percentage to hdynamixel -> _lastLoad
     *
     * Arguments: hdynamixel, the motor handle
     *
     * Returns: none
     */

    /* Read data from motor. */
    uint16_t retVal = Dynamixel_DataReader(hdynamixel, REG_CURRENT_LOAD, 2);

    /* Parse data and write it into motor handle. */
    uint8_t isNegative = (retVal >> 9) & 0x1;
    if (retVal > 1023) {
        retVal = retVal - 1023;
    }

    float retValf = (float) (retVal / 1023.0 * 100.0);
    if (isNegative) {
        retValf *= -1;
    }
    hdynamixel->_lastLoad = retValf;
}

float Dynamixel_GetVoltage(Dynamixel_HandleTypeDef* hdynamixel) {
    /* Reads address 0x2A in the motor RAM to see what the current voltage is.
     * Value retrieved from motor is 10 times the actual voltage.
     *
     * Arguments: hdynamixel, the motor handle
     *
     * Returns: voltage in volts
     */

    /* Read data from motor. */
    uint8_t retVal = (uint8_t) Dynamixel_DataReader(hdynamixel,
    REG_CURRENT_VOLTAGE, 1);

    return ((float) (retVal / 10.0));
}

uint8_t Dynamixel_GetTemperature(Dynamixel_HandleTypeDef* hdynamixel) {
    /* Reads address 0x2B in the motor RAM to see what the current temperature is inside the motor.
     * Results in degrees Celsius.
     *
     * Arguments: hdynamixel, the motor handle
     *
     * Returns: internal motor temperature in degrees Celsius
     */

    /* Read data from motor straight into the motor handle. */
    return ((uint8_t) Dynamixel_DataReader(hdynamixel, REG_CURRENT_TEMPERATURE,
            1));
}

uint8_t Dynamixel_IsRegistered(Dynamixel_HandleTypeDef* hdynamixel) {
    /* Used to tell if command sent was written to motor registers.
     * Can also be used to see if instruction in motor register has been executed.
     *
     * Read address: 0x2C
     * Default value: 0x00
     *
     * Arguments: hdynamixel, the motor handle
     *
     * Returns: 1 if there are commands transmitted by REG_WRITE
     * 			0 otherwise.
     * 			If ACTION command is executed, the value is changed into 0.
     */

    /* Return the data read from the motor. */
    return ((uint8_t) Dynamixel_DataReader(hdynamixel, REG_REGISTERED, 1));
}

uint8_t Dynamixel_IsMoving(Dynamixel_HandleTypeDef* hdynamixel) {
    /* Reads the 0x2E address in motor RAM to see if motor is moving.
     *
     * Arguments: hdynamixel, the motor handle
     *
     * Returns: 0 if not moving
     * 			1 if moving
     */

    /* Return the data from the motor. */
    return ((uint8_t) Dynamixel_DataReader(hdynamixel, REG_MOVING, 1));
}

uint8_t Dynamixel_IsJointMode(Dynamixel_HandleTypeDef* hdynamixel) {
    /* Reads the CW (addr: 0x06) and CCW (addr: 0x08) angle limits. If both are 0, motor is in wheel mode
     * and can spin indefinitely. Otherwise, motor is in joint mode and has angle limits set
     *
     * Arguments: hdynamixel, the motor handle
     *
     * Returns: 0 if in wheel mode
     * 			1 if in joint mode
     */

    /* Read data from motor. */
    uint16_t retValCW = Dynamixel_DataReader(hdynamixel, REG_CW_ANGLE_LIMIT, 2);
    uint16_t retValCCW = Dynamixel_DataReader(hdynamixel, REG_CCW_ANGLE_LIMIT,
            2);

    /* Parse data and write it into motor handle. */
    hdynamixel->_isJointMode = (uint8_t) ((retValCW | retValCCW) != 0);

    /* Return. */
    return (hdynamixel->_isJointMode);
}

/*******************************************************************************/
/*	Transmit/receive helper functions										   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*******************************************************************************/
void Dynamixel_DataWriter(Dynamixel_HandleTypeDef* hdynamixel, uint8_t* args,
        uint8_t numArgs) {
    /* Handles sending of data since this nearly identical for all setters.
     * Uses the WRITE DATA instruction, 0x03, in the motor instruction set.
     *
     * Arguments: hdynamixel, the motor handle
     * 			  args is an array of arguments of the form
     * 						{ADDR, PARAM1, ...}
     *			  numArgs must be equal to sizeof(args), and must either be 2 or 3
     *
     * Returns: none
     */

    /* Check validity so that we don't accidentally write something invalid */
    if (numArgs <= 3) {
        /* Do assignments and computations. */
        uint8_t ID = hdynamixel->_ID; //should be default
        if (ID == BROADCAST_ID) {
            ID = 0;
        }

        arrTransmit[ID][3] = 2 + numArgs; // Length of message following length argument (arguments & checksum)
        for (uint8_t i = 0; i < numArgs; i++) {
            arrTransmit[ID][5 + i] = args[i];
        }

        /* Checksum. */
        arrTransmit[ID][4 + numArgs + 1] = Dynamixel_ComputeChecksum(
                arrTransmit[ID], 4 + numArgs + 2);

        /* Set data direction for transmit. */
        __DYNAMIXEL_TRANSMIT(hdynamixel->_dataDirPort,
                hdynamixel->_dataDirPinNum);

        /* Transmit. */
////		if(hdynamixel -> _UART_Handle == &huart3){
////			HAL_UART_Transmit_DMA(hdynamixel -> _UART_Handle, arrTransmit[ID], 4 + numArgs + 2);
////		}
//		if(hdynamixel -> _UART_Handle == &huart6){
//			HAL_UART_Transmit_IT(hdynamixel -> _UART_Handle, arrTransmit[ID], 4 + numArgs + 2);
//		}
//		else{
        HAL_UART_Transmit(hdynamixel->_UART_Handle, arrTransmit[ID],
                4 + numArgs + 2, TRANSMIT_TIMEOUT);
        //sent the entire row: arrTransmit[ID]
//		}

    }
}

uint16_t Dynamixel_DataReader(Dynamixel_HandleTypeDef* hdynamixel,
        uint8_t readAddr, uint8_t readLength) {
    /* Reads data back from the motor passed in by the handle. This process is identical
     * for all the getters which is why it is encapsulated in this function. Reading data
     * uses the READ DATA instruction, 0x02, in the motor instruction set.
     *
     * The status packet returned will be of them form:
     * 0xFF 0xFF ID LENGTH ERR PARAM_1...PARAM_N CHECKSUM, where N = readLength
     *
     * Arguments: hdynamixel, the motor handle
     *			  readAddr, the address inside the motor memory table where reading is to begin
     *			  readLength, the number of bytes to be read. Must be either 1 or 2.
     *
     * Returns: a 16-bit value containing 1 or both bytes received, as applicable. The
     * 			1st byte received will be the LSB and the 2nd byte received will be the MSB
     */

    /* Clear array for reception. */
    uint8_t ID = hdynamixel->_ID;
    for (uint8_t i = 0; i < BUFF_SIZE_RX; i++) {
        arrReceive[ID][i] = 0;
    }

    /* Do assignments and computations. */
    arrTransmit[ID][3] = 4; // Length of message minus the obligatory bytes
    arrTransmit[ID][4] = INST_READ_DATA; // READ DATA instruction
    arrTransmit[ID][5] = readAddr; // Write address for register
    arrTransmit[ID][6] = readLength; // Number of bytes to be read from motor
    arrTransmit[ID][7] = Dynamixel_ComputeChecksum(arrTransmit[ID], 8);

    // Set data direction for transmit
    __DYNAMIXEL_TRANSMIT(hdynamixel->_dataDirPort, hdynamixel->_dataDirPinNum);

    // Transmit
    HAL_UART_Transmit(hdynamixel->_UART_Handle, arrTransmit[ID], 8,
    TRANSMIT_TIMEOUT);

    // Set data direction for receive
    __DYNAMIXEL_RECEIVE(hdynamixel->_dataDirPort, hdynamixel->_dataDirPinNum);

    /* Set the instruction back to INST_WRITE_DATA in the buffer because writing
     * is expected to be more common than reading. */
    arrTransmit[ID][4] = INST_WRITE_DATA; // WRITE DATA instruction

    // Call appropriate UART receive function depending on if 1 or 2 bytes are to be read
    if (readLength == 1) {
        HAL_UART_Receive(hdynamixel->_UART_Handle, arrReceive[ID], 7,
        RECEIVE_TIMEOUT);
    } else {
        HAL_UART_Receive(hdynamixel->_UART_Handle, arrReceive[ID], 8,
        RECEIVE_TIMEOUT);
    }

    if (readLength == 1) {
        return (uint16_t) arrReceive[ID][5];
    } else {
        return (uint16_t) (arrReceive[ID][5] | (arrReceive[ID][6] << 8));
    }
}

// TODO: Test this
void Dynamixel_SyncWritePosition(Dynamixel_HandleTypeDef* hdynamixel,
        SyncWriteBlock_t* syncWriteBlock) {
    /* Implements the sync write instruction (0x83) for updating the positions of several motors connected
     * via the same UART module.
     *
     * Arguments: hdynamixel, the motor handle
     * 			  syncWriteBlock, contains all the fields necessary for transmitting the data
     *
     * Returns: none
     */

    /* Compute the first index into the sync write array */
    uint8_t arrIdx = 0;
    UART_HandleTypeDef* theUART = syncWriteBlock->_UART_Handle;

    /*if(theUART == &huart1){ arrIdx = 0; }
     else if(theUART == &huart2){ arrIdx = 1; }
     else if(theUART == &huart3){ arrIdx = 2; }
     else if(theUART == &huart4){ arrIdx = 3; }
     else if(theUART == &huart6){ arrIdx = 4; }
     else{
     // If we are here, then a UART that is not supported by the robot is attempting to be used
     return;
     }*/
    //Only huart 1 and 2 available for NUCLEOL432KC
    if (theUART == &huart1) {
        arrIdx = 0;
    } else if (theUART == &huart2) {
        arrIdx = 1;
    } else {
        // If we are here, then a UART that is not supported by the robot is attempting to be used
        return;
    }

    /* Write packet length, based on the number of motors on this UART */
    arrSyncWritePosition[arrIdx][3] = 4 + (syncWriteBlock->numMotors) * (1 + 2);

    /* Loop through motors in the syncWriteBlock, adding their IDs and arguments into the packet */
    uint8_t idx = 7;
    float goalAngle = 0;
    uint16_t normalized_value = 0;
    for (uint8_t motor = 0; motor < (syncWriteBlock->numMotors); motor++) {
        // Put ID into packet
        arrSyncWritePosition[arrIdx][idx] =
                (syncWriteBlock->motorHandles)[motor]->_ID;

        /* Check for input validity. If input not valid, replace goalAngle with closest
         * valid value to ensure code won't halt. */
        goalAngle = (syncWriteBlock->motorArgs)[motor];
        if ((goalAngle < MIN_ANGLE) || (goalAngle > MAX_ANGLE)) {
            if (goalAngle > MIN_ANGLE) {
                goalAngle = MAX_ANGLE;
            } else {
                goalAngle = MIN_ANGLE;
            }
        }

        /* Translate the angle from degrees into a 10- or 12-bit number. */
        if ((syncWriteBlock->motorHandles)[motor]->_motorType == AX12ATYPE) {
            normalized_value = (uint16_t) (goalAngle / MAX_ANGLE * 1023);
        } else if ((syncWriteBlock->motorHandles)[motor]->_motorType
                == MX28TYPE) {
            normalized_value = (uint16_t) (goalAngle / MAX_ANGLE * 4095);
        } else {
            // Should NEVER reach here!
            return;
        }

        uint8_t lowByte = (uint8_t) (normalized_value & 0xFF); // Low byte of goal position
        uint8_t highByte = (uint8_t) ((normalized_value >> 8) & 0xFF); // High byte of goal position

        arrSyncWritePosition[arrIdx][idx + 1] = lowByte;
        arrSyncWritePosition[arrIdx][idx + 2] = highByte;

        idx += 3;
    }

    HAL_UART_Transmit(syncWriteBlock->_UART_Handle,
            arrSyncWritePosition[arrIdx], 4 + arrSyncWritePosition[arrIdx][3],
            TRANSMIT_TIMEOUT);
}

// TODO: Test this
void Dynamixel_SyncWritePositionAndVelocity(Dynamixel_HandleTypeDef* hdynamixel,
        SyncWriteBlock_t* syncWriteBlock) {
    /* Implements the sync write instruction (0x83) for updating the positions and velocities
     * of several motors connected via the same UART module.
     *
     * Arguments: hdynamixel, the motor handle
     * 			  syncWriteBlock, contains all the fields necessary for transmitting the data
     *
     * Returns: none
     */

    /* Compute the first index into the sync write array */
    uint8_t arrIdx = 0;
    UART_HandleTypeDef* theUART = syncWriteBlock->_UART_Handle;

    if (theUART == &huart1) {
        arrIdx = 0;
    } else if (theUART == &huart2) {
        arrIdx = 1;
    } else {
        // If we are here, then a UART that is not supported by the robot is attempting to be used
        return;
    }

    /* Write packet length, based on the number of motors on this UART */
    arrSyncWritePosition[arrIdx][3] = 4
            + (syncWriteBlock->numMotors) * (1 + 2 + 2);

    /* Loop through motors in the syncWriteBlock, adding their IDs and arguments into the packet */
    uint8_t idx = 7;
    uint8_t lowByte = 0;
    uint8_t highByte = 0;
    uint16_t normalized_value = 0;
    float goalAngle = 0;
    float goalVelocity = 0;

    for (uint8_t motor = 0; motor < (syncWriteBlock->numMotors); motor += 2) {
        // Put ID into packet
        arrSyncWritePosition[arrIdx][idx] =
                (syncWriteBlock->motorHandles)[motor]->_ID;

        /********** POSITION **********/
        /* Check for input validity for position. If input not valid, replace goalAngle with closest
         * valid value to ensure code won't halt. */
        goalAngle = (syncWriteBlock->motorArgs)[motor]; // Position argument
        if ((goalAngle < MIN_ANGLE) || (goalAngle > MAX_ANGLE)) {
            if (goalAngle > MIN_ANGLE) {
                goalAngle = MAX_ANGLE;
            } else {
                goalAngle = MIN_ANGLE;
            }
        }

        /* Translate the angle from degrees into a 10- or 12-bit number. */
        normalized_value = 0;
        if ((syncWriteBlock->motorHandles)[motor]->_motorType == AX12ATYPE) {
            normalized_value = (uint16_t) (goalAngle / MAX_ANGLE * 1023);
        } else if ((syncWriteBlock->motorHandles)[motor]->_motorType
                == MX28TYPE) {
            normalized_value = (uint16_t) (goalAngle / MAX_ANGLE * 4095);
        } else {
            // Should NEVER reach here!
            return;
        }

        lowByte = (uint8_t) (normalized_value & 0xFF); // Low byte of goal position
        highByte = (uint8_t) ((normalized_value >> 8) & 0xFF); // High byte of goal position

        arrSyncWritePosition[arrIdx][idx + 1] = lowByte;
        arrSyncWritePosition[arrIdx][idx + 2] = highByte;

        /********** VELOCITY **********/
        /* Translate the position from RPM into a 10-bit number. */
        normalized_value = 0;
        goalVelocity = (syncWriteBlock->motorArgs)[motor + 1]; // Velocity argument
        if (hdynamixel->_isJointMode) {
            /* Check for input validity. If input not valid, replace goalAngle with closest
             * valid value to ensure code won't halt. */
            if (goalVelocity != 0) {
                if (hdynamixel->_motorType == AX12ATYPE) {
                    if ((goalVelocity < MIN_VELOCITY)
                            || (goalVelocity > AX12A_MAX_VELOCITY)) {
                        if (goalVelocity > MIN_VELOCITY) {
                            goalVelocity = AX12A_MAX_VELOCITY;
                        } else {
                            goalVelocity = MIN_VELOCITY;
                        }
                    }
                } else if (hdynamixel->_motorType == MX28TYPE) {
                    if ((goalVelocity < MIN_VELOCITY)
                            || (goalVelocity > MX28_MAX_VELOCITY)) {
                        if (goalVelocity > MIN_VELOCITY) {
                            goalVelocity = MX28_MAX_VELOCITY;
                        } else {
                            goalVelocity = MIN_VELOCITY;
                        }
                    }
                }
            }
        }

        if (hdynamixel->_motorType == AX12ATYPE) {
            normalized_value = (uint16_t) (goalVelocity / AX12A_MAX_VELOCITY
                    * 1023);
            if (goalVelocity < 0) {
                normalized_value = ((uint16_t) ((goalVelocity * -1)
                        / AX12A_MAX_VELOCITY * 1023)) | 0b000010000000000;
            }
        } else if (hdynamixel->_motorType == MX28TYPE) {
            normalized_value = (uint16_t) (goalVelocity / MX28_MAX_VELOCITY
                    * 1023);
            if (goalVelocity < 0) {
                normalized_value = ((uint16_t) ((goalVelocity * -1)
                        / MX28_MAX_VELOCITY * 1023)) | 0b000010000000000;
            }
        }

        lowByte = (uint8_t) (normalized_value & 0xFF); // Low byte of goal velocity
        highByte = (uint8_t) ((normalized_value >> 8) & 0xFF); // High byte of goal velocity

        arrSyncWritePosition[arrIdx][idx + 3] = lowByte;
        arrSyncWritePosition[arrIdx][idx + 4] = highByte;

        idx += 5;
    }

    HAL_UART_Transmit(syncWriteBlock->_UART_Handle,
            arrSyncWritePosition[arrIdx], 4 + arrSyncWritePosition[arrIdx][3],
            TRANSMIT_TIMEOUT);
}

/*******************************************************************************/
/*	Other motor instruction helper functions								   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*******************************************************************************/
void Dynamixel_RegWrite(Dynamixel_HandleTypeDef* hdynamixel, uint8_t arrSize,
        uint8_t writeAddr, uint8_t param1, uint8_t param2) {
    /* Implementation of REG WRITE instruction with 2 parameters.
     *
     * Arguments: hdynamixel, the motor handle
     * 			  writeAddr, thhe starting address for where the data is to be written
     * 			  param1, the first parameter
     * 			  param2, the second parameter
     *
     * Returns: none
     */

    /* Define arrays for transmission. */
    uint8_t arrTransmit[arrSize];

    /* Do assignments and computations. */
    arrTransmit[0] = 0xFF; // Obligatory bytes for starting communication
    arrTransmit[1] = 0xFF; // Obligatory bytes for starting communication
    arrTransmit[2] = hdynamixel->_ID; // Motor ID
    arrTransmit[3] = arrSize - 4; // Length of message minus the obligatory bytes
    arrTransmit[4] = INST_REG_WRITE; // REG WRITE instruction
    arrTransmit[5] = writeAddr;
    arrTransmit[7] =
            (arrSize == 8) ?
                    Dynamixel_ComputeChecksum(arrTransmit, arrSize) : param2;
    if (arrSize == 9) {
        arrTransmit[8] = Dynamixel_ComputeChecksum(arrTransmit, arrSize);
    }

    /* Set data direction. */
    __DYNAMIXEL_TRANSMIT(hdynamixel->_dataDirPort, hdynamixel->_dataDirPinNum);

    /* Transmit. */
    HAL_UART_Transmit(hdynamixel->_UART_Handle, arrTransmit, arrSize,
    TRANSMIT_TIMEOUT);
}

void Dynamixel_Action(Dynamixel_HandleTypeDef* hdynamixel) {
    /* Implements the ACTION instruction. This triggers the instruction registered by the REG WRITE
     * instruction. This way, time delays can be reduced for the concurrent motion of several
     * motors.
     *
     * Arguments: hdynamixel, the motor handle
     *
     * Returns: none
     */

    /* Define arrays for transmission and reception. */
    uint8_t arrTransmit[6];

    /* Do assignments and computations. */
    arrTransmit[0] = 0xFF; // Obligatory bytes for starting communication
    arrTransmit[1] = 0xFF; // Obligatory bytes for starting communication
    arrTransmit[2] = hdynamixel->_ID; // Motor ID
    arrTransmit[3] = 2; // Length of message minus the obligatory bytes
    arrTransmit[4] = INST_ACTION; // ACTION instruction
    arrTransmit[5] = Dynamixel_ComputeChecksum(arrTransmit, 6);

    /* Set data direction. */
    __DYNAMIXEL_TRANSMIT(hdynamixel->_dataDirPort, hdynamixel->_dataDirPinNum);

    /* Transmit. */
    HAL_UART_Transmit(hdynamixel->_UART_Handle, arrTransmit, 6,
    TRANSMIT_TIMEOUT);
}

int8_t Dynamixel_Ping(Dynamixel_HandleTypeDef* hdynamixel) {
    /* Used only for returning a status packet or checking the existence of a motor
     * with a specified ID. Does not command any operations.
     *
     * Arguments: hdynamixel, the motor handle
     *
     * Returns: motor ID seen in status packet if received a valid status packet,
     * 				otherwise -1
     */

    /* Define arrays for transmission and reception. */
    uint8_t arr[6];

    /* Do assignments and computations. */
    arr[0] = 0xff; // Obligatory bytes for starting communication
    arr[1] = 0xff; // Obligatory bytes for starting communication
    arr[2] = hdynamixel->_ID; // Motor ID
    arr[3] = 2; // Length of message minus the obligatory bytes
    arr[4] = INST_PING; // PING instruction
    arr[5] = Dynamixel_ComputeChecksum(arr, 6);

    /* Set data direction for transmit. */
    __DYNAMIXEL_TRANSMIT(hdynamixel->_dataDirPort, hdynamixel->_dataDirPinNum);

    /* Transmit. */
    HAL_UART_Transmit(hdynamixel->_UART_Handle, arr, 6, TRANSMIT_TIMEOUT);

    /* Set data direction for receive. */
    __DYNAMIXEL_RECEIVE(hdynamixel->_dataDirPort, hdynamixel->_dataDirPinNum);

    /* Receive. */
    if (HAL_UART_Receive(hdynamixel->_UART_Handle, arr, 6, RECEIVE_TIMEOUT)) {
        return (arr[2]);
    } else {
        return -1;
    }
}

/*******************************************************************************/
/*	Setup helper functions      											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*******************************************************************************/
void Dynamixel_Init(Dynamixel_HandleTypeDef* hdynamixel, uint8_t ID,
        UART_HandleTypeDef *UART_Handle, GPIO_TypeDef* DataDirPort,
        uint16_t DataDirPinNum, enum motorTypes_e motorType) {
    /* Initializes the motor handle.
     *
     * Arguments: hdynamixel, the motor handle to be initialized
     * 			  ID, the ID the motor has. Note that this function will not set
     * 			  	  the ID in case there are multiple actuators on the same bus
     * 			  UART_Handle, the handle to the UART that will be used to
     * 			      communicate with this motor
     * 			  DataDirPort, the pointer to the port that the data direction pin
     * 			  	  for the motor is on
     * 			  DataDirPinNum, the number corresponding to the pin that controls
     * 			      data direction (a power of two, e.g. 2^0 for pin 0, 2^15 for pin 15)
     * 			  motorType, indicates whether motor is AX12A or MX28
     *
     * Returns: none
     */

    /* Set fields in motor handle. */
    hdynamixel->_motorType = motorType;	// Identifies the type of actuator; used in certain functions
    hdynamixel->_ID = ID; 					// Motor ID (unique or global)
    hdynamixel->_lastPosition = -1; // In future, could initialize this accurately
    hdynamixel->_lastVelocity = -1; // In future, could initialize this accurately
    hdynamixel->_lastLoad = -1; // In future, could initialize this accurately
    hdynamixel->_isJointMode = 1; // In future, could initialize this accurately
    hdynamixel->_UART_Handle = UART_Handle; 	// For UART TX and RX
    hdynamixel->_dataDirPort = DataDirPort;
    hdynamixel->_dataDirPinNum = DataDirPinNum;
}

void Dynamixel_Reset(Dynamixel_HandleTypeDef* hdynamixel) {
    /* Resets the control table values of the motor to the Factory Default Value settings.
     * Note that post-reset, motor ID will be 1. Thus, if several motors with ID 1 are
     * connected on the same bus, there will not be a way to assign them unique IDs without
     * first disconnecting them. Need to wait around 500 ms before motor becomes valid again.
     *
     * Arguments: hdynamixel, the motor handle
     *
     * Returns: none
     */

    /* Define array for transmission. */
    uint8_t arrTransmit[6];

    /* Do assignments and computations. */
    arrTransmit[0] = 0xff; // Obligatory bytes for starting communication
    arrTransmit[1] = 0xff; // Obligatory bytes for starting communication
    arrTransmit[2] = hdynamixel->_ID; // Motor ID: 1, 2, 3, or 4,..
    arrTransmit[3] = 2; // Length of message minus the obligatory bytes
    arrTransmit[4] = INST_RESET; // #define INST_RESET=0x06, Resets the control tables of the Dynamixel actuator(s) specified
    arrTransmit[5] = Dynamixel_ComputeChecksum(arrTransmit, 6);

    /* Set data direction. */
    __DYNAMIXEL_TRANSMIT(hdynamixel->_dataDirPort, hdynamixel->_dataDirPinNum); //write 1

    /* Transmit. */
    HAL_UART_Transmit(hdynamixel->_UART_Handle, arrTransmit, 6,
    TRANSMIT_TIMEOUT);
    hdynamixel->_ID = DEFAULT_ID;
}

/*******************************************************************************/
/*	Interfaces for previously-defined functions								   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*******************************************************************************/
void Dynamixel_EnterWheelMode(Dynamixel_HandleTypeDef* hdynamixel,
        float goalVelocity) {
    /* Sets the control registers such that the rotational angle of the motor
     * is not bounded.
     *
     * Arguments: hdynamixel, the motor handle
     * 			  goalVelocity, the desired velocity to use when entering wheel mode
     *
     * Returns: none
     */

    /* When the angle limits are both set to 0, then motor will attempt to
     * rotate with maximum velocity. To prevent undesired behaviour, the
     * goal velocity should be set right after calling this function */
    Dynamixel_SetCWAngleLimit(hdynamixel, 0);
    Dynamixel_SetCCWAngleLimit(hdynamixel, 0);
    hdynamixel->_isJointMode = 0;
    Dynamixel_SetGoalVelocity(hdynamixel, goalVelocity);
}

void Dynamixel_EnterJointMode(Dynamixel_HandleTypeDef* hdynamixel) {
    /* Sets the control registers such that the rotational angle of the motor
     * is constrained between the default values.
     *
     * Arguments: hdynamixel, the motor handle
     *
     * Returns: none
     */

    Dynamixel_SetCWAngleLimit(hdynamixel, MIN_ANGLE);
    Dynamixel_SetCCWAngleLimit(hdynamixel, MAX_ANGLE);
    hdynamixel->_isJointMode = 1;
}

/*******************************************************************************/
/*	Computation-based helper functions										   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*******************************************************************************/
inline uint8_t Dynamixel_ComputeChecksum(uint8_t *arr, int length) {
    /* Compute the checksum for data to be transmitted.
     *
     * Arguments: arr, the array to be transmitted and ran through the checksum function
     * 			  length, the total length of the array arr
     *
     * Returns: the 1-byte number that is the checksum
     */

    /* Local variable declaration. */
    uint8_t accumulate = 0;

    /* Loop through the array starting from the 2nd element of the array and finishing before the last
     * since the last is where the checksum will be stored. */
    for (uint8_t i = 2; i < length - 1; i++) {
        accumulate += arr[i];
    }

    return (~accumulate) & 0xFF; // Lower 8 bits of the logical NOT of the sum
}

void syncWriteBlockInit(Dynamixel_HandleTypeDef** arrDynamixel,
        uint8_t numMotors, UART_HandleTypeDef* UART_Handle,
        SyncWriteBlock_t* syncWriteBlock) {
    /* Initializes the fields in the syncWriteBlock passed in by reference.
     *
     * Arguments: arrDynamixel, address of the array of motor handles
     *            numMotors, the size of the arrDynamixel array
     *            UART_Handle, the handle to the UART that will be used to
     * 			      communicate with this set of motors
     *            syncWriteBlock, the SyncWriteBlock_t structure to initialize
     *
     * Returns: none
     */

    syncWriteBlock->_UART_Handle = UART_Handle;

    uint8_t idx = 0;
    for (uint8_t i = 0; i < numMotors; i++) {
        if (arrDynamixel[i]->_UART_Handle == syncWriteBlock->_UART_Handle) {
            syncWriteBlock->motorHandles[idx] = arrDynamixel[i];
            syncWriteBlock->numMotors = syncWriteBlock->numMotors + 1;
            idx++;
        }
    }
}
