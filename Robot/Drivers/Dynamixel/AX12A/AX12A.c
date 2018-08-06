/**
  ******************************************************************************
  * @file    AX12A.c
  * @author  Tyler
  * @brief   This file implements AX12A-specific functions.
  ******************************************************************************
  */




/********************************* Includes **********************************/
#include "../h/AX12A.h"




/********************************* Externs ***********************************/
extern void Dynamixel_DataWriter(Dynamixel_HandleTypeDef* hdynamixel, uint8_t* args, uint8_t numArgs);




/********************************** Macros ***********************************/
// Register addresses
#define AX12A_REG_CW_COMPLIANCE_MARGIN  0x1A        /**< Clockwise compliance margin register */
#define AX12A_REG_CCW_COMPLIANCE_MARGIN 0x1B        /**< Counter-clockwise compliance margin register */
#define AX12A_REG_CW_COMPLIANCE_SLOPE   0x1C        /**< Clockwise compliance slope register */
#define AX12A_REG_CCW_COMPLIANCE_SLOPE  0x1D        /**< Counter-clockwise compliance slope register */




/********************************* Constants *********************************/
// Value limit definitions
const uint8_t AX12A_MAX_VELOCITY = 114; /**< Maximum angular velocity (RPM) */

// Default register values
const uint8_t AX12A_DEFAULT_BAUD_RATE              =  0x01;    /**< Default baud rate register setting */
const uint16_t AX12A_DEFAULT_CCW_ANGLE_LIMIT       =  0x03FF;  /**< Default counter-clockwise angle limit */
const uint8_t AX12A_DEFAULT_HIGHEST_VOLTAGE_LIMIT  =  0xBE;    /**< Default permitted maximum voltage (0xBE = 140 -> 14.0 V) */
const uint8_t AX12A_DEFAULT_CW_COMPLIANCE_MARGIN   =  0x01;    /**< Default clockwise compliance margin (position error) */
const uint8_t AX12A_DEFAULT_CCW_COMPLIANCE_MARGIN  =  0x01;    /**< Default counter-clockwise compliance margin (position error) */
const uint8_t AX12A_DEFAULT_CW_COMPLIANCE_SLOPE    =  0x20;    /**< Default clockwise compliance slope (torque near goal position) */
const uint8_t AX12A_DEFAULT_CCW_COMPLIANCE_SLOPE   =  0x20;    /**< Default counter-clockwise compliance slope (torque near goal position) */
const uint16_t AX12A_DEFAULT_PUNCH                 =  0x0020;  /**< Default punch */




/******************************** Functions **********************************/
/**
 * @defgroup AX12A AX12A
 * @brief    Globally-accessible functions for interfacing with AX12A actuators.
 *           These functions are specific to AX12As and have no analogue for
 *           other actuators supported by this library. All of these functions
 *           will return and have no effect if the motor structure type field
 *           is not AX12ATYPE
 * @ingroup  Dynamixel
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
 * @defgroup AX12A_Setters Setters
 * @brief    Register-setting functions
 *
 * # Setter functions #
 *
 * This subsection provides a set of functions which provide interfaces for
 * setting motor register values.
 *
 * @ingroup AX12A
 * @{
 */

/**
 * @brief   Sets the clockwise compliance margin for the current motor
 * @details The compliance margin is the acceptable error between the current
 *          position and goal position. The greater the value, the more error
 *          is acceptable
 *
 *          Instruction register address: 0x1A (RAM)
 *
 *          Default value: 0x01
 * @param 	hdynamixel pointer to a Dynamixel_HandleTypeDef structure that
 *          contains the configuration information for the motor
 * @param   CWcomplianceMargin the acceptable error between the current and
 *          goal position. Arguments in range [0, 255]
 * @return  None
 */
void AX12A_SetCWComplianceMargin(Dynamixel_HandleTypeDef* hdynamixel, uint8_t CWcomplianceMargin){
    /* Write data to motor. */
    if(hdynamixel -> _motorType == AX12ATYPE){
        uint8_t arr[2] = {AX12A_REG_CW_COMPLIANCE_MARGIN, CWcomplianceMargin};
        Dynamixel_DataWriter(hdynamixel, arr, sizeof(arr));
    }
}

/**
 * @brief   Sets the counter-clockwise compliance margin for the current motor
 * @details The compliance margin is the acceptable error between the current
 *          position and goal position. The greater the value, the more error
 *          is acceptable
 *
 *          Instruction register address: 0x1B (RAM)
 *
 *          Default value: 0x01
 * @param   hdynamixel pointer to a Dynamixel_HandleTypeDef structure that
 *          contains the configuration information for the motor
 * @param   CCWcomplianceMargin the acceptable error between the current and
 *          goal position. Arguments in range [0, 255]
 * @return  None
 */
void AX12A_SetCCWComplianceMargin(Dynamixel_HandleTypeDef* hdynamixel, uint8_t CCWcomplianceMargin){
    /* Write data to motor. */
    if(hdynamixel -> _motorType == AX12ATYPE){
        uint8_t arr[2] = {AX12A_REG_CCW_COMPLIANCE_MARGIN, CCWcomplianceMargin};
        Dynamixel_DataWriter(hdynamixel, arr, sizeof(arr));
    }
}

/**
 * @brief   Sets the clockwise compliance slope for the current motor
 * @details The compliance slope sets the level of torque near the goal
 *          position. The higher the value, the more flexibility that is
 *          obtained. That is, a high value means that as the goal position is
 *          approached, the torque will be significantly reduced. If a low
 *          value is used, then as the goal position is approached, the torque
 *          will not be reduced all that much
 *
 *          Instruction register address: 0x1C (RAM)
 *
 *          Default value: 0x20
 * @param   hdynamixel pointer to a Dynamixel_HandleTypeDef structure that
 *          contains the configuration information for the motor
 * @param   CWcomplianceSlope arguments in range [1, 7], with 1 being the
 *          least flexible
 * @return  None
 */
void AX12A_SetCWComplianceSlope(Dynamixel_HandleTypeDef* hdynamixel, uint8_t CWcomplianceSlope){
    if(hdynamixel -> _motorType == AX12ATYPE){
        /* Translate the step into motor data. */
        uint8_t step;

        if((CWcomplianceSlope > 0) && (CWcomplianceSlope <= 7)){
            step = 1 << CWcomplianceSlope; // 2 to the power of CWcomplianceSlope
        }
        else{
            step = AX12A_DEFAULT_CW_COMPLIANCE_SLOPE;
        }

        /* Write data to motor. */
        uint8_t arr[2] = {AX12A_REG_CW_COMPLIANCE_SLOPE, step};
        Dynamixel_DataWriter(hdynamixel, arr, sizeof(arr));
    }
}

/**
 * @brief   Sets the counter-clockwise compliance slope for the current motor
 * @details The compliance slope sets the level of torque near the goal
 *          position. The higher the value, the more flexibility that is
 *          obtained. That is, a high value means that as the goal position is
 *          approached, the torque will be significantly reduced. If a low
 *          value is used, then as the goal position is approached, the torque
 *          will not be reduced all that much
 *
 *          Instruction register address: 0x1D (RAM)
 *
 *          Default value: 0x20
 * @param   hdynamixel pointer to a Dynamixel_HandleTypeDef structure that
 *          contains the configuration information for the motor
 * @param   CCWcomplianceSlope arguments in range [1, 7], with 1 being the
 *          least flexible
 * @return  None
 */
void AX12A_SetCCWComplianceSlope(Dynamixel_HandleTypeDef* hdynamixel, uint8_t CCWcomplianceSlope){
    if(hdynamixel -> _motorType == AX12ATYPE){
        /* Translate the step into motor data. */
        uint8_t step;

        if((CCWcomplianceSlope > 0) && (CCWcomplianceSlope <= 7)){
            step = 1 << CCWcomplianceSlope; // 2 to the power of CCWcomplianceSlope
        }
        else{
            step = AX12A_DEFAULT_CCW_COMPLIANCE_SLOPE;
        }

        /* Write data to motor. */
        uint8_t arr[2] = {AX12A_REG_CCW_COMPLIANCE_SLOPE, step};
        Dynamixel_DataWriter(hdynamixel, arr, sizeof(arr));
    }
}

/**
 * @}
 */
/* end AX12A_Setters */



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
 * @defgroup AX12A_Interfaces Interfaces for previously-defined functions
 * @brief    Interfaces for previously-defined functions
 *
 * # Interfaces for previously-defined functions #
 *
 * This subsection provides a set of functions which implement functions
 * which call previously-defined functions in order to accomplish specific
 * tasks.
 *
 * @ingroup AX12A
 * @{
 */

/**
 * @brief   Sets both the CW and CCW compliance slope
 * @details "Sets the compliance slope for the current motor, which sets the
 *          level of torque near the goal position. The higher the value, the
 *          more flexibility that is obtained. That is, a high value means that
 *          as the goal position is approached, the torque will be
 *          significantly reduced. If a low value is used, then as the goal
 *          position is approached, the torque will not be reduced all that
 *          much"
 * @param 	hdynamixel pointer to a Dynamixel_HandleTypeDef structure that
 *          contains the configuration information for the motor
 * @param   complianceSlope arguments in range [1, 7], with 1 being the least flexible
 * @return  None
 */
void AX12A_SetComplianceSlope(Dynamixel_HandleTypeDef* hdynamixel, uint8_t complianceSlope){
    AX12A_SetCWComplianceSlope(hdynamixel, complianceSlope);
    AX12A_SetCCWComplianceSlope(hdynamixel, complianceSlope);
}

/**
 * @brief   Sets both the CW and CCW compliance margin
 * @details "Sets the compliance margin for the current motor. The compliance
 *          margin is the acceptable error between the current position and
 *          goal position. The greater the value, the more error is
 *          acceptable"
 * @param 	hdynamixel pointer to a Dynamixel_HandleTypeDef structure that
 *          contains the configuration information for the motor
 * @param   complianceMargin the acceptable error between the current and
 *          goal position. Arguments in range [0, 255]
 * @return  None
 */
void AX12A_SetComplianceMargin(Dynamixel_HandleTypeDef* hdynamixel, uint8_t complianceMargin){
    AX12A_SetCWComplianceMargin(hdynamixel, complianceMargin);
    AX12A_SetCCWComplianceMargin(hdynamixel, complianceMargin);
}

/**
 * @}
 */
/* end AX12A_Interfaces */
