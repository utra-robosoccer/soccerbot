/**
  *****************************************************************************
  * @file    template.c
  * @author  YourNameHere
  * @brief   This file is a template
  *
  * @defgroup Module_Name Module Name
  * @brief 	  This is a brief description of the module
  *****************************************************************************
  */




/********************************* Includes **********************************/
#include "template.h"




/****************************** Public Variables *****************************/
uint8_t publicVar;




/***************************** Private Variables *****************************/
static uint8_t privateVar;




/******************************** Functions **********************************/
/*****************************************************************************/
/*  Submodule name here                                                      */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/
/**
 * @defgroup ModuleName_SubmoduleName SubmoduleName
 * @brief    Brief description of the submodule
 *
 * # Submodule name here #
 *
 * This subsection provides a set of functions which do X Y Z
 *
 * @ingroup ModuleName
 * @{
 */

/**
 * @brief   For an actuator in multi-turn mode, this applies a tunable offset
 *          to all positions. That is it allows you to change where the
 *          actuator considers position 0 to be. For an actuator not in
 *          multi-turn mode, this setting has no effect
 * @param 	hdynamixel pointer to a Dynamixel_HandleTypeDef structure that
 *          contains the configuration information for the motor
 * @param   offset the signed offset argument indicating the offset value to
 *          program
 * @retval None
 */
void MX28_SetMultiTurnOffset(Dynamixel_HandleTypeDef* hdynamixel, int16_t offset){
	if(offset > 28672){
		offset = 28672;
	}
	else if(offset < -28672){
		offset = -28672;
	}

	uint8_t args[3] = {MX28_REG_MULTI_TURN_OFFSET, offset & 0xFF, (offset >> 16) & 0xFF};
	Dynamixel_DataWriter(hdynamixel, args, sizeof(args));
}


/**
  * @}
  */
/* end MX28_Setters */


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
 * @defgroup MX28_Interfaces Interfaces for previously-defined functions
 * @brief    Interfaces for previously-defined functions
 *
 * # Interfaces for previously-defined functions #
 *
 * This subsection provides a set of functions which implement functions
 * which call previously-defined functions in order to accomplish specific
 * tasks.
 *
 * @ingroup MX28
 * @{
 */

/**
 * @brief   Activates multi-turn mode, which allows the actuator to have a
 *          range of controllable position values from -28672 to 28672
 * @param 	hdynamixel pointer to a Dynamixel_HandleTypeDef structure that
 *          contains the configuration information for the motor
 * @retval  None
 */
void MX28_EnterMultiTurnMode(Dynamixel_HandleTypeDef* hdynamixel){
	if(hdynamixel -> _motorType == MX28TYPE){
		Dynamixel_SetCWAngleLimit(hdynamixel, 4095);
		Dynamixel_SetCCWAngleLimit(hdynamixel, 4095);
	}
}

/**
 * @}
 */
/* end MX28_Interfaces */
