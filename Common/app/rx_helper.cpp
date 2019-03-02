/**
 *****************************************************************************
 * @file
 * @author  Hannah
 * @brief   Helper file for the function StartRXTask in freertos.cpp
 *
 * @defgroup Helpers
 * @ingroup Threads
 * @brief Helper functions to help the read-ability of freertos.cpp
 * @{
 *****************************************************************************
 */

/********************************* Includes **********************************/
#include "rx_helper.h"
#include "Notification.h"
#include "cmsis_os.h"
#include "Communication.h"
#include "usart.h"

using namespace soccerbot; // TODO(tgamvrel) using namespace

/***************************** Private Variables *****************************/
static uint8_t robotGoalData[sizeof(comm::RobotGoal_t)];
static uint8_t *robotGoalDataPtr;

/********************************  Functions  ********************************/
/*****************************************************************************/
/*  StartRxTask Helper Functions                                             */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/
/**
 * @defgroup RxHelperFunctions StartRxTask Helper Functions
 * @ingroup Helpers
 * @brief Helper functions for StartRxTask()
 *
 * # StartRXTask Helper Functions #
 *
 * This subsection includes helper functions for the StartRXTask() function in
 * freertos.cpp in order to make the the file more readable.
 *
 * @{
 */

/**
 * @brief   Initializes the private variables for StartRxTask
 * @param 	None
 * @return  None
 */
void initializeVars(void) {
    //sending
    comm::RobotGoal_t& robot_goal = comm::get_robot_goal();
    robot_goal.id = 0;
    robotGoalDataPtr = robotGoalData;

    //receiving
    comm::RobotState_t& robot_state = comm::get_robot_state();
    robot_state.id = 0;
    robot_state.start_seq = UINT32_MAX;
    robot_state.end_seq = 0;
}

// TODO: refactor this after researching more standard parsing techniques.
static RxParseState readByte(const uint8_t byte_in, bool& complete_out) {
    constexpr size_t SIZE_START_SEQ = 4;
    constexpr size_t SIZE_DATA = sizeof(comm::RobotGoal_t);
    static RxParseState state = RxParseState::CHECKING_HEADER;
    static size_t startSeqCount = 0;
    static size_t dataBytesRead = 0;
    RxParseState prevState = state;

    switch(byte_in) {
    case 0xFF:
        if (state == RxParseState::CHECKING_HEADER) {
            startSeqCount++;
        }
        if (state == RxParseState::READING_DATA) {
            dataBytesRead++;
        }
        break;
    default:
        if (state == RxParseState::READING_DATA) {
            dataBytesRead++;
        }
        break;
    }

    if (state == RxParseState::CHECKING_HEADER && startSeqCount == SIZE_START_SEQ) {
        state = RxParseState::READING_DATA;
    }
    else if (state == RxParseState::READING_DATA && dataBytesRead == SIZE_DATA) {
        complete_out = true;
        state = RxParseState::CHECKING_HEADER;
        startSeqCount = 0;
        dataBytesRead = 0;
    }

    return prevState;
}

void parseByteSequence(uint8_t *in_buff, size_t in_buff_size, bool& complete) {
    for (size_t i = 0; i < in_buff_size; i++) {
        if (readByte(in_buff[i], complete) == RxParseState::READING_DATA) {
            *(robotGoalDataPtr++) = in_buff[i];
        }
        if (complete) {
            // Reset the variables to help with reception of a RobotGoal
            robotGoalDataPtr = robotGoalData;
            break;
        }
    }
}

void copyParsedData(void) {
    comm::RobotGoal_t& robot_goal = comm::get_robot_goal();
    comm::RobotState_t& robot_state = comm::get_robot_state();

    robot_state.id = robot_goal.id;
    memcpy(&robot_goal, robotGoalData, sizeof(comm::RobotGoal_t));
}

/**
 * @}
 */
/* end - RxHelperFunctions */

/**
 * @}
 */
/* end - Helpers */

