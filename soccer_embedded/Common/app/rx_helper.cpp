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
static uint8_t robot_goal_data[sizeof(comm::RobotGoal_t)];
static uint8_t *robot_goal_data_ptr;

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
    comm::RobotGoal_t& robot_goal = comm::getRobotGoal();
    robot_goal.id = 0;
    robot_goal_data_ptr = robot_goal_data;

    //receiving
    comm::RobotState_t& robot_state = comm::getRobotState();
    robot_state.id = 0;
    robot_state.start_seq = UINT32_MAX;
    robot_state.end_seq = 0;
}

// TODO: refactor this after researching more standard parsing techniques.
static RxParseState readByte(const uint8_t byte_in, bool& complete_out) {
    constexpr size_t SIZE_START_SEQ = 4;
    constexpr size_t SIZE_DATA = sizeof(comm::RobotGoal_t);
    static RxParseState state = RxParseState::CHECKING_HEADER;
    static size_t start_seq_count = 0;
    static size_t data_bytes_read = 0;
    RxParseState prev_state = state;

    switch(byte_in) {
    case 0xFF:
        if (state == RxParseState::CHECKING_HEADER) {
            start_seq_count++;
        }
        if (state == RxParseState::READING_DATA) {
            data_bytes_read++;
        }
        break;
    default:
        if (state == RxParseState::READING_DATA) {
            data_bytes_read++;
        }
        break;
    }

    if (state == RxParseState::CHECKING_HEADER && start_seq_count == SIZE_START_SEQ) {
        state = RxParseState::READING_DATA;
    }
    else if (state == RxParseState::READING_DATA && data_bytes_read == SIZE_DATA) {
        complete_out = true;
        state = RxParseState::CHECKING_HEADER;
        start_seq_count = 0;
        data_bytes_read = 0;
    }

    return prev_state;
}

void parseByteSequence(uint8_t *in_buff, size_t in_buff_size, bool& complete) {
    for (size_t i = 0; i < in_buff_size; i++) {
        if (readByte(in_buff[i], complete) == RxParseState::READING_DATA) {
            *(robot_goal_data_ptr++) = in_buff[i];
        }
        if (complete) {
            // Reset the variables to help with reception of a RobotGoal
            robot_goal_data_ptr = robot_goal_data;
            break;
        }
    }
}

void copyParsedData(void) {
    comm::RobotGoal_t& robot_goal = comm::getRobotGoal();
    comm::RobotState_t& robot_state = comm::getRobotState();

    robot_state.id = robot_goal.id;
    memcpy(&robot_goal, robot_goal_data, sizeof(comm::RobotGoal_t));
}

/**
 * @}
 */
/* end - RxHelperFunctions */

/**
 * @}
 */
/* end - Helpers */

