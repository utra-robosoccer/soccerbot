#include <serial.h>

void send_state(RobotState* robotstate) {
	HAL_UART_AbortReceive_IT(&huart2);
	HAL_UART_Transmit(&huart2, (uint8_t *) robotstate, sizeof(RobotState), 10);
	HAL_UART_Receive_IT(&huart2, robotGoalBuffer, sizeof(RobotGoal));
}

RobotGoal receive_state() {
	RobotGoal r;

	return r;
}
