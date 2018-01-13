#include <serial.h>
#include <packet.h>
#include <string.h>

void send_state(RobotState* robotstate) {
	// Encapsulates the data into a packet
	Packet p, *p_ptr;
	p_ptr = &p;
	p.start_pattern = START_PATTERN;
	p.byte_count = sizeof(RobotState);
	memcpy(&p.data, robotstate, sizeof(RobotState));

	HAL_UART_AbortReceive_IT(&huart2);
	// Transmit cycle

	int size = sizeof(RobotState) + 8;
	int i = 0;
	while(size > 0) {
		uint8_t* c = (uint8_t * ) (p_ptr) + i;
		HAL_UART_Transmit(&huart2, (uint8_t * ) (p_ptr) + i, sizeof(uint8_t), 10);
		size = size - 1;
		i++;
	}

	HAL_UART_Receive_IT(&huart2, robotGoalBuffer, sizeof(RobotGoal));
}

RobotGoal receive_state() {
	RobotGoal r;

	return r;
}

