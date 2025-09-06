#ifndef UART_DBG
#define UART_DBG
#include "main.h"

typedef struct{
    char* cmd;
    float motor_config;
} set_cmd_t;

HAL_StatusTypeDef uart_get_new_line(UART_HandleTypeDef* phuart, char* msg_buf, int buf_maxlen);
void uart_parse_cmd(UART_HandleTypeDef* phuart, char* msg, int msg_len);

#endif