#include "uart_dbg.h"


set_cmd_t set_cmd_list[] = {
    {"pos", 0.0},
    {"rpm", 0.0},
    {"kp" , 0.0},
    {"kd" , 0.0},
    {"torq", 0.0}
};

HAL_StatusTypeDef uart_get_new_line(UART_HandleTypeDef* phuart, char* msg_buf, int buf_maxlen)
{
    int n = 0;
    char ch;
    while (n < buf_maxlen - 1){
        if (HAL_UART_Receive(phuart, (uint8_t*)&ch, 1, HAL_MAX_DELAY) != HAL_OK) {return HAL_ERROR;}
        msg_buf[n] = ch;
        n ++;
        if (ch == '\n' || ch == '\r'){
            break;
        }
    }

    msg_buf[n] = '\0';
    return HAL_OK;
}

/*
Available UART cmds
cubemars demo -> run cubemars demo program
         start -> enter mit mode
         stop -> exit from mit mode
         read -> request parameters from the motor
         send -> send MIT parameters to the motor
         set <pos> <rpm> <kp> <kd> <torq> -> all floats
*/

void uart_parse_cmd(UART_HandleTypeDef* phuart, char* msg, int msg_len)
{
    char copy[100];
    strncpy(copy, msg, msg_len);
    copy[msg_len] = '\0';
    char* token = strtok(copy, " ");
    if (!token){
        snprintf(copy, sizeof(copy), "ERROR: empty cmd\n\r");
        HAL_UART_Transmit(phuart, copy, strlen(copy), HAL_MAX_DELAY);
        return;
    }

    if (!strncmp(token, "cubemars", strlen(token))){
        //cubemars motor
        token = strtok(NULL, " \n\r");
        if (!strncmp(token, "demo", strlen(token))){
            //exe demo
        }
        else if (!strncmp(token, "start", strlen(token))){
            snprintf(copy, sizeof(copy), "Enable Cubemars MIT control\n\r");
            HAL_UART_Transmit(phuart, copy, strlen(copy), HAL_MAX_DELAY);
            CUBEMARS_enable_motion_ctrl(&hcan1, &TxMailbox);
        }
        else if (!strncmp(token, "read", strlen(token))){
            snprintf(copy, sizeof(copy), "Requested Cubemars parameters\n\r");
            HAL_UART_Transmit(phuart, copy, strlen(copy), HAL_MAX_DELAY);
            CUBEMARS_enable_motion_ctrl(&hcan1, &TxMailbox);
            uint32_t timeout = 1000;
            while(!can_receive_flag){
                timeout --;
                if (!timeout){
                    snprintf(copy, sizeof(copy), "CAN read timeout\n\r");
                    HAL_UART_Transmit(phuart, copy, strlen(copy), HAL_MAX_DELAY);
                    return;
                }
            }
            can_receive_flag = 0;
            snprintf(copy, sizeof(copy), "Motor Report:\r\nTEMP: %d | ERROR_CODE: %0x\r\nPOS: %d | SPD: %d | TORQ: %d\r\n",
                (int)motor_temp, error_code, (int)pos_float, (int)spd_float, (int)torq_float);
            HAL_UART_Transmit(phuart, copy, strlen(copy), HAL_MAX_DELAY);
        }
        else if (!strncmp(token, "stop", strlen(token))){
            snprintf(copy, sizeof(copy), "Disable Cubemars MIT control\n\r");
            HAL_UART_Transmit(phuart, copy, strlen(copy), HAL_MAX_DELAY);
            CUBEMARS_disable_motion_ctrl(&hcan1, &TxMailbox);
        }
        else if (!strncmp(token, "send", strlen(token))){
            snprintf(copy, sizeof(copy), "Cubemars send MIT parameters: pos=%d, rpm=%d, kp=%d, kd=%d, torq=%d\n\r",
                (int)set_cmd_list[0].motor_config,
                (int)set_cmd_list[1].motor_config,
                (int)set_cmd_list[2].motor_config,
                (int)set_cmd_list[3].motor_config,
                (int)set_cmd_list[4].motor_config);
            HAL_UART_Transmit(phuart, copy, strlen(copy), HAL_MAX_DELAY);
            CUBEMARS_set_motion_ctrl_parameters(&hcan1, &TxMailbox, set_cmd_list[0].motor_config, set_cmd_list[1].motor_config, set_cmd_list[2].motor_config, set_cmd_list[3].motor_config, set_cmd_list[4].motor_config);
        }
        else if (!strncmp(token, "set", strlen(token))){
            char* set_type = strtok(NULL, " \n\r");
            char* config = strtok(NULL, " \n\r");
            
            if (!set_type){
                snprintf(copy, sizeof(copy), "ERROR: Missing set type\n\r");
                HAL_UART_Transmit(phuart, copy, strlen(copy), HAL_MAX_DELAY);
            }

            if (!config){
                snprintf(copy, sizeof(copy), "ERROR: Missing set config\n\r");
                HAL_UART_Transmit(phuart, copy, strlen(copy), HAL_MAX_DELAY);
            }

            float config_float = atof(config);

            for (int i = 0; i< sizeof set_cmd_list; i ++){
                if (!strncmp(set_type, set_cmd_list[i].cmd, strlen(set_type))){
                    set_cmd_list[i].motor_config = config_float;
                    snprintf(copy, sizeof(copy), "Cubemars set: %s = %.3f\n\r", set_cmd_list[i].cmd, set_cmd_list[i].motor_config);
                    HAL_UART_Transmit(&huart2, copy, strlen(copy), HAL_MAX_DELAY);
                    return;
                }
            }
            snprintf(copy, sizeof(copy), "Invalid set type\n\rYou should only input: \n\r");
            HAL_UART_Transmit(phuart, copy, strlen(copy), HAL_MAX_DELAY);
            for (int i = 0; i < sizeof set_cmd_list; i ++){
                snprintf(copy, sizeof(copy), "%s <float> \n\r", set_cmd_list[i].cmd);
                HAL_UART_Transmit(phuart, copy, strlen(copy), HAL_MAX_DELAY);
            }
            return;
   
        }
        else{
            snprintf(copy, sizeof(copy), "ERROR: invalid motor cmd\n\r");
            HAL_UART_Transmit(phuart, copy, strlen(copy), HAL_MAX_DELAY);
            return;
        }

    }
    else if(!strncmp(token, "rs", strlen(token))){
        token = strtok(NULL, " \n\r");
        if (!strncmp(token, "demo", strlen(token))){
            //exe demo
        }
        else if (!strncmp(token, "start", strlen(token))){
            snprintf(copy, sizeof(copy), "Enable rs motor\n\r");
            HAL_UART_Transmit(phuart, copy, strlen(copy), HAL_MAX_DELAY);
            can_enable_motor(RS_test_motor_id, CAN_master_id);
        }
        else if (!strncmp(token, "stop", strlen(token))){
            snprintf(copy, sizeof(copy), "Disable rs motor\n\r");
            HAL_UART_Transmit(phuart, copy, strlen(copy), HAL_MAX_DELAY);
            can_disable_motor(RS_test_motor_id, CAN_master_id);
        }
        else if (!strncmp(token, "send", strlen(token))){
            snprintf(copy, sizeof(copy), "rs send MIT parameters: pos=%.3f, rpm=%.3f, kp=%.3f, kd=%.3f, torq=%.3f\n\r",
                set_cmd_list[0].motor_config,
                set_cmd_list[1].motor_config,
                set_cmd_list[2].motor_config,
                set_cmd_list[3].motor_config,
                set_cmd_list[4].motor_config);
            HAL_UART_Transmit(phuart, copy, strlen(copy), HAL_MAX_DELAY);
            can_mit_control_set(RS_test_motor_id, set_cmd_list[4].motor_config, set_cmd_list[0].motor_config, set_cmd_list[1].motor_config,set_cmd_list[2].motor_config, set_cmd_list[3].motor_config);
        }
        else if (!strncmp(token, "set", strlen(token))){
            char* set_type = strtok(NULL, " \n\r");
            char* config = strtok(NULL, " \n\r");

            if (!set_type){
                snprintf(copy, sizeof(copy), "ERROR: Missing set type\n\r");
                HAL_UART_Transmit(phuart, copy, strlen(copy), HAL_MAX_DELAY);
                return;
            }

            if (!config){
                snprintf(copy, sizeof(copy), "ERROR: Missing set config\n\r");
                HAL_UART_Transmit(phuart, copy, strlen(copy), HAL_MAX_DELAY);
                return;
            }

            float config_float = atof(config);

            for (int i = 0; i< sizeof set_cmd_list; i ++){
                if (!strncmp(set_type, set_cmd_list[i].cmd, strlen(set_type))){
                    set_cmd_list[i].motor_config = config_float;
                    snprintf(copy, sizeof(copy), "rs set: %s = %.3f\n\r", set_cmd_list[i].cmd, set_cmd_list[i].motor_config);
                    HAL_UART_Transmit(&huart2, copy, strlen(copy), HAL_MAX_DELAY);
                    return;
                }
            }
            snprintf(copy, sizeof(copy), "Invalid set type\n\r");
            HAL_UART_Transmit(phuart, copy, strlen(copy), HAL_MAX_DELAY);

        }
        else{
            snprintf(copy, sizeof(copy), "ERROR: invalid motor cmd\n\r");
            HAL_UART_Transmit(phuart, copy, strlen(copy), HAL_MAX_DELAY);
            return;
        }
    }

    else{
        snprintf(copy, sizeof(copy), "ERROR: invalid motor series\n\r");
        HAL_UART_Transmit(phuart, copy, strlen(copy), HAL_MAX_DELAY);
        return;
    }

}
