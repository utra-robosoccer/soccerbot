#ifndef ROBOSTRIDE
#define ROBOSTRIDE
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"
#include "cubemars.h"
#include "main.h"

#define P_MIN   -12.57f
#define P_MAX    12.57f
#define V_MIN   -20.0f
#define V_MAX    20.0f
#define KP_MIN    0.0f
#define KP_MAX 5000.0f
#define KD_MIN    0.0f
#define KD_MAX  100.0f
#define T_MIN   -60.0f
#define T_MAX    60.0f

typedef struct {
    uint32_t id   : 8; //Usually the Device ID
    uint32_t data : 16; //Master CAN ID
    uint32_t mode : 5; //Which Comm type
    uint32_t res  : 3; //Will never be used as can extended id is 29 bits
} exCanIdInfo;

typedef enum{
    MIT_MODE        = 0,
    POS_PP_MODE     = 1,
    VELOCITY_MODE   = 2,
    CURRENT_MODE    = 3,
    CSP_MODE        = 4
} rs_runmode_t;

#define MOTOR_ERROR_OFFSET_UNCALIBRATED    21
#define MOTOR_ERROR_OFFSET_STALL_OVERLOAD  20
#define MOTOR_ERROR_OFFSET_ENCODER_FAULT   19
#define MOTOR_ERROR_OFFSET_OVERHEAT        18
#define MOTOR_ERROR_OFFSET_DRIVER_FAULT    17
#define MOTOR_ERROR_OFFSET_UNDERVOLTAGE    16

#define MOTOR_ERROR_MASK_UNCALIBRATED    (1U << MOTOR_ERROR_OFFSET_UNCALIBRATED) // bit21: Uncalibrated
#define MOTOR_ERROR_MASK_STALL_OVERLOAD  (1U << MOTOR_ERROR_OFFSET_STALL_OVERLOAD) // bit20: Stall/Overload
#define MOTOR_ERROR_MASK_ENCODER_FAULT   (1U << MOTOR_ERROR_OFFSET_ENCODER_FAULT) // bit19: Magnetic encoder fault
#define MOTOR_ERROR_MASK_OVERHEAT        (1U << MOTOR_ERROR_OFFSET_OVERHEAT) // bit18: Overheat
#define MOTOR_ERROR_MASK_DRIVER_FAULT    (1U << MOTOR_ERROR_OFFSET_DRIVER_FAULT) // bit17: Driver fault
#define MOTOR_ERROR_MASK_UNDERVOLTAGE    (1U << MOTOR_ERROR_OFFSET_UNDERVOLTAGE) // bit16: Undervoltage fault

#define MOTOR_FEEDBACK_MODE_OFFSET    22
#define MOTOR_FEEDBACK_ID_OFFSET      8
#define MOTOR_FEEDBACK_MASK_MODE      (3U << MOTOR_FEEDBACK_MODE_OFFSET)
#define MOTOR_FEEDBACK_MASK_ID        (0xFFU << MOTOR_FEEDBACK_ID_OFFSET)

typedef enum{
    RS_MODE_RESET = 0,
    RS_MODE_CALI,
    RS_MODE_NORMAL
} motor_status_t;

typedef struct {
    uint8_t uncalibrated;
    uint8_t stall_overload;
    uint8_t encoder_fault;
    uint8_t overheat;
    uint8_t driver_fault;
    uint8_t undervoltage;
} motor_error_t;

typedef struct {
    uint8_t id;
    uint16_t master_id;
    motor_status_t status;
    motor_error_t motor_errors;
    rs_runmode_t motor_mode;

    float temperature; //motor temperature

    //mit parameters
    float pos; //rad
    float rpm; //rad/s
    float kp;
    float kd;
    float torq; //Nm
} motor_t;

extern CAN_RxHeaderTypeDef rs_can_rx_header;

HAL_StatusTypeDef can_enable_motor(uint8_t id, uint16_t master_id);
HAL_StatusTypeDef can_mit_control_set(uint8_t id, float torque, float MechPosition, float speed, float kp, float kd);
HAL_StatusTypeDef can_disable_motor(uint8_t id, uint16_t master_id);
HAL_StatusTypeDef can_change_motor_mode(uint8_t id, uint16_t master_id, rs_runmode_t rs_runmode);
HAL_StatusTypeDef can_unpack_motor_feedback(motor_t* motor, uint8_t* recv_buf);


#endif
