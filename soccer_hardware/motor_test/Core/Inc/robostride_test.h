#ifndef ROBOSTRIDE
#define ROBOSTRIDE
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"
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
    uint32_t id   : 8;
    uint32_t data : 16;
    uint32_t mode : 5;
    uint32_t res  : 3;
} exCanIdInfo;



#endif
