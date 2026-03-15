#ifndef SPEED_PID_H
#define SPEED_PID_H

#include "motor_driver.h"
#include "encoder.h"






typedef struct {
    int32_t integral;     // 积分项
    int32_t err;          // 误差
    int32_t last_error;   // 上一次误差
} Speed_PID_t;




#endif // SPEED_H