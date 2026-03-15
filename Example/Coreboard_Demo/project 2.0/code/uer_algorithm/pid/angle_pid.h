#ifndef ANGLE_PID_H
#define ANGLE_PID_H

#include "motor_driver.h"


typedef struct {
    int32_t kp;           // 比例系数
    int32_t ki;           // 积分系数
    int32_t kd;           // 微分系数
    int32_t integral;     // 积分项
    int32_t last_error;   // 上一次误差
} MecanumPID_t;




static int32_t Mecanum_PIDCalc(MecanumPID_t *pid, int32_t target, int32_t current) {
    if (pid == NULL) return 0;
    
    // 1. 计算误差
    int32_t error = target - current;
    
    // 2. 死区过滤（零漂阈值）
    if (abs(error) <= SPEED_ZERO_OFFSET) {
        pid->integral = 0;
        pid->last_error = 0;
        return 0;
    }
    
    // 3. 积分项计算（限幅避免饱和）
    pid->integral += error * PID_CONTROL_PERIOD; // 积分=误差×周期（10ms）
    if (pid->integral > pid->max_output * 10 / pid->ki) { // 放大系数补偿
        pid->integral = pid->max_output * 10 / pid->ki;
    } else if (pid->integral < pid->min_output * 10 / pid->ki) {
        pid->integral = pid->min_output * 10 / pid->ki;
    }
    
    // 4. PID输出计算（参数放大10倍，最终除以10恢复）
    int32_t output = (pid->kp * error + pid->ki * pid->integral + 
                      pid->kd * (error - pid->last_error)) / 10;
    
    // 5. 输出限幅
    output = (output > pid->max_output) ? pid->max_output : output;
    output = (output < pid->min_output) ? pid->min_output : output;
    
    // 6. 更新上一次误差
    pid->last_error = error;
    
    return output;
}
















#endif
