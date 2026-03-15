#include "speed_pid.h"

// 全局变量定义
Speed_PID_t g_speed_pid[MOTOR_MAX] = {0};

/**
 * @brief 速度环PID计算
 * @param motor_id 电机编号
 */
int32_t speed_pid_calc(MotorID motor_id)
{
    if(motor_id >= MOTOR_MAX) return 0;
    
    // 更新当前速度
    motor_update_speed(motor_id);
    
    // 计算当前误差（目标速度 - 实际速度）
    g_speed_pid[motor_id].err = g_motor[motor_id].speed_target - g_motor[motor_id].speed_now;
    
    // 死区处理
    if(abs(g_speed_pid[motor_id].err) < SPEED_DEAD_ZONE)
    {
        g_speed_pid[motor_id].err = 0;
        g_motor[motor_id].pwm_out = 0;
        return 0;
    }
    
    // 积分项计算（带限幅）
    g_speed_pid[motor_id].integral += g_speed_pid[motor_id].err * PID_CONTROL_PERIOD / 1000.0f;
    g_speed_pid[motor_id].integral = LIMIT_ABS(g_speed_pid[motor_id].integral, SPEED_PID_I_LIMIT);
    
    // 微分项计算
    float derivative = (g_speed_pid[motor_id].err - g_speed_pid[motor_id].last_error) / (PID_CONTROL_PERIOD / 1000.0f);
    
    // PID输出计算（映射到PWM）
    g_motor[motor_id].pwm_out = (int32_t)(SPEED_PID_KP * g_speed_pid[motor_id].err + 
                                 SPEED_PID_KI * g_speed_pid[motor_id].integral + 
                                 SPEED_PID_KD * derivative);
    
    // 输出限幅
    g_motor[motor_id].pwm_out = LIMIT_ABS(g_motor[motor_id].pwm_out, SPEED_PWM_MAX_OUTPUT);
    
    // 更新上一次误差
    g_speed_pid[motor_id].last_error = g_speed_pid[motor_id].err;
}