#include "position_pid.h"

// 全局变量定义
PID_Params_t g_pos_pid_params[MOTOR_MAX] = {0};
PID_Pos_State_t g_pos_pid_state[MOTOR_MAX] = {0};
Vehicle_Pos_t g_vehicle_pos = {0};
const PID_Params_t g_default_pos_pid = {2.0f, 0.1f, 0.5f, 100};

/**
 * @brief 位置环PID初始化
 */
void pos_pid_init(void)
{
    for(int i = 0; i < MOTOR_MAX; i++)
    {
        // 设置默认PID参数
        g_pos_pid_params[i] = g_default_pos_pid;
        
        // 清零PID状态
        memset(&g_pos_pid_state[i], 0, sizeof(PID_Pos_State_t));
    }
    
    // 初始化位置
    g_vehicle_pos.target_x = 0;
    g_vehicle_pos.target_y = 0;
    g_vehicle_pos.current_x = 0;
    g_vehicle_pos.current_y = 0;
}

/**
 * @brief 计算整车位置（里程计）
 */
void calc_vehicle_position(void)
{
    // 计算各电机移动距离（mm）
    float fl_mm = PULSE_TO_MM(g_motor[MOTOR_LF].pos_now);
    float fr_mm = PULSE_TO_MM(g_motor[MOTOR_RF].pos_now);
    float bl_mm = PULSE_TO_MM(g_motor[MOTOR_LB].pos_now);
    float br_mm = PULSE_TO_MM(g_motor[MOTOR_RB].pos_now);
    
    // 麦克纳姆轮里程计解算
    g_vehicle_pos.current_x = (int32_t)((fl_mm - fr_mm + bl_mm - br_mm) / 4);
    g_vehicle_pos.current_y = (int32_t)((fl_mm + fr_mm + bl_mm + br_mm) / 4);
}

/**
 * @brief 设置电机位置目标
 * @param motor_id 电机编号
 * @param pos 目标位置（脉冲）
 */
void motor_set_pos_target(MotorID motor_id, int32_t pos)
{
    if(motor_id >= MOTOR_MAX) return;
    
    g_motor[motor_id].pos_target = pos;
    
    // 清零PID状态
    g_pos_pid_state[motor_id].err_sum = 0;
    g_pos_pid_state[motor_id].last_err = 0;
}

/**
 * @brief 位置环PID计算
 * @param motor_id 电机编号
 * @return PID输出（速度目标 mm/s）
 */
int32_t pos_pid_calc(MotorID motor_id)
{
    if(motor_id >= MOTOR_MAX) return 0;
    
    PID_Params_t *pid_params = &g_pos_pid_params[motor_id];
    PID_Pos_State_t *pid_state = &g_pos_pid_state[motor_id];
    Motor_Control_t *motor = &g_motor[motor_id];
    
    // 计算当前误差（目标位置 - 实际位置）
    pid_state->err = motor->pos_target - motor->pos_now;
    
    // 死区处理
    if(abs(pid_state->err) < POS_ZERO_OFFSET)
    {
        pid_state->err = 0;
        pid_state->output = 0;
        motor_set_speed_target(motor_id, 0);
        return 0;
    }
    
    // 积分项计算（带限幅）
    pid_state->err_sum += pid_state->err * PID_CONTROL_PERIOD / 1000.0f;
    pid_state->err_sum = LIMIT_ABS(pid_state->err_sum, pid_params->i_limit);
    
    // 微分项计算
    float derivative = (pid_state->err - pid_state->last_err) / (PID_CONTROL_PERIOD / 1000.0f);
    
    // PID输出（速度目标，转换为mm/s）
    pid_state->output = (int32_t)(pid_params->kp * pid_state->err / PULSE_PER_MM + 
                                 pid_params->ki * pid_state->err_sum / PULSE_PER_MM + 
                                 pid_params->kd * derivative / PULSE_PER_MM);
    
    // 加速度限制
    static int32_t last_output[MOTOR_MAX] = {0};
    int32_t delta = pid_state->output - last_output[motor_id];
    delta = LIMIT_ABS(delta, MAX_ACCEL * PID_CONTROL_PERIOD / 1000);
    pid_state->output = last_output[motor_id] + delta;
    last_output[motor_id] = pid_state->output;
    
    // 速度限幅
    pid_state->output = LIMIT_ABS(pid_state->output, SPEED_MAX_LIMIT);
    
    // 减速处理（剩余距离小于减速阈值时）
    if(abs(pid_state->err) < MM_TO_PULSE(DECEL_DISTANCE))
    {
        float decel_ratio = (float)abs(pid_state->err) / MM_TO_PULSE(DECEL_DISTANCE);
        pid_state->output = (int32_t)(pid_state->output * decel_ratio);
    }
    
    // 更新速度目标
    motor_set_speed_target(motor_id, pid_state->output);
    
    // 更新上一次误差
    pid_state->last_err = pid_state->err;
    
    return pid_state->output;
}

/**
 * @brief 检查是否到达位置目标
 * @return true=到达，false=未到达
 */
bool check_pos_arrived(void)
{
    // 检查位置误差
    int32_t err_x = abs(g_vehicle_pos.target_x - g_vehicle_pos.current_x);
    int32_t err_y = abs(g_vehicle_pos.target_y - g_vehicle_pos.current_y);
    
    // 检查所有电机位置误差
    bool all_motor_arrived = true;
    for(int i = 0; i < MOTOR_MAX; i++)
    {
        if(abs(g_motor[i].pos_target - g_motor[i].pos_now) > POS_ZERO_OFFSET)
        {
            all_motor_arrived = false;
            break;
        }
    }
    
    return (err_x < POS_ZERO_OFFSET && err_y < POS_ZERO_OFFSET) || all_motor_arrived;
}