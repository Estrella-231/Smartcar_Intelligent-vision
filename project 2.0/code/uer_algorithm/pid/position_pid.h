#ifndef POSITION_PID_H
#define POSITION_PID_H

#include "speed_pid.h"
#include "encoder.h"





typedef struct {
    int32_t kp;           // 比例系数
    int32_t ki;           // 积分系数
    int32_t kd;           // 微分系数
    int32_t integral;     // 积分项
    int32_t last_error;   // 上一次误差
} MecanumPID_t;









/************************ PID运行状态结构体 ************************/
/*
typedef struct {
    float err;         // 当前误差（目标-实际）
    float err_sum;     // 误差积分累计
    float last_err;    // 上一次误差
    int32_t output;    // PID输出值（速度目标 mm/s）
} PID_Pos_State_t;
*/
/************************ 位置控制结构体 ************************/
/*
typedef struct {
    int32_t target_x;  // 目标X坐标（mm）
    int32_t target_y;  // 目标Y坐标（mm）
    int32_t current_x; // 当前X坐标（mm）
    int32_t current_y; // 当前Y坐标（mm）
} Vehicle_Pos_t;
*/
/************************ 全局变量声明 ************************/
/*
extern PID_Params_t g_pos_pid_params[MOTOR_MAX];
extern PID_Pos_State_t g_pos_pid_state[MOTOR_MAX];
extern Vehicle_Pos_t g_vehicle_pos;
extern const PID_Params_t g_default_pos_pid; // 默认位置环PID参数
*/
/************************ 函数声明 ************************/
/*
void pos_pid_init(void);                                     // 位置环PID初始化
int32_t pos_pid_calc(MotorID motor_id);                      // 位置环PID计算
void calc_vehicle_position(void);                            // 计算整车位置（里程计）
void motor_set_pos_target(MotorID motor_id, int32_t pos);    // 设置电机位置目标
bool check_pos_arrived(void);                                // 检查是否到达位置目标
*/
#endif // POSITION_H