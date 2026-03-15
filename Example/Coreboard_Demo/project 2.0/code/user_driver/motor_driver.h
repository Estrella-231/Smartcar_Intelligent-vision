#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "hardware_config.h"
#include "robot_param.h"

/************************ 电机硬件配置结构体 ************************/
typedef struct {
    uint8_t dir_pin;   // 方向引脚编号
    uint8_t pwm_pin;   // PWM引脚编号
    bool dir_level;    // 正转电平（true=高电平，false=低电平）
} Motor_HwConfig_t;

/************************ 电机控制结构体 ************************/
typedef struct {
    Motor_HwConfig_t hw;       // 硬件配置
    int32_t speed_target;      // 目标速度（mm/s）
    int32_t speed_now;         // 当前速度（mm/s）
    int32_t speed_last;        // 之前速度
    int32_t pos_target;        // 目标位置（脉冲）
    int32_t pos_now;           // 当前位置（脉冲）
    int32_t pos_last;          // 之前位置
    int32_t pwm_out;           // 实际输出PWM值
} Motor_Control_t;

/************************ 全局变量声明 ************************/
extern Motor_Control_t g_motor[MOTOR_MAX];

/************************ 函数声明 ************************/
void motor_driver_init(void);                                        // 电机驱动初始化（所有电机）
void motor_hw_init(MotorID motor_id);                                // 单个电机硬件初始化
void motor_set_pwm(MotorID motor_id, int32_t pwm);                   // 设置电机PWM
void motor_stop(MotorID motor_id);                                   // 停止某一电机
void motor_stop_all(void);                                           // 停止所有电机
int32_t get_pos_now(MotorID motor_id);                               // 获取电机的当前位置
void change_pos_now(MotorID motor_id, int32_t position);             // 更改电机的当前位置
void change_speed_target(MotorID motor_id, int32_t speed);           // 更改电机的目标位置
int32_t get_speed_now(MotorID motor_id);                             // 获取电机的当前速度
void change_pos_now(MotorID motor_id, int32_t speed);                // 更改电机的当前速度
void change_speed_target(MotorID motor_id, int32_t speed);           // 更改电机的目标位置

#endif // MOTOR_DRIVER_H
