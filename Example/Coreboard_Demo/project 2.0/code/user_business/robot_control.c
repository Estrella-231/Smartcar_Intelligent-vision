#include "robot_control.h"

// 全局变量定义
Vehicle_Control_t g_vehicle_ctrl = {0};
int32_t g_adaptive_max_speed = SPEED_MAX_LIMIT;
int32_t g_current_speed = 0;

/**
 * @brief 机器人控制系统初始化
 */
void robot_control_init(void)
{
    // 初始化底层驱动
    motor_driver_init();
    Encoder_Init();
    speed_pid_init();
    pos_pid_init();
    
    // 初始化整车控制参数
    g_vehicle_ctrl.count_time = 0;
    g_vehicle_ctrl.global_speed = SPEED_MAX_LIMIT;
    g_vehicle_ctrl.enable_closed_loop = true;
    g_vehicle_ctrl.current_dir = MOVE_STOP;
    g_vehicle_ctrl.move_state = MOVE_STATE_IDLE;
    
    // 初始化全局速度参数
    g_adaptive_max_speed = SPEED_MAX_LIMIT;
    g_current_speed = 0;
    
    // 停止所有电机
    mecanum_stop();
}

/**
 * @brief 机器人控制主循环（10ms调用一次）
 */
void robot_control_loop(void)
{
    // 更新全局计时器
    g_vehicle_ctrl.count_time += PID_CONTROL_PERIOD;
    
    // 计算整车位置
    calc_vehicle_position();
    
    // 检查是否到达位置目标
    if(g_vehicle_ctrl.move_state == MOVE_STATE_RUNNING && check_pos_arrived())
    {
        mecanum_stop();
        g_vehicle_ctrl.move_state = MOVE_STATE_FINISH;
        return;
    }
    
    // 闭环控制计算
    if(g_vehicle_ctrl.enable_closed_loop)
    {
        // 位置环计算（生成速度目标）
        for(int i = 0; i < MOTOR_MAX; i++)
        {
            pos_pid_calc((MotorID)i);
        }
        
        // 速度环计算（生成PWM输出）
        for(int i = 0; i < MOTOR_MAX; i++)
        {
            int32_t pwm = speed_pid_calc((MotorID)i);
            motor_set_pwm((MotorID)i, abs(pwm));
        }
    }
}

/**
 * @brief 紧急停止
 */
void robot_emergency_stop(void)
{
    mecanum_stop();
    g_vehicle_ctrl.move_state = MOVE_STATE_IDLE;
    g_vehicle_ctrl.current_dir = MOVE_STOP;
    g_current_speed = 0;
}


// 将车头转向前进方向