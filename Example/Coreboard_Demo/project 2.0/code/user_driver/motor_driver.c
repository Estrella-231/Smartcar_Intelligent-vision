#include "motor_driver.h"

// 全局变量定义
Motor_Control_t g_motor[MOTOR_MAX] = {0};
pwm_channel_enum channel_list[MOTOR_MAX] = {MOTOR1_PWM, MOTOR2_PWM, MOTOR3_PWM, MOTOR4_PWM};

/**
 * @brief 单个电机硬件初始化
 * @param motor_id 电机编号
 */
void motor_hw_init(MotorID motor_id)
{
    if(motor_id >= MOTOR_MAX) return;
    
    // 配置电机硬件引脚
    switch(motor_id)
    {
        case MOTOR_LF:
            g_motor[motor_id].hw.dir_pin = MOTOR1_DIR;
            g_motor[motor_id].hw.pwm_pin = MOTOR1_PWM;
            break;
        case MOTOR_RF:
            g_motor[motor_id].hw.dir_pin = MOTOR2_DIR;
            g_motor[motor_id].hw.pwm_pin = MOTOR2_PWM;
            break;
        case MOTOR_LB:
            g_motor[motor_id].hw.dir_pin = MOTOR3_DIR;
            g_motor[motor_id].hw.pwm_pin = MOTOR3_PWM;
            break;
        case MOTOR_RB:
            g_motor[motor_id].hw.dir_pin = MOTOR4_DIR;
            g_motor[motor_id].hw.pwm_pin = MOTOR4_PWM;
            break;
        default:
            return;
    }
    
    // 初始化PWM（20kHz频率，0占空比）
    pwm_init(g_motor[motor_id].hw.pwm_pin, PWM_FREQUENCY, 0);
    
    // 初始化电机控制参数
    g_motor[motor_id].speed_target = 0;
    g_motor[motor_id].speed_now = 0;
    g_motor[motor_id].pos_target = 0;
    g_motor[motor_id].pos_now = 0;
    g_motor[motor_id].pwm_out = 0;
    g_motor[motor_id].hw.dir_level = true; // 默认高电平正转
}

/* @brief 电机驱动初始化（所有电机）*/
void motor_driver_init(void)
{
    for(int i = 0; i < MOTOR_MAX; i++)
    {
        motor_hw_init((MotorID)i);
    }
    motor_stop_all();
}

/**
 * @brief 设置电机PWM
 * @param motor_id 电机编号
 * @param pwm pwm（0-1000）
 */
void motor_set_pwm(MotorID motor_id, int32_t pwm)
{
    if(motor_id >= MOTOR_MAX) return;

    // 设置PWM占空比
    pwm_set_duty(channel_list[motor_id], pwm);
    
    // 更新实际输出PWM
    g_motor[motor_id].pwm_out = pwm;
}

/**
 * @brief 电机停止
 * @param motor_id 电机编号
 */
void motor_stop(MotorID motor_id)
{
    motor_set_pwm((MotorID)motor_id, 0);
    g_motor[motor_id].speed_target = 0;
    g_motor[motor_id].pwm_out = 0;    

}

/* @brief 停止所有电机 */
void motor_stop_all(void)
{
    for(int i = 0; i < MOTOR_MAX; i++)
    {
        motor_stop(i);
    }
}

/**
 * @brief 获取电机的当前位置
 * @param motor_id 电机编号
 * @return int32_t 电机当前位置
 */
int32_t get_pos_now(MotorID motor_id)
{
    return g_motor[motor_id].pos_now;
}
 
/**
 * @brief 更改电机的当前位置
 * @param motor_id 电机编号
 * @param position 更新后的当前位置
 */
void change_pos_now(MotorID motor_id, int32_t position)
{
    g_motor[motor_id].pos_now = position;
}

/**
 * @brief 获取电机的当前速度
 * @param motor_id 电机编号
 * @return int32_t 电机当前速度
 */
int32_t get_speed_now(MotorID motor_id)
{
    return g_motor[motor_id].speed_now;
}
 
/**
 * @brief 更改电机的当前速度
 * @param motor_id 电机编号
 * @param position 更新后的当前速度
 */
void change_pos_now(MotorID motor_id, int32_t speed)
{
    g_motor[motor_id].speed_now = speed;
}

/**
 * @brief 更改电机的目标速度
 * @param motor_id 电机编号
 * @param position 更新后的目标速度
 */
void change_speed_target(MotorID motor_id, int32_t speed)
{
    g_motor[motor_id].speed_target = speed;
}

/**
 * @brief 更改电机的目标位置
 * @param motor_id 电机编号
 * @param position 更新后的目标位置
 */
void change_speed_target(MotorID motor_id, int32_t position)
{
    g_motor[motor_id].pos_target = position;
}
