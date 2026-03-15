#include "odometry.h"




// 整车目标坐标 -> 4电机目标位置
void Odometry_entirety2part(int32_t x, int32_t y) 
{
    int32_t dx = x - g_car.x_now;
    int32_t dy = y - g_car.y_now;
    int32_t forward_pulse =  MM_TO_PULSE(dx +dy);
    g_motor[MOTOR_LF].pos_target = forward_pulse;
    g_motor[MOTOR_RF].pos_target = forward_pulse;
    g_motor[MOTOR_LB].pos_target = forward_pulse;
    g_motor[MOTOR_RB].pos_target = forward_pulse; 
}

void update_coord(int32_t x, int32_t y){
    g_car.x_now = x;
    g_car.y_now = y;
}

// 目标角度 -> 电机
void Odometry_rotate(int32_t angle){
    // 计算角度差
    int32_t da = (g_car.angle_now - angle)*100;
    // 计算弧长
    int32_t arc_length = (da * PI * WHEEL_BASE) / 18000;   
    // 弧长->脉冲
    int32_t rotate_pulse = arc_length * PULSE_PER_MM;

    g_motor[MOTOR_LF].pos_target = g_motor[MOTOR_LF].pos_now + rotate_pulse;
    g_motor[MOTOR_RF].pos_target = g_motor[MOTOR_RF].pos_now - rotate_pulse;
    g_motor[MOTOR_LB].pos_target = g_motor[MOTOR_LB].pos_now + rotate_pulse;
    g_motor[MOTOR_RB].pos_target = g_motor[MOTOR_RB].pos_now - rotate_pulse; 
}

// 更新电机当前位置
void updata_PosNow(MotorID motor_id){
    g_motor[motor_id].pos_now = 
}