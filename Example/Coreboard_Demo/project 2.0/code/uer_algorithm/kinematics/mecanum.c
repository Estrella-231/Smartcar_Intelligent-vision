#include "mecanum.h"


   
// 整车目标速度 -> 4电机目标速度
void Mecanum_entirety2part(MoveDirection Car_direction, int32_t Car_target_speed) 
{
    // mm/s → 脉冲/秒；0.1°/s → 脉冲/秒
    int32_t v_pulse = Car_target_speed * PULSE_PER_MM;
    int32_t w_pulse = (Car_target_speed * 314) / (1800 * 10);   

    switch(Car_direction)
    {
        case MOVE_STOP:
            g_motor[MOTOR_LF].speed_target = 0;
            g_motor[MOTOR_RF].speed_target = 0;
            g_motor[MOTOR_LB].speed_target = 0;
            g_motor[MOTOR_RB].speed_target = 0; 
            break;
        case MOVE_FORWARD:
            g_motor[MOTOR_LF].speed_target = v_pulse;
            g_motor[MOTOR_RF].speed_target = v_pulse;
            g_motor[MOTOR_LB].speed_target = v_pulse;
            g_motor[MOTOR_RB].speed_target = v_pulse; 
            break;
        case MOVE_BACKWARD:
            g_motor[MOTOR_LF].speed_target = -v_pulse;
            g_motor[MOTOR_RF].speed_target = -v_pulse;
            g_motor[MOTOR_LB].speed_target = -v_pulse;
            g_motor[MOTOR_RB].speed_target = -v_pulse;    
            break;
        case MOVE_ROTATE_CW:
            g_motor[MOTOR_LF].speed_target = w_pulse;
            g_motor[MOTOR_RF].speed_target = -w_pulse;
            g_motor[MOTOR_LB].speed_target = w_pulse;
            g_motor[MOTOR_RB].speed_target = -w_pulse;
            break;
        case MOVE_ROTATE_CCW:
            g_motor[MOTOR_LF].speed_target = -w_pulse;
            g_motor[MOTOR_RF].speed_target = w_pulse;
            g_motor[MOTOR_LB].speed_target = -w_pulse;
            g_motor[MOTOR_RB].speed_target = w_pulse;    
            break;
        default:
            // 预留状态或非法状态 → 停止
            g_motor[MOTOR_LF].speed_target = 0;
            g_motor[MOTOR_RF].speed_target = 0;
            g_motor[MOTOR_LB].speed_target = 0;
            g_motor[MOTOR_RB].speed_target = 0; 
            break;      
    }
}

// 运动学正解：4电机速度 → 车体合成速度
void Mecanum_part2entirety(MoveDirection Car_direction) {
    if (g_motor[MOTOR_LF].speed_now == 0 &&
        g_motor[MOTOR_RF].speed_now == 0 &&
        g_motor[MOTOR_LB].speed_now == 0 &&
        g_motor[MOTOR_RB].speed_now == 0){
        g_car.speed_now = 0;
    }
    
    // 读取电机当前速度（脉冲/秒）
    int32_t lf = g_motor[MOTOR_LF].speed_now;
    int32_t rf = g_motor[MOTOR_RF].speed_now;
    int32_t lb = g_motor[MOTOR_LB].speed_now;
    int32_t rb = g_motor[MOTOR_RB].speed_now;
    // 正解公式：电机速度 → 车体速度（脉冲/秒）
    int32_t v_pulse = (lf + rf + lb + rb) / 4;
    int32_t w_pulse = (-lf - rf + lb + rb) / 4;
    
    if(Car_direction == MOVE_FORWARD || Car_direction == MOVE_BACKWARD) g_car.speed_now = v_pulse;
    else if(Car_direction == MOVE_ROTATE_CW || Car_direction == MOVE_ROTATE_CCW)  g_car.speed_now = w_pulse;
    else g_car.speed_now = 0;
}

// 计算当前速度
void Calc_SpeedNow(MotorID motor_id){

    if (motor_id >= 4) return 1;

    // 计算出的原始速度（mm/s）
    int32_t speed_calc;        
    // 10ms内脉冲增量
    int32_t pos_diff = g_motor[motor_id].pos_now - g_motor[motor_id].pos_last;

    // 处理编码器溢出导致的异常差值（如从32767→-32768，差值应为-1，而非-65535）
    if (pos_diff > ENCODER_PULSE_MAX) {
        pos_diff -= (ENCODER_PULSE_MAX - ENCODER_PULSE_MIN + 1);
    } else if (pos_diff < ENCODER_PULSE_MIN) {
        pos_diff += (ENCODER_PULSE_MAX - ENCODER_PULSE_MIN + 1);
    }   

    // 计算当前速度
    speed_calc = (pos_diff * 1000) / (PULSE_PER_MM * PID_CONTROL_PERIOD);
    // 速度低通滤波（减少抖动）
    g_motor[motor_id].speed_now = (SPEED_FILTER_FACTOR * g_motor[motor_id].speed_last + (10 - SPEED_FILTER_FACTOR) * speed_calc) / 10;

    g_motor[motor_id].pos_last = g_motor[motor_id].pos_now;
}



