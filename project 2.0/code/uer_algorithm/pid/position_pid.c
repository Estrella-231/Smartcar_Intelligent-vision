// 位置环PID控制模块
// 根据目标位置和当前位置计算速度目标值

#include "position_pid.h"

PID_Pram_t g_position_pid[MOTOR_MAX] = {0};

void position_pid_reset(MotorID motor_id)
{
    if(motor_id >= MOTOR_MAX)
    {
        return;
    }

    g_position_pid[motor_id].integral = 0.0f;
    g_position_pid[motor_id].err = 0;
    g_position_pid[motor_id].last_error = 0;
    g_position_pid[motor_id].derivative_lpf = 0.0f;
    g_position_pid[motor_id].output = 0;
    g_position_pid[motor_id].last_output = 0;
}

void position_pid_reset_all(void)
{
    for(int i = 0; i < MOTOR_MAX; i++)
    {
        position_pid_reset((MotorID)i);
    }
}

/*
 * Legacy wheel-position outer loop.
 *
 * This module is currently not part of the active motion-control path. The
 * project now drives motion through chassis-level point_move + angle_pid +
 * speed_pid. Keep this function only as a preserved alternative implementation.
 */
void pos_pid_calc(MotorID motor_id)
{
    int32_t output_step_limit;
    int32_t raw_output;
    int32_t delta;

    if(motor_id >= MOTOR_MAX)
    {
        return;
    }

    g_position_pid[motor_id].err = g_motor[motor_id].pos_target - g_motor[motor_id].pos_now;

    if(abs(g_position_pid[motor_id].err) < POSITION_DEAD_ZONE)
    {
        position_pid_reset(motor_id);
        change_speed_target(motor_id, 0);
        return;
    }

    g_position_pid[motor_id].integral +=
        g_position_pid[motor_id].err * PID_CONTROL_PERIOD / 1000.0f;
    g_position_pid[motor_id].integral =
        LIMIT_ABS(g_position_pid[motor_id].integral, POSITION_PID_I_LIMIT);

    float derivative =
        (g_position_pid[motor_id].err - g_position_pid[motor_id].last_error) /
        (PID_CONTROL_PERIOD / 1000.0f);

    raw_output =
        (int32_t)(POSITION_PID_KP * g_position_pid[motor_id].err / PULSE_PER_MM +
                  POSITION_PID_KI * g_position_pid[motor_id].integral / PULSE_PER_MM +
                  POSITION_PID_KD * derivative / PULSE_PER_MM);

    output_step_limit =
        (int32_t)((float)SPEED_MAX_ACCEL * (float)PID_CONTROL_PERIOD / 1000.0f + 0.5f);
    if(output_step_limit < 1)
    {
        output_step_limit = 1;
    }

    delta = raw_output - g_position_pid[motor_id].last_output;
    delta = LIMIT_ABS(delta, output_step_limit);
    g_position_pid[motor_id].output = g_position_pid[motor_id].last_output + delta;

    if(abs(g_position_pid[motor_id].err) < MM_TO_PULSE(DECEL_DISTANCE))
    {
        float decel_ratio =
            (float)abs(g_position_pid[motor_id].err) / MM_TO_PULSE(DECEL_DISTANCE);
        g_position_pid[motor_id].output =
            (int32_t)(g_position_pid[motor_id].output * decel_ratio);
    }

    change_speed_target(motor_id, g_position_pid[motor_id].output);
    g_position_pid[motor_id].last_error = g_position_pid[motor_id].err;
    g_position_pid[motor_id].last_output = g_position_pid[motor_id].output;
}

/*
 * Legacy arrival check for the preserved position-loop path.
 *
 * Task-4 refactor note:
 * query aggregate car state and wheel targets through accessors so this
 * algorithm file is less tightly coupled to the raw global storage layout.
 */
bool check_pos_arrived(void)
{
    int32_t err_x = abs(car_get_x_target() - car_get_x_now());
    int32_t err_y = abs(car_get_y_target() - car_get_y_now());

    bool all_motor_arrived = true;
    for(int i = 0; i < MOTOR_MAX; i++)
    {
        if(abs(get_pos_target((MotorID)i) - get_pos_now((MotorID)i)) > POS_ZERO_OFFSET)
        {
            all_motor_arrived = false;
            break;
        }
    }

    return (err_x < POS_ZERO_OFFSET && err_y < POS_ZERO_OFFSET) || all_motor_arrived;
}
