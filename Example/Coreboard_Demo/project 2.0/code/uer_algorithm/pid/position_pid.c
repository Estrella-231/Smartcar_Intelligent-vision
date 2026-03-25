#include "position_pid.h"

PID_Pram_t g_position_pid[MOTOR_MAX] = {0};

/*
 * Run the outer position loop for one wheel.
 *
 * The implementation is intentionally kept behavior-equivalent to the existing
 * project logic: position error is converted into a speed target, then limited
 * by acceleration and near-target deceleration rules.
 */
void pos_pid_calc(MotorID motor_id)
{
    g_position_pid[motor_id].err = g_motor[motor_id].pos_target - g_motor[motor_id].pos_now;

    if(abs(g_position_pid[motor_id].err) < POSITION_DEAD_ZONE)
    {
        g_position_pid[motor_id].err = 0;
        g_position_pid[motor_id].output = 0;
    }

    g_position_pid[motor_id].integral +=
        g_position_pid[motor_id].err * PID_CONTROL_PERIOD / 1000.0f;
    g_position_pid[motor_id].integral =
        LIMIT_ABS(g_position_pid[motor_id].integral, POSITION_PID_I_LIMIT);

    float derivative =
        (g_position_pid[motor_id].err - g_position_pid[motor_id].last_error) /
        (PID_CONTROL_PERIOD / 1000.0f);

    g_position_pid[motor_id].output =
        (int32_t)(POSITION_PID_KP * g_position_pid[motor_id].err / PULSE_PER_MM +
                  POSITION_PID_KI * g_position_pid[motor_id].integral / PULSE_PER_MM +
                  POSITION_PID_KD * derivative / PULSE_PER_MM);

    int32_t delta = g_position_pid[motor_id].output - g_position_pid[motor_id].last_output;
    delta = LIMIT_ABS(delta, SPEED_MAX_ACCEL * PID_CONTROL_PERIOD / 1000);
    g_position_pid[motor_id].output = g_position_pid[motor_id].last_output + delta;
    g_position_pid[motor_id].last_output = g_position_pid[motor_id].output;

    if(abs(g_position_pid[motor_id].err) < MM_TO_PULSE(DECEL_DISTANCE))
    {
        float decel_ratio =
            (float)abs(g_position_pid[motor_id].err) / MM_TO_PULSE(DECEL_DISTANCE);
        g_position_pid[motor_id].output =
            (int32_t)(g_position_pid[motor_id].output * decel_ratio);
    }

    change_speed_target(motor_id, g_position_pid[motor_id].output);
    g_position_pid[motor_id].last_error = g_position_pid[motor_id].err;
}

/*
 * Determine whether the current position goal is considered reached.
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
