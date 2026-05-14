#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "../uer_algorithm/pid/speed_pid.h"

Motor_Control_t g_motor[MOTOR_MAX] = {0};

void change_speed_target(MotorID motor_id, int32_t speed)
{
    g_motor[motor_id].speed_target = speed;
}

void change_speed_now(MotorID motor_id, int32_t speed)
{
    g_motor[motor_id].speed_now = speed;
}

void motor_set_pwm(MotorID motor_id, int32_t pwm)
{
    g_motor[motor_id].pwm_out = pwm;
}

void motor_stop_all(void)
{
    for(int i = 0; i < MOTOR_MAX; i++)
    {
        g_motor[i].pwm_out = 0;
        g_motor[i].speed_target = 0;
    }
}

int32_t get_speed_target(MotorID motor_id)
{
    return g_motor[motor_id].speed_target;
}

static void test_target_is_ramped_before_control_output(void)
{
    int32_t pwm;

    speed_pid_reset_all();
    change_speed_target(MOTOR_LF, 400);

    pwm = speed_pid_calc(MOTOR_LF, get_speed_target(MOTOR_LF), 0, 20);

    assert(g_motor[MOTOR_LF].speed_target == 400);
    assert(g_speed_pid[MOTOR_LF].target_ramped == 20);
    assert(speed_pid_get_target_ramped(MOTOR_LF) == 20);
    assert(speed_pid_get_error(MOTOR_LF) == 20);
    assert(speed_pid_get_output(MOTOR_LF) == pwm);
    assert(speed_pid_get_output_limit(MOTOR_LF) == 850);
    assert(pwm > 0);
    assert(pwm <= 850);
}

static void test_low_speed_target_uses_min_effective_pwm(void)
{
    int32_t pwm;

    speed_pid_reset_all();
    change_speed_target(MOTOR_LF, 5);

    pwm = speed_pid_calc(MOTOR_LF, get_speed_target(MOTOR_LF), 0, 20);

    assert(g_motor[MOTOR_LF].speed_target == 5);
    assert(g_speed_pid[MOTOR_LF].target_ramped == 5);
    assert(pwm == 70);
}

static void test_point_move_minimum_speed_is_not_silently_zeroed(void)
{
    int32_t target_pulse =
        (int32_t)(((float)POINT_MOVE_MIN_EFFECTIVE_MMPS *
                   PULSE_PER_MM *
                   (float)EXEC_CONTROL_PERIOD_MS) / 1000.0f);
    int32_t pwm;

    speed_pid_reset_all();
    change_speed_target(MOTOR_LF, target_pulse);

    pwm = speed_pid_calc(MOTOR_LF,
                         get_speed_target(MOTOR_LF),
                         0,
                         EXEC_CONTROL_PERIOD_MS);

    assert(target_pulse > 0);
    assert(pwm >= g_speed_pid[MOTOR_LF].min_effect_pwm_fwd);
}

static void test_zero_target_clears_output_state(void)
{
    int32_t pwm;

    speed_pid_reset_all();

    change_speed_target(MOTOR_LF, 20);
    (void)speed_pid_calc(MOTOR_LF, get_speed_target(MOTOR_LF), 0, 20);
    change_speed_target(MOTOR_LF, 0);
    pwm = speed_pid_calc(MOTOR_LF, get_speed_target(MOTOR_LF), 0, 20);

    assert(g_motor[MOTOR_LF].speed_target == 0);
    assert(g_speed_pid[MOTOR_LF].target_ramped == 0);
    assert(g_motor[MOTOR_LF].pwm_out == 0);
    assert(pwm == 0);
}

int main(void)
{
    test_target_is_ramped_before_control_output();
    test_low_speed_target_uses_min_effective_pwm();
    test_point_move_minimum_speed_is_not_silently_zeroed();
    test_zero_target_clears_output_state();

    puts("speed_pid_test passed");
    return 0;
}
