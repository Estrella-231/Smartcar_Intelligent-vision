#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "../uer_algorithm/pid/angle_pid.h"

Motor_Control_t g_motor[MOTOR_MAX] = {0};

static int32_t g_test_gyro_z_cdps = 0;
static int32_t g_test_yaw_cd = 0;

int32_t imu_get_yaw_cd(void)
{
    return g_test_yaw_cd;
}

int32_t imu_get_gyro_z_cdps(void)
{
    return g_test_gyro_z_cdps;
}

void motor_set_pwm_all(void)
{
}

void system_delay_ms(uint32_t ms)
{
    (void)ms;
}

static void test_heading_output_slew_limit_scales_with_period(void)
{
    int32_t output_20ms;
    int32_t output_10ms;

    angle_pid_reset_state();
    output_20ms = angle_pid_calc_output(10000, 0, EXEC_CONTROL_PERIOD_MS);

    angle_pid_reset_state();
    output_10ms = angle_pid_calc_output(10000, 0, EXEC_CONTROL_PERIOD_MS / 2);

    assert(output_20ms == 25);
    assert(output_10ms == 13);
}

int main(void)
{
    test_heading_output_slew_limit_scales_with_period();

    puts("angle_pid_test passed");
    return 0;
}
