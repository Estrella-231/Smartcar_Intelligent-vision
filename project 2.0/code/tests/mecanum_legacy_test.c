#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../uer_algorithm/kinematics/mecanum.h"

Motor_Control_t g_motor[MOTOR_MAX] = {0};
static int32_t g_car_speed_now = 0;

void change_speed_target(MotorID motor_id, int32_t speed)
{
    g_motor[motor_id].speed_target = speed;
}

int32_t get_speed_now(MotorID motor_id)
{
    return g_motor[motor_id].speed_now;
}

void car_set_speed_now(int32_t speed)
{
    g_car_speed_now = speed;
}

static void test_calc_speed_now_uses_software_position_delta_without_int16_wrap(void)
{
    memset(g_motor, 0, sizeof(g_motor));
    g_car_speed_now = 0;

    g_motor[MOTOR_LF].pos_last = 0;
    g_motor[MOTOR_LF].pos_now = 40000;

    Calc_SpeedNow(MOTOR_LF);

    assert(g_motor[MOTOR_LF].speed_now > 0);
    assert(g_motor[MOTOR_LF].pos_last == 40000);
}

int main(void)
{
    test_calc_speed_now_uses_software_position_delta_without_int16_wrap();

    puts("mecanum_legacy_test passed");
    return 0;
}
