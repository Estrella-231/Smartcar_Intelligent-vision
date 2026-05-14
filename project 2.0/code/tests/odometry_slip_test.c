#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../uer_algorithm/kinematics/odometry.h"
#include "speed_pid.h"

Motor_Control_t g_motor[MOTOR_MAX] = {0};

static int32_t g_test_target_ramped[MOTOR_MAX] = {0};
static int32_t g_test_encoder_data[MOTOR_MAX] = {0};
static int16_t g_test_encoder_delta[MOTOR_MAX] = {0};
static int32_t g_test_yaw_cd = 0;

int32_t speed_pid_get_target_ramped(MotorID motor_id)
{
    return g_test_target_ramped[motor_id];
}

int32_t speed_pid_get_error(MotorID motor_id)
{
    return g_test_target_ramped[motor_id] - g_motor[motor_id].speed_now;
}

int32_t speed_pid_get_output(MotorID motor_id)
{
    return g_motor[motor_id].pwm_out;
}

int32_t speed_pid_get_output_limit(MotorID motor_id)
{
    (void)motor_id;
    return 850;
}

int32_t get_speed_target(MotorID motor_id)
{
    return g_motor[motor_id].speed_target;
}

int32_t get_speed_now(MotorID motor_id)
{
    return g_motor[motor_id].speed_now;
}

int32_t get_encoder_data(MotorID motor_id)
{
    return g_test_encoder_data[motor_id];
}

int16_t get_encoder_position_delta(MotorID motor_id)
{
    return g_test_encoder_delta[motor_id];
}

int32_t imu_get_yaw_cd(void)
{
    return g_test_yaw_cd;
}

void car_set_position_now(int32_t x, int32_t y)
{
    (void)x;
    (void)y;
}

void car_set_position_target(int32_t x, int32_t y)
{
    (void)x;
    (void)y;
}

void car_set_angle_now(int32_t angle)
{
    (void)angle;
}

void Mecanum_forward_kinematics_mmps(int32_t lf_mmps,
                                     int32_t rf_mmps,
                                     int32_t lb_mmps,
                                     int32_t rb_mmps,
                                     int32_t *vx_body_mmps,
                                     int32_t *vy_body_mmps)
{
    if(vx_body_mmps != NULL)
    {
        *vx_body_mmps = (lf_mmps - rf_mmps - lb_mmps + rb_mmps) / 4;
    }

    if(vy_body_mmps != NULL)
    {
        *vy_body_mmps = (lf_mmps + rf_mmps + lb_mmps + rb_mmps) / 4;
    }
}

int32_t Mecanum_pulse_per_period_to_mmps(int32_t pulse_per_period, uint32_t period_ms)
{
    if(period_ms == 0)
    {
        return 0;
    }

    return (int32_t)(((float)pulse_per_period / PULSE_PER_MM) * 1000.0f / (float)period_ms);
}

static void set_all_wheels(int32_t target_raw, int32_t target_ramped, int32_t feedback)
{
    for(int i = 0; i < MOTOR_MAX; i++)
    {
        g_motor[i].speed_target = target_raw;
        g_motor[i].speed_now = feedback;
        g_test_target_ramped[i] = target_ramped;
        g_test_encoder_data[i] = feedback;
        g_test_encoder_delta[i] = 0;
    }
}

static void reset_test_state(void)
{
    memset(g_motor, 0, sizeof(g_motor));
    memset(g_test_target_ramped, 0, sizeof(g_test_target_ramped));
    memset(g_test_encoder_data, 0, sizeof(g_test_encoder_data));
    memset(g_test_encoder_delta, 0, sizeof(g_test_encoder_delta));
    g_test_yaw_cd = 0;
    odometry_reset_pose(0, 0);
}

static void test_slip_weight_is_continuous_between_soft_and_hard_error(void)
{
    reset_test_state();

    set_all_wheels(60, 60, 0);
    odometry_update(EXEC_CONTROL_PERIOD_MS);

    assert(odometry_get_slip_weight_pct() < 100);
    assert(odometry_get_slip_weight_pct() > 0);
}

static void test_slip_detection_uses_ramped_target_not_raw_target(void)
{
    reset_test_state();

    set_all_wheels(400, 20, 20);
    odometry_update(EXEC_CONTROL_PERIOD_MS);

    assert(odometry_get_slip_weight_pct() == 100);
}

static void test_finish_tolerance_cannot_be_smaller_than_axis_stop_deadband(void)
{
    int32_t vx_cmd_mmps = -1;
    int32_t vy_cmd_mmps = -1;

    reset_test_state();
    odometry_set_finish_tolerance_mm(20);
    odometry_set_target_point(24, 0);

    assert(odometry_update_point_move_command(&vx_cmd_mmps, &vy_cmd_mmps) ==
           MOVE_STATE_RUNNING);
    assert(vx_cmd_mmps == 0);
    assert(vy_cmd_mmps == 0);

    assert(odometry_update_point_move_command(&vx_cmd_mmps, &vy_cmd_mmps) ==
           MOVE_STATE_FINISH);
}

static void test_near_target_command_settles_below_effective_motion_threshold(void)
{
    int32_t vx_cmd_mmps = 0;
    int32_t vy_cmd_mmps = 0;

    reset_test_state();
    odometry_set_finish_tolerance_mm(POINT_MOVE_AXIS_STOP_TOL_MM);
    odometry_set_target_point(80, 0);

    for(int i = 0; i < 4; i++)
    {
        assert(odometry_update_point_move_command(&vx_cmd_mmps, &vy_cmd_mmps) ==
               MOVE_STATE_RUNNING);
    }

    assert(vx_cmd_mmps >= POINT_MOVE_MIN_EFFECTIVE_MMPS);
    assert(vx_cmd_mmps <= POINT_MOVE_NEAR_MAX_SPEED_MMPS);
    assert(vy_cmd_mmps == 0);
}

static void test_far_target_command_keeps_effective_motion_threshold(void)
{
    int32_t vx_cmd_mmps = 0;
    int32_t vy_cmd_mmps = 0;

    reset_test_state();
    odometry_set_finish_tolerance_mm(POINT_MOVE_AXIS_STOP_TOL_MM);
    odometry_set_target_point(120, 0);

    for(int i = 0; i < 4; i++)
    {
        assert(odometry_update_point_move_command(&vx_cmd_mmps, &vy_cmd_mmps) ==
               MOVE_STATE_RUNNING);
    }

    assert(vx_cmd_mmps >= POINT_MOVE_MIN_EFFECTIVE_MMPS);
    assert(vy_cmd_mmps == 0);
}

/*
 * Verify the static integration gate: when the chassis is truly idle
 * (targets ≈ 0 AND all wheel deltas ≤ noise floor), raw-delta position
 * integration must be suppressed so quantisation noise does not accumulate
 * into the odometry coordinate.
 */
static void test_static_gate_suppresses_noise_when_chassis_idle(void)
{
    int i;

    reset_test_state();

    /*
     * target_ramped = 0  (≤ ODOM_MOTION_TARGET_DEADBAND_PULSE = 2)
     * encoder_delta  = 1 (≤ ODOM_RAW_DELTA_DEADBAND_PULSE     = 1)
     * body_um from delta=1 all-wheels ≈ 130 um
     *               (≤ ODOM_RAW_BODY_DEADBAND_UM              = 500)
     * All three gate conditions satisfied → must skip integration.
     */
    for(i = 0; i < MOTOR_MAX; i++)
    {
        g_test_target_ramped[i] = 0;
        g_test_encoder_delta[i] = 1;
    }

    odometry_update(EXEC_CONTROL_PERIOD_MS);

    assert(odometry_get_is_integrating() == 0);
    assert(odometry_get_x_mm() == 0);
    assert(odometry_get_y_mm() == 0);
}

/*
 * Verify the static integration gate does NOT suppress when the chassis is
 * actively commanded to move.
 *
 * Even though the encoder delta per cycle is small (low-speed creep), the
 * non-zero wheel target must keep the gate open so slow approach moves are
 * not silently dropped from the pose estimate.
 */
static void test_static_gate_keeps_integrating_when_target_nonzero(void)
{
    int i;

    reset_test_state();

    /*
     * target_ramped = 10 (> ODOM_MOTION_TARGET_DEADBAND_PULSE = 2)
     * → target condition fails → gate stays open regardless of delta size.
     *
     * delta = 5 for all wheels (uniform forward):
     * vy_body_um ≈ 5*4*1000/(4*7.7) ≈ 649 um per cycle.
     * After two cycles (×ODOM_SCALE_Y ≈ 0.977) → y_um ≈ 1268 → y_mm = 1.
     */
    for(i = 0; i < MOTOR_MAX; i++)
    {
        g_test_target_ramped[i] = 10;
        g_test_encoder_delta[i] = 5;
    }

    odometry_update(EXEC_CONTROL_PERIOD_MS);
    assert(odometry_get_is_integrating() == 1);

    odometry_update(EXEC_CONTROL_PERIOD_MS);
    assert(odometry_get_y_mm() > 0);
}

int main(void)
{
    test_slip_weight_is_continuous_between_soft_and_hard_error();
    test_slip_detection_uses_ramped_target_not_raw_target();
    test_finish_tolerance_cannot_be_smaller_than_axis_stop_deadband();
    test_near_target_command_settles_below_effective_motion_threshold();
    test_far_target_command_keeps_effective_motion_threshold();
    test_static_gate_suppresses_noise_when_chassis_idle();
    test_static_gate_keeps_integrating_when_target_nonzero();

    puts("odometry_slip_test passed");
    return 0;
}
