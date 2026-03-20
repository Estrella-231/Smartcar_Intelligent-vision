#include "angle_pid.h"

ROTATEPID_t g_rotate = {0};
ROTATEPID_t g_angle = {0};
int32_t g_stable_frame_cnt = 0;
int32_t rotate_finish_flag = 0;
int32_t wheel_pwm[MOTOR_MAX] = {0};
Rotate_State_Type g_rot_state = ROT_STATE_IDLE;

/*
 * Normalize a heading difference into the range [-18000, 18000] centi-degrees.
 *
 * Why this matters:
 * - IMU yaw can keep accumulating beyond one full turn
 * - heading-hold control should prefer the shortest correction direction
 * - without normalization, target=0 and current=359 deg would produce a huge
 *   negative error instead of a small positive one
 */
static int32_t angle_error_normalize_cd(int32_t error_cd)
{
    while(error_cd > 18000)
    {
        error_cd -= 36000;
    }

    while(error_cd < -18000)
    {
        error_cd += 36000;
    }

    return error_cd;
}

void angle_pid_set_target(int32_t angle)
{
    g_angle.target = angle;
}

/*
 * Clear all angle-loop runtime history.
 *
 * This must be done when switching test modes or when a motion task finishes,
 * otherwise the next motion can inherit stale integral and derivative state.
 */
void angle_pid_reset_state(void)
{
    memset(&g_rotate, 0, sizeof(g_rotate));
    memset(&g_angle, 0, sizeof(g_angle));
    g_stable_frame_cnt = 0;
    rotate_finish_flag = 0;
    g_rot_state = ROT_STATE_IDLE;
}

/*
 * Task 9 outer-loop implementation.
 *
 * This computes a yaw correction command from:
 * - desired heading
 * - current IMU heading
 * - real control period
 *
 * The result is not sent directly to the motor PWM layer. Instead, it is used
 * as the Vz contribution before the already validated four-wheel speed loop.
 */
int32_t angle_pid_calc_output(int32_t target_angle_cd,
                              int32_t current_angle_cd,
                              uint32_t period_ms)
{
    int32_t gyro_z_cdps;
    int32_t raw_output;
    int32_t output_step;

    /*
     * Task 10 tuning note:
     *
     * The previous PI-style outer loop made the chassis shake more during
     * lateral translation. That is a common failure mode when integral action is
     * added too early in heading-hold control:
     * - the chassis is already moving
     * - heading error changes sign frequently near the setpoint
     * - integral memory keeps pushing after the body has started coming back
     * - the result is left-right oscillation
     *
     * For the current stage, a more suitable heading-hold outer loop is:
     * - proportional term on heading error
     * - gyro-rate damping term on actual yaw rate
     * - no integral accumulation
     * - output slew limiting so Vz does not jump violently between samples
     *
     * This is effectively a "P + rate damping" controller tailored for moving
     * heading hold, and it is usually easier to stabilize than a full PID.
     */
    const int32_t heading_hold_deadband_cd = 80;    // 0.80 deg
    const int32_t heading_hold_output_deadband = 15;
    const int32_t heading_hold_output_step_limit = 25;
    const float heading_hold_kp = 0.35f;
    const float heading_hold_gyro_damping = 0.018f;

    if(period_ms == 0)
    {
        return 0;
    }

    g_angle.target = target_angle_cd;
    g_angle.feedback = current_angle_cd;
    g_angle.err = angle_error_normalize_cd(target_angle_cd - current_angle_cd);

    if(abs(g_angle.err) < heading_hold_deadband_cd)
    {
        g_angle.err = 0;
        g_angle.integral = 0;
        g_angle.output = 0;
        g_angle.last_output = 0;
        g_angle.last_error = 0;
        return 0;
    }

    /*
     * The damping term uses the measured yaw rate directly:
     * - if the body is already rotating in the correction direction, reduce output
     * - if it is rotating away from the target, increase output
     *
     * This is much better behaved than differentiating the angle error here,
     * because the IMU already provides the physically meaningful angular-rate
     * signal we need for damping.
     */
    gyro_z_cdps = imu_get_gyro_z_cdps();
    raw_output = (int32_t)(heading_hold_kp * (float)g_angle.err -
                           heading_hold_gyro_damping * (float)gyro_z_cdps);

    if(abs(raw_output) < heading_hold_output_deadband)
    {
        raw_output = 0;
    }

    /*
     * Limit how much the heading correction is allowed to change each control
     * period. This directly targets the observed "left-right twitch" behavior by
     * preventing the Vz command from flipping too abruptly between samples.
     */
    output_step = raw_output - g_angle.last_output;

    if(output_step > heading_hold_output_step_limit)
    {
        raw_output = g_angle.last_output + heading_hold_output_step_limit;
    }
    else if(output_step < -heading_hold_output_step_limit)
    {
        raw_output = g_angle.last_output - heading_hold_output_step_limit;
    }

    g_angle.integral = 0;
    g_angle.output = LIMIT_ABS(raw_output, ANGLE_MAX_OUTPUT);
    g_angle.last_error = g_angle.err;
    g_angle.last_output = g_angle.output;

    return g_angle.output;
}

int32_t angle_pid_get_error(void)
{
    return g_angle.err;
}

int32_t angle_pid_get_output(void)
{
    return g_angle.output;
}

/*
 * Legacy wrapper kept so older code paths still build.
 */
void angle_pid_calc(void)
{
    (void)angle_pid_calc_output(g_angle.target, imu_get_yaw_cd(), PID_CONTROL_PERIOD);
}

/*
 * Legacy inner rotation-rate PID kept for compatibility with the older
 * rotation-only path. Task 9 does not depend on it.
 */
void rotate_pid_calc(void)
{
    g_rotate.err = g_rotate.target - imu_get_gyro_z_cdps();
    g_rotate.integral += g_rotate.err * PID_CONTROL_PERIOD / 1000;
    g_rotate.integral = LIMIT_ABS(g_rotate.integral, ROTATE_PID_I_LIMIT);

    g_rotate.output = (int32_t)(ROTATE_PID_KP * g_rotate.err +
                                ROTATE_PID_KI * g_rotate.integral +
                                ROTATE_PID_KD * (g_rotate.err - g_rotate.last_error) /
                                    PID_CONTROL_PERIOD * 1000);

    g_rotate.output = (int32_t)(g_rotate.output * 3 / 10);
    g_rotate.output = LIMIT_ABS(g_rotate.output, ROTATE_MAX_OUTPUT);
    g_rotate.last_error = g_rotate.err;
}

/*
 * Legacy PWM distributor for the rotation-only path.
 */
void Rotate_PWM_Calc(void)
{
    int32_t dir = MOVE_ROTATE_CCW;
    int32_t target_pwm_base;
    static int32_t current_pwm_base = 0;
    int32_t diff;
    int32_t pwm_base;

    if(g_angle.err > 0)
    {
        dir = MOVE_ROTATE_CW;
    }

    target_pwm_base = abs(g_angle.output);
    target_pwm_base = LIMIT_ABS(target_pwm_base, ROTATE_MAX_OUTPUT);

    diff = target_pwm_base - current_pwm_base;

    if(diff > 50)
    {
        current_pwm_base += 50;
    }
    else if(diff < -50)
    {
        current_pwm_base -= 50;
    }
    else
    {
        current_pwm_base = target_pwm_base;
    }

    pwm_base = current_pwm_base;

    if(dir == MOVE_ROTATE_CW)
    {
        wheel_pwm[MOTOR_LF] = -pwm_base;
        wheel_pwm[MOTOR_RF] = pwm_base;
        wheel_pwm[MOTOR_LB] = -pwm_base;
        wheel_pwm[MOTOR_RB] = pwm_base;
    }
    else if(dir == MOVE_ROTATE_CCW)
    {
        wheel_pwm[MOTOR_LF] = pwm_base;
        wheel_pwm[MOTOR_RF] = -pwm_base;
        wheel_pwm[MOTOR_LB] = pwm_base;
        wheel_pwm[MOTOR_RB] = -pwm_base;
    }
    else
    {
        memset(wheel_pwm, 0, sizeof(wheel_pwm));
        current_pwm_base = 0;
    }

    for(int32_t i = 0; i < MOTOR_MAX; i++)
    {
        wheel_pwm[i] = LIMIT_ABS(wheel_pwm[i], ROTATE_MAX_OUTPUT);
        wheel_pwm[i] = (abs(wheel_pwm[i]) < 500) ? 0 : wheel_pwm[i];
        g_motor[i].pwm_out = wheel_pwm[i];
    }

    system_delay_ms(10);
    motor_set_pwm_all();
}

int32_t Rotate_Finish_Judge(int32_t target_angle, int32_t curr_angle, int32_t curr_gyro)
{
    int32_t angle_err = abs(angle_error_normalize_cd(curr_angle - target_angle));
    int32_t is_gyro_static = (abs(curr_gyro) <= ANGLE_DEAD_ZONE) ? 1 : 0;
    int32_t is_pwm_valid = (abs(g_angle.output) > 500) ? 1 : 0;

    switch(g_rot_state)
    {
        case ROT_STATE_IDLE:
            g_rot_state = is_pwm_valid ? ROT_STATE_RUNNING : ROT_STATE_IDLE;
            g_stable_frame_cnt = 0;
            break;

        case ROT_STATE_RUNNING:
            if(angle_err <= ANGLE_DEAD_ZONE && is_gyro_static)
            {
                g_rot_state = ROT_STATE_STABLE;
                g_stable_frame_cnt = 1;
            }
            break;

        case ROT_STATE_STABLE:
            if(angle_err <= ANGLE_DEAD_ZONE && is_gyro_static)
            {
                g_stable_frame_cnt++;
                g_rot_state = (g_stable_frame_cnt >= 5) ? ROT_STATE_FINISHED : ROT_STATE_STABLE;
            }
            else
            {
                g_rot_state = ROT_STATE_RUNNING;
                g_stable_frame_cnt = 0;
            }
            break;

        case ROT_STATE_FINISHED:
            g_rot_state = is_pwm_valid ? ROT_STATE_RUNNING : ROT_STATE_FINISHED;
            g_stable_frame_cnt = 0;
            break;

        default:
            g_rot_state = ROT_STATE_IDLE;
            g_stable_frame_cnt = 0;
            break;
    }

    rotate_finish_flag = (g_rot_state == ROT_STATE_FINISHED) ? 1 : 0;
    return g_rot_state;
}
