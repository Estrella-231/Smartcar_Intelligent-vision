#include "odometry.h"
#include "encoder.h"

Odometry_State_t g_odometry = {0};

/*
 * Keep the previous odometry velocity so we can apply a simple step limit
 * against sudden spikes from encoder mismatch or wheel slip.
 */
static int32_t g_last_vx_body_mmps = 0;
static int32_t g_last_vy_body_mmps = 0;
static uint8_t g_point_move_stable_count = 0;
static int32_t g_last_vx_point_cmd_mmps = 0;
static int32_t g_last_vy_point_cmd_mmps = 0;
static PointMoveProfile_t g_point_move_profile = POINT_MOVE_PROFILE_WALK;

static int32_t odometry_limit_velocity_step(int32_t current_value, int32_t last_value)
{
    int32_t delta = current_value - last_value;

    if(delta > ODOM_MAX_VEL_STEP_MMPS)
    {
        return last_value + ODOM_MAX_VEL_STEP_MMPS;
    }

    if(delta < -ODOM_MAX_VEL_STEP_MMPS)
    {
        return last_value - ODOM_MAX_VEL_STEP_MMPS;
    }

    return current_value;
}

/*
 * Limit how quickly the point-move planner is allowed to change one axis
 * command between adjacent control cycles.
 *
 * This is separate from the odometry velocity spike limiter:
 * - odometry_limit_velocity_step() protects the measured velocity estimate
 * - odometry_limit_point_cmd_step() smooths the commanded translation
 *
 * The goal here is to reduce target-near oscillation and chassis twitch.
 */
static int32_t odometry_limit_point_cmd_step(int32_t current_value, int32_t last_value)
{
    int32_t delta = current_value - last_value;

    if(delta > POINT_MOVE_CMD_STEP_MMPS)
    {
        return last_value + POINT_MOVE_CMD_STEP_MMPS;
    }

    if(delta < -POINT_MOVE_CMD_STEP_MMPS)
    {
        return last_value - POINT_MOVE_CMD_STEP_MMPS;
    }

    return current_value;
}

/*
 * Estimate how trustworthy the current wheel-speed set is by comparing
 * closed-loop wheel targets with measured wheel feedback.
 *
 * This is a lightweight engineering anti-slip heuristic, not a full observer.
 * If wheel tracking error becomes very large, pose integration is down-weighted
 * so one bad traction event does not immediately destroy the odometry state.
 */
static int32_t odometry_calc_slip_weight_pct(void)
{
    int32_t total_error = 0;

    for(int i = 0; i < MOTOR_MAX; i++)
    {
        total_error += abs(get_speed_target((MotorID)i) - get_speed_now((MotorID)i));
    }

    total_error /= MOTOR_MAX;

    if(total_error >= ODOM_TRACK_ERROR_HARD_PULSE)
    {
        return 0;
    }

    if(total_error >= ODOM_TRACK_ERROR_SOFT_PULSE)
    {
        return ODOM_SOFT_WEIGHT_PCT;
    }

    return 100;
}

/*
 * Convert one global-axis position error directly into one global-axis velocity
 * command.
 *
 * Why the round-two planner now uses axis-by-axis control instead of the old
 * "atan2 + minimum speed" strategy:
 * - the old version kept forcing a non-zero speed even when the chassis was
 *   already very close to the target
 * - that made the car keep pushing right near the end of a 0.6 m move
 * - axis P control lets each axis naturally decay to zero as the position error
 *   shrinks, which is much better for an omni chassis doing point-to-point work
 */
static int32_t odometry_axis_error_to_cmd_mmps(int32_t axis_error_mm)
{
    int32_t axis_cmd_mmps;
    int32_t abs_error_mm = abs(axis_error_mm);
    int32_t max_speed_mmps = POINT_MOVE_MAX_SPEED_MMPS;
    int32_t min_effective_mmps = POINT_MOVE_MIN_EFFECTIVE_MMPS;
    int32_t near_max_speed_mmps = POINT_MOVE_NEAR_MAX_SPEED_MMPS;

    if(abs_error_mm <= POINT_MOVE_AXIS_STOP_TOL_MM)
    {
        return 0;
    }

    /*
     * Push segments should move more conservatively than free walk segments.
     * The state machine switches the profile before starting each segment, so
     * the exact same point-move logic can serve both without duplicating the
     * controller.
     */
    if(POINT_MOVE_PROFILE_PUSH == g_point_move_profile)
    {
        max_speed_mmps = (POINT_MOVE_MAX_SPEED_MMPS * POINT_MOVE_PUSH_SPEED_SCALE_PCT) / 100;
        near_max_speed_mmps =
            (POINT_MOVE_NEAR_MAX_SPEED_MMPS * POINT_MOVE_PUSH_SPEED_SCALE_PCT) / 100;
        min_effective_mmps = POINT_MOVE_PUSH_MIN_EFFECTIVE_MMPS;
    }

    axis_cmd_mmps = (int32_t)((float)axis_error_mm * POINT_MOVE_KP);
    /*
     * Use two move regimes:
     * - far from target: keep the command above the current chassis low-speed
     *   dead region so the move stays decisive
     * - near target: allow a much softer command so the chassis can settle
     *   instead of weaving around the final point
     */
    if(abs_error_mm > POINT_MOVE_NEAR_SLOWDOWN_MM)
    {
        axis_cmd_mmps = LIMIT_ABS(axis_cmd_mmps, max_speed_mmps);

        /*
         * Keep the command out of the known low-speed ineffective region.
         *
         * Why this is needed:
         * - the current chassis and speed-loop tuning were validated around a
         *   relatively high wheel target
         * - if point-move commands become too small, the real motion becomes
         *   sticky, uneven, and prone to twitch
         */
        if(axis_cmd_mmps > 0 && axis_cmd_mmps < min_effective_mmps)
        {
            axis_cmd_mmps = min_effective_mmps;
        }
        else if(axis_cmd_mmps < 0 && axis_cmd_mmps > -min_effective_mmps)
        {
            axis_cmd_mmps = -min_effective_mmps;
        }
    }
    else
    {
        axis_cmd_mmps = LIMIT_ABS(axis_cmd_mmps, near_max_speed_mmps);
    }

    return axis_cmd_mmps;
}

void odometry_init(int32_t start_x_mm, int32_t start_y_mm)
{
    odometry_reset_pose(start_x_mm, start_y_mm);
}

/*
 * Reset integrated odometry pose and runtime filters.
 *
 * This should be called at the known field start position before calibration or
 * before starting a new point-to-point test so the pose estimate is anchored to
 * a known coordinate.
 */
void odometry_reset_pose(int32_t start_x_mm, int32_t start_y_mm)
{
    memset(&g_odometry, 0, sizeof(g_odometry));
    g_odometry.x_mm = start_x_mm;
    g_odometry.y_mm = start_y_mm;
    g_odometry.target_x_mm = start_x_mm;
    g_odometry.target_y_mm = start_y_mm;
    g_odometry.slip_weight_pct = 100;
    g_odometry.move_state = MOVE_STATE_IDLE;

    g_last_vx_body_mmps = 0;
    g_last_vy_body_mmps = 0;
    g_point_move_stable_count = 0;
    g_last_vx_point_cmd_mmps = 0;
    g_last_vy_point_cmd_mmps = 0;
    g_point_move_profile = POINT_MOVE_PROFILE_WALK;

    car_set_position_now(start_x_mm, start_y_mm);
    car_set_position_target(start_x_mm, start_y_mm);
    car_set_angle_now(imu_get_yaw_cd());
}

/*
 * Main second-round odometry update.
 *
 * Data flow:
 * 1. Convert wheel feedback from encoder pulses / control period into mm/s.
 * 2. Use mecanum forward kinematics to recover body-frame vx/vy.
 * 3. Apply basic deadband, step limiting, and wheel-tracking-based slip weight.
 * 4. Rotate body-frame velocity into the global frame using current IMU yaw.
 * 5. Integrate global velocity into global X/Y position.
 */
void odometry_update(uint32_t period_ms)
{
    int32_t lf_mmps;
    int32_t rf_mmps;
    int32_t lb_mmps;
    int32_t rb_mmps;
    int32_t vx_body_mmps;
    int32_t vy_body_mmps;
    int32_t vx_global_mmps;
    int32_t vy_global_mmps;
    int32_t yaw_cd;
    float yaw_rad;
    float cos_yaw;
    float sin_yaw;

    if(period_ms == 0)
    {
        return;
    }

    lf_mmps = Mecanum_pulse_per_period_to_mmps(get_encoder_data(MOTOR_LF), period_ms);
    rf_mmps = Mecanum_pulse_per_period_to_mmps(get_encoder_data(MOTOR_RF), period_ms);
    lb_mmps = Mecanum_pulse_per_period_to_mmps(get_encoder_data(MOTOR_LB), period_ms);
    rb_mmps = Mecanum_pulse_per_period_to_mmps(get_encoder_data(MOTOR_RB), period_ms);

    Mecanum_forward_kinematics_mmps(lf_mmps,
                                    rf_mmps,
                                    lb_mmps,
                                    rb_mmps,
                                    &vx_body_mmps,
                                    &vy_body_mmps);

    if(abs(vx_body_mmps) < ODOM_BODY_SPEED_DEADBAND_MMPS)
    {
        vx_body_mmps = 0;
    }

    if(abs(vy_body_mmps) < ODOM_BODY_SPEED_DEADBAND_MMPS)
    {
        vy_body_mmps = 0;
    }

    vx_body_mmps = odometry_limit_velocity_step(vx_body_mmps, g_last_vx_body_mmps);
    vy_body_mmps = odometry_limit_velocity_step(vy_body_mmps, g_last_vy_body_mmps);
    g_last_vx_body_mmps = vx_body_mmps;
    g_last_vy_body_mmps = vy_body_mmps;

    yaw_cd = imu_get_yaw_cd();
    yaw_rad = ((float)yaw_cd * 3.1415926f) / 18000.0f;
    cos_yaw = cosf(yaw_rad);
    sin_yaw = sinf(yaw_rad);

    /*
     * The project uses clockwise-positive yaw, body x=right, body y=forward.
     * Under that convention:
     * - VX =  vx*cos(yaw) + vy*sin(yaw)
     * - VY = -vx*sin(yaw) + vy*cos(yaw)
     */
    vx_global_mmps = (int32_t)((float)vx_body_mmps * cos_yaw +
                               (float)vy_body_mmps * sin_yaw);
    vy_global_mmps = (int32_t)(-(float)vx_body_mmps * sin_yaw +
                               (float)vy_body_mmps * cos_yaw);

    /*
     * Apply simple engineering correction factors between the ideal model and
     * the real field. These are meant to be tuned through the 0.4 m calibration
     * runs in task 4 of round two.
     */
    vx_global_mmps = (int32_t)((float)vx_global_mmps * ODOM_SCALE_X);
    vy_global_mmps = (int32_t)((float)vy_global_mmps * ODOM_SCALE_Y);

    g_odometry.slip_weight_pct = odometry_calc_slip_weight_pct();

    if(g_odometry.slip_weight_pct != 100)
    {
        vx_global_mmps = (vx_global_mmps * g_odometry.slip_weight_pct) / 100;
        vy_global_mmps = (vy_global_mmps * g_odometry.slip_weight_pct) / 100;
    }

    if(abs(vx_global_mmps) < ODOM_GLOBAL_SPEED_DEADBAND_MMPS)
    {
        vx_global_mmps = 0;
    }

    if(abs(vy_global_mmps) < ODOM_GLOBAL_SPEED_DEADBAND_MMPS)
    {
        vy_global_mmps = 0;
    }

    g_odometry.vx_body_mmps = vx_body_mmps;
    g_odometry.vy_body_mmps = vy_body_mmps;
    g_odometry.vx_global_mmps = vx_global_mmps;
    g_odometry.vy_global_mmps = vy_global_mmps;

    g_odometry.x_mm += (vx_global_mmps * (int32_t)period_ms) / 1000;
    g_odometry.y_mm += (vy_global_mmps * (int32_t)period_ms) / 1000;

    car_set_position_now(g_odometry.x_mm, g_odometry.y_mm);
    car_set_angle_now(yaw_cd);
}

void odometry_set_target_point(int32_t target_x_mm, int32_t target_y_mm)
{
    g_odometry.target_x_mm = target_x_mm;
    g_odometry.target_y_mm = target_y_mm;
    g_odometry.move_state = MOVE_STATE_RUNNING;
    g_point_move_stable_count = 0;
    g_last_vx_point_cmd_mmps = 0;
    g_last_vy_point_cmd_mmps = 0;
    car_set_position_target(target_x_mm, target_y_mm);
}

/*
 * Select the point-move profile used by subsequent odometry_update_point_move_command()
 * calls.
 *
 * The execution scheduler sets this before each segment:
 * - WALK: normal free translation
 * - PUSH: lower-speed translation while pushing a box
 */
void odometry_set_point_move_profile(PointMoveProfile_t profile)
{
    g_point_move_profile = profile;
}

/*
 * Generate a body-frame linear velocity command toward the currently configured
 * target point while keeping heading control separate.
 *
 * The position planner works in the global frame first, then rotates the desired
 * global velocity back into the body frame so the existing inverse kinematics can
 * keep using body-frame vx/vy inputs.
 */
MoveState odometry_update_point_move_command(int32_t *vx_cmd_mmps,
                                             int32_t *vy_cmd_mmps)
{
    int32_t dx;
    int32_t dy;
    float yaw_rad;
    float cos_yaw;
    float sin_yaw;
    float vx_global_cmd;
    float vy_global_cmd;

    if(vx_cmd_mmps == NULL || vy_cmd_mmps == NULL)
    {
        return g_odometry.move_state;
    }

    dx = g_odometry.target_x_mm - g_odometry.x_mm;
    dy = g_odometry.target_y_mm - g_odometry.y_mm;

    /*
     * Finish decision is based on a slightly looser tolerance than the command
     * deadband used for individual axis correction.
     *
     * Why separate tolerances are needed:
     * - command deadband should be tight, so the planner does not keep injecting
     *   unnecessary correction on one axis
     * - finish tolerance should be slightly looser, so the full move can really
     *   finish instead of being held hostage by a tiny cross-axis residual
     */
    if(abs(dx) <= POINT_MOVE_FINISH_TOL_MM &&
       abs(dy) <= POINT_MOVE_FINISH_TOL_MM)
    {
        g_point_move_stable_count++;
    }
    else
    {
        g_point_move_stable_count = 0;
    }

    if(g_point_move_stable_count >= POINT_MOVE_STABLE_COUNT)
    {
        *vx_cmd_mmps = 0;
        *vy_cmd_mmps = 0;
        g_last_vx_point_cmd_mmps = 0;
        g_last_vy_point_cmd_mmps = 0;
        g_odometry.move_state = MOVE_STATE_FINISH;
        return g_odometry.move_state;
    }

    /*
     * Build the desired motion directly in the global frame:
     * - X error drives VX
     * - Y error drives VY
     *
     * This avoids the near-target overshoot introduced by forcing a minimum
     * speed along the atan2 direction vector.
     */
    /*
     * First compute the raw global-frame axis commands from the current pose
     * error, then smooth them so the chassis does not jerk left-right while the
     * residual error changes sign near the target.
     */
    /*
     * Use a dominant-axis strategy:
     * - if X is the main unfinished axis, ignore a small Y residual for now
     * - if Y is the main unfinished axis, ignore a small X residual for now
     *
     * This prevents the chassis from weaving left-right just because the
     * secondary axis is off by only a few centimeters while the main axis is
     * still far from the target.
     */
    if(abs(dx) > abs(dy))
    {
        g_last_vx_point_cmd_mmps =
            odometry_limit_point_cmd_step(odometry_axis_error_to_cmd_mmps(dx),
                                          g_last_vx_point_cmd_mmps);

        if(abs(dy) <= POINT_MOVE_CROSS_AXIS_TOL_MM &&
           abs(dx) > POINT_MOVE_FINISH_TOL_MM)
        {
            g_last_vy_point_cmd_mmps =
                odometry_limit_point_cmd_step(0, g_last_vy_point_cmd_mmps);
        }
        else
        {
            g_last_vy_point_cmd_mmps =
                odometry_limit_point_cmd_step(odometry_axis_error_to_cmd_mmps(dy),
                                              g_last_vy_point_cmd_mmps);
        }
    }
    else
    {
        g_last_vy_point_cmd_mmps =
            odometry_limit_point_cmd_step(odometry_axis_error_to_cmd_mmps(dy),
                                          g_last_vy_point_cmd_mmps);

        if(abs(dx) <= POINT_MOVE_CROSS_AXIS_TOL_MM &&
           abs(dy) > POINT_MOVE_FINISH_TOL_MM)
        {
            g_last_vx_point_cmd_mmps =
                odometry_limit_point_cmd_step(0, g_last_vx_point_cmd_mmps);
        }
        else
        {
            g_last_vx_point_cmd_mmps =
                odometry_limit_point_cmd_step(odometry_axis_error_to_cmd_mmps(dx),
                                              g_last_vx_point_cmd_mmps);
        }
    }

    vx_global_cmd = (float)g_last_vx_point_cmd_mmps;
    vy_global_cmd = (float)g_last_vy_point_cmd_mmps;

    yaw_rad = ((float)imu_get_yaw_cd() * 3.1415926f) / 18000.0f;
    cos_yaw = cosf(yaw_rad);
    sin_yaw = sinf(yaw_rad);

    /*
     * Inverse rotation of the clockwise-positive body/global mapping:
     * - vx = VX*cos(yaw) - VY*sin(yaw)
     * - vy = VX*sin(yaw) + VY*cos(yaw)
     */
    *vx_cmd_mmps = (int32_t)(vx_global_cmd * cos_yaw - vy_global_cmd * sin_yaw);
    *vy_cmd_mmps = (int32_t)(vx_global_cmd * sin_yaw + vy_global_cmd * cos_yaw);

    g_odometry.move_state = MOVE_STATE_RUNNING;
    return g_odometry.move_state;
}

int32_t odometry_get_vx_body_mmps(void)
{
    return g_odometry.vx_body_mmps;
}

int32_t odometry_get_vy_body_mmps(void)
{
    return g_odometry.vy_body_mmps;
}

int32_t odometry_get_vx_global_mmps(void)
{
    return g_odometry.vx_global_mmps;
}

int32_t odometry_get_vy_global_mmps(void)
{
    return g_odometry.vy_global_mmps;
}

int32_t odometry_get_x_mm(void)
{
    return g_odometry.x_mm;
}

int32_t odometry_get_y_mm(void)
{
    return g_odometry.y_mm;
}

int32_t odometry_get_target_dx_mm(void)
{
    return g_odometry.target_x_mm - g_odometry.x_mm;
}

int32_t odometry_get_target_dy_mm(void)
{
    return g_odometry.target_y_mm - g_odometry.y_mm;
}

int32_t odometry_get_slip_weight_pct(void)
{
    return g_odometry.slip_weight_pct;
}

MoveState odometry_get_move_state(void)
{
    return g_odometry.move_state;
}
