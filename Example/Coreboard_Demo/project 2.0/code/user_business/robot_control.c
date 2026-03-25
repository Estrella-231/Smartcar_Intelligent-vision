#include "robot_control.h"

#include "angle_pid.h"
#include "bluetooth.h"
#include "encoder.h"
#include "gyroscope.h"
#include "mecanum.h"
#include "motor_driver.h"
#include "odometry.h"
#include "speed_pid.h"
#include "state_machine.h"
#include "zf_device_ips200.h"

#include <string.h>

/*
 * Current business-layer rotation wrapper.
 *
 * This function is intentionally kept small. It orchestrates the already
 * existing angle/gyro/PWM logic but leaves the low-level controller details in
 * the PID and driver modules. Moving the heavy dependencies into this .c file
 * keeps robot_control.h from leaking the whole stack to any unrelated module
 * that only wants to call Control_Rotate().
 */
void Control_Rotate(int32_t angle)
{
    static uint8_t finish_cnt = 0;

    /*
     * The finish judge belongs to the algorithm layer, but the decision of
     * "what to do once rotation is considered finished" belongs to the business
     * layer. That is why the wrapper checks the state and then performs stop /
     * reset actions here.
     */
    int32_t state = Rotate_Finish_Judge(angle, imu_get_yaw_cd(), imu_get_gyro_z_cdps());

    if(state == ROT_STATE_FINISHED)
    {
        finish_cnt++;
        if(finish_cnt >= 3)
        {
            motor_stop_all();
            send_data(0, 6, 6, 0);

            /*
             * Clear controller history when the action is complete. Otherwise
             * the next rotation command can inherit stale integral and derivative
             * state from the previous maneuver.
             */
            angle_pid_reset_state();
        }
    }
    else
    {
        finish_cnt = 0;
        angle_pid_set_target(angle);

        /*
         * The actual closed-loop calls are still commented out in the current
         * project stage because task 9 has not started yet. Keep this wrapper in
         * place so later enabling the full chain only requires restoring those
         * calls here rather than re-spreading control logic across modules.
         */
//        angle_pid_calc();
//        system_delay_ms(10);
//        rotate_pid_calc();
//        system_delay_ms(10);
//        Rotate_PWM_Calc();
    }
}

static motion_exec_runtime_state_t g_motion_exec_state = {0};
static MotionPlan g_motion_exec_plan = {0};
static uint8_t g_motion_exec_has_plan = 0;
static uint8_t g_segment_target_grid_x = 0;
static uint8_t g_segment_target_grid_y = 0;

#define MOTION_DISPLAY_WIDTH_PX              (320)
#define MOTION_DISPLAY_HEIGHT_PX             (240)
#define MOTION_DISPLAY_CELL_PX               (20)
#define MOTION_DISPLAY_GRID_COLOR            (RGB565_GRAY)
#define MOTION_DISPLAY_BG_COLOR              (RGB565_WHITE)
#define MOTION_DISPLAY_CAR_COLOR             (RGB565_RED)
#define MOTION_DISPLAY_TARGET_COLOR          (RGB565_BLUE)
#define MOTION_DISPLAY_CAR_RADIUS_PX         (3)
#define MOTION_DISPLAY_TARGET_RADIUS_PX      (2)

static uint8_t g_motion_display_initialized = 0U;
static int16_t g_last_car_px = -1;
static int16_t g_last_car_py = -1;
static int16_t g_last_target_px = -1;
static int16_t g_last_target_py = -1;
static uint8_t g_last_target_valid = 0U;
static uint8_t g_motion_display_skip_divider = 0U;

/*
 * Decide whether one pixel belongs to the static 16 x 12 grid.
 *
 * The IPS200 runs in 320 x 240 crosswise mode, so every 200 mm field cell maps
 * to exactly 20 pixels. Keeping this test centralized lets us restore only the
 * small patch around the moving markers instead of redrawing the full screen.
 */
static uint8_t motion_display_is_grid_pixel(uint16_t x_px, uint16_t y_px)
{
    if((x_px >= (MOTION_DISPLAY_WIDTH_PX - 1U)) ||
       (y_px >= (MOTION_DISPLAY_HEIGHT_PX - 1U)))
    {
        return 1U;
    }

    if((0U == (x_px % MOTION_DISPLAY_CELL_PX)) ||
       (0U == (y_px % MOTION_DISPLAY_CELL_PX)))
    {
        return 1U;
    }

    return 0U;
}

/*
 * Convert one physical field coordinate to one screen pixel.
 *
 * Mapping convention:
 * - field origin is bottom-left
 * - IPS200 origin is top-left
 * - 10 mm in field space equals 1 pixel on the 320 x 240 screen
 */
static void motion_display_mm_to_pixel(int32_t x_mm,
                                       int32_t y_mm,
                                       int16_t *x_px,
                                       int16_t *y_px)
{
    int32_t clamped_x_mm = x_mm;
    int32_t clamped_y_mm = y_mm;
    int32_t scaled_x_px;
    int32_t scaled_y_px;

    if(0 == x_px || 0 == y_px)
    {
        return;
    }

    if(clamped_x_mm < 0)
    {
        clamped_x_mm = 0;
    }
    else if(clamped_x_mm > FIELD_WIDTH_MM)
    {
        clamped_x_mm = FIELD_WIDTH_MM;
    }

    if(clamped_y_mm < 0)
    {
        clamped_y_mm = 0;
    }
    else if(clamped_y_mm > FIELD_HEIGHT_MM)
    {
        clamped_y_mm = FIELD_HEIGHT_MM;
    }

    scaled_x_px = clamped_x_mm / 10;
    scaled_y_px = clamped_y_mm / 10;

    if(scaled_x_px >= MOTION_DISPLAY_WIDTH_PX)
    {
        scaled_x_px = MOTION_DISPLAY_WIDTH_PX - 1;
    }

    if(scaled_y_px >= MOTION_DISPLAY_HEIGHT_PX)
    {
        scaled_y_px = MOTION_DISPLAY_HEIGHT_PX - 1;
    }

    *x_px = (int16_t)scaled_x_px;
    *y_px = (int16_t)((MOTION_DISPLAY_HEIGHT_PX - 1) - scaled_y_px);
}

/*
 * Redraw only a small local patch back to the static grid background.
 *
 * This avoids heavy full-screen refreshes on every 20 ms control cycle and
 * keeps the position marker stable while still preserving grid lines.
 */
static void motion_display_restore_patch(int16_t center_x_px,
                                         int16_t center_y_px,
                                         int16_t radius_px)
{
    int16_t x_start;
    int16_t x_end;
    int16_t y_start;
    int16_t y_end;

    if((center_x_px < 0) || (center_y_px < 0))
    {
        return;
    }

    x_start = center_x_px - radius_px;
    x_end = center_x_px + radius_px;
    y_start = center_y_px - radius_px;
    y_end = center_y_px + radius_px;

    if(x_start < 0)
    {
        x_start = 0;
    }
    if(y_start < 0)
    {
        y_start = 0;
    }
    if(x_end >= MOTION_DISPLAY_WIDTH_PX)
    {
        x_end = MOTION_DISPLAY_WIDTH_PX - 1;
    }
    if(y_end >= MOTION_DISPLAY_HEIGHT_PX)
    {
        y_end = MOTION_DISPLAY_HEIGHT_PX - 1;
    }

    for(int16_t y_px = y_start; y_px <= y_end; y_px++)
    {
        for(int16_t x_px = x_start; x_px <= x_end; x_px++)
        {
            ips200_draw_point((uint16_t)x_px,
                              (uint16_t)y_px,
                              motion_display_is_grid_pixel((uint16_t)x_px, (uint16_t)y_px) ?
                                  MOTION_DISPLAY_GRID_COLOR :
                                  MOTION_DISPLAY_BG_COLOR);
        }
    }
}

/*
 * Draw a filled square marker for the current odometry position.
 *
 * A filled block is easier to see than a single pixel while the car is moving.
 */
static void motion_display_draw_car_marker(int16_t center_x_px, int16_t center_y_px)
{
    for(int16_t y_px = center_y_px - MOTION_DISPLAY_CAR_RADIUS_PX;
        y_px <= center_y_px + MOTION_DISPLAY_CAR_RADIUS_PX;
        y_px++)
    {
        for(int16_t x_px = center_x_px - MOTION_DISPLAY_CAR_RADIUS_PX;
            x_px <= center_x_px + MOTION_DISPLAY_CAR_RADIUS_PX;
            x_px++)
        {
            if((x_px >= 0) && (x_px < MOTION_DISPLAY_WIDTH_PX) &&
               (y_px >= 0) && (y_px < MOTION_DISPLAY_HEIGHT_PX))
            {
                ips200_draw_point((uint16_t)x_px, (uint16_t)y_px, MOTION_DISPLAY_CAR_COLOR);
            }
        }
    }
}

/*
 * Draw a hollow target box so the target and the live car marker can be
 * distinguished at a glance.
 */
static void motion_display_draw_target_marker(int16_t center_x_px, int16_t center_y_px)
{
    for(int16_t offset = -MOTION_DISPLAY_TARGET_RADIUS_PX;
        offset <= MOTION_DISPLAY_TARGET_RADIUS_PX;
        offset++)
    {
        int16_t left_x = center_x_px - MOTION_DISPLAY_TARGET_RADIUS_PX;
        int16_t right_x = center_x_px + MOTION_DISPLAY_TARGET_RADIUS_PX;
        int16_t top_y = center_y_px - MOTION_DISPLAY_TARGET_RADIUS_PX;
        int16_t bottom_y = center_y_px + MOTION_DISPLAY_TARGET_RADIUS_PX;

        if((left_x >= 0) && (left_x < MOTION_DISPLAY_WIDTH_PX) &&
           (center_y_px + offset >= 0) && (center_y_px + offset < MOTION_DISPLAY_HEIGHT_PX))
        {
            ips200_draw_point((uint16_t)left_x,
                              (uint16_t)(center_y_px + offset),
                              MOTION_DISPLAY_TARGET_COLOR);
        }

        if((right_x >= 0) && (right_x < MOTION_DISPLAY_WIDTH_PX) &&
           (center_y_px + offset >= 0) && (center_y_px + offset < MOTION_DISPLAY_HEIGHT_PX))
        {
            ips200_draw_point((uint16_t)right_x,
                              (uint16_t)(center_y_px + offset),
                              MOTION_DISPLAY_TARGET_COLOR);
        }

        if((center_x_px + offset >= 0) && (center_x_px + offset < MOTION_DISPLAY_WIDTH_PX) &&
           (top_y >= 0) && (top_y < MOTION_DISPLAY_HEIGHT_PX))
        {
            ips200_draw_point((uint16_t)(center_x_px + offset),
                              (uint16_t)top_y,
                              MOTION_DISPLAY_TARGET_COLOR);
        }

        if((center_x_px + offset >= 0) && (center_x_px + offset < MOTION_DISPLAY_WIDTH_PX) &&
           (bottom_y >= 0) && (bottom_y < MOTION_DISPLAY_HEIGHT_PX))
        {
            ips200_draw_point((uint16_t)(center_x_px + offset),
                              (uint16_t)bottom_y,
                              MOTION_DISPLAY_TARGET_COLOR);
        }
    }
}

/*
 * Draw the static 16 x 12 field grid once after IPS200 init.
 *
 * With the current screen resolution, each virtual cell is exactly 20 x 20
 * pixels, so the display becomes a direct visual copy of the field grid.
 */
static void motion_display_draw_static_grid(void)
{
    ips200_full(MOTION_DISPLAY_BG_COLOR);

    for(uint16_t x_px = 0; x_px < MOTION_DISPLAY_WIDTH_PX; x_px += MOTION_DISPLAY_CELL_PX)
    {
        ips200_draw_line(x_px, 0, x_px, MOTION_DISPLAY_HEIGHT_PX - 1, MOTION_DISPLAY_GRID_COLOR);
    }

    ips200_draw_line(MOTION_DISPLAY_WIDTH_PX - 1,
                     0,
                     MOTION_DISPLAY_WIDTH_PX - 1,
                     MOTION_DISPLAY_HEIGHT_PX - 1,
                     MOTION_DISPLAY_GRID_COLOR);

    for(uint16_t y_px = 0; y_px < MOTION_DISPLAY_HEIGHT_PX; y_px += MOTION_DISPLAY_CELL_PX)
    {
        ips200_draw_line(0, y_px, MOTION_DISPLAY_WIDTH_PX - 1, y_px, MOTION_DISPLAY_GRID_COLOR);
    }

    ips200_draw_line(0,
                     MOTION_DISPLAY_HEIGHT_PX - 1,
                     MOTION_DISPLAY_WIDTH_PX - 1,
                     MOTION_DISPLAY_HEIGHT_PX - 1,
                     MOTION_DISPLAY_GRID_COLOR);
}

/*
 * Convert one logical grid coordinate into the physical center of that cell.
 *
 * Field convention fixed for this project stage:
 * - origin: bottom-left corner
 * - grid size: 16 x 12
 * - cell size: 200 mm x 200 mm
 *
 * The scheduler always drives to cell centers rather than edges. This makes the
 * relation between BFS cells and odometry target points explicit and stable.
 */
static uint8_t motion_exec_grid_to_mm_center(uint8_t grid_x,
                                             uint8_t grid_y,
                                             int32_t *x_mm,
                                             int32_t *y_mm)
{
    if((grid_x >= FIELD_GRID_COLS) || (grid_y >= FIELD_GRID_ROWS) ||
       (0 == x_mm) || (0 == y_mm))
    {
        return 0U;
    }

    *x_mm = (int32_t)grid_x * FIELD_GRID_CELL_MM + FIELD_GRID_CELL_MM / 2;
    *y_mm = (int32_t)grid_y * FIELD_GRID_CELL_MM + FIELD_GRID_CELL_MM / 2;
    return 1U;
}

/*
 * Push the current local execution state into the shared formal runtime store.
 *
 * The scheduler owns the writable copy, while the rest of the system should
 * only read the state through motion_exec_runtime_state_copy().
 */
static void motion_exec_publish_state(void)
{
    motion_exec_runtime_state_store(&g_motion_exec_state);
}

/*
 * Stop all drive outputs and clear controller residues.
 *
 * This is used in every non-moving state so the chassis never keeps stale wheel
 * targets or stale integral/PWM history while waiting for the next segment.
 */
static void motion_exec_stop_chassis(void)
{
    Mecanum_inverse_kinematics(0, 0, 0);
    speed_pid_reset_all();
    motor_stop_all();
}

/*
 * Run the already validated low-level motion chain for one control period:
 * - heading hold outer loop
 * - inverse kinematics
 * - per-wheel speed PID
 * - signed PWM output
 */
static void motion_exec_apply_body_command(int32_t vx_cmd_mmps,
                                           int32_t vy_cmd_mmps,
                                           uint32_t control_period_ms)
{
    int32_t vz_cmd;
    int32_t vx_cmd_pulse;
    int32_t vy_cmd_pulse;

    vz_cmd = angle_pid_calc_output(EXEC_YAW_TARGET_CD, imu_get_yaw_cd(), control_period_ms);
    vz_cmd = LIMIT_ABS(vz_cmd, EXEC_YAW_VZ_LIMIT);

    vx_cmd_pulse = Mecanum_mmps_to_pulse_per_period(vx_cmd_mmps, control_period_ms);
    vy_cmd_pulse = Mecanum_mmps_to_pulse_per_period(vy_cmd_mmps, control_period_ms);

    Mecanum_inverse_kinematics(vx_cmd_pulse, vy_cmd_pulse, vz_cmd);

    for(int32_t i = 0; i < MOTOR_MAX; i++)
    {
        int32_t pwm_command;

        pwm_command = speed_pid_calc((MotorID)i,
                                     get_speed_target((MotorID)i),
                                     get_encoder_data((MotorID)i),
                                     control_period_ms);
        motor_set_pwm((MotorID)i, pwm_command);
    }
}

/*
 * Refresh all motion feedback that the scheduler depends on.
 *
 * Ordering matters:
 * 1. read fresh encoder deltas for this control cycle
 * 2. update IMU yaw/rate for this control cycle
 * 3. fuse both into odometry
 */
static void motion_exec_update_feedback(uint32_t control_period_ms)
{
    encoder_read_data();
    imu_deal_data_period(control_period_ms);
    odometry_update(control_period_ms);
}

/*
 * Mark the scheduler as failed and force the chassis into a safe stop.
 */
static void motion_exec_enter_error(exec_error_t error_code)
{
    g_motion_exec_state.phase = CAR_STATE_ERROR;
    g_motion_exec_state.error_code = error_code;
    g_motion_exec_state.plan_finished = 0U;
    g_motion_exec_state.plan_loaded = g_motion_exec_has_plan;
    motion_exec_stop_chassis();
    motion_exec_publish_state();
}

/*
 * Start execution of the next motion segment from the currently active plan.
 *
 * Grid progression is tracked explicitly. This is safer than adding deltas to
 * the current odometry pose, because every segment target remains anchored to a
 * known virtual grid cell center.
 */
static void motion_exec_start_next_segment(void)
{
    MotionSegment *segment;
    int32_t target_x_mm;
    int32_t target_y_mm;
    int32_t next_grid_x;
    int32_t next_grid_y;

    if(!g_motion_exec_has_plan)
    {
        motion_exec_enter_error(EXEC_ERROR_NO_BFS_PLAN);
        return;
    }

    if(g_motion_exec_state.current_segment_index >= g_motion_exec_plan.count)
    {
        g_motion_exec_state.phase = CAR_STATE_PLAN_DONE;
        g_motion_exec_state.plan_finished = 1U;
        motion_exec_stop_chassis();
        motion_exec_publish_state();
        return;
    }

    segment = &g_motion_exec_plan.data[g_motion_exec_state.current_segment_index];
    g_motion_exec_state.current_segment = *segment;
    g_motion_exec_state.segment_wait_remaining_ms = 0;

    switch(segment->type)
    {
        case SEG_WALK:
        case SEG_PUSH:
            next_grid_x = g_motion_exec_state.current_grid_x;
            next_grid_y = g_motion_exec_state.current_grid_y;

            switch(segment->dir)
            {
                case DIR_UP:    next_grid_y += segment->cells; break;
                case DIR_DOWN:  next_grid_y -= segment->cells; break;
                case DIR_LEFT:  next_grid_x -= segment->cells; break;
                case DIR_RIGHT: next_grid_x += segment->cells; break;
                default:
                    motion_exec_enter_error(EXEC_ERROR_UNSUPPORTED_SEGMENT);
                    return;
            }

            if((next_grid_x < 0) || (next_grid_x >= FIELD_GRID_COLS) ||
               (next_grid_y < 0) || (next_grid_y >= FIELD_GRID_ROWS))
            {
                motion_exec_enter_error(EXEC_ERROR_SEGMENT_RANGE);
                return;
            }

            g_segment_target_grid_x = (uint8_t)next_grid_x;
            g_segment_target_grid_y = (uint8_t)next_grid_y;

            if(!motion_exec_grid_to_mm_center(g_segment_target_grid_x,
                                              g_segment_target_grid_y,
                                              &target_x_mm,
                                              &target_y_mm))
            {
                motion_exec_enter_error(EXEC_ERROR_SEGMENT_RANGE);
                return;
            }

            odometry_set_point_move_profile((SEG_PUSH == segment->type) ?
                                                POINT_MOVE_PROFILE_PUSH :
                                                POINT_MOVE_PROFILE_WALK);
            odometry_set_target_point(target_x_mm, target_y_mm);

            g_motion_exec_state.segment_target_x_mm = target_x_mm;
            g_motion_exec_state.segment_target_y_mm = target_y_mm;
            g_motion_exec_state.phase = CAR_STATE_WAIT_SEGMENT_FINISH;
            break;

        case SEG_WAIT:
            g_motion_exec_state.segment_wait_remaining_ms =
                (segment->time_ms > 0) ? segment->time_ms : EXEC_WAIT_SEGMENT_DEFAULT_MS;
            g_motion_exec_state.segment_target_x_mm = odometry_get_x_mm();
            g_motion_exec_state.segment_target_y_mm = odometry_get_y_mm();
            g_motion_exec_state.phase = CAR_STATE_WAIT_SEGMENT_FINISH;
            break;

        default:
            motion_exec_enter_error(EXEC_ERROR_UNSUPPORTED_SEGMENT);
            return;
    }

    motion_exec_publish_state();
}

/*
 * Accept a segment once the chassis is already close enough to that segment's
 * target point.
 *
 * Why this soft-finish gate is needed:
 * - grid execution cares about reaching the next logical cell region, not about
 *   mathematically sitting on the exact center pixel of the blue marker
 * - on the real field the car can lose momentum near the target because of
 *   small floor ripple or static friction
 * - if we keep insisting on the exact final centimeters, the car may appear to
 *   "freeze" until someone nudges it
 *
 * The scheduler therefore accepts the current segment once both position errors
 * are inside a practical execution tolerance.
 */
static uint8_t motion_exec_segment_soft_finish_reached(void)
{
    return (uint8_t)((abs(odometry_get_target_dx_mm()) <= EXEC_SEGMENT_ACCEPT_TOL_MM) &&
                     (abs(odometry_get_target_dy_mm()) <= EXEC_SEGMENT_ACCEPT_TOL_MM));
}

void motion_exec_init(void)
{
    memset(&g_motion_exec_state, 0, sizeof(g_motion_exec_state));
    memset(&g_motion_exec_plan, 0, sizeof(g_motion_exec_plan));

    g_motion_exec_state.phase = CAR_STATE_WAIT_START;
    g_motion_exec_state.error_code = EXEC_ERROR_NONE;
    g_motion_exec_state.current_segment_index = 0;
    g_motion_exec_state.segment_wait_remaining_ms = 0;

    g_motion_exec_has_plan = 0U;
    g_segment_target_grid_x = 0U;
    g_segment_target_grid_y = 0U;

    odometry_init(ODOM_START_X_MM, ODOM_START_Y_MM);
    angle_pid_reset_state();
    speed_pid_reset_all();
    motion_exec_stop_chassis();
    motion_exec_runtime_state_reset();
    motion_exec_publish_state();
}

void motion_exec_request_start(void)
{
    g_motion_exec_state.start_requested = 1U;
    g_motion_exec_state.plan_finished = 0U;
    g_motion_exec_state.error_code = EXEC_ERROR_NONE;

    if((g_motion_exec_state.phase == CAR_STATE_WAIT_START) ||
       (g_motion_exec_state.phase == CAR_STATE_PLAN_DONE))
    {
        g_motion_exec_state.phase = CAR_STATE_WAIT_MAP;
    }

    motion_exec_publish_state();
}

void motion_exec_tick(uint32_t control_period_ms)
{
    bfs_runtime_state_t bfs_state;
    int32_t start_x_mm;
    int32_t start_y_mm;
    int32_t vx_cmd_mmps = 0;
    int32_t vy_cmd_mmps = 0;

    if(control_period_ms == 0U)
    {
        return;
    }

    motion_exec_update_feedback(control_period_ms);

    if(!bfs_runtime_state_copy(&bfs_state))
    {
        memset(&bfs_state, 0, sizeof(bfs_state));
    }

    switch(g_motion_exec_state.phase)
    {
        case CAR_STATE_WAIT_START:
            motion_exec_stop_chassis();
            break;

        case CAR_STATE_WAIT_MAP:
            motion_exec_stop_chassis();

            if(!g_motion_exec_state.start_requested)
            {
                g_motion_exec_state.phase = CAR_STATE_WAIT_START;
            }
            else if(bfs_state.has_filtered_map)
            {
                g_motion_exec_state.phase = CAR_STATE_WAIT_BFS;
            }
            break;

        case CAR_STATE_WAIT_BFS:
            motion_exec_stop_chassis();

            if(!g_motion_exec_state.start_requested)
            {
                g_motion_exec_state.phase = CAR_STATE_WAIT_START;
            }
            else if(bfs_state.has_plan &&
                    (bfs_state.phase == BFS_RUNTIME_PLAN_OK) &&
                    (bfs_state.last_plan_status == SOKO_OK))
            {
                g_motion_exec_state.phase = CAR_STATE_LOAD_PLAN;
            }
            break;

        case CAR_STATE_LOAD_PLAN:
            if(!bfs_state.has_plan || (bfs_state.last_plan_status != SOKO_OK))
            {
                motion_exec_enter_error(EXEC_ERROR_NO_BFS_PLAN);
                return;
            }

            if(0U == bfs_state.last_motion_plan.count)
            {
                motion_exec_enter_error(EXEC_ERROR_EMPTY_PLAN);
                return;
            }

            if(!motion_exec_grid_to_mm_center(bfs_state.player_x,
                                              bfs_state.player_y,
                                              &start_x_mm,
                                              &start_y_mm))
            {
                motion_exec_enter_error(EXEC_ERROR_SEGMENT_RANGE);
                return;
            }

            memcpy(&g_motion_exec_plan, &bfs_state.last_motion_plan, sizeof(MotionPlan));
            g_motion_exec_has_plan = 1U;
            g_motion_exec_state.plan_loaded = 1U;
            g_motion_exec_state.plan_finished = 0U;
            g_motion_exec_state.error_code = EXEC_ERROR_NONE;
            g_motion_exec_state.source_plan_signature = bfs_state.planned_map_signature;
            g_motion_exec_state.current_segment_index = 0U;
            g_motion_exec_state.current_grid_x = bfs_state.player_x;
            g_motion_exec_state.current_grid_y = bfs_state.player_y;
            g_motion_exec_state.segment_target_x_mm = start_x_mm;
            g_motion_exec_state.segment_target_y_mm = start_y_mm;

            odometry_reset_pose(start_x_mm, start_y_mm);
            angle_pid_reset_state();
            g_motion_exec_state.phase = CAR_STATE_EXEC_SEGMENT;
            break;

        case CAR_STATE_EXEC_SEGMENT:
            motion_exec_start_next_segment();
            return;

        case CAR_STATE_WAIT_SEGMENT_FINISH:
            if(SEG_WAIT == g_motion_exec_state.current_segment.type)
            {
                motion_exec_stop_chassis();

                if(g_motion_exec_state.segment_wait_remaining_ms > (int32_t)control_period_ms)
                {
                    g_motion_exec_state.segment_wait_remaining_ms -= (int32_t)control_period_ms;
                }
                else
                {
                    g_motion_exec_state.segment_wait_remaining_ms = 0;
                    g_motion_exec_state.current_segment_index++;
                    g_motion_exec_state.phase = CAR_STATE_EXEC_SEGMENT;
                }
            }
            else
            {
                if(motion_exec_segment_soft_finish_reached() ||
                   (odometry_update_point_move_command(&vx_cmd_mmps, &vy_cmd_mmps) == MOVE_STATE_FINISH))
                {
                    /*
                     * Keep the finish transition conservative.
                     *
                     * We tried loading the next segment immediately in the same
                     * control tick, but on real ground that made the chassis
                     * advance segments too early and occasionally drift or stop
                     * one cell before the intended target. Returning to the
                     * simpler stop-then-next-tick transition is slower, but
                     * much more stable for the current tuning stage.
                     */
                    motion_exec_stop_chassis();
                    g_motion_exec_state.current_grid_x = g_segment_target_grid_x;
                    g_motion_exec_state.current_grid_y = g_segment_target_grid_y;
                    g_motion_exec_state.current_segment_index++;
                    g_motion_exec_state.phase = CAR_STATE_EXEC_SEGMENT;
                }
                else
                {
                    motion_exec_apply_body_command(vx_cmd_mmps, vy_cmd_mmps, control_period_ms);
                }
            }
            break;

        case CAR_STATE_PLAN_DONE:
            motion_exec_stop_chassis();
            break;

        case CAR_STATE_ERROR:
        default:
            motion_exec_stop_chassis();
            break;
    }

    motion_exec_publish_state();
}

void motion_display_init(void)
{
    ips200_set_dir(IPS200_CROSSWISE);
    ips200_init(IPS200_TYPE_SPI);
    ips200_set_font(IPS200_6X8_FONT);
    ips200_set_color(RGB565_BLACK, MOTION_DISPLAY_BG_COLOR);

    motion_display_draw_static_grid();

    g_motion_display_initialized = 1U;
    g_last_car_px = -1;
    g_last_car_py = -1;
    g_last_target_px = -1;
    g_last_target_py = -1;
    g_last_target_valid = 0U;
    g_motion_display_skip_divider = 0U;
}

void motion_display_tick(void)
{
    motion_exec_runtime_state_t exec_state;
    int16_t car_x_px;
    int16_t car_y_px;
    int16_t target_x_px = -1;
    int16_t target_y_px = -1;
    uint8_t target_valid = 0U;

    if(!g_motion_display_initialized)
    {
        return;
    }

    /*
     * The motion control loop runs every 20 ms, but the IPS200 does not need
     * to be refreshed that fast. Updating every 100 ms is enough for a live
     * position view and keeps SPI drawing load modest.
     */
    g_motion_display_skip_divider++;
    if(g_motion_display_skip_divider < 5U)
    {
        return;
    }
    g_motion_display_skip_divider = 0U;

    motion_display_mm_to_pixel(odometry_get_x_mm(), odometry_get_y_mm(), &car_x_px, &car_y_px);

    if(motion_exec_runtime_state_copy(&exec_state) &&
       ((exec_state.phase == CAR_STATE_EXEC_SEGMENT) ||
        (exec_state.phase == CAR_STATE_WAIT_SEGMENT_FINISH)))
    {
        motion_display_mm_to_pixel(exec_state.segment_target_x_mm,
                                   exec_state.segment_target_y_mm,
                                   &target_x_px,
                                   &target_y_px);
        target_valid = 1U;
    }

    if((g_last_car_px != car_x_px) || (g_last_car_py != car_y_px))
    {
        motion_display_restore_patch(g_last_car_px,
                                     g_last_car_py,
                                     MOTION_DISPLAY_CAR_RADIUS_PX + 1);
    }

    if(g_last_target_valid &&
       ((!target_valid) ||
        (g_last_target_px != target_x_px) ||
        (g_last_target_py != target_y_px)))
    {
        motion_display_restore_patch(g_last_target_px,
                                     g_last_target_py,
                                     MOTION_DISPLAY_TARGET_RADIUS_PX + 1);
    }

    if(target_valid)
    {
        motion_display_draw_target_marker(target_x_px, target_y_px);
    }

    motion_display_draw_car_marker(car_x_px, car_y_px);

    g_last_car_px = car_x_px;
    g_last_car_py = car_y_px;
    g_last_target_px = target_x_px;
    g_last_target_py = target_y_px;
    g_last_target_valid = target_valid;
}
