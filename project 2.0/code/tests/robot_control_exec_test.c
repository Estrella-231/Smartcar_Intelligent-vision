#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "angle_pid.h"
#include "motor_driver.h"
#include "odometry.h"
#include "robot_param.h"
#include "../user_business/robot_control.h"
#include "../user_business/state_machine.h"

static bfs_runtime_state_t g_bfs_state;
static motion_exec_runtime_state_t g_motion_exec_runtime_state;
static int32_t g_yaw_cd = 0;
static int32_t g_gyro_z_cdps = 0;
static int32_t g_odometry_x_mm = 0;
static int32_t g_odometry_y_mm = 0;
static int32_t g_odometry_target_x_mm = 0;
static int32_t g_odometry_target_y_mm = 0;
static MoveState g_odometry_move_state = MOVE_STATE_IDLE;
static PointMoveProfile_t g_last_point_move_profile = POINT_MOVE_PROFILE_WALK;
static int32_t g_speed_targets[MOTOR_MAX] = {0};
static int32_t g_last_angle_pid_target_cd = 0;
static int32_t g_last_motor_pwm[MOTOR_MAX] = {0};
static int32_t g_test_point_vx_cmd_mmps = 0;
static int32_t g_test_point_vy_cmd_mmps = 0;
static int32_t g_last_finish_tolerance_mm = 0;
static int32_t g_speed_pid_error[MOTOR_MAX] = {0};
static int32_t g_speed_pid_output[MOTOR_MAX] = {0};
static int32_t g_speed_pid_output_limit[MOTOR_MAX] = {0};
static int32_t g_speed_pid_reset_all_call_count = 0;
static int32_t g_motor_stop_all_call_count = 0;
static uint8_t g_force_speed_pid_saturation = 0U;

void system_delay_ms(uint32_t ms)
{
    (void)ms;
}

void send_data(int32_t a, int32_t b, int32_t c, int32_t d)
{
    (void)a;
    (void)b;
    (void)c;
    (void)d;
}

uint32_t ble6a20_send_string(const char *str)
{
    (void)str;
    return 0;
}

void encoder_read_data(void)
{
}

int32_t get_encoder_data(MotorID motor_id)
{
    (void)motor_id;
    return 0;
}

uint8_t imu_is_ready(void)
{
    return 1U;
}

int32_t imu_get_yaw_cd(void)
{
    return g_yaw_cd;
}

int32_t imu_get_gyro_z_cdps(void)
{
    return g_gyro_z_cdps;
}

void imu_deal_data_period(uint32_t period_ms)
{
    (void)period_ms;
}

void angle_pid_set_target(int32_t angle)
{
    g_last_angle_pid_target_cd = angle;
}

void angle_pid_reset_state(void)
{
    g_last_angle_pid_target_cd = 0;
}

int32_t angle_pid_calc_output(int32_t target_angle_cd,
                              int32_t current_angle_cd,
                              uint32_t period_ms)
{
    (void)period_ms;
    g_last_angle_pid_target_cd = target_angle_cd;

    if(target_angle_cd > current_angle_cd)
    {
        return 30;
    }
    if(target_angle_cd < current_angle_cd)
    {
        return -30;
    }

    return 0;
}

int32_t Rotate_Finish_Judge(int32_t target_angle, int32_t curr_angle, int32_t curr_gyro)
{
    (void)target_angle;
    (void)curr_angle;
    (void)curr_gyro;
    return ROT_STATE_IDLE;
}

void Mecanum_inverse_kinematics(int32_t vx_cmd, int32_t vy_cmd, int32_t vz_cmd)
{
    change_speed_target(MOTOR_LF, vy_cmd + vx_cmd + vz_cmd);
    change_speed_target(MOTOR_RF, vy_cmd - vx_cmd - vz_cmd);
    change_speed_target(MOTOR_LB, vy_cmd - vx_cmd + vz_cmd);
    change_speed_target(MOTOR_RB, vy_cmd + vx_cmd - vz_cmd);
}

int32_t Mecanum_mmps_to_pulse_per_period(int32_t velocity_mmps, uint32_t period_ms)
{
    (void)period_ms;
    return velocity_mmps;
}

void motor_stop_all(void)
{
    g_motor_stop_all_call_count++;

    for(int i = 0; i < MOTOR_MAX; i++)
    {
        g_last_motor_pwm[i] = 0;
    }
}

void motor_set_pwm(MotorID motor_id, int32_t pwm)
{
    g_last_motor_pwm[motor_id] = pwm;
}

void change_speed_target(MotorID motor_id, int32_t speed)
{
    g_speed_targets[motor_id] = speed;
}

int32_t get_speed_target(MotorID motor_id)
{
    return g_speed_targets[motor_id];
}

int32_t get_speed_now(MotorID motor_id)
{
    (void)motor_id;
    return 0;
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

void odometry_init(int32_t start_x_mm, int32_t start_y_mm)
{
    odometry_reset_pose(start_x_mm, start_y_mm);
}

void odometry_reset_pose(int32_t start_x_mm, int32_t start_y_mm)
{
    g_odometry_x_mm = start_x_mm;
    g_odometry_y_mm = start_y_mm;
    g_odometry_target_x_mm = start_x_mm;
    g_odometry_target_y_mm = start_y_mm;
    g_odometry_move_state = MOVE_STATE_IDLE;
}

void odometry_update(uint32_t period_ms)
{
    (void)period_ms;
}

void odometry_set_point_move_profile(PointMoveProfile_t profile)
{
    g_last_point_move_profile = profile;
}

void odometry_set_target_point(int32_t target_x_mm, int32_t target_y_mm)
{
    g_odometry_target_x_mm = target_x_mm;
    g_odometry_target_y_mm = target_y_mm;
    g_odometry_move_state = MOVE_STATE_RUNNING;
}

void odometry_set_finish_tolerance_mm(int32_t tolerance_mm)
{
    g_last_finish_tolerance_mm = tolerance_mm;
}

MoveState odometry_update_point_move_command(int32_t *vx_cmd_mmps,
                                             int32_t *vy_cmd_mmps)
{
    if(vx_cmd_mmps != NULL)
    {
        *vx_cmd_mmps = g_test_point_vx_cmd_mmps;
    }
    if(vy_cmd_mmps != NULL)
    {
        *vy_cmd_mmps = g_test_point_vy_cmd_mmps;
    }
    return g_odometry_move_state;
}

int32_t odometry_get_x_mm(void)
{
    return g_odometry_x_mm;
}

int32_t odometry_get_y_mm(void)
{
    return g_odometry_y_mm;
}

int32_t odometry_get_target_dx_mm(void)
{
    return g_odometry_target_x_mm - g_odometry_x_mm;
}

int32_t odometry_get_target_dy_mm(void)
{
    return g_odometry_target_y_mm - g_odometry_y_mm;
}

void speed_pid_reset_all(void)
{
    g_speed_pid_reset_all_call_count++;
}

void speed_pid_stall_protection_init(void)
{
}

void speed_pid_stall_protection_tick(uint32_t period_ms)
{
    (void)period_ms;
}

uint8_t speed_pid_is_stalled(MotorID motor_id)
{
    (void)motor_id;
    return 0U;
}

uint8_t speed_pid_any_motor_stalled(void)
{
    return 0U;
}

int32_t speed_pid_calc(MotorID motor_id,
                       int32_t target_speed,
                       int32_t current_speed,
                       uint32_t period_ms)
{
    (void)motor_id;
    (void)current_speed;
    (void)period_ms;

    if(g_force_speed_pid_saturation)
    {
        g_speed_pid_error[motor_id] = MOTION_CMD_SCALE_TRACK_ERROR_PULSE;
        g_speed_pid_output[motor_id] =
            (target_speed >= 0) ? MOTION_CMD_SCALE_PWM_SAT_PCT : -MOTION_CMD_SCALE_PWM_SAT_PCT;
        g_speed_pid_output_limit[motor_id] = 100;
    }
    else
    {
        g_speed_pid_error[motor_id] = 0;
        g_speed_pid_output[motor_id] = target_speed;
        g_speed_pid_output_limit[motor_id] = 100;
    }

    return target_speed;
}

int32_t speed_pid_get_target_ramped(MotorID motor_id)
{
    return g_speed_targets[motor_id];
}

int32_t speed_pid_get_error(MotorID motor_id)
{
    return g_speed_pid_error[motor_id];
}

int32_t speed_pid_get_output(MotorID motor_id)
{
    return g_speed_pid_output[motor_id];
}

int32_t speed_pid_get_output_limit(MotorID motor_id)
{
    return g_speed_pid_output_limit[motor_id];
}

void bfs_runtime_state_reset(void)
{
    memset(&g_bfs_state, 0, sizeof(g_bfs_state));
}

void bfs_runtime_state_update_bridge(uint32_t valid_frame_count,
                                     uint32_t stable_frame_count,
                                     uint32_t filtered_map_signature,
                                     uint8_t player_x,
                                     uint8_t player_y,
                                     uint8_t box_count,
                                     uint8_t target_count,
                                     uint8_t bomb_count,
                                     uint8_t stable_ready)
{
    (void)valid_frame_count;
    (void)stable_frame_count;
    (void)filtered_map_signature;
    (void)player_x;
    (void)player_y;
    (void)box_count;
    (void)target_count;
    (void)bomb_count;
    (void)stable_ready;
}

void bfs_runtime_state_update_plan(uint32_t planned_map_signature,
                                   SokoStatus plan_status,
                                   const ActionSeq *action_seq,
                                   const MotionPlan *motion_plan)
{
    (void)planned_map_signature;
    (void)plan_status;
    (void)action_seq;
    (void)motion_plan;
}

uint8_t bfs_runtime_state_copy(bfs_runtime_state_t *out_state)
{
    if(out_state == NULL)
    {
        return 0U;
    }

    *out_state = g_bfs_state;
    return 1U;
}

void motion_exec_runtime_state_reset(void)
{
    memset(&g_motion_exec_runtime_state, 0, sizeof(g_motion_exec_runtime_state));
}

void motion_exec_runtime_state_store(const motion_exec_runtime_state_t *state)
{
    g_motion_exec_runtime_state = *state;
}

uint8_t motion_exec_runtime_state_copy(motion_exec_runtime_state_t *out_state)
{
    if(out_state == NULL)
    {
        return 0U;
    }

    *out_state = g_motion_exec_runtime_state;
    return 1U;
}

void ips200_set_dir(int dir)
{
    (void)dir;
}

void ips200_init(int type)
{
    (void)type;
}

void ips200_set_font(int font)
{
    (void)font;
}

void ips200_set_color(uint16_t fg, uint16_t bg)
{
    (void)fg;
    (void)bg;
}

void ips200_full(uint16_t color)
{
    (void)color;
}

void ips200_draw_point(uint16_t x, uint16_t y, uint16_t color)
{
    (void)x;
    (void)y;
    (void)color;
}

void ips200_draw_line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
    (void)x0;
    (void)y0;
    (void)x1;
    (void)y1;
    (void)color;
}

static void reset_test_state(void)
{
    memset(&g_bfs_state, 0, sizeof(g_bfs_state));
    memset(&g_motion_exec_runtime_state, 0, sizeof(g_motion_exec_runtime_state));
    memset(g_speed_targets, 0, sizeof(g_speed_targets));
    memset(g_last_motor_pwm, 0, sizeof(g_last_motor_pwm));
    memset(g_speed_pid_error, 0, sizeof(g_speed_pid_error));
    memset(g_speed_pid_output, 0, sizeof(g_speed_pid_output));
    memset(g_speed_pid_output_limit, 0, sizeof(g_speed_pid_output_limit));

    g_yaw_cd = 0;
    g_gyro_z_cdps = 0;
    g_odometry_x_mm = 0;
    g_odometry_y_mm = 0;
    g_odometry_target_x_mm = 0;
    g_odometry_target_y_mm = 0;
    g_odometry_move_state = MOVE_STATE_IDLE;
    g_last_point_move_profile = POINT_MOVE_PROFILE_WALK;
    g_last_angle_pid_target_cd = 0;
    g_test_point_vx_cmd_mmps = 0;
    g_test_point_vy_cmd_mmps = 0;
    g_last_finish_tolerance_mm = 0;
    g_speed_pid_reset_all_call_count = 0;
    g_motor_stop_all_call_count = 0;
    g_force_speed_pid_saturation = 0U;
}

static void setup_single_rotate_plan(uint16_t angle_deg)
{
    g_bfs_state.has_filtered_map = 1U;
    g_bfs_state.has_plan = 1U;
    g_bfs_state.phase = BFS_RUNTIME_PLAN_OK;
    g_bfs_state.last_plan_status = SOKO_OK;
    g_bfs_state.player_x = 1U;
    g_bfs_state.player_y = 6U;
    g_bfs_state.last_motion_plan.count = 1U;
    g_bfs_state.last_motion_plan.data[0].type = SEG_ROTATE;
    g_bfs_state.last_motion_plan.data[0].angle_deg = angle_deg;
}

static void test_motion_exec_init_resets_speed_pid_once(void)
{
    reset_test_state();
    motion_exec_init();
    assert(g_speed_pid_reset_all_call_count == 1);
}

static void test_rotate_segment_is_not_rejected(void)
{
    motion_exec_runtime_state_t runtime_state;

    reset_test_state();
    setup_single_rotate_plan(90U);

    motion_exec_init();
    motion_exec_request_start();

    motion_exec_tick(EXEC_CONTROL_PERIOD_MS);
    motion_exec_tick(EXEC_CONTROL_PERIOD_MS);
    motion_exec_tick(EXEC_CONTROL_PERIOD_MS);
    motion_exec_tick(EXEC_CONTROL_PERIOD_MS);

    assert(motion_exec_runtime_state_copy(&runtime_state));
    assert(runtime_state.error_code == EXEC_ERROR_NONE);
    assert(runtime_state.phase == CAR_STATE_WAIT_SEGMENT_FINISH);
    assert(runtime_state.current_segment.type == SEG_ROTATE);

    motion_exec_tick(EXEC_CONTROL_PERIOD_MS);
    assert(g_last_angle_pid_target_cd == 9000);
}

static void test_manual_plan_load_bypasses_bfs_wait_states(void)
{
    motion_exec_runtime_state_t runtime_state;
    MotionPlan manual_plan;

    reset_test_state();
    memset(&manual_plan, 0, sizeof(manual_plan));
    manual_plan.count = 1U;
    manual_plan.data[0].type = SEG_WALK;
    manual_plan.data[0].dir = DIR_RIGHT;
    manual_plan.data[0].cells = 2U;

    motion_exec_init();
    motion_exec_load_manual_plan(&manual_plan, ODOM_START_GRID_X, ODOM_START_GRID_Y);
    motion_exec_request_start();
    motion_exec_tick(EXEC_CONTROL_PERIOD_MS);

    assert(motion_exec_runtime_state_copy(&runtime_state));
    assert(runtime_state.error_code == EXEC_ERROR_NONE);
    assert(runtime_state.plan_loaded == 1U);
    assert(runtime_state.phase == CAR_STATE_WAIT_SEGMENT_FINISH);
    assert(runtime_state.current_segment.type == SEG_WALK);
    assert(runtime_state.current_grid_x == ODOM_START_GRID_X);
    assert(runtime_state.current_grid_y == ODOM_START_GRID_Y);
    assert(runtime_state.segment_target_x_mm ==
           ((ODOM_START_GRID_X + 2) * FIELD_GRID_CELL_MM + FIELD_GRID_CELL_MM / 2));
    assert(runtime_state.segment_target_y_mm == ODOM_START_Y_MM);
    assert(g_last_point_move_profile == POINT_MOVE_PROFILE_WALK);
}

static void test_manual_debug_mode_does_not_accept_segment_at_default_tolerance(void)
{
    motion_exec_runtime_state_t runtime_state;
    MotionPlan manual_plan;

    reset_test_state();
    memset(&manual_plan, 0, sizeof(manual_plan));
    manual_plan.count = 1U;
    manual_plan.data[0].type = SEG_WALK;
    manual_plan.data[0].dir = DIR_RIGHT;
    manual_plan.data[0].cells = 2U;

    g_test_point_vx_cmd_mmps = POINT_MOVE_MIN_EFFECTIVE_MMPS;
    g_test_point_vy_cmd_mmps = 0;

    motion_exec_init();
    assert(g_last_finish_tolerance_mm == DEBUG_POINT_MOVE_FINISH_TOL_MM);

    motion_exec_load_manual_plan(&manual_plan, ODOM_START_GRID_X, ODOM_START_GRID_Y);
    motion_exec_request_start();
    motion_exec_tick(EXEC_CONTROL_PERIOD_MS);

    g_odometry_x_mm = g_odometry_target_x_mm - (DEBUG_EXEC_SEGMENT_ACCEPT_TOL_MM + 5);
    g_odometry_y_mm = g_odometry_target_y_mm;
    g_odometry_move_state = MOVE_STATE_RUNNING;

    motion_exec_tick(EXEC_CONTROL_PERIOD_MS);

    assert(motion_exec_runtime_state_copy(&runtime_state));
    assert(runtime_state.phase == CAR_STATE_WAIT_SEGMENT_FINISH);
    assert(runtime_state.current_segment_index == 0U);
    assert(g_speed_targets[MOTOR_LF] == POINT_MOVE_MIN_EFFECTIVE_MMPS);
}

static void test_completed_segment_snaps_odometry_to_grid_target(void)
{
    motion_exec_runtime_state_t runtime_state;
    MotionPlan manual_plan;

    reset_test_state();
    memset(&manual_plan, 0, sizeof(manual_plan));
    manual_plan.count = 2U;
    manual_plan.data[0].type = SEG_WALK;
    manual_plan.data[0].dir = DIR_RIGHT;
    manual_plan.data[0].cells = 1U;
    manual_plan.data[1].type = SEG_WALK;
    manual_plan.data[1].dir = DIR_LEFT;
    manual_plan.data[1].cells = 1U;

    motion_exec_init();
    motion_exec_load_manual_plan(&manual_plan, ODOM_START_GRID_X, ODOM_START_GRID_Y);
    motion_exec_request_start();
    motion_exec_tick(EXEC_CONTROL_PERIOD_MS);

    g_odometry_x_mm = g_odometry_target_x_mm - DEBUG_EXEC_SEGMENT_ACCEPT_TOL_MM;
    g_odometry_y_mm = g_odometry_target_y_mm;
    g_odometry_move_state = MOVE_STATE_RUNNING;
    motion_exec_tick(EXEC_CONTROL_PERIOD_MS);

    assert(g_odometry_x_mm == ((ODOM_START_GRID_X + 1) * FIELD_GRID_CELL_MM + FIELD_GRID_CELL_MM / 2));
    assert(g_odometry_y_mm == ODOM_START_Y_MM);

    motion_exec_tick(EXEC_CONTROL_PERIOD_MS);

    assert(motion_exec_runtime_state_copy(&runtime_state));
    assert(runtime_state.current_segment_index == 1U);
    assert(runtime_state.current_segment.dir == DIR_LEFT);
    assert(runtime_state.segment_target_x_mm == ODOM_START_X_MM);
    assert(g_odometry_target_x_mm == ODOM_START_X_MM);
    assert(g_odometry_target_x_mm - g_odometry_x_mm == -FIELD_GRID_CELL_MM);
}

static void test_consecutive_move_segments_do_not_full_stop_between_segments(void)
{
    motion_exec_runtime_state_t runtime_state;
    MotionPlan manual_plan;
    int32_t stop_count_after_init;

    reset_test_state();
    memset(&manual_plan, 0, sizeof(manual_plan));
    manual_plan.count = 2U;
    manual_plan.data[0].type = SEG_WALK;
    manual_plan.data[0].dir = DIR_RIGHT;
    manual_plan.data[0].cells = 1U;
    manual_plan.data[1].type = SEG_WALK;
    manual_plan.data[1].dir = DIR_UP;
    manual_plan.data[1].cells = 1U;

    motion_exec_init();
    stop_count_after_init = g_motor_stop_all_call_count;
    motion_exec_load_manual_plan(&manual_plan, ODOM_START_GRID_X, ODOM_START_GRID_Y);
    motion_exec_request_start();
    motion_exec_tick(EXEC_CONTROL_PERIOD_MS);

    g_odometry_x_mm = g_odometry_target_x_mm - DEBUG_EXEC_SEGMENT_ACCEPT_TOL_MM;
    g_odometry_y_mm = g_odometry_target_y_mm;
    g_odometry_move_state = MOVE_STATE_RUNNING;
    motion_exec_tick(EXEC_CONTROL_PERIOD_MS);

    assert(motion_exec_runtime_state_copy(&runtime_state));
    assert(runtime_state.current_segment_index == 1U);
    assert(runtime_state.phase == CAR_STATE_EXEC_SEGMENT);
    assert(g_motor_stop_all_call_count == stop_count_after_init);
    assert(g_speed_pid_reset_all_call_count == 1);
}

static void test_near_target_dwell_accepts_sticky_segment(void)
{
    motion_exec_runtime_state_t runtime_state;
    MotionPlan manual_plan;

    reset_test_state();
    memset(&manual_plan, 0, sizeof(manual_plan));
    manual_plan.count = 1U;
    manual_plan.data[0].type = SEG_WALK;
    manual_plan.data[0].dir = DIR_RIGHT;
    manual_plan.data[0].cells = 1U;

    g_test_point_vx_cmd_mmps = 0;
    g_test_point_vy_cmd_mmps = 0;

    motion_exec_init();
    motion_exec_load_manual_plan(&manual_plan, ODOM_START_GRID_X, ODOM_START_GRID_Y);
    motion_exec_request_start();
    motion_exec_tick(EXEC_CONTROL_PERIOD_MS);

    g_odometry_x_mm = g_odometry_target_x_mm - EXEC_SEGMENT_NEAR_ACCEPT_TOL_MM;
    g_odometry_y_mm = g_odometry_target_y_mm;
    g_odometry_move_state = MOVE_STATE_RUNNING;

    for(int i = 0; i < ((EXEC_SEGMENT_NEAR_ACCEPT_MS / EXEC_CONTROL_PERIOD_MS) + 2); i++)
    {
        motion_exec_tick(EXEC_CONTROL_PERIOD_MS);
    }

    assert(motion_exec_runtime_state_copy(&runtime_state));
    assert(runtime_state.current_segment_index == 1U);
    assert(runtime_state.phase == CAR_STATE_PLAN_DONE);
    assert(runtime_state.plan_finished == 1U);
    assert(g_odometry_x_mm == g_odometry_target_x_mm);
}

static void test_sustained_saturation_scales_body_command_next_cycle(void)
{
    MotionPlan manual_plan;

    reset_test_state();
    memset(&manual_plan, 0, sizeof(manual_plan));
    manual_plan.count = 1U;
    manual_plan.data[0].type = SEG_WALK;
    manual_plan.data[0].dir = DIR_RIGHT;
    manual_plan.data[0].cells = 2U;

    g_test_point_vx_cmd_mmps = 100;
    g_test_point_vy_cmd_mmps = 0;
    g_force_speed_pid_saturation = 1U;

    motion_exec_init();
    motion_exec_load_manual_plan(&manual_plan, ODOM_START_GRID_X, ODOM_START_GRID_Y);
    motion_exec_request_start();

    motion_exec_tick(EXEC_CONTROL_PERIOD_MS);
    motion_exec_tick(EXEC_CONTROL_PERIOD_MS);
    assert(g_speed_targets[MOTOR_LF] == 100);

    motion_exec_tick(EXEC_CONTROL_PERIOD_MS);
    motion_exec_tick(EXEC_CONTROL_PERIOD_MS);
    motion_exec_tick(EXEC_CONTROL_PERIOD_MS);

    assert(g_speed_targets[MOTOR_LF] == 95);
    assert(g_speed_targets[MOTOR_RF] == -95);
    assert(g_speed_targets[MOTOR_LB] == -95);
    assert(g_speed_targets[MOTOR_RB] == 95);
}

static void test_command_scale_keeps_translation_above_effective_motion_threshold(void)
{
    MotionPlan manual_plan;

    reset_test_state();
    memset(&manual_plan, 0, sizeof(manual_plan));
    manual_plan.count = 1U;
    manual_plan.data[0].type = SEG_WALK;
    manual_plan.data[0].dir = DIR_RIGHT;
    manual_plan.data[0].cells = 2U;

    g_test_point_vx_cmd_mmps = 100;
    g_test_point_vy_cmd_mmps = 0;
    g_force_speed_pid_saturation = 1U;

    motion_exec_init();
    motion_exec_load_manual_plan(&manual_plan, ODOM_START_GRID_X, ODOM_START_GRID_Y);
    motion_exec_request_start();

    for(int i = 0; i < 30; i++)
    {
        motion_exec_tick(EXEC_CONTROL_PERIOD_MS);
    }

    assert(motion_exec_get_command_scale_pct() == MOTION_CMD_SCALE_MIN_PCT);
    assert(g_speed_targets[MOTOR_LF] == POINT_MOVE_MIN_EFFECTIVE_MMPS);
    assert(g_speed_targets[MOTOR_RF] == -POINT_MOVE_MIN_EFFECTIVE_MMPS);
    assert(g_speed_targets[MOTOR_LB] == -POINT_MOVE_MIN_EFFECTIVE_MMPS);
    assert(g_speed_targets[MOTOR_RB] == POINT_MOVE_MIN_EFFECTIVE_MMPS);
}

static void test_heading_correction_does_not_dominate_lateral_translation(void)
{
    MotionPlan manual_plan;

    reset_test_state();
    memset(&manual_plan, 0, sizeof(manual_plan));
    manual_plan.count = 1U;
    manual_plan.data[0].type = SEG_WALK;
    manual_plan.data[0].dir = DIR_RIGHT;
    manual_plan.data[0].cells = 2U;

    g_test_point_vx_cmd_mmps = 100;
    g_test_point_vy_cmd_mmps = 0;
    g_yaw_cd = -9000;

    motion_exec_init();
    motion_exec_load_manual_plan(&manual_plan, ODOM_START_GRID_X, ODOM_START_GRID_Y);
    motion_exec_request_start();

    motion_exec_tick(EXEC_CONTROL_PERIOD_MS);
    motion_exec_tick(EXEC_CONTROL_PERIOD_MS);

    assert(g_speed_targets[MOTOR_LF] <= 100 + EXEC_YAW_VZ_LIMIT);
    assert(g_speed_targets[MOTOR_RF] >= -100 - EXEC_YAW_VZ_LIMIT);
    assert(g_speed_targets[MOTOR_LB] >= -100 + EXEC_YAW_VZ_LIMIT);
    assert(g_speed_targets[MOTOR_RB] <= 100 - EXEC_YAW_VZ_LIMIT);
}

int main(void)
{
    test_motion_exec_init_resets_speed_pid_once();
    test_rotate_segment_is_not_rejected();
    test_manual_plan_load_bypasses_bfs_wait_states();
    test_manual_debug_mode_does_not_accept_segment_at_default_tolerance();
    test_completed_segment_snaps_odometry_to_grid_target();
    test_consecutive_move_segments_do_not_full_stop_between_segments();
    test_near_target_dwell_accepts_sticky_segment();
    test_sustained_saturation_scales_body_command_next_cycle();
    test_command_scale_keeps_translation_above_effective_motion_threshold();
    test_heading_correction_does_not_dominate_lateral_translation();

    puts("robot_control_exec_test passed");
    return 0;
}
