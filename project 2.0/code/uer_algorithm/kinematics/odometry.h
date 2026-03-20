#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "gyroscope.h"
#include "mecanum.h"
#include "motor_driver.h"
#include "robot_param.h"
#include "math_tools.h"

typedef struct {
    int32_t vx_body_mmps;
    int32_t vy_body_mmps;
    int32_t vx_global_mmps;
    int32_t vy_global_mmps;
    int32_t x_mm;
    int32_t y_mm;
    int32_t target_x_mm;
    int32_t target_y_mm;
    int32_t slip_weight_pct;
    MoveState move_state;
} Odometry_State_t;

extern Odometry_State_t g_odometry;

/*
 * Initialize/reset odometry pose and runtime state.
 */
void odometry_init(int32_t start_x_mm, int32_t start_y_mm);
void odometry_reset_pose(int32_t start_x_mm, int32_t start_y_mm);

/*
 * Update body velocity, global velocity, and integrated pose from wheel speed
 * feedback plus current IMU yaw.
 */
void odometry_update(uint32_t period_ms);

/*
 * Configure and run simple point-to-point motion generation.
 *
 * The generated commands are body-frame linear velocity commands in mm/s.
 * Heading hold is still handled by the separate yaw outer loop.
 */
void odometry_set_target_point(int32_t target_x_mm, int32_t target_y_mm);
MoveState odometry_update_point_move_command(int32_t *vx_cmd_mmps,
                                             int32_t *vy_cmd_mmps);

/*
 * Read-only helpers for telemetry and upper-layer logic.
 */
int32_t odometry_get_vx_body_mmps(void);
int32_t odometry_get_vy_body_mmps(void);
int32_t odometry_get_vx_global_mmps(void);
int32_t odometry_get_vy_global_mmps(void);
int32_t odometry_get_x_mm(void);
int32_t odometry_get_y_mm(void);
int32_t odometry_get_target_dx_mm(void);
int32_t odometry_get_target_dy_mm(void);
int32_t odometry_get_slip_weight_pct(void);
MoveState odometry_get_move_state(void);

#endif
