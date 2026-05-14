#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stdint.h>

#include "robot_param.h"

typedef enum {
    POINT_MOVE_PROFILE_WALK = 0,
    POINT_MOVE_PROFILE_PUSH = 1
} PointMoveProfile_t;

void odometry_init(int32_t start_x_mm, int32_t start_y_mm);
void odometry_reset_pose(int32_t start_x_mm, int32_t start_y_mm);
void odometry_update(uint32_t period_ms);
void odometry_set_point_move_profile(PointMoveProfile_t profile);
void odometry_set_target_point(int32_t target_x_mm, int32_t target_y_mm);
void odometry_set_finish_tolerance_mm(int32_t tolerance_mm);
MoveState odometry_update_point_move_command(int32_t *vx_cmd_mmps,
                                             int32_t *vy_cmd_mmps);
int32_t odometry_get_x_mm(void);
int32_t odometry_get_y_mm(void);
int32_t odometry_get_target_dx_mm(void);
int32_t odometry_get_target_dy_mm(void);

#endif
