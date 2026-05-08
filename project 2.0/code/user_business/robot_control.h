#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include "zf_common_headfile.h"
#include "soko_types.h"

/*
 * Business-layer control entry points should expose only the API that higher
 * layers need to call.
 *
 * Avoid turning this header into a catch-all include for drivers, algorithms,
 * Bluetooth, or IMU state. Those implementation details belong in
 * robot_control.c so module boundaries stay readable.
 */
void Control_Rotate(int32_t angle);

/*
 * Motion execution scheduler entry points.
 *
 * These APIs sit above BFS and odometry:
 * - BFS provides a MotionPlan
 * - the scheduler consumes that MotionPlan segment by segment
 * - the scheduler drives the already existing point-move + heading-hold stack
 */
void motion_exec_init(void);
void motion_exec_load_manual_plan(const MotionPlan *plan,
                                  uint8_t start_grid_x,
                                  uint8_t start_grid_y);
void motion_exec_request_start(void);
void motion_exec_tick(uint32_t control_period_ms);
void motion_exec_set_segment_accept_tolerance_mm(int32_t tolerance_mm);
int32_t motion_exec_get_command_scale_pct(void);

/*
 * IPS200 execution-view display entry points.
 *
 * The screen shows:
 * - one 16 x 12 grid mapped 1:1 to the virtual field
 * - the current odometry position
 * - the current segment target point while a segment is running
 */
void motion_display_init(void);
void motion_display_tick(void);

#endif // ROBOT_CONTROL_H
