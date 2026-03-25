#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include "zf_common_headfile.h"

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
void motion_exec_request_start(void);
void motion_exec_tick(uint32_t control_period_ms);

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
