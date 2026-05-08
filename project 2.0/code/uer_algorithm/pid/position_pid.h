#ifndef POSITION_PID_H
#define POSITION_PID_H

#include "speed_pid.h"
#include "encoder.h"

/*
 * Legacy wheel-position outer-loop state.
 *
 * Current project motion execution does not call this module. The active path
 * is point_move + angle_pid + speed_pid at chassis level.
 *
 * Keep this module only as a preserved alternative/bench reference unless the
 * project explicitly switches back to a wheel-position outer loop.
 */
extern PID_Pram_t g_position_pid[MOTOR_MAX];

/*
 * Clear one wheel's position-loop history.
 */
void position_pid_reset(MotorID motor_id);

/*
 * Clear all wheels' position-loop history.
 */
void position_pid_reset_all(void);

/*
 * Legacy wheel-position outer-loop step.
 *
 * This converts one wheel's position error into a wheel speed target. It is
 * currently unused by the active motion executor.
 */
void pos_pid_calc(MotorID motor_id);

/*
 * Legacy helper for the preserved position-loop path.
 *
 * Return whether the current position target is considered reached. The current
 * implementation accepts either:
 * 1. the vehicle-level X/Y error being small enough, or
 * 2. all wheel position errors being within the configured dead zone.
 */
bool check_pos_arrived(void);

#endif // POSITION_PID_H
