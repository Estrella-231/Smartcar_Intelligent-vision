#ifndef POSITION_PID_H
#define POSITION_PID_H

#include "speed_pid.h"
#include "encoder.h"

/*
 * One position-loop state block is kept for each wheel. The position loop feeds
 * the speed loop, so it reuses the same PID state structure type.
 */
extern PID_Pram_t g_position_pid[MOTOR_MAX];

/*
 * Run the position PID for a single wheel. The function converts position error
 * into a speed target and writes that target through change_speed_target().
 */
void pos_pid_calc(MotorID motor_id);

/*
 * Return whether the current position target is considered reached. The current
 * implementation accepts either:
 * 1. the vehicle-level X/Y error being small enough, or
 * 2. all wheel position errors being within the configured dead zone.
 */
bool check_pos_arrived(void);

#endif // POSITION_PID_H
