#ifndef MECANUM_H
#define MECANUM_H

#include "motor_driver.h"
#include "robot_param.h"

/*
 * Task 7 inverse kinematics interface.
 *
 * Coordinate/sign convention used by this project:
 * - vx_cmd > 0: chassis translates to the right
 * - vy_cmd > 0: chassis translates forward
 * - vz_cmd > 0: chassis rotates clockwise
 *
 * Unit convention at the current project stage:
 * - all three command inputs are expressed as "equivalent wheel speed command"
 * - practical unit = encoder pulses per control period contribution
 *
 * Why this unit is used now:
 * - Task 6 has already tuned the speed loop in encoder-pulse space
 * - wheelbase and geometry constants are not fully calibrated yet
 * - keeping Task 7 in the same unit makes bench and ground debugging easier
 */
void Mecanum_inverse_kinematics(int32_t vx_cmd, int32_t vy_cmd, int32_t vz_cmd);

/*
 * Round-2 forward kinematics helper.
 *
 * Input:
 * - four wheel linear speeds in mm/s
 *
 * Output:
 * - chassis body-frame linear velocity in mm/s
 * - vx_body > 0: move right
 * - vy_body > 0: move forward
 */
void Mecanum_forward_kinematics_mmps(int32_t lf_mmps,
                                     int32_t rf_mmps,
                                     int32_t lb_mmps,
                                     int32_t rb_mmps,
                                     int32_t *vx_body_mmps,
                                     int32_t *vy_body_mmps);

/*
 * Unit conversion helpers between wheel linear speed and encoder-pulse feedback.
 */
int32_t Mecanum_pulse_per_period_to_mmps(int32_t pulse_per_period, uint32_t period_ms);
int32_t Mecanum_mmps_to_pulse_per_period(int32_t velocity_mmps, uint32_t period_ms);

/*
 * Legacy wrapper kept for compatibility with the existing direction-based calls.
 *
 * This function now forwards to the general inverse kinematics entry instead of
 * hard-coding wheel targets itself.
 */
void Mecanum_entirety2part(MoveDirection car_direction, int32_t car_target_speed);

/*
 * Rough forward-kinematics helper that synthesizes a single scalar vehicle speed
 * for legacy code paths. This project is not using a full pose estimator here yet.
 */
void Mecanum_part2entirety(MoveDirection car_direction);

/*
 * Legacy helper that estimates wheel speed from encoder position change.
 */
void Calc_SpeedNow(MotorID motor_id);

#endif // MECANUM_H
