#ifndef SPEED_PID_H
#define SPEED_PID_H

#include <stdint.h>

#include "robot_param.h"

void speed_pid_reset_all(void);
int32_t speed_pid_calc(MotorID motor_id,
                       int32_t target_speed,
                       int32_t current_speed,
                       uint32_t period_ms);
int32_t speed_pid_get_target_ramped(MotorID motor_id);
int32_t speed_pid_get_error(MotorID motor_id);
int32_t speed_pid_get_output(MotorID motor_id);
int32_t speed_pid_get_output_limit(MotorID motor_id);

#endif
