#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>

#include "robot_param.h"

typedef struct {
    int32_t speed_target;
    int32_t speed_now;
    int32_t speed_last;
    int32_t pos_now;
    int32_t pos_last;
    int32_t pwm_out;
} Motor_Control_t;

extern Motor_Control_t g_motor[MOTOR_MAX];

void motor_stop_all(void);
void motor_set_pwm(MotorID motor_id, int32_t pwm);
void motor_set_pwm_all(void);
void change_speed_now(MotorID motor_id, int32_t speed);
void change_speed_target(MotorID motor_id, int32_t speed);
int32_t get_speed_target(MotorID motor_id);
int32_t get_speed_now(MotorID motor_id);
void car_set_speed_now(int32_t speed);
void car_set_position_now(int32_t x, int32_t y);
void car_set_position_target(int32_t x, int32_t y);
void car_set_angle_now(int32_t angle);

#endif
