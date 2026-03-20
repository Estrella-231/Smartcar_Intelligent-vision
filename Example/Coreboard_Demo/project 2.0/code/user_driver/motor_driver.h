#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "hardware_config.h"
#include "robot_param.h"

/************************ Motor hardware config ************************/
typedef struct {
    gpio_pin_enum dir_pin;
    pwm_channel_enum pwm_pin;
    gpio_level_enum dir_level;   // Logic level that means forward for this wheel.
} Motor_HwConfig_t;

/************************ Motor control state ************************/
typedef struct {
    Motor_HwConfig_t hw;
    int32_t speed_target;
    int32_t speed_now;
    int32_t speed_last;
    int32_t pos_target;
    int32_t pos_now;
    int32_t pos_last;
    int32_t pwm_out;             // Signed PWM command.
} Motor_Control_t;

/************************ Vehicle control state ************************/
typedef struct {
    int32_t speed_target;
    int32_t speed_now;
    int32_t angle_target;
    int32_t angle_now;
    int32_t x_target;
    int32_t y_target;
    int32_t x_now;
    int32_t y_now;
} Car_Control_t;

/************************ Globals ************************/
extern Motor_Control_t g_motor[MOTOR_MAX];
extern Car_Control_t g_car;

/************************ API ************************/
void motor_driver_init(void);
void motor_hw_init(MotorID motor_id);
void motor_set_direction(MotorID motor_id, bool forward);
void motor_set_pwm(MotorID motor_id, int32_t pwm);
void motor_set_pwm_all(void);
void motor_stop(MotorID motor_id);
void motor_stop_all(void);
int32_t get_pos_now(MotorID motor_id);
void change_pos_now(MotorID motor_id, int32_t position);
void change_pos_target(MotorID motor_id, int32_t position);
int32_t get_speed_now(MotorID motor_id);
void change_speed_now(MotorID motor_id, int32_t speed);
void change_speed_target(MotorID motor_id, int32_t speed);
int32_t get_speed_target(MotorID motor_id);
int32_t get_pos_target(MotorID motor_id);

/*
 * Car-level state accessors.
 *
 * The project still stores the aggregate vehicle state in the global g_car
 * struct. These helpers provide a narrow API for other modules so they can stop
 * reaching into g_car fields directly one by one.
 */
int32_t car_get_speed_now(void);
void car_set_speed_now(int32_t speed);
int32_t car_get_angle_now(void);
void car_set_angle_now(int32_t angle);
int32_t car_get_x_now(void);
int32_t car_get_y_now(void);
void car_set_position_now(int32_t x, int32_t y);
int32_t car_get_x_target(void);
int32_t car_get_y_target(void);
void car_set_position_target(int32_t x, int32_t y);

#endif // MOTOR_DRIVER_H
