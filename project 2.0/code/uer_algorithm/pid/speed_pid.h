#ifndef SPEED_PID_H
#define SPEED_PID_H

#include "motor_driver.h"
#include "encoder.h"
#include "math_tools.h"

#include <stdbool.h>

#define MAX_WHEEL_SPEED          (400)

typedef enum {
    PID_ZERO_RESET_IMMEDIATE = 0,
    PID_ZERO_RESET_DECAY     = 1
} PID_Zero_Reset_Mode_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    float kf;

    int32_t i_limit;
    float integral;

    int32_t err;
    int32_t last_error;
    float derivative_lpf;

    int32_t output;
    int32_t last_output;

    int32_t output_limit;
    int32_t min_effect_pwm_fwd;
    int32_t min_effect_pwm_rev;

    int32_t integral_full_error;
    int32_t integral_half_error;

    int32_t target_step_limit;
    int32_t target_ramped;

    PID_Zero_Reset_Mode_t zero_reset_mode;
    float zero_decay_factor;

    bool enable_d;
    float d_filter_alpha;
    float d_state;
    int32_t last_measurement;
} PID_Pram_t;

extern PID_Pram_t g_speed_pid[MOTOR_MAX];

void speed_pid_reset(MotorID motor_id);
void speed_pid_reset_all(void);

int32_t speed_pid_calc(MotorID motor_id,
                       int32_t target_speed,
                       int32_t current_speed,
                       uint32_t period_ms);

int32_t speed_pid_get_target_ramped(MotorID motor_id);
int32_t speed_pid_get_error(MotorID motor_id);
int32_t speed_pid_get_output(MotorID motor_id);
int32_t speed_pid_get_output_limit(MotorID motor_id);

#endif // SPEED_PID_H
