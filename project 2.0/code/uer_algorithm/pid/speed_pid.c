#include "speed_pid.h"

#include <stdlib.h>


PID_Pram_t g_speed_pid[MOTOR_MAX] =
{

	{
        .kp = 1.8f, .ki = 0.25f, .kd = 0.0f, .kf = 1.8f,
        .i_limit = 220,
        .output_limit = 650,
        .min_effect_pwm_fwd = 70, .min_effect_pwm_rev = 75,
        .integral_full_error = 50, .integral_half_error = 120,
        .target_step_limit = 20,
        .zero_reset_mode = PID_ZERO_RESET_IMMEDIATE, .zero_decay_factor = 0.8f,
        .enable_d = false, .d_filter_alpha = 0.7f,
        .stall_params = {
            .pwm_threshold_pct = STALL_PWM_THRESHOLD_PCT,
            .speed_threshold_pulse = STALL_SPEED_THRESHOLD_PULSE,
            .confirm_time_ms = STALL_CONFIRM_TIME_MS,
            .recovery_delay_ms = STALL_RECOVERY_DELAY_MS,
            .stop_all_on_stall = (STALL_STOP_ALL_MOTORS != 0)
        },
        .stall_state = {0}
    },
    {
        .kp = 1.8f, .ki = 0.25f, .kd = 0.0f, .kf = 1.8f,
        .i_limit = 220,
        .output_limit = 650,
        .min_effect_pwm_fwd = 70, .min_effect_pwm_rev = 75,
        .integral_full_error = 50, .integral_half_error = 120,
        .target_step_limit = 20,
        .zero_reset_mode = PID_ZERO_RESET_IMMEDIATE, .zero_decay_factor = 0.8f,
        .enable_d = false, .d_filter_alpha = 0.7f,
        .stall_params = {
            .pwm_threshold_pct = STALL_PWM_THRESHOLD_PCT,
            .speed_threshold_pulse = STALL_SPEED_THRESHOLD_PULSE,
            .confirm_time_ms = STALL_CONFIRM_TIME_MS,
            .recovery_delay_ms = STALL_RECOVERY_DELAY_MS,
            .stop_all_on_stall = (STALL_STOP_ALL_MOTORS != 0)
        },
        .stall_state = {0}
    },
    {
        .kp = 1.8f, .ki = 0.25f, .kd = 0.0f, .kf = 1.9f,
        .i_limit = 240,
        .output_limit = 650,
        .min_effect_pwm_fwd = 72, .min_effect_pwm_rev = 78,
        .integral_full_error = 50, .integral_half_error = 120,
        .target_step_limit = 20,
        .zero_reset_mode = PID_ZERO_RESET_IMMEDIATE, .zero_decay_factor = 0.8f,
        .enable_d = false, .d_filter_alpha = 0.7f,
        .stall_params = {
            .pwm_threshold_pct = STALL_PWM_THRESHOLD_PCT,
            .speed_threshold_pulse = STALL_SPEED_THRESHOLD_PULSE,
            .confirm_time_ms = STALL_CONFIRM_TIME_MS,
            .recovery_delay_ms = STALL_RECOVERY_DELAY_MS,
            .stop_all_on_stall = (STALL_STOP_ALL_MOTORS != 0)
        },
        .stall_state = {0}
    },
    {
        .kp = 1.8f, .ki = 0.25f, .kd = 0.0f, .kf = 1.9f,
        .i_limit = 240,
        .output_limit = 650,
        .min_effect_pwm_fwd = 72, .min_effect_pwm_rev = 78,
        .integral_full_error = 50, .integral_half_error = 120,
        .target_step_limit = 20,
        .zero_reset_mode = PID_ZERO_RESET_IMMEDIATE, .zero_decay_factor = 0.8f,
        .enable_d = false, .d_filter_alpha = 0.7f,
        .stall_params = {
            .pwm_threshold_pct = STALL_PWM_THRESHOLD_PCT,
            .speed_threshold_pulse = STALL_SPEED_THRESHOLD_PULSE,
            .confirm_time_ms = STALL_CONFIRM_TIME_MS,
            .recovery_delay_ms = STALL_RECOVERY_DELAY_MS,
            .stop_all_on_stall = (STALL_STOP_ALL_MOTORS != 0)
        },
        .stall_state = {0}
    }
};

static float speed_pid_get_integral_scale(const PID_Pram_t *pid, int32_t err_abs)
{
    if(err_abs < pid->integral_full_error)
    {
        return 1.0f;
    }

    if(err_abs < pid->integral_half_error)
    {
        return 0.5f;
    }

    return 0.0f;
}

static int32_t speed_pid_ramp_target(int32_t target, int32_t last_target, int32_t step_limit)
{
    int32_t delta = target - last_target;

    if(step_limit <= 0)
    {
        return target;
    }

    if(delta > step_limit)
    {
        return last_target + step_limit;
    }

    if(delta < -step_limit)
    {
        return last_target - step_limit;
    }

    return target;
}

void speed_pid_reset(MotorID motor_id)
{
    if(motor_id >= MOTOR_MAX)
    {
        return;
    }

    g_speed_pid[motor_id].integral = 0.0f;
    g_speed_pid[motor_id].err = 0;
    g_speed_pid[motor_id].last_error = 0;
    g_speed_pid[motor_id].derivative_lpf = 0.0f;
    g_speed_pid[motor_id].output = 0;
    g_speed_pid[motor_id].last_output = 0;
    g_speed_pid[motor_id].target_ramped = 0;
    g_speed_pid[motor_id].d_state = 0.0f;
    g_speed_pid[motor_id].last_measurement = 0;
}

void speed_pid_reset_all(void)
{
    for(int i = 0; i < MOTOR_MAX; i++)
    {
        speed_pid_reset((MotorID)i);
    }
}

int32_t speed_pid_get_target_ramped(MotorID motor_id)
{
    if(motor_id >= MOTOR_MAX)
    {
        return 0;
    }

    return g_speed_pid[motor_id].target_ramped;
}

int32_t speed_pid_get_error(MotorID motor_id)
{
    if(motor_id >= MOTOR_MAX)
    {
        return 0;
    }

    return g_speed_pid[motor_id].err;
}

int32_t speed_pid_get_output(MotorID motor_id)
{
    if(motor_id >= MOTOR_MAX)
    {
        return 0;
    }

    return g_speed_pid[motor_id].output;
}

int32_t speed_pid_get_output_limit(MotorID motor_id)
{
    if(motor_id >= MOTOR_MAX)
    {
        return 0;
    }

    return g_speed_pid[motor_id].output_limit;
}

/************************ Stall Protection Implementation ************************/

void speed_pid_stall_protection_init(void)
{
    for(int i = 0; i < MOTOR_MAX; i++)
    {
        g_speed_pid[i].stall_params.pwm_threshold_pct = STALL_PWM_THRESHOLD_PCT;
        g_speed_pid[i].stall_params.speed_threshold_pulse = STALL_SPEED_THRESHOLD_PULSE;
        g_speed_pid[i].stall_params.confirm_time_ms = STALL_CONFIRM_TIME_MS;
        g_speed_pid[i].stall_params.recovery_delay_ms = STALL_RECOVERY_DELAY_MS;
        g_speed_pid[i].stall_params.stop_all_on_stall = (STALL_STOP_ALL_MOTORS != 0);

        g_speed_pid[i].stall_state.state = STALL_STATE_NORMAL;
        g_speed_pid[i].stall_state.stall_counter_ms = 0;
        g_speed_pid[i].stall_state.recovery_counter_ms = 0;
        g_speed_pid[i].stall_state.stall_detected = 0;
        g_speed_pid[i].stall_state.stalled_pwm_value = 0;
    }
}

uint8_t speed_pid_is_stalled(MotorID motor_id)
{
    if(motor_id >= MOTOR_MAX)
    {
        return 0;
    }

    return g_speed_pid[motor_id].stall_state.stall_detected;
}

void speed_pid_clear_stall(MotorID motor_id)
{
    if(motor_id >= MOTOR_MAX)
    {
        return;
    }

    g_speed_pid[motor_id].stall_state.state = STALL_STATE_NORMAL;
    g_speed_pid[motor_id].stall_state.stall_counter_ms = 0;
    g_speed_pid[motor_id].stall_state.recovery_counter_ms = 0;
    g_speed_pid[motor_id].stall_state.stall_detected = 0;
    g_speed_pid[motor_id].stall_state.stalled_pwm_value = 0;
}

uint8_t speed_pid_any_motor_stalled(void)
{
    for(int i = 0; i < MOTOR_MAX; i++)
    {
        if(g_speed_pid[i].stall_state.stall_detected)
        {
            return 1;
        }
    }
    return 0;
}

void speed_pid_stall_protection_tick(uint32_t period_ms)
{
#if STALL_PROTECTION_ENABLE
    for(int i = 0; i < MOTOR_MAX; i++)
    {
        StallDetectionState_t *state = &g_speed_pid[i].stall_state;
        StallProtectionParams_t *params = &g_speed_pid[i].stall_params;

        if(state->state == STALL_STATE_RECOVERY)
        {
            state->recovery_counter_ms += (uint16_t)period_ms;

            if(state->recovery_counter_ms >= params->recovery_delay_ms)
            {
                state->state = STALL_STATE_NORMAL;
                state->recovery_counter_ms = 0;
                state->stall_counter_ms = 0;
                state->stall_detected = 0;
            }
        }
    }
#else
    (void)period_ms;
#endif
}

static uint8_t speed_pid_detect_stall(MotorID motor_id,
                                       int32_t pwm_output,
                                       int32_t current_speed,
                                       uint32_t period_ms)
{
    PID_Pram_t *pid;
    StallProtectionParams_t *params;
    StallDetectionState_t *state;
    int32_t pwm_threshold;
    int32_t speed_abs;

#if !STALL_PROTECTION_ENABLE
    (void)motor_id;
    (void)pwm_output;
    (void)current_speed;
    (void)period_ms;
    return 0;
#endif

    if(motor_id >= MOTOR_MAX)
    {
        return 0;
    }

    pid = &g_speed_pid[motor_id];
    params = &pid->stall_params;
    state = &pid->stall_state;

    if(state->state == STALL_STATE_CONFIRMED)
    {
        return 1;
    }

    if(state->state == STALL_STATE_RECOVERY)
    {
        return 0;
    }

    if(pid->output_limit <= 0)
    {
        return 0;
    }

    pwm_threshold = (pid->output_limit * params->pwm_threshold_pct + 99) / 100;
    speed_abs = abs(current_speed);

    if((abs(pwm_output) >= pwm_threshold) &&
       (speed_abs < params->speed_threshold_pulse) &&
       (pid->target_ramped != 0))
    {
        if(state->state == STALL_STATE_NORMAL)
        {
            state->state = STALL_STATE_DETECTING;
            state->stall_counter_ms = 0;
        }

        if(state->state == STALL_STATE_DETECTING)
        {
            state->stall_counter_ms += (uint16_t)period_ms;

            if(state->stall_counter_ms >= params->confirm_time_ms)
            {
                state->state = STALL_STATE_CONFIRMED;
                state->stall_detected = 1;
                state->stalled_pwm_value = pwm_output;
                return 1;
            }
        }
    }
    else
    {
        if(state->state == STALL_STATE_DETECTING)
        {
            state->state = STALL_STATE_NORMAL;
            state->stall_counter_ms = 0;
        }
    }

    return 0;
}

static void speed_pid_handle_stall(MotorID motor_id)
{
    PID_Pram_t *pid;

    if(motor_id >= MOTOR_MAX)
    {
        return;
    }

    pid = &g_speed_pid[motor_id];

    pid->integral = 0.0f;
    pid->err = 0;
    pid->last_error = 0;
    pid->output = 0;
    pid->last_output = 0;
    pid->target_ramped = 0;
    pid->derivative_lpf = 0.0f;
    pid->d_state = 0.0f;

    if(pid->stall_params.stop_all_on_stall)
    {
        motor_stop_all();

        for(int i = 0; i < MOTOR_MAX; i++)
        {
            g_speed_pid[i].integral = 0.0f;
            g_speed_pid[i].output = 0;
            g_motor[i].pwm_out = 0;
        }
    }
    else
    {
        motor_set_pwm(motor_id, 0);
        g_motor[motor_id].pwm_out = 0;
    }

    pid->stall_state.state = STALL_STATE_RECOVERY;
    pid->stall_state.recovery_counter_ms = 0;
}

int32_t speed_pid_calc(MotorID motor_id,
                       int32_t target_speed,
                       int32_t current_speed,
                       uint32_t period_ms)
{
    PID_Pram_t *pid;
    int32_t limited_target;
    int32_t ramped_target;
    int32_t err;
    int32_t err_abs;
    float dt_s;
    float integral_scale;
    float candidate_integral;
    float ff_output;
    float d_term = 0.0f;
    float pre_output;
    float final_output;
    int32_t pwm_unsat_candidate;
    int32_t pwm_command;
    bool allow_integral = false;

    if(motor_id >= MOTOR_MAX || period_ms == 0)
    {
        return 0;
    }

    pid = &g_speed_pid[motor_id];
    dt_s = (float)period_ms / 1000.0f;

    limited_target = LIMIT_ABS(target_speed, MAX_WHEEL_SPEED);
    ramped_target = speed_pid_ramp_target(limited_target,
                                          pid->target_ramped,
                                          pid->target_step_limit);
    pid->target_ramped = ramped_target;

    change_speed_now(motor_id, current_speed);

    if(ramped_target == 0)
    {
        if(pid->zero_reset_mode == PID_ZERO_RESET_IMMEDIATE)
        {
            pid->integral = 0.0f;
        }
        else
        {
            pid->integral *= pid->zero_decay_factor;

            if(pid->integral > -1.0f && pid->integral < 1.0f)
            {
                pid->integral = 0.0f;
            }
        }

        pid->err = 0;
        pid->last_error = 0;
        pid->derivative_lpf = 0.0f;
        pid->output = 0;
        pid->last_output = 0;
        pid->d_state = 0.0f;
        pid->last_measurement = current_speed;
        g_motor[motor_id].pwm_out = 0;
        return 0;
    }

    err = ramped_target - current_speed;
    err_abs = abs(err);
    pid->err = err;

    ff_output = pid->kf * (float)ramped_target;

    if(pid->enable_d && pid->kd > 0.0f)
    {
        float measurement_derivative =
            ((float)current_speed - (float)pid->last_measurement) / dt_s;
        float d_raw = -pid->kd * measurement_derivative;

        pid->d_state = pid->d_filter_alpha * pid->d_state +
                       (1.0f - pid->d_filter_alpha) * d_raw;
        d_term = pid->d_state;
        pid->derivative_lpf = d_term;
    }
    else
    {
        pid->d_state = 0.0f;
        pid->derivative_lpf = 0.0f;
    }

    integral_scale = speed_pid_get_integral_scale(pid, err_abs);
    candidate_integral = pid->integral + ((float)err * dt_s * integral_scale);
    candidate_integral = LIMIT_ABS(candidate_integral, (float)pid->i_limit);

    pre_output = ff_output +
                 pid->kp * (float)err +
                 pid->ki * candidate_integral +
                 d_term;
    pwm_unsat_candidate = (int32_t)pre_output;

    if(abs(pwm_unsat_candidate) <= pid->output_limit)
    {
        allow_integral = true;
    }
    else if(pwm_unsat_candidate > pid->output_limit && err < 0)
    {
        allow_integral = true;
    }
    else if(pwm_unsat_candidate < -pid->output_limit && err > 0)
    {
        allow_integral = true;
    }

    if(allow_integral)
    {
        pid->integral = candidate_integral;
        final_output = pre_output;
    }
    else
    {
        final_output = ff_output +
                       pid->kp * (float)err +
                       pid->ki * pid->integral +
                       d_term;
    }

    pwm_command = LIMIT_ABS((int32_t)final_output, pid->output_limit);

    if(pwm_command > 0 && pwm_command < pid->min_effect_pwm_fwd)
    {
        pwm_command = pid->min_effect_pwm_fwd;
    }
    else if(pwm_command < 0 && pwm_command > -pid->min_effect_pwm_rev)
    {
        pwm_command = -pid->min_effect_pwm_rev;
    }

    pid->output = pwm_command;
    pid->last_output = pwm_command;
    pid->last_error = err;
    pid->last_measurement = current_speed;
    g_motor[motor_id].pwm_out = pwm_command;

#if STALL_PROTECTION_ENABLE
    if(speed_pid_detect_stall(motor_id, pwm_command, current_speed, period_ms))
    {
        speed_pid_handle_stall(motor_id);
        return 0;
    }
#endif

    return pwm_command;
}
