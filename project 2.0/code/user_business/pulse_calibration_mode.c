#include "pulse_calibration_mode.h"

#include "encoder.h"
#include "gyroscope.h"
#include "mecanum.h"
#include "motor_driver.h"
#include "odometry.h"
#include "speed_pid.h"

#define PULSE_CAL_TARGET_DISTANCE_MM     (3200)
#define PULSE_CAL_FORWARD_TARGET_PULSE   (42)
#define PULSE_CAL_STALL_NONE             (0xFFU)

static PulseCalStatus g_pulse_cal_status = {0};

static void pulse_cal_stop_chassis(void)
{
    Mecanum_inverse_kinematics(0, 0, 0);
    speed_pid_reset_all();
    motor_stop_all();
}

static void pulse_cal_apply_same_wheel_target(uint32_t control_period_ms)
{
    Mecanum_inverse_kinematics(0, PULSE_CAL_FORWARD_TARGET_PULSE, 0);

    for(int32_t i = 0; i < MOTOR_MAX; i++)
    {
        int32_t pwm_command = speed_pid_calc((MotorID)i,
                                             get_speed_target((MotorID)i),
                                             get_encoder_data((MotorID)i),
                                             control_period_ms);

        if(speed_pid_is_stalled((MotorID)i))
        {
            g_pulse_cal_status.state = PULSE_CAL_STATE_ERROR;
            g_pulse_cal_status.stall_motor = (uint8_t)i;
            pulse_cal_stop_chassis();
            return;
        }

        motor_set_pwm((MotorID)i, pwm_command);
    }
}

static void pulse_cal_update_status(void)
{
    g_pulse_cal_status.current_x_mm = odometry_get_x_mm();
    g_pulse_cal_status.current_y_mm = odometry_get_y_mm();
    g_pulse_cal_status.odom_delta_mm =
        g_pulse_cal_status.current_y_mm - g_pulse_cal_status.start_y_mm;
    g_pulse_cal_status.target_error_mm =
        PULSE_CAL_TARGET_DISTANCE_MM - g_pulse_cal_status.odom_delta_mm;
    g_pulse_cal_status.yaw_cd = imu_get_yaw_cd();
    g_pulse_cal_status.yaw_error_cd = 0;
}

void pulse_cal_3200_init(void)
{
    g_pulse_cal_status.state = PULSE_CAL_STATE_RUNNING;
    g_pulse_cal_status.target_distance_mm = PULSE_CAL_TARGET_DISTANCE_MM;
    g_pulse_cal_status.start_x_mm = 0;
    g_pulse_cal_status.start_y_mm = 0;
    g_pulse_cal_status.current_x_mm = 0;
    g_pulse_cal_status.current_y_mm = 0;
    g_pulse_cal_status.odom_delta_mm = 0;
    g_pulse_cal_status.target_error_mm = PULSE_CAL_TARGET_DISTANCE_MM;
    g_pulse_cal_status.yaw_cd = 0;
    g_pulse_cal_status.yaw_error_cd = 0;
    g_pulse_cal_status.stall_motor = PULSE_CAL_STALL_NONE;

    pulse_cal_stop_chassis();
    encoder_reset_all();
    imu_reset_yaw();
    odometry_reset_pose(0, 0);
}

void pulse_cal_3200_tick(uint32_t control_period_ms)
{
    if(g_pulse_cal_status.state == PULSE_CAL_STATE_FINISHED ||
       g_pulse_cal_status.state == PULSE_CAL_STATE_ERROR)
    {
        pulse_cal_stop_chassis();
        pulse_cal_update_status();
        return;
    }

    if(g_pulse_cal_status.state == PULSE_CAL_STATE_IDLE)
    {
        pulse_cal_3200_init();
    }

    encoder_read_data();
    imu_deal_data_period(control_period_ms);
    odometry_update(control_period_ms);
    pulse_cal_update_status();

    if(g_pulse_cal_status.odom_delta_mm >= PULSE_CAL_TARGET_DISTANCE_MM)
    {
        g_pulse_cal_status.state = PULSE_CAL_STATE_FINISHED;
        pulse_cal_stop_chassis();
        return;
    }

    pulse_cal_apply_same_wheel_target(control_period_ms);
    pulse_cal_update_status();
}

PulseCalState pulse_cal_3200_get_state(void)
{
    return g_pulse_cal_status.state;
}

uint8_t pulse_cal_3200_is_finished(void)
{
    return (uint8_t)(g_pulse_cal_status.state == PULSE_CAL_STATE_FINISHED);
}

void pulse_cal_3200_get_status(PulseCalStatus *out_status)
{
    if(out_status == 0)
    {
        return;
    }

    *out_status = g_pulse_cal_status;
}
