#ifndef PULSE_CALIBRATION_MODE_H
#define PULSE_CALIBRATION_MODE_H

#include <stdint.h>

#include "robot_param.h"

typedef enum {
    PULSE_CAL_STATE_IDLE = 0,
    PULSE_CAL_STATE_RUNNING,
    PULSE_CAL_STATE_FINISHED,
    PULSE_CAL_STATE_ERROR
} PulseCalState;

typedef struct {
    PulseCalState state;
    int32_t target_distance_mm;
    int32_t start_x_mm;
    int32_t start_y_mm;
    int32_t current_x_mm;
    int32_t current_y_mm;
    int32_t odom_delta_mm;
    int32_t target_error_mm;
    int32_t yaw_cd;
    int32_t yaw_error_cd;
    uint8_t stall_motor;
} PulseCalStatus;

/*
 * Dedicated 3200 mm forward calibration mode.
 *
 * This mode intentionally does not run point-to-point correction or heading PID.
 * It sends the same forward wheel target to all four wheels and stops when
 * odometry CH11 reaches 3200 mm. Use it to calibrate PULSE_PER_MM without
 * mixing in path-following behavior.
 *
 * Integration:
 * - Call pulse_cal_3200_init() after motor/encoder/IMU/PID init is complete.
 * - Call pulse_cal_3200_tick(EXEC_CONTROL_PERIOD_MS) once every control period.
 * - Keep monitor_tick()/send_data() in your main loop if you want PC telemetry.
 */
void pulse_cal_3200_init(void);
void pulse_cal_3200_tick(uint32_t control_period_ms);
PulseCalState pulse_cal_3200_get_state(void);
uint8_t pulse_cal_3200_is_finished(void);
void pulse_cal_3200_get_status(PulseCalStatus *out_status);

#endif
