#ifndef GYROSCOPE_H
#define GYROSCOPE_H

#include "hardware_config.h"
#include "robot_param.h"

/************************ IMU basic bring-up parameters ************************/

/*
 * The library default gyro range is +/-2000 dps.
 * For this range the IMU963RA driver uses about 14.3 LSB per degree/s.
 * Public values are kept in centi-units so they print cleanly while preserving
 * enough precision for the current motion-control stage:
 * - gyro_z_dps   : 0.01 degree/s
 * - rotate_angle : 0.01 degree
 */
#define GYRO_LSB_PER_DPS_X10          (143)
#define GYRO_RATE_DEADBAND_CDPS       (50)
#define IMU_CALIB_SAMPLE_COUNT        (200)
#define IMU_CALIB_SAMPLE_DELAY_MS     (5)

typedef struct {
    uint8_t init_ok;
    int32_t gyro_z_raw;
    int32_t gyro_z_bias_raw;
    int32_t gyro_z_calib;
    int32_t gyro_z_dps;
    int32_t gyro_acc_filtered;
    int32_t gyro_z_dps2;
    int32_t rotate_angle;
    int32_t rotate_angle_total;
    int32_t last_gyro_z_dps;
} IMU_Data_t;

extern IMU_Data_t g_imu;

uint8_t imu_init_and_calibrate(void);
uint8_t imu_is_ready(void);
int32_t imu_get_gyro_z_raw(void);
int32_t imu_get_yaw_cd(void);
int32_t imu_get_gyro_z_cdps(void);
void gyro_read_data(void);
void IMU_calib_data(void);
void IMU_Convert_Physical(void);

/*
 * Period-aware integration helpers.
 *
 * Task 8 introduces a dedicated 5 ms IMU timer, so angle and angular
 * acceleration updates can no longer assume one fixed compile-time period.
 */
void IMU_Calc_Rotate_Angle_Period(uint32_t period_ms);
void IMU_Calc_Rotate_Acc_Period(uint32_t period_ms);
void imu_deal_data_period(uint32_t period_ms);

/*
 * Legacy wrappers kept for older call sites that still use PID_CONTROL_PERIOD.
 */
void IMU_Calc_Rotate_Angle(void);
void IMU_Calc_Rotate_Acc(void);
void imu_deal_data(void);

void imu_reset_yaw(void);

#endif
