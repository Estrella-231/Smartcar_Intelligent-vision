#ifndef GYROSCOPE_H
#define GYROSCOPE_H

#include <stdint.h>

uint8_t imu_is_ready(void);
int32_t imu_get_yaw_cd(void);
int32_t imu_get_gyro_z_cdps(void);
void imu_deal_data_period(uint32_t period_ms);

#endif
