#ifndef MECANUM_H
#define MECANUM_H

#include <stdint.h>

void Mecanum_inverse_kinematics(int32_t vx_cmd, int32_t vy_cmd, int32_t vz_cmd);
void Mecanum_forward_kinematics_mmps(int32_t lf_mmps,
                                     int32_t rf_mmps,
                                     int32_t lb_mmps,
                                     int32_t rb_mmps,
                                     int32_t *vx_body_mmps,
                                     int32_t *vy_body_mmps);
int32_t Mecanum_mmps_to_pulse_per_period(int32_t velocity_mmps, uint32_t period_ms);
int32_t Mecanum_pulse_per_period_to_mmps(int32_t pulse_per_period, uint32_t period_ms);

#endif
