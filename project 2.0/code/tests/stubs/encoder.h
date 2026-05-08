#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

#include "robot_param.h"

void encoder_read_data(void);
int32_t get_encoder_data(MotorID motor_id);
int16_t get_encoder_position_delta(MotorID motor_id);

#endif
