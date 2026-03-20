#ifndef ENCODER_H
#define ENCODER_H

#include "zf_common_headfile.h"
#include "motor_driver.h"

// Pulse increment measured during one PIT period after sign normalization.
// Positive means "same direction as positive motor command".
extern volatile int16_t encoder_data[MOTOR_MAX];

void Encoder_Init(void);
void encoder_reset_all(void);
void encoder_read_data(void);
int16_t get_encoder_data(MotorID motor_id);

#endif
