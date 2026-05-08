#ifndef ANGLE_PID_H
#define ANGLE_PID_H

#include <stdint.h>

typedef enum {
    ROT_STATE_IDLE = 0,
    ROT_STATE_RUNNING = 1,
    ROT_STATE_STABLE = 2,
    ROT_STATE_FINISHED = 3
} Rotate_State_Type;

void angle_pid_set_target(int32_t angle);
void angle_pid_reset_state(void);
int32_t angle_pid_calc_output(int32_t target_angle_cd,
                              int32_t current_angle_cd,
                              uint32_t period_ms);
int32_t Rotate_Finish_Judge(int32_t target_angle, int32_t curr_angle, int32_t curr_gyro);

#endif
