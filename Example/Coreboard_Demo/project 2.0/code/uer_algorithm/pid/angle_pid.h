#ifndef ANGLE_PID_H
#define ANGLE_PID_H

#include "motor_driver.h"
#include "math_tools.h"
#include "gyroscope.h"

typedef struct {
    int32_t integral;
    int32_t target;
    int32_t feedback;
    int32_t err;
    int32_t last_error;
    int32_t last_output;
    int32_t output;
} ROTATEPID_t;

typedef enum {
    ROT_STATE_IDLE = 0,
    ROT_STATE_RUNNING = 1,
    ROT_STATE_STABLE = 2,
    ROT_STATE_FINISHED = 3
} Rotate_State_Type;

extern Rotate_State_Type g_rot_state;
extern ROTATEPID_t g_rotate;
extern ROTATEPID_t g_angle;

/*
 * Business-layer helper wrappers.
 */
void angle_pid_set_target(int32_t angle);
void angle_pid_reset_state(void);

/*
 * Task 9 outer-loop helper.
 *
 * Input/return unit:
 * - target_angle_cd  : 0.01 degree
 * - current_angle_cd : 0.01 degree
 * - return value     : yaw-correction command used as Vz contribution
 *
 * This function keeps the angle-loop state in g_angle while exposing a clean
 * interface to the caller.
 */
int32_t angle_pid_calc_output(int32_t target_angle_cd,
                              int32_t current_angle_cd,
                              uint32_t period_ms);

/*
 * Read-only helpers for debug output and upper-layer inspection.
 */
int32_t angle_pid_get_error(void);
int32_t angle_pid_get_output(void);

/*
 * Legacy interfaces kept for the existing rotation-only code path.
 */
void angle_pid_calc(void);
void rotate_pid_calc(void);
void Rotate_PWM_Calc(void);
int32_t Rotate_Finish_Judge(int32_t target_angle, int32_t curr_angle, int32_t curr_gyro);

#endif
