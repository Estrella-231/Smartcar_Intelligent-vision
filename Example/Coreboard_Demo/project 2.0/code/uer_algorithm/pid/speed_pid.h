#ifndef SPEED_PID_H
#define SPEED_PID_H

#include "motor_driver.h"
#include "encoder.h"
#include "math_tools.h"

/*
 * Per-wheel speed-loop state.
 *
 * Task 5 requires an incremental speed loop:
 *   target_speed - current_speed -> PID -> pwm_adjust
 *   pwm_command = last_pwm_command + pwm_adjust
 *
 * Therefore we explicitly keep:
 * - the accumulated integral term,
 * - the current and previous error,
 * - the last delta output,
 * - the final PWM command that was applied last cycle.
 */
typedef struct {
    int32_t integral;      // Accumulated integral term
    int32_t err;           // Current speed error
    int32_t last_error;    // Previous speed error
    int32_t output;        // Incremental PWM adjustment for this cycle
    int32_t last_output;   // Final PWM command applied in the previous cycle
} PID_Pram_t;

extern PID_Pram_t g_speed_pid[MOTOR_MAX];

/*
 * Clear one wheel's speed-loop history.
 *
 * Use this before starting a fresh bench test so the controller does not carry
 * integral residue or an old PWM baseline from a previous run.
 */
void speed_pid_reset(MotorID motor_id);

/*
 * Clear the speed-loop history of all wheels.
 */
void speed_pid_reset_all(void);

/*
 * Run one wheel's incremental speed PID.
 *
 * Parameters:
 * - motor_id:        wheel being controlled
 * - target_speed:    target speed, unit = encoder pulses per control period
 * - current_speed:   measured speed, unit = encoder pulses per control period
 * - period_ms:       control period in milliseconds
 *
 * Return value:
 * - final PWM command after adding this cycle's adjustment to last cycle's PWM
 *
 * Important design note for Task 5:
 * this function matches the required bench procedure exactly:
 * "PID output directly adds onto the previous PWM value".
 */
int32_t speed_pid_calc(MotorID motor_id,
                       int32_t target_speed,
                       int32_t current_speed,
                       uint32_t period_ms);

#endif // SPEED_PID_H
