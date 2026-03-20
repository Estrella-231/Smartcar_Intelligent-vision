#include "robot_control.h"

#include "angle_pid.h"
#include "bluetooth.h"
#include "gyroscope.h"
#include "motor_driver.h"

/*
 * Current business-layer rotation wrapper.
 *
 * This function is intentionally kept small. It orchestrates the already
 * existing angle/gyro/PWM logic but leaves the low-level controller details in
 * the PID and driver modules. Moving the heavy dependencies into this .c file
 * keeps robot_control.h from leaking the whole stack to any unrelated module
 * that only wants to call Control_Rotate().
 */
void Control_Rotate(int32_t angle)
{
    static uint8_t finish_cnt = 0;

    /*
     * The finish judge belongs to the algorithm layer, but the decision of
     * "what to do once rotation is considered finished" belongs to the business
     * layer. That is why the wrapper checks the state and then performs stop /
     * reset actions here.
     */
    int32_t state = Rotate_Finish_Judge(angle, imu_get_yaw_cd(), imu_get_gyro_z_cdps());

    if(state == ROT_STATE_FINISHED)
    {
        finish_cnt++;
        if(finish_cnt >= 3)
        {
            motor_stop_all();
            send_data(0, 6, 6, 0);

            /*
             * Clear controller history when the action is complete. Otherwise
             * the next rotation command can inherit stale integral and derivative
             * state from the previous maneuver.
             */
            angle_pid_reset_state();
        }
    }
    else
    {
        finish_cnt = 0;
        angle_pid_set_target(angle);

        /*
         * The actual closed-loop calls are still commented out in the current
         * project stage because task 9 has not started yet. Keep this wrapper in
         * place so later enabling the full chain only requires restoring those
         * calls here rather than re-spreading control logic across modules.
         */
//        angle_pid_calc();
//        system_delay_ms(10);
//        rotate_pid_calc();
//        system_delay_ms(10);
//        Rotate_PWM_Calc();
    }
}
