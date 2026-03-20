#include "zf_common_headfile.h"

#include "hardware_config.h"
#include "robot_param.h"

#include "angle_pid.h"
#include "bluetooth.h"
#include "encoder.h"
#include "gyroscope.h"
#include "mecanum.h"
#include "motor_driver.h"
#include "odometry.h"
#include "speed_pid.h"

/*
 * Round-2 task 5 only.
 *
 * This file runs the first point-to-point motion mode built on top of the four
 * completed foundation tasks:
 * 1. body-frame forward kinematics
 * 2. global velocity mapping
 * 3. X/Y integration
 * 4. field calibration and basic anti-slip handling
 *
 * Scope of this first version:
 * - one target point only
 * - fixed heading during the move
 * - no path planning
 * - no obstacle avoidance
 *
 * Control chain:
 * 1. read encoder feedback
 * 2. update odometry pose
 * 3. compute vx/vy point-move command from current pose and target pose
 * 4. compute vz correction from heading hold
 * 5. run inverse kinematics and four wheel speed loops
 *
 * Bluetooth output:
 * - a = x_mm
 * - b = y_mm
 * - c = dx_mm = target_x - x_now
 * - d = dy_mm = target_y - y_now
 */
#define TASK5_CONTROL_PERIOD_MS          (20)
#define TASK5_IMU_FAST_PERIOD_MS         (5)
#define TASK5_DEBUG_SEND_PERIOD_MS       (100)

#define TASK5_TARGET_YAW_CD              (0)
#define TASK5_VZ_CMD_LIMIT               (90)
#define TASK5_MID_VZ_CMD_LIMIT           (30)
#define TASK5_NEAR_VZ_CMD_LIMIT          (8)
#define TASK5_YAW_SOFTEN_MM              (450)
#define TASK5_NEAR_YAW_DEADBAND_CD       (250)

/*
 * Default point-to-point target.
 *
 * Recommended validation order:
 * 1. (600, 0)
 * 2. (0, 600)
 * 3. (600, 600)
 */
#define TASK5_TARGET_X_MM                (600)
#define TASK5_TARGET_Y_MM                (0)

#define TASK5_DEBUG_MODE_POSE            (0)
#define TASK5_DEBUG_MODE_COMMAND         (1)
#define TASK5_DEBUG_MODE                 (TASK5_DEBUG_MODE_POSE)

static volatile int32_t g_task5_debug_a = 0;
static volatile int32_t g_task5_debug_b = 0;
static volatile int32_t g_task5_debug_c = 0;
static volatile int32_t g_task5_debug_d = 0;

/*
 * Prepare the complete point-to-point stack.
 *
 * Startup rules:
 * - IMU yaw is reset so heading target 0 has a consistent meaning
 * - odometry pose is reset to the configured global start point
 * - target point is configured once at startup
 * - point-move logic generates vx/vy commands from then on
 */
static void task5_system_init(void)
{
    motor_driver_init();
    Encoder_Init();
    encoder_reset_all();
    speed_pid_reset_all();
    angle_pid_reset_state();
    motor_stop_all();

    if(imu_init_and_calibrate())
    {
        imu_reset_yaw();
    }

    odometry_init(ODOM_START_X_MM, ODOM_START_Y_MM);
    odometry_set_target_point(TASK5_TARGET_X_MM, TASK5_TARGET_Y_MM);

    g_task5_debug_a = 0;
    g_task5_debug_b = 0;
    g_task5_debug_c = TASK5_TARGET_X_MM;
    g_task5_debug_d = TASK5_TARGET_Y_MM;
}

/*
 * Run one 20 ms point-to-point control cycle.
 *
 * Important behavior:
 * - odometry_update_point_move_command() works in the global frame first
 * - it converts the desired translation back into body-frame vx/vy
 * - heading hold is handled separately through vz correction
 * - once the point-move state reports finish, translation commands become zero
 *   and the speed loops naturally bring the chassis to a stop
 */
static void task5_control_loop(void)
{
    int32_t vx_cmd_mmps;
    int32_t vy_cmd_mmps;
    int32_t vz_correction;
    int32_t vx_cmd_pulse;
    int32_t vy_cmd_pulse;
    int32_t current_speed;
    int32_t pwm_command;
    int32_t dx_mm;
    int32_t dy_mm;
    int32_t dominant_error_mm;

    encoder_read_data();
    odometry_update(TASK5_CONTROL_PERIOD_MS);

    vx_cmd_mmps = 0;
    vy_cmd_mmps = 0;
    (void)odometry_update_point_move_command(&vx_cmd_mmps, &vy_cmd_mmps);

    vz_correction = angle_pid_calc_output(TASK5_TARGET_YAW_CD,
                                          imu_get_yaw_cd(),
                                          TASK5_CONTROL_PERIOD_MS);
    dx_mm = odometry_get_target_dx_mm();
    dy_mm = odometry_get_target_dy_mm();
    dominant_error_mm = (abs(dx_mm) >= abs(dy_mm)) ? abs(dx_mm) : abs(dy_mm);

    /*
     * Near the target, reduce the yaw-correction authority.
     *
     * This prevents heading hold from fighting tiny translation corrections too
     * aggressively during the final convergence phase.
     */
    if(dominant_error_mm <= POINT_MOVE_NEAR_SLOWDOWN_MM)
    {
        /*
         * In the final convergence stage, ignore small heading errors and keep
         * yaw correction very soft. This prevents heading hold from constantly
         * kicking the chassis while translation is trying to settle.
         */
        if(abs(angle_pid_get_error()) <= TASK5_NEAR_YAW_DEADBAND_CD)
        {
            vz_correction = 0;
        }
        else
        {
            vz_correction = LIMIT_ABS(vz_correction, TASK5_NEAR_VZ_CMD_LIMIT);
        }
    }
    else if(dominant_error_mm <= TASK5_YAW_SOFTEN_MM)
    {
        /*
         * The user-observed jitter starts well before the final stop band.
         * Reduce heading-hold authority through the whole second half of the
         * move so Vz correction stops fighting the main translation command.
         */
        vz_correction = LIMIT_ABS(vz_correction, TASK5_MID_VZ_CMD_LIMIT);
    }
    else
    {
        vz_correction = LIMIT_ABS(vz_correction, TASK5_VZ_CMD_LIMIT);
    }

    vx_cmd_pulse = Mecanum_mmps_to_pulse_per_period(vx_cmd_mmps, TASK5_CONTROL_PERIOD_MS);
    vy_cmd_pulse = Mecanum_mmps_to_pulse_per_period(vy_cmd_mmps, TASK5_CONTROL_PERIOD_MS);

    Mecanum_inverse_kinematics(vx_cmd_pulse, vy_cmd_pulse, vz_correction);

    for(int i = 0; i < MOTOR_MAX; i++)
    {
        current_speed = get_encoder_data((MotorID)i);
        pwm_command = speed_pid_calc((MotorID)i,
                                     get_speed_target((MotorID)i),
                                     current_speed,
                                     TASK5_CONTROL_PERIOD_MS);
        motor_set_pwm((MotorID)i, pwm_command);
    }

    if(TASK5_DEBUG_MODE == TASK5_DEBUG_MODE_COMMAND)
    {
        /*
         * Command debug mode:
         * - a = vx_cmd_mmps
         * - b = vy_cmd_mmps
         * - c = vz_correction
         * - d = angle error in centi-degree
         *
         * This mode is used to locate whether final-stage jitter comes from:
         * - translation command oscillation
         * - yaw-hold correction fighting the translation
         */
        g_task5_debug_a = vx_cmd_mmps;
        g_task5_debug_b = vy_cmd_mmps;
        g_task5_debug_c = vz_correction;
        g_task5_debug_d = angle_pid_get_error();
    }
    else
    {
        g_task5_debug_a = odometry_get_x_mm();
        g_task5_debug_b = odometry_get_y_mm();
        g_task5_debug_c = odometry_get_target_dx_mm();
        g_task5_debug_d = odometry_get_target_dy_mm();
    }
}

/*
 * High-rate IMU loop reused from the verified yaw integration path.
 */
static void task5_imu_fast_loop(void)
{
    imu_deal_data_period(TASK5_IMU_FAST_PERIOD_MS);
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);
    debug_init();
    system_delay_ms(300);

    BlueTooth_Init();
    task5_system_init();

    pit_ms_init(PIT_CH_CONTROL, TASK5_CONTROL_PERIOD_MS);
    pit_ms_init(PIT_CH_IMU_FAST, TASK5_IMU_FAST_PERIOD_MS);
    interrupt_global_enable(0);

    while(1)
    {
        send_data(g_task5_debug_a,
                  g_task5_debug_b,
                  g_task5_debug_c,
                  g_task5_debug_d);
        system_delay_ms(TASK5_DEBUG_SEND_PERIOD_MS);
    }
}

void pit_handler(void)
{
    task5_control_loop();
}

void pit1_handler(void)
{
    task5_imu_fast_loop();
}
