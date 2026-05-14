// 机器人全局参数配置头文件
// 集中定义控制周期、PID参数、底盘尺寸、地图网格与执行器阈值等常量

#ifndef ROBOT_PARAM_H
#define ROBOT_PARAM_H

// Unit: mm, ms, degree, centi-degree, encoder pulse.

/************************ Global control ************************/
#define PID_CONTROL_PERIOD       (10)           // PID control period in ms
#define PWM_FREQUENCY            (20000)        // PWM frequency in Hz
#define PWM_MAX_VALUE            (1000)         // 100 percent duty command
#define DUTY_MAX                 (100U)         // Max duty percentage

/************************ Chassis size ************************/
#define ROBOT_WIDTH              (500)
#define ROBOT_LENGTH             (282)

#define WHEEL_BASE_X             (500)          // Left-right wheel spacing
#define WHEEL_BASE_Y             (500)          // Front-back wheel spacing
#define WHEEL_BASE               (WHEEL_BASE_X / 2)

/************************ Encoder ************************/
// Encoder pulses per millimeter of actual chassis travel.
//
// This value directly affects:
// - mm/s -> wheel target pulses per control period
// - wheel pulses -> odometry x/y distance
// - point-to-point move distance accuracy
//
// Current value is provisional. Recalibrate it on the real chassis:
// PULSE_PER_MM = average_encoder_pulses / measured_travel_mm.
#define PULSE_PER_MM             (9.407f)

// Raw int16 encoder-count boundaries for one hardware read. These are not the
// encoder resolution and should not be used to wrap software-accumulated pos_now.
#define ENCODER_PULSE_MAX        (32767)
#define ENCODER_PULSE_MIN        (-32767)
#define SPEED_ZERO_OFFSET        (3)
#define POS_ZERO_OFFSET          (20)
#define ROTATE_ZERO_OFFSET       (20)

/************************ Speed loop ************************/
#define SPEED_PID_I_LIMIT        (30.0f)

#define SPEED_FILTER_FACTOR      (9)
#define SPEED_D_FILTER_ALPHA     (0.7f)
#define SPEED_PWM_MAX_OUTPUT     (150)
#define SPEED_PWM_MIN_OUTPUT     (-150)
#define SPEED_MAX_ACCEL          (50)
#define SPEED_ERR_DEAD_ZONE      (5)
#define SPEED_START_KICK_PWM     (20)

/************************ Position loop ************************/
#define POSITION_PID_KP          (5.0f)
#define POSITION_PID_KI          (0.05f)
#define POSITION_PID_KD          (1.0f)
#define POSITION_PID_I_LIMIT     (100.0f)

#define POSITION_PWM_MAX_OUTPUT  (500)
#define DECEL_DISTANCE           (200)
#define POSITION_DEAD_ZONE       (10)

/************************ Rotation-rate loop ************************/
#define ROTATE_PID_KP            (0.465)
#define ROTATE_PID_KI            (0.05)
#define ROTATE_PID_KD            (1)
#define ROTATE_PID_I_LIMIT       (12000)

#define ROTATE_MAX_OUTPUT        (1000)
#define ROTATE_MAX_ACCEL         (600)
#define ROTATE_DEAD_ZONE         (50)

/************************ Angle loop ************************/
#define ANGLE_PID_I_LIMIT        (80000)

#define ANGLE_MAX_OUTPUT         (6000)
#define ANGLE_DECEL_DISTANCE     (200)
#define ANGLE_DEAD_ZONE          (50)

/************************ Map ************************/
#define MAP_WIDTH                (500)
#define MAP_LENGTH               (500)

/************************ Field / grid execution ************************/
// Competition field size: 3.2 m x 2.4 m, mapped to a 16 x 12 virtual grid.
#define FIELD_WIDTH_MM                   (3200)
#define FIELD_HEIGHT_MM                  (2400)
#define FIELD_GRID_COLS                  (16)
#define FIELD_GRID_ROWS                  (12)
#define FIELD_GRID_CELL_MM               (200)

// Motion execution scheduler timing and heading policy.
#define EXEC_CONTROL_PERIOD_MS           (20)
#define EXEC_YAW_TARGET_CD               (0)
#define EXEC_YAW_VZ_LIMIT                (20)

// When the chassis is already close enough to a segment target, let the
// scheduler accept that segment and move on instead of waiting for the point
// controller to fight over the last few centimeters.
#define EXEC_SEGMENT_ACCEPT_TOL_MM       (30)

// Because the team does not yet have a dedicated start trigger fixture, keep a
// software auto-start path enabled for bench testing. A future start button or
// referee signal should call motion_exec_request_start() instead of relying on
// this test macro.
#define EXEC_AUTO_START_FOR_TEST         (1)

// Motion-segment wait handling.
#define EXEC_WAIT_SEGMENT_DEFAULT_MS     (100)

// Push segments reuse the same point-move controller but with lower speed caps.
#define POINT_MOVE_PUSH_SPEED_SCALE_PCT       (70)
#define POINT_MOVE_PUSH_MIN_EFFECTIVE_MMPS    (180)

/************************ Runtime debug modes ************************/
#define SMARTCAR_MODE_WHEEL_SIGN_CHECK        (0)
#define SMARTCAR_MODE_MANUAL_RECT_LAP         (1)
#define SMARTCAR_MODE_BFS_FIXED_MAP           (2)
#define SMARTCAR_MODE_OPENART_BFS             (3)
#define SMARTCAR_MODE_PULSE_CAL_3200          (4)

// Fixed-map bring-up path: use the built-in stage1 ASCII map and execute BFS output.
#define SMARTCAR_RUNTIME_MODE                 (SMARTCAR_MODE_BFS_FIXED_MAP)

// Manual-lap debug mode uses tighter completion thresholds so the car does not
// stop one half-cell early during calibration runs.
#define DEBUG_EXEC_SEGMENT_ACCEPT_TOL_MM      (20)
#define DEBUG_POINT_MOVE_FINISH_TOL_MM        (20)
#define DEBUG_TELEMETRY_PAGE_PERIOD_MS        (1000U)

// If the chassis is close to a grid target but cannot settle the final few cm,
// accept the segment after a short dwell and snap odometry to the target cell.
#define EXEC_SEGMENT_NEAR_ACCEPT_TOL_MM       (45)
#define EXEC_SEGMENT_NEAR_ACCEPT_MS           (500)

// If the chassis is close to the target but odometry error stops improving,
// accept the segment to avoid waiting forever in the low-speed sticky region.
#define EXEC_SEGMENT_STICKY_ACCEPT_TOL_MM     (80)
#define EXEC_SEGMENT_STICKY_ACCEPT_MS         (700)
#define EXEC_SEGMENT_STICKY_PROGRESS_MM       (5)

/************************ Round-2 odometry ************************/
// Scale factors used after field calibration. Leave at 1.0 until measured.
#define ODOM_SCALE_X                     (0.940f)
#define ODOM_SCALE_Y                     (0.977f)

// Default start cell on the virtual 16 x 12 map.
//
// Team convention for the execution stage:
// - the car starts from grid (1, 6)
// - odometry should therefore boot at the physical center of that cell
//
// Physical center conversion:
// x_mm = grid_x * 200 + 100
// y_mm = grid_y * 200 + 100
#define ODOM_START_GRID_X                (1)
#define ODOM_START_GRID_Y                (6)
#define ODOM_START_X_MM                  (ODOM_START_GRID_X * FIELD_GRID_CELL_MM + FIELD_GRID_CELL_MM / 2)
#define ODOM_START_Y_MM                  (ODOM_START_GRID_Y * FIELD_GRID_CELL_MM + FIELD_GRID_CELL_MM / 2)

// Small body/global velocity values are treated as zero to suppress noise.
#define ODOM_BODY_SPEED_DEADBAND_MMPS    (15)
#define ODOM_GLOBAL_SPEED_DEADBAND_MMPS  (10)

// Limit single-cycle velocity jumps caused by encoder spikes or slip.
#define ODOM_MAX_VEL_STEP_MMPS           (120)

// Slip heuristic thresholds, based on wheel target/feedback mismatch.
// The weight now falls continuously between SOFT and HARD. Recovery is filtered
// so odometry trust returns gradually after a suspicious traction event.
#define ODOM_TRACK_ERROR_SOFT_PULSE      (40)
#define ODOM_TRACK_ERROR_HARD_PULSE      (80)
#define ODOM_SOFT_WEIGHT_PCT             (50)
#define ODOM_SLIP_WEIGHT_RECOVER_STEP_PCT (5)

// Static gate: suppress raw-delta integration when the chassis is stationary.
// Only skip integration when both the wheel commands and measured deltas are tiny.
// If the chassis is actively moving, keep all small deltas for low-speed creep.
#define ODOM_RAW_DELTA_DEADBAND_PULSE      (1)
#define ODOM_RAW_BODY_DEADBAND_UM          (500)
#define ODOM_MOTION_TARGET_DEADBAND_PULSE  (2)

/************************ Motion saturation guard ************************/
// Unified body-command scaling applied before mecanum inverse kinematics.
#define MOTION_CMD_SCALE_ENABLE               (1)
#define MOTION_CMD_SCALE_MIN_PCT              (60)
#define MOTION_CMD_SCALE_PWM_SAT_PCT          (90)
#define MOTION_CMD_SCALE_TRACK_ERROR_PULSE    (40)
#define MOTION_CMD_SCALE_BAD_CONFIRM_CYCLES   (3)
#define MOTION_CMD_SCALE_RECOVER_CYCLES       (10)
#define MOTION_CMD_SCALE_DOWN_STEP_PCT        (5)
#define MOTION_CMD_SCALE_UP_STEP_PCT          (2)

// Reserved for a later chassis-speed outer loop. Keep disabled until the
// odometry scale and saturation guard are validated on hardware.
#define BODY_SPEED_LOOP_ENABLE                (0)

/************************ Motor Stall Protection ************************/
// Detects "high PWM + low speed" condition to prevent motor burnout.
// When PWM output exceeds threshold while encoder speed remains below threshold
// for a confirmation time, the motor is stopped and integral is cleared.
#define STALL_PROTECTION_ENABLE           (1)
#define STALL_PWM_THRESHOLD_PCT          (50)    // PWM saturation threshold (50%)
#define STALL_SPEED_THRESHOLD_PULSE      (15)    // Speed threshold (pulses/period)
#define STALL_CONFIRM_TIME_MS            (500)   // Stall confirmation time
#define STALL_RECOVERY_DELAY_MS          (2000)  // Recovery delay time
#define STALL_STOP_ALL_MOTORS            (1)     // Stop all motors on stall

/************************ Round-2 point move ************************/
// Global-frame axis P controller. Output unit is mm/s.
#define POINT_MOVE_KP                    (0.5f)

// Upper limit of the point-move linear speed command.
#define POINT_MOVE_MAX_SPEED_MMPS        (220)

// When a move axis is still far from the target, keep the command above the
// low-speed dead region of the current chassis.
#define POINT_MOVE_MIN_EFFECTIVE_MMPS    (80)

// Once the dominant axis enters the near-target zone, switch to a softer
// convergence mode so the chassis can settle instead of weaving.
#define POINT_MOVE_NEAR_SLOWDOWN_MM      (160)
#define POINT_MOVE_NEAR_MAX_SPEED_MMPS   (90)

// If one axis error is below this threshold, that axis command becomes zero.
#define POINT_MOVE_AXIS_STOP_TOL_MM      (25)

// Full move completion is judged with a slightly looser tolerance than the
// per-axis command deadband. This avoids endless small correction attempts when
// the chassis is already close enough for the current hardware stage.
#define POINT_MOVE_FINISH_TOL_MM         (70)

// If one axis is much smaller than the dominant move axis, suppress that small
// cross-axis correction until the dominant axis is nearly settled.
#define POINT_MOVE_CROSS_AXIS_TOL_MM     (60)

// Limit how much the planner can change one axis command per control cycle.
// This directly reduces left-right twitch near the target.
#define POINT_MOVE_CMD_STEP_MMPS         (25)

// Full point move is finished only after both axes stay inside tolerance for
// several control cycles in a row.
#define POINT_MOVE_STABLE_COUNT          (2)

/************************ Enum types ************************/
typedef enum {
    MOTOR_LF   = 0,
    MOTOR_RF   = 1,
    MOTOR_LB   = 2,
    MOTOR_RB   = 3,
    MOTOR_MAX  = 4
} MotorID;

typedef enum {
    MOVE_STOP       = 0,
    MOVE_FORWARD    = 1,
    MOVE_BACKWARD   = 2,
    MOVE_LEFT       = 3,
    MOVE_RIGHT      = 4,
    MOVE_ROTATE_CW  = 5,
    MOVE_ROTATE_CCW = 6
} MoveDirection;

typedef enum {
    MOVE_STATE_IDLE    = 0,
    MOVE_STATE_RUNNING = 1,
    MOVE_STATE_FINISH  = 2
} MoveState;

#endif
