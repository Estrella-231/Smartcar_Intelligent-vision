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
#define PULSE_PER_MM             (7.7f)         // Encoder pulse count per mm

#define ENCODER_PULSE_MAX        (32767)
#define ENCODER_PULSE_MIN        (-32767)
#define SPEED_ZERO_OFFSET        (20)
#define POS_ZERO_OFFSET          (20)
#define ROTATE_ZERO_OFFSET       (20)

/************************ Speed loop ************************/
#define SPEED_PID_KP             (2)
#define SPEED_PID_KI             (0.1f)
#define SPEED_PID_KD             (0.5f)
#define SPEED_PID_I_LIMIT        (100.0f)

#define SPEED_FILTER_FACTOR      (9)
#define SPEED_PWM_MAX_OUTPUT     (800)
#define SPEED_PWM_MIN_OUTPUT     (-800)
#define SPEED_MAX_ACCEL          (50)
#define SPEED_DEAD_ZONE          (10)

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
#define ANGLE_PID_KP             (0.5)
#define ANGLE_PID_KI             (0.02)
#define ANGLE_PID_KD             (0.1)
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
#define EXEC_YAW_VZ_LIMIT                (70)

// When the chassis is already close enough to a segment target, let the
// scheduler accept that segment and move on instead of waiting for the point
// controller to fight over the last few centimeters.
#define EXEC_SEGMENT_ACCEPT_TOL_MM       (100)

// Because the team does not yet have a dedicated start trigger fixture, keep a
// software auto-start path enabled for bench testing. A future start button or
// referee signal should call motion_exec_request_start() instead of relying on
// this test macro.
#define EXEC_AUTO_START_FOR_TEST         (1)

// Motion-segment wait handling.
#define EXEC_WAIT_SEGMENT_DEFAULT_MS     (300)

// Push segments reuse the same point-move controller but with lower speed caps.
#define POINT_MOVE_PUSH_SPEED_SCALE_PCT       (70)
#define POINT_MOVE_PUSH_MIN_EFFECTIVE_MMPS    (180)

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
#define ODOM_TRACK_ERROR_SOFT_PULSE      (40)
#define ODOM_TRACK_ERROR_HARD_PULSE      (80)
#define ODOM_SOFT_WEIGHT_PCT             (50)

/************************ Round-2 point move ************************/
// Global-frame axis P controller. Output unit is mm/s.
#define POINT_MOVE_KP                    (1.0f)

// Upper limit of the point-move linear speed command.
#define POINT_MOVE_MAX_SPEED_MMPS        (600)

// When a move axis is still far from the target, keep the command above the
// low-speed dead region of the current chassis.
#define POINT_MOVE_MIN_EFFECTIVE_MMPS    (280)

// Once the dominant axis enters the near-target zone, switch to a softer
// convergence mode so the chassis can settle instead of weaving.
#define POINT_MOVE_NEAR_SLOWDOWN_MM      (160)
#define POINT_MOVE_NEAR_MAX_SPEED_MMPS   (220)

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
#define POINT_MOVE_CMD_STEP_MMPS         (60)

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
