#ifndef ROBOT_PARAM_H
#define ROBOT_PARAM_H

// 单位：mm、ms

/************************ 全局控制参数 ************************/
#define PID_CONTROL_PERIOD       (10)           // PID控制周期(ms), 10ms=100Hz
#define PWM_FREQUENCY            (20000)        // PWM频率(Hz), 20kHz
#define PWM_MAX_VALUE            (1000)         // PWM最大数值（对应100%占空比）
#define DUTY_MAX                 (100U)         // 最大占空比（%）

#define PI                       (314)          // 圆周率
/************************ 车体尺寸 ************************/
#define ROBOT_WIDTH              ()             // 车体宽度
#define ROBOT_LENGTH             (282)          // 车体长度

#define WHEEL_BASE_X             ()             // 麦克纳姆轮x轴轴距(左右轮)
#define WHEEL_BASE_Y             ()             // 麦克纳姆轮y轴轴距(前后轮)
#define WHEEL_BASE               (WHEEL_BASE_X / 2)// 旋转轴距
/************************ 编码器 ************************/
#define PULSE_PER_MM             (7.7f)         // 每毫米对应编码器脉冲数

#define ENCODER_PULSE_MAX        (32767)        // 编码器最大计数值（16位）
#define ENCODER_PULSE_MIN        (-32767)       // 编码器最小计数值（16位）
#define SPEED_ZERO_OFFSET        (20)           // 速度零漂阈值(脉冲)
#define POS_ZERO_OFFSET          (20)           // 位置零漂阈值(脉冲)
#define ROTATE_ZERO_OFFSET       (20)           // 旋转零漂阈值(脉冲)
/************************ 速度 ************************/
#define SPEED_PID_KP             (2)            // 比例系数
#define SPEED_PID_KI             (0.1f)         // 积分系数
#define SPEED_PID_KD             (0.5f)         // 微分系数
#define SPEED_PID_I_LIMIT        (100.0f)       // 积分限幅，避免积分饱和

#define SPEED_FILTER_FACTOR      (9)            // 速度滤波系数（0~9，越大滤波越强）
#define SPEED_PWM_MAX_OUTPUT     (800)          // 最大pwm输出
#define SPEED_PWM_MIN_OUTPUT     (-800)         // 最小输出（反向）
#define SPEED_MAX_ACCEL          (50)           // 最大加速度
#define SPEED_DEAD_ZONE          (10)           // 速度死区
/************************ 位置 ************************/
#define POSITION_PID_KP          (5.0f)
#define POSITION_PID_KI          (0.05f)
#define POSITION_PID_KD          (1.0f)
#define POSITION_PID_I_LIMIT     (100.0f)       // 积分限幅

#define POSITION_PWM_MAX_OUTPUT  (500)          // 最大pwm输出（位置限制）
#define DECEL_DISTANCE           (200)          // 减速触发距离
#define POSITION_DEAD_ZONE       (10)           // 位置死区
/************************ 旋转速度 *********************/
#define ROTATE_PID_KP            ()
#define ROTATE_PID_KI            ()
#define ROTATE_PID_KD            ()
#define ROTATE_PID_I_LIMIT       (100.0f)       // 积分限幅

#define ROTATE_PWM_MAX_OUTPUT    (500)          // 最大旋转pwm输出
#define ROTATE_PWM_MIN_OUTPUT    (-500)
#define ROTATE_MAX_ACCEL         (50)           // 最大加速度
#define ROTATE_DEAD_ZONE         ()             // 旋转死区
/************************ 角度 ************************/
#define ANGLE_PID_KP             ()
#define ANGLE_PID_KI             ()
#define ANGLE_PID_KD             ()
#define ANGLE_PID_I_LIMIT        (100.0f)       // 积分限幅
 
#define ANGLE_PWM_MAX_OUTPUT     (500)          // 最大pwm输出（角度限制）
#define ANGLE_DECEL_DISTANCE     (200)          // 减速触发角度
#define ANGLE_DEAD_ZONE          ()             // 角度死区
/************************ 地图 ************************/
#define MAP_WIDTH                ()             // 地图宽度
#define MAP_LENGTH               ()             // 地图长度 



/************************ 枚举类型定义 ************************/
// 电机编号
typedef enum {
    MOTOR_LF   = 0,          // 左前电机
    MOTOR_RF   = 1,          // 右前电机
    MOTOR_LB   = 2,          // 左后电机
    MOTOR_RB   = 3,          // 右后电机
    MOTOR_MAX  = 4           // 电机总数
} MotorID;

// 麦克纳姆轮运动方向
typedef enum {
    MOVE_STOP       = 0,     // 停止
    MOVE_FORWARD    = 1,     // 前进
    MOVE_BACKWARD   = 2,     // 后退
    MOVE_LEFT       = 3,     // 左移
    MOVE_RIGHT      = 4,     // 右移
    MOVE_ROTATE_CW  = 5,     // 顺时针旋转
    MOVE_ROTATE_CCW = 6      // 逆时针旋转
} MoveDirection;

// 运动状态
typedef enum {
    MOVE_STATE_IDLE    = 0,  // 空闲
    MOVE_STATE_RUNNING = 1,  // 运动中
    MOVE_STATE_FINISH  = 2   // 完成
} MoveState;

/************************ 整车控制结构体 ************************/
typedef struct {
    int32_t speed_target;      // 目标速度（mm/s）
    int32_t speed_now;         // 当前速度（mm/s）
    int32_t angle_target;      // 目标角度
    int32_t angle_now;         // 当前角度(初始为90)      
    int32_t x_target;          // 目标位置
    int32_t y_target;
    int32_t x_now;             // 当前位置
    int32_t y_now;
} Car_Control_t;

/************************ 全局变量声明 ************************/
extern Car_Control_t g_car;













#endif
