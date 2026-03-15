#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include "mecanum.h"


/************************ 整车控制结构体 ************************/
typedef struct {
    uint32 count_time;               // 全局计时器（ms）
    int32_t global_speed;              // 全局速度缩放系数
    bool enable_closed_loop;           // 是否启用闭环控制
    MoveDirection current_dir;       // 当前运动方向
    MoveState move_state;            // 当前运动状态
} Vehicle_Control_t;

/************************ 全局变量声明 ************************/
extern Vehicle_Control_t g_vehicle_ctrl;
extern int32_t g_adaptive_max_speed;   // 适配后的最大速度
extern int32_t g_current_speed;        // 当前运动速度

/************************ 函数声明 ************************/
void robot_control_init(void);                  // 机器人控制系统初始化
void robot_control_loop(void);                  // 机器人控制主循环（10ms调用一次）
void robot_emergency_stop(void);                // 紧急停止

#endif // ROBOT_CONTROL_H