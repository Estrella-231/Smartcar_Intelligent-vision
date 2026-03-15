#ifndef MECANUM_H
#define MECANUM_H

#include "motor_driver.h"
#include "robot_param.h"








void Mecanum_entirety2part(MoveDirection Car_direction, int32_t Car_target_speed);   // 整车目标速度 -> 4电机目标速度

void Mecanum_part2entirety(MoveDirection Car_direction);                             // 运动学正解：4电机速度 → 车体合成速度

void Calc_SpeedNow(MotorID motor_id);                                                // 计算当前速度






#endif 