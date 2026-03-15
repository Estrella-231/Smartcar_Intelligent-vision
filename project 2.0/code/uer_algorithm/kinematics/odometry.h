#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "motor_driver.h"
#include "robot_param.h"
#include "math_tools.h"


void Odometry_entirety2part(int32_t x, int32_t y);  // 整车目标坐标 -> 4电机目标位置

void update_coord(int32_t x, int32_t y);            // 更新坐标

void Odometry_rotate(int32_t angle);                // 目标角度 -> 电机









#endif
