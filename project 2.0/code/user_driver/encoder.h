#ifndef ENCODER_H
#define ENCODER_H

#include "zf_common_headfile.h"
#include "motor_driver.h"


extern volatile int16_t encoder_data[MOTOR_MAX]; // 编码器增量值

// 函数声明
void Encoder_Init(void);                           // 编码器初始化
void encoder_read_data(void);                      // 读取编码器数据（并更新每个轮子当前位置）
int16_t get_encoder_data(MotorID motor_id);        // 获取编码器数据






#endif 
