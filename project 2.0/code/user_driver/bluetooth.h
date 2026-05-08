#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include "zf_common_headfile.h"
#include "hardware_config.h"
#include "robot_param.h"
#include "vision_config.h"

void BlueTooth_Init(void);               // 蓝牙初始化
void send_wheel_data(int16_t Encoder0, int32_t speed0, int32_t position0, 
                     int16_t Encoder1, int32_t speed1, int32_t position1, 
                     int16_t Encoder2, int32_t speed2, int32_t position2, 
                     int16_t Encoder3, int32_t speed3, int32_t position3);                
                                        // 发送各个轮子
void send_txt_data(int State, int32_t Pos_X, int32_t Pos_Y,
               int32_t Angle, int32_t Speed, 
               int16_t Encoder0, int16_t Encoder1, int16_t Encoder2, int16_t Encoder3);
                                        // 发送文档数据

void send_imu_data(void);

void send_data(int32_t a, int32_t b, int32_t c, int32_t d);
void send_openart_map_debug(const char map[OPENART_MAP_HEIGHT][OPENART_MAP_WIDTH],
                            int32_t player_x,
                            int32_t player_y,
                            int32_t valid_count,
                            int32_t invalid_count,
                            int32_t parse_error_count);









#endif
