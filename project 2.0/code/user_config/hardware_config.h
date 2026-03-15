#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

#include "zf_common_headfile.h"

/************************** 中断定义 **************************/

#define PIT_CH                          (PIT_CH0 )                  // 使用PIT_CH0中断


/************************ 硬件引脚定义 ************************/

// #define LED1                    (B9 )               //检测蓝牙初始化




// 电机接入
//      模块管脚            单片机管脚
//       E1                       C9
//       P1                       C8
//       E2                       C7
//       P2                       C6 
//       E3                       D3
//       P3                       D2
//       E4                       C11
//       P4                       C10

#define MOTOR1_DIR          (C9 )               // 左前电机方向
#define MOTOR1_PWM          (PWM2_MODULE1_CHA_C8)
#define MOTOR2_DIR          (C7 )               // 右前电机方向
#define MOTOR2_PWM          (PWM2_MODULE0_CHA_C6)
#define MOTOR3_DIR          (D2 )               // 左后电机方向
#define MOTOR3_PWM          (PWM2_MODULE3_CHB_D3)
#define MOTOR4_DIR          (C10 )              // 右后电机方向
#define MOTOR4_PWM          (PWM2_MODULE2_CHB_C11)

// 方向编码器连接
//      模块管脚            单片机管脚
//      ENCODER_1_LSB             C0
//      ENCODER_1_DIR             C1
//      GND                 GND 
//      ENCODER_2_LSB             C2
//      ENCODER_2_DIR             C24
//      GND                 GND
//      ENCODER_3_LSB             C3
//      ENCODER_3_DIR             C4
//      GND                 GND                                                                                                   
//      ENCODER_4_LSB             C5
//      ENCODER_4_DIR             C25
//      GND                 GND

#define ENCODER_1                       (QTIMER1_ENCODER1)           // 左前
#define ENCODER_1_LSB                   (QTIMER1_ENCODER1_CH1_C0)
#define ENCODER_1_DIR                   (QTIMER1_ENCODER1_CH2_C1)
                                        
#define ENCODER_2                       (QTIMER1_ENCODER2)           // 右前
#define ENCODER_2_LSB                   (QTIMER1_ENCODER2_CH1_C2)
#define ENCODER_2_DIR                   (QTIMER1_ENCODER2_CH2_C24)
                                        
#define ENCODER_3                       (QTIMER2_ENCODER1)           // 左后
#define ENCODER_3_LSB                   (QTIMER2_ENCODER1_CH1_C3)
#define ENCODER_3_DIR                   (QTIMER2_ENCODER1_CH2_C4)
                                        
#define ENCODER_4                       (QTIMER2_ENCODER2)           // 右后
#define ENCODER_4_LSB                   (QTIMER2_ENCODER2_CH1_C5)
#define ENCODER_4_DIR                   (QTIMER2_ENCODER2_CH2_C25)

// 接入 IMU963RA
//      模块管脚            单片机管脚
//      SCL/SPC              C23 
//      SDA/DSI              C22
//      SA0/SDO              C21
//      CS                   C20
//      GND                 GND
//      3V3                 3V3

#define acc_x                       (imu963ra_acc_x)           // 加速度
#define acc_y                       (imu963ra_acc_y)           
#define acc_z                       (imu963ra_acc_z)
#define gyro_x                      (imu963ra_gyro_x)          // 角速度
#define gyro_y                      (imu963ra_gyro_y)
#define gyro_z                      (imu963ra_gyro_z)
#define mag_x                       (imu963ra_mag_x)           // 地磁计
#define mag_y                       (imu963ra_mag_y)
#define mag_z                       (imu963ra_mag_z)

// 接入 BLE6A20蓝牙模块
//      模块管脚            单片机管脚
//      RX                  D16
//      TX                  D17
//      RTS                 D26
//      CMD                 悬空
//      NC                  悬空
//      GND                 GND
//      3V3                 3V3








#endif 
