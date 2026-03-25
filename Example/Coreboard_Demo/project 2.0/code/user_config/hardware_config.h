#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

#include "zf_common_headfile.h"

/************************** 中断定义 **************************/

#define PIT_CH_CONTROL                  (PIT_CH0)                   // 20ms 控制/调试节拍
#define PIT_CH_IMU_FAST                 (PIT_CH1)                   // 5ms IMU 高频积分节拍
#define PIT_CH                          (PIT_CH_CONTROL)            // 兼容旧代码中的 PIT_CH 宏


/************************ 硬件引脚定义 ************************/

// #define LED1                    (B9 )               //检测蓝牙初始化




// 电机接入
//      模块管脚            单片机管脚
//       E1                       C11
//       P1                       C10
//       E2                       D3
//       P2                       D2
//       E3                       C9
//       P3                       C8
//       E4                       C7
//       P4                       C6 

#define MOTOR1_DIR          (C10 )              // 左前电机方向
#define MOTOR1_PWM          (PWM2_MODULE2_CHB_C11)
#define MOTOR2_DIR          (D2 )               // 右前电机方向
#define MOTOR2_PWM          (PWM2_MODULE3_CHB_D3)
#define MOTOR3_DIR          (C9 )               // 左后电机方向
#define MOTOR3_PWM          (PWM2_MODULE1_CHA_C8)
#define MOTOR4_DIR          (C7 )               // 右后电机方向
#define MOTOR4_PWM          (PWM2_MODULE0_CHA_C6)

// Per-wheel forward polarity. Adjust after the first single-wheel direction check.
#define MOTOR1_FORWARD_LEVEL    (GPIO_HIGH)
#define MOTOR2_FORWARD_LEVEL    (GPIO_HIGH)
#define MOTOR3_FORWARD_LEVEL    (GPIO_LOW)
#define MOTOR4_FORWARD_LEVEL    (GPIO_LOW)

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

#define ENCODER_1                       (QTIMER2_ENCODER2)           // 左前
#define ENCODER_1_LSB                   (QTIMER2_ENCODER2_CH1_C5)
#define ENCODER_1_DIR                   (QTIMER2_ENCODER2_CH2_C25)

#define ENCODER_2                       (QTIMER2_ENCODER1)           // 右前
#define ENCODER_2_LSB                   (QTIMER2_ENCODER1_CH1_C3)
#define ENCODER_2_DIR                   (QTIMER2_ENCODER1_CH2_C4)

#define ENCODER_3                       (QTIMER1_ENCODER1)           // 左后
#define ENCODER_3_LSB                   (QTIMER1_ENCODER1_CH1_C0)
#define ENCODER_3_DIR                   (QTIMER1_ENCODER1_CH2_C1)
                                        
#define ENCODER_4                       (QTIMER1_ENCODER2)           // 右后
#define ENCODER_4_LSB                   (QTIMER1_ENCODER2_CH1_C2)
#define ENCODER_4_DIR                   (QTIMER1_ENCODER2_CH2_C24)

// Encoder sign normalization.
// After this normalization, a positive motor command must produce a positive encoder increment.
// Keep the current project behavior as default, then only flip the wheel that bench test proves wrong.
#define ENCODER1_FORWARD_SIGN           (-1)
#define ENCODER2_FORWARD_SIGN           (1)
#define ENCODER3_FORWARD_SIGN           (-1)
#define ENCODER4_FORWARD_SIGN           (1)

// Task 2 debug switch.
// 1: Bluetooth prints encoder increments in the order LF, RF, LB, RB.
// 0: Keep the original upper-layer debug telemetry.
#define ENABLE_ENCODER_SIGN_DEBUG       (1)
                                        
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
