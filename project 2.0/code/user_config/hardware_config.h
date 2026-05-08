#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

#include "zf_common_headfile.h"

/************************** Timer **************************/

#define PIT_CH_CONTROL                  (PIT_CH0)
#define PIT_CH_IMU_FAST                 (PIT_CH1)
#define PIT_CH                          (PIT_CH_CONTROL)


/************************ Hardware Pins ************************/

// Motor wiring:
// E1 -> C11, P1 -> C10
// E2 -> D3,  P2 -> D2
// E3 -> C9,  P3 -> C8
// E4 -> C7,  P4 -> C6

#define MOTOR1_DIR          (C10)
#define MOTOR1_PWM          (PWM2_MODULE2_CHB_C11)
#define MOTOR2_DIR          (D2)
#define MOTOR2_PWM          (PWM2_MODULE3_CHB_D3)
#define MOTOR3_DIR          (C7)
#define MOTOR3_PWM          (PWM2_MODULE0_CHA_C6)
#define MOTOR4_DIR          (C9)
#define MOTOR4_PWM          (PWM2_MODULE1_CHA_C8)

// Logic level that means "forward" for each logical wheel.
#define MOTOR1_FORWARD_LEVEL    (GPIO_HIGH)
#define MOTOR2_FORWARD_LEVEL    (GPIO_HIGH)
#define MOTOR3_FORWARD_LEVEL    (GPIO_LOW)
#define MOTOR4_FORWARD_LEVEL    (GPIO_LOW)

// Encoder wiring:
// ENCODER_1 -> C5/C25
// ENCODER_2 -> C3/C4
// ENCODER_3 -> C0/C1
// ENCODER_4 -> C2/C24

#define ENCODER_1                       (QTIMER2_ENCODER2)
#define ENCODER_1_LSB                   (QTIMER2_ENCODER2_CH1_C5)
#define ENCODER_1_DIR                   (QTIMER2_ENCODER2_CH2_C25)

#define ENCODER_2                       (QTIMER2_ENCODER1)
#define ENCODER_2_LSB                   (QTIMER2_ENCODER1_CH1_C3)
#define ENCODER_2_DIR                   (QTIMER2_ENCODER1_CH2_C4)

#define ENCODER_3                       (QTIMER1_ENCODER1)
#define ENCODER_3_LSB                   (QTIMER1_ENCODER1_CH1_C0)
#define ENCODER_3_DIR                   (QTIMER1_ENCODER1_CH2_C1)

#define ENCODER_4                       (QTIMER1_ENCODER2)
#define ENCODER_4_LSB                   (QTIMER1_ENCODER2_CH1_C2)
#define ENCODER_4_DIR                   (QTIMER1_ENCODER2_CH2_C24)

// After normalization, positive motor command should produce positive encoder delta.
#define ENCODER1_FORWARD_SIGN           (-1)
#define ENCODER2_FORWARD_SIGN           (1)
#define ENCODER3_FORWARD_SIGN           (-1)
#define ENCODER4_FORWARD_SIGN           (1)

// 1: print LF/RF/LB/RB encoder debug
// 0: keep original upper-layer telemetry
#define ENABLE_ENCODER_SIGN_DEBUG       (1)

// IMU963RA aliases
#define acc_x                       (imu963ra_acc_x)
#define acc_y                       (imu963ra_acc_y)
#define acc_z                       (imu963ra_acc_z)
#define gyro_x                      (imu963ra_gyro_x)
#define gyro_y                      (imu963ra_gyro_y)
#define gyro_z                      (imu963ra_gyro_z)
#define mag_x                       (imu963ra_mag_x)
#define mag_y                       (imu963ra_mag_y)
#define mag_z                       (imu963ra_mag_z)

#endif
