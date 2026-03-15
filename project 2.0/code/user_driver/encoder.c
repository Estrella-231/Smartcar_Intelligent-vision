#include "encoder.h"

volatile int16_t encoder_data[MOTOR_MAX] = {0};


/* @brief 编码器初始化 */
void Encoder_Init(void)
{
	encoder_dir_init(ENCODER_1, ENCODER_1_LSB, ENCODER_1_DIR);                  // 
    encoder_dir_init(ENCODER_2, ENCODER_2_LSB, ENCODER_2_DIR);                  
    encoder_dir_init(ENCODER_3, ENCODER_3_LSB, ENCODER_3_DIR);                  
    encoder_dir_init(ENCODER_4, ENCODER_4_LSB, ENCODER_4_DIR); 
}

/**
 * @brief 读取编码器数据（并更新每个轮子当前位置）
 */
void encoder_read_data(void)
{
    // 左前电机
    encoder_data[MOTOR_LF] = encoder_get_count(ENCODER_1);
    encoder_clear_count(ENCODER_1);
    encoder_data[MOTOR_LF] = -encoder_data[MOTOR_LF];
    encoder_data[MOTOR_LF] = abs(encoder_data[MOTOR_LF]) <= 20 ? 0 : encoder_data[MOTOR_LF];
    
    // 右前电机
    encoder_data[MOTOR_RF] = encoder_get_count(ENCODER_2);
    encoder_clear_count(ENCODER_2);
    encoder_data[MOTOR_RF] = -encoder_data[MOTOR_RF];
    encoder_data[MOTOR_RF] = abs(encoder_data[MOTOR_RF]) <= 20 ? 0 : encoder_data[MOTOR_RF];
    
    // 左后电机
    encoder_data[MOTOR_LB] = encoder_get_count(ENCODER_3);
    encoder_clear_count(ENCODER_3);
    encoder_data[MOTOR_LB] = abs(encoder_data[MOTOR_LB]) <= 20 ? 0 : encoder_data[MOTOR_LB];
    
    // 右后电机
    encoder_data[MOTOR_RB] = encoder_get_count(ENCODER_4);
    encoder_clear_count(ENCODER_4);
    encoder_data[MOTOR_RB] = abs(encoder_data[MOTOR_RB]) <= 20 ? 0 : encoder_data[MOTOR_RB];
    
    // 更新电机位置
//    for(int i = 0; i < MOTOR_MAX; i++)
//    {
//        change_pos_now(i, get_pos_now(i) + encoder_data[i]);
//    }
}

/* @brief 获取编码器数据 */
 int16_t get_encoder_data(MotorID motor_id)
 {
     return encoder_data[motor_id];
 }