#include "gyroscope.h"


/* 读取陀螺仪数据 */
void gyro_read_data(void)
{
    imu963ra_get_acc();
    imu963ra_get_gyro();
    imu963ra_get_mag();
}



