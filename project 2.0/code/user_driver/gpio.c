#include "gpio.h"


void GPIO_Init(void)
{
	// 电机方向 初始化为输出 默认上拉输出高
	gpio_init(MOTOR1_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
	gpio_init(MOTOR2_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
	gpio_init(MOTOR3_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
	gpio_init(MOTOR4_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);

	// LED 初始化为输出 默认上拉输出高
//	gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    

	
}

