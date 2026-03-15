#include "zf_common_headfile.h"
#include "gpio.h"
#include "robot_config.h"
#include "motor_driver.h"
#include "encoder.h"
#include "robot_control.h"
#include "bluetooth.h"
#include "gyroscope.h"



//--------------------------------------------------------------------------------------------------------------//
// 函数名称       main
// 函数说明       主函数，系统初始化和主循环
// 输入参数       void
// 返回参数       void
//--------------------------------------------------------------------------------------------------------------//
int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);  // 初始化单片机时钟，时钟频率为600MHz
    debug_init();                   // 调试串口初始化
    system_delay_ms(300);

	BlueTooth_Init();
    robot_control_init();

    pit_ms_init(PIT_CH, 10);         // PIT中断初始化，触发10ms一次

    interrupt_global_enable(0);     // 使能全局中断
    
    // 主循环
    while(1)
    {
   //     key_scanner();              // 按键扫描
  //      system_delay_ms(10);        // 缩短主循环延时（10ms）

  //      send_status();        // 发送车的状态


    }
}

//--------------------------------------------------------------------------------------------------------------//
// 函数名称       pit_handler
// 函数说明       PIT中断服务函数，10ms一次，更新编码器数据+计算速度/位置
// 输入参数       void
// 返回参数       void
//--------------------------------------------------------------------------------------------------------------//
void pit_handler(void)
{
    // 读取编码器计数器
    encoder_read_data();
    // 读取陀螺仪测量值
    gyro_read_data();
    // 执行车控制循环
    robot_control_loop();
}
