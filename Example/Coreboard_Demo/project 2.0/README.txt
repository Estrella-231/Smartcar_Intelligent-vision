本程序为2.0版

将车的移动更新为在3.2*2.4m的二维坐标，以车初始的位置为坐标原点，上机位输入（x，y）的目标位置的指令

pwm为800时，speed为多少

pwm：20KHz








硬件分配：

     LED引脚分配
            LED1                     B9 

     电机驱动引脚分配
           模块管脚           主控管脚
           E1                       C9
           P1                       C8
           GND                 GND   
           E2                       C7
           P2                       C6
           GND                 GND   
           E3                       D3
           P3                       D2
           GND                 GND   
           E4                       C11
           P4                       C10
           GND                 GND
           接线端子 +          电池正极
           接线端子 -          电池负极

     方向编码器引脚分配
           模块管脚            单片机管脚
           ENCODER_1_LSB             C0
           ENCODER_1_DIR             C1

           ENCODER_2_LSB             C2
           ENCODER_2_DIR             C24

           ENCODER_3_LSB             C3
           ENCODER_3_DIR             C4
                                                                                                 
           ENCODER_4_LSB             C5
           ENCODER_4_DIR             C25



