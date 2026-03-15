# RT1064 zf_device 外设驱动库学习笔记

> 基于逐飞科技 RT1064 开源库 v3.9.x

## 1. 目录结构概述

`zf_device` 目录位于 `libraries/` 目录下，包含所有外设驱动的头文件和源文件。

```
zf_device/
├── zf_device_xxx.h          # 外设头文件（接口定义）
├── zf_device_xxx.c          # 外设源文件（实现代码）
├── zf_device_type.h/c       # 类型定义和回调函数
├── zf_device_config.h       # 配置相关
└── 外设文件说明.txt          # 外设清单
```

## 2. 外设分类

| 类别 | 外设 | 说明 |
|------|------|------|
| **TOF测距** | DL1A, DL1B | 激光测距模块 |
| **IMU传感器** | MPU6050, ICM20602, IMU660RA, IMU963RA | 惯性测量单元（加速度计+陀螺仪） |
| **液晶显示** | IPS114, IPS200, TFT180, OLED | 屏幕显示驱动 |
| **摄像头** | MT9V03X, OV7725, SCC8660, TSL1401 | 图像采集 |
| **无线通信** | BLE6A20, CH9141, WIFI-SPI, WIFI-UART | 无线数据传输 |
| **其他** | 按键、编码器、GNSS、虚拟示波器 | 杂项外设 |

## 3. 设计模式与特点

### 3.1 统一的文件结构

每个外设驱动包含：
- **.h 文件**：接口声明、宏定义、枚举类型
- **.c 文件**：具体实现

### 3.2 宏定义配置

使用宏定义来配置硬件引脚和通信方式：

```c
// 引脚配置示例（按键驱动）
#define KEY_LIST {C15, C14, C13, C12}

// 通信模式选择（屏幕驱动）
#define IPS200_USE_SOFT_SPI (0)    // 0=硬件SPI, 1=软件SPI

// 通信模式选择（IMU驱动）
#define MPU6050_USE_SOFT_IIC (1)   // 0=硬件IIC, 1=软件IIC
```

### 3.3 枚举类型定义

定义外设类型和工作状态：

```c
// 按键索引
typedef enum {
    KEY_1,
    KEY_2,
    KEY_3,
    KEY_4,
    KEY_NUMBER,
} key_index_enum;

// 按键状态
typedef enum {
    KEY_RELEASE,       // 释放
    KEY_SHORT_PRESS,   // 短按
    KEY_LONG_PRESS,    // 长按
} key_state_enum;
```

### 3.4 统一的初始化接口

每个外设都有标准的初始化函数：

```c
// 按键：period 为扫描周期（毫秒）
void key_init(uint32 period);

// 屏幕：type_select 选择通信方式
void ips200_init(ips200_type_enum type_select);

// IMU：返回初始化状态
uint8 mpu6050_init(void);

// 摄像头：返回初始化状态
uint8 mt9v03x_init(void);
```

### 3.5 外部变量声明

使用 `extern` 声明全局数据供用户访问：

```c
// IMU 数据（可直接读取）
extern int16 mpu6050_gyro_x, mpu6050_gyro_y, mpu6050_gyro_z;
extern int16 mpu6050_acc_x,  mpu6050_acc_y,  mpu6050_acc_z;

// 摄像头数据
extern uint8 mt9v03x_finish_flag;        // 图像采集完成标志
extern uint8 (*mt9v03x_image)[MT9V03X_W]; // 图像数据数组
```

### 3.6 回调函数机制

使用函数指针处理异步事件（主要用于摄像头和无线通信）：

```c
// 类型定义（zf_device_type.h）
typedef void (*callback_function)(void);

// 设置回调（用户需要提供处理函数）
extern callback_function camera_uart_handler;
extern callback_function wireless_module_uart_handler;
```

## 4. 典型使用示例

### 4.1 按键驱动

```c
#include "zf_common_headfile.h"

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);
    debug_init();

    // 初始化按键，10ms扫描周期
    key_init(10);

    while(1)
    {
        // 在定时器中断中调用：key_scanner();

        // 获取按键状态
        if (key_get_state(KEY_1) == KEY_SHORT_PRESS)
        {
            // 处理短按事件
            key_clear_state(KEY_1);  // 清除状态
        }

        if (key_get_state(KEY_1) == KEY_LONG_PRESS)
        {
            // 处理长按事件
            key_clear_state(KEY_1);
        }
    }
}
```

### 4.2 屏幕驱动（IPS200）

```c
// 初始化IPS200屏幕（SPI模式）
ips200_init(IPS200_TYPE_SPI);

// 设置显示方向（竖屏/横屏）
ips200_set_dir(IPS200_PORTAIT);

// 设置颜色（画笔颜色, 背景颜色）
ips200_set_color(RGB565_RED, RGB565_WHITE);

// 显示内容
ips200_show_string(0, 0, "Hello RT1064!");
ips200_show_int(0, 20, 12345, 5);       // 显示整数
ips200_show_float(0, 40, 3.14, 4, 2);   // 显示浮点数

// 显示图像（灰度摄像头）
ips200_displayimage03x(mt9v03x_image[0], 94, 60);

// 清屏
ips200_clear();
```

### 4.3 IMU传感器（MPU6050）

```c
int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);
    debug_init();

    // 初始化MPU6050
    uint8 status = mpu6050_init();
    if (status == 0)
    {
        printf("MPU6050 Init OK!\r\n");
    }

    while(1)
    {
        // 读取加速度计数据
        mpu6050_get_acc();

        // 读取陀螺仪数据
        mpu6050_get_gyro();

        // 转换为实际物理量（单位：g 和 度/秒）
        float acc_x = mpu6050_acc_transition(mpu6050_acc_x);
        float gyro_x = mpu6050_gyro_transition(mpu6050_gyro_x);

        printf("Acc: %.2f %.2f %.2f\r\n",
            mpu6050_acc_transition(mpu6050_acc_x),
            mpu6050_acc_transition(mpu6050_acc_y),
            mpu6050_acc_transition(mpu6050_acc_z));

        system_delay_ms(10);
    }
}
```

### 4.4 摄像头驱动（MT9V03X）

```c
uint8 image_data[MT9V03X_H][MT9V03X_W];

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);
    debug_init();

    // 初始化摄像头
    mt9v03x_init();

    while(1)
    {
        // 等待一帧图像采集完成
        while (!mt9v03x_finish_flag);
        mt9v03x_finish_flag = 0;

        // 复制图像数据进行处理
        memcpy(image_data, mt9v03x_image, MT9V03X_IMAGE_SIZE);

        // 在屏幕上显示
        ips200_displayimage03x(image_data[0], 94, 60);
    }
}
```

### 4.5 配置摄像头参数

```c
// 设置曝光时间（范围0-63，值越大曝光越强）
mt9v03x_set_exposure_time(30);

// 设置帧率（可选50/75/100等）
// 需要通过配置寄存器实现

// 设置增益（范围16-64）
// 通过配置寄存器实现
```

## 5. 库的整体架构

```
zf_common_headfile.h  (总头文件)
    │
    ├── SDK底层 (fsl_xxx.h)
    │
    ├── 通用基础库 (zf_common_*.h)
    │     ├── zf_common_clock.h    时钟
    │     ├── zf_common_debug.h    调试
    │     ├── zf_common_fifo.h     FIFO队列
    │     └── ...
    │
    ├── 驱动层 (zf_driver_*.h)
    │     ├── zf_driver_gpio.h     GPIO
    │     ├── zf_driver_uart.h     串口
    │     ├── zf_driver_iic.h      IIC
    │     ├── zf_driver_spi.h      SPI
    │     └── ...
    │
    └── 外设驱动层 (zf_device_*.h)
          ├── zf_device_key.h      按键
          ├── zf_device_ips200.h   屏幕
          ├── zf_device_mpu6050.h IMU
          ├── zf_device_mt9v03x.h  摄像头
          └── ...
```

## 6. 使用注意事项

### 6.1 引脚配置
- 大多数外设的引脚可以通过宏定义修改
- 修改前请查看对应 `.h` 文件中的引脚定义

### 6.2 通信方式选择
- IIC、SPI、UART 都支持软件模拟和硬件外设两种方式
- 硬件方式性能更好，软件方式更灵活

### 6.3 中断处理
- 按键扫描需要在定时器中断中调用 `key_scanner()`
- 摄像头的 DMA 和 UART 回调函数需要用户实现

### 6.4 图像缓冲区
- 摄像头图像数据使用 DMA 传输，用户只需检查标志位
- 图像数组大小与分辨率相关，需要根据实际配置使用

## 7. 快速开始清单

1. **包含头文件**：在 main.c 中包含 `zf_common_headfile.h`
2. **初始化时钟**：`clock_init(SYSTEM_CLOCK_600M);`
3. **初始化调试**：`debug_init();`
4. **初始化外设**：调用对应的 `xxx_init()` 函数
5. **使用外设**：调用对应的 API 函数或读取全局变量

## 8. 相关资源

- 淘宝店铺：https://seekfree.taobao.com/
- 例程目录：`D:\SmartCar\RT1064_Library\Example\Coreboard_Demo\`
