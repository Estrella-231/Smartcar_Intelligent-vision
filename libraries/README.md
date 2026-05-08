# Smartcar 底层驱动库 (SeekFree RT1064 Library)

## 简介

本目录包含逐飞科技 RT1064 核心板的底层驱动库，是 [Smartcar_Intelligent-vision](https://github.com/Estrella-231/Smartcar_Intelligent-vision) 项目的底层依赖。

## 目录结构

```
libraries/
├── sdk/               # NXP i.MX-RT1064 官方 SDK
│   └── CMSIS/         # ARM Cortex-M7 内核头文件
├── zf_driver/         # 逐飞 GPIO/PWM/编码器/定时器驱动
├── zf_device/         # 逐飞外设驱动
│   ├── imu963ra.c/h   # IMU963RA 陀螺仪驱动
│   ├── imu660ra.c/h   # IMU660RA 陀螺仪驱动
│   └── ips200.c/h     # IPS200 LCD 屏幕驱动
├── zf_components/     # 逐飞通用组件
├── zf_common/         # 公共头文件、宏定义、通用函数
└── components/        # 第三方组件
    ├── fatfs/         # FatFS 文件系统
    └── sdmmc/         # SD 卡驱动
```

## 驱动列表

| 驱动 | 文件 | 说明 |
|------|------|------|
| GPIO | `zf_driver/gpio.h` | 通用 IO 操作 |
| PWM | `zf_driver/pwm.h` | 电机 PWM 控制 |
| 编码器 | `zf_driver/encoder.h` | 正交编码器读取（脉冲+方向模式） |
| 定时器 | `zf_driver/pit.h` | 周期定时中断 |
| IMU | `zf_device/imu963ra.h` | 六轴陀螺仪 |
| 屏幕 | `zf_device/ips200.h` | 200×320 SPI LCD |
| Flash | `zf_driver/flash.h` | 片内 Flash 操作 |

## 使用说明

本库作为 `project 2.0` 的子模块（git submodule）使用，引用方式：

```c
#include "zf_common_headfile.h"
#include "motor_driver.h"
#include "encoder.h"
```

所有驱动均已集成到 `zf_common_headfile.h`，无需单独包含。

## 开发环境

- **MCU**: NXP i.MX-RT1064 (600MHz, Cortex-M7)
- **IDE**: MDK5 / IAR EWARM
- **依赖**: NXP RT1064 SDK

## 版权说明

各组件遵循原有版权协议：
- `sdk/` : NXP Software License
- `zf_driver/` / `zf_device/` : 逐飞科技 License
- `components/fatfs/` : FatFS License (GNU GPLv3)
- `components/sdmmc/` : NXP License

详情请参阅各目录下的 `LICENSE` / `LICENSE.txt` 文件。

## 逐飞科技

- 官网: https://www.seekfree.com/
- 论坛: https://www.seekfree.com/forum/
