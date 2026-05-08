# Smartcar Intelligent Vision

智能车视觉识别与控制库，基于 RT1064 微控制器。

## 目录结构

```
├── libraries/          # 底层驱动与组件库
│   ├── components/     # 外设组件
│   ├── sdk/            # SDK 驱动
│   ├── zf_common/      # 通用模块
│   ├── zf_components/   # 组件模块
│   ├── zf_device/      # 设备驱动
│   └── zf_driver/      # 基础驱动
│
└── project 2.0/       # 项目 2.0 代码与配置
    ├── code/           # 源代码
    ├── iar/            # IAR 工程文件
    ├── mdk/            # MDK 工程文件
    └── user/          # 用户代码
```

## libraries 模块说明

| 模块 | 描述 |
|------|------|
| zf_driver | 基础驱动层，提供 GPIO、时钟等底层接口 |
| zf_device | 设备驱动，涵盖摄像头、显示等外设 |
| zf_components | 功能组件，图像处理、控制算法等 |
| zf_common | 通用工具模块 |
| sdk | 芯片原厂 SDK 封装 |
| components | 第三方组件 |

## project 2.0 模块说明

- **code/** - 核心算法代码
- **user/** - 用户应用代码
- **mdk/** - Keil MDK 工程配置
- **iar/** - IAR 工程配置

## 硬件平台

- MCU: NXP RT1064
- 主频: 600MHz
- 架构: ARM Cortex-M7

## 快速开始

1. 使用 MDK 或 IAR 打开对应工程文件
2. 配置调试器并下载程序
3. 根据具体项目配置参数

## 文档

- `project 2.0/MOTION_CHAIN_REVIEW.md` - 运动链评审文档
- `project 2.0/PID_MIGRATION_PLAN.md` - PID 迁移计划
- `project 2.0/DEBUG_FLOW_MANUAL_RECT_LAP.md` - 调试流程手册

## License

详见 `libraries/LICENSE` 文件
