# Smartcar Intelligent Vision - 智能视觉麦克纳姆轮推箱子机器人

## 项目简介

本项目基于逐飞科技 RT1064 核心板开发，实现了一款**全自主路径规划的麦克纳姆轮推箱子机器人**。机器人通过 OpenART 摄像头识别场地地图，使用 BFS（广度优先搜索）算法规划路径，配合里程计与 IMU 融合定位，完成箱子的推送任务。

## 仓库结构

```
Smartcar_Intelligent-vision/
├── project 2.0/          # 上层运动控制与路径规划
│   ├── code/
│   │   ├── user_business/         # 业务层（运动调度、BFS执行）
│   │   ├── user_driver/           # 驱动层（编码器等）
│   │   ├── user_config/           # 参数配置
│   │   ├── user_utils/            # 工具函数
│   │   ├── uer_algorithm/         # 算法层（PID、运动学、里程计、BFS）
│   │   │   ├── pid/                # 速度环与角度环PID
│   │   │   ├── kinematics/         # 麦克纳姆轮运动学、里程计
│   │   │   └── BFS/               # 推箱子路径规划
│   │   └── tests/                  # PC端单元测试
│   └── user/src/main.c             # 主程序入口
│
├── libraries/             # 底层驱动与SDK（逐飞科技开源库）
│   ├── sdk/               # NXP RT1064 SDK
│   ├── zf_driver/         # 逐飞GPIO/PWM/编码器驱动
│   ├── zf_device/         # 逐飞外设驱动（IPS200屏幕、IMU等）
│   ├── zf_components/     # 逐飞组件
│   └── zf_common/         # 通用头文件与宏
│
└── README.md
```

## 核心技术模块

### 1. 运动控制链路

```
BFS路径规划
    ↓
motion_exec_tick() (20ms调度周期)
    ↓
encoder_read_data() + imu_deal_data_period() + odometry_update()
    ↓
odometry_update_point_move_command() → vx/vy (global frame)
    ↓
angle_pid_calc_output() → vz (heading hold, P+角速度阻尼)
    ↓
Mecanum_inverse_kinematics(vx, vy, vz) → 4轮速度目标
    ↓
speed_pid_calc() → PWM
    ↓
motor_set_pwm()
```

### 2. 麦克纳姆轮逆运动学

```
LF = vy + vx + vz   (左前轮)
RF = vy - vx - vz   (右前轮)
LB = vy - vx + vz   (左后轮)
RB = vy + vx - vz   (右后轮)
```

### 3. 里程计融合

- **编码器** → 车身坐标系 vx/vy（正运动学）
- **IMU yaw** → 将车身速度旋转到全局坐标系
- **防滑权重** → 当轮速跟踪误差大时降低积分权重

### 4. PID 控制

| 环路 | 类型 | 特点 |
|------|------|------|
| 速度环 | PI+前馈 | 目标斜率限制，最小有效PWM |
| 角度环 | P+角速度阻尼 | 无积分项，防止移动中抖动 |
| 旋转环 | 兼容旧代码 | 状态机判断完成状态 |

## 开发环境

- **MCU**: NXP i.MX-RT1064 (600MHz)
- **IDE**: MDK5 (Keil) / IAR
- **仿真器**: DAP / J-Link V9+
- **传感器**: OpenART 摄像头、IMU963RA 陀螺仪、光电编码器

## 调试与测试

项目内置多级测试模式，切换 `robot_param.h` 中的 `SMARTCAR_RUNTIME_MODE`:

| 模式 | 值 | 用途 |
|------|---|------|
| `SMARTCAR_MODE_WHEEL_SIGN_CHECK` | 0 | 逐个轮子驱动，验证编码器符号 |
| `SMARTCAR_MODE_MANUAL_RECT_LAP` | 1 | 固定矩形路径，验证 chassis 闭环 |
| `SMARTCAR_MODE_BFS_FIXED_MAP` | 2 | 固定地图 + BFS 执行 |
| `SMARTCAR_MODE_OPENART_BFS` | 3 | 完整 OpenART + BFS 路径规划 |

## 分层测试建议（按顺序）

```
1. WHEEL_SIGN_CHECK  → 确认四个编码器符号一致
2. MANUAL_RECT_LAP   → 验证 chassis 能走完闭环
3. BFS_FIXED_MAP     → 验证 BFS 路径执行
4. OPENART_BFS       → 完整视觉方案
```

## 主要参数

| 参数 | 值 | 说明 |
|------|---|------|
| 控制周期 | 20ms | 调度器主周期 |
| 场地尺寸 | 3200×2400mm | 16×12 格，每格 200mm |
| 到达容忍度 | 30mm | segment 接受阈值 |
| 角度死区 | 0.5° | 旋转完成判定 |
| heading hold deadband | 0.8° | 航向保持死区 |

## 致谢

- [逐飞科技](https://www.seekfree.com/) - RT1064 开源库与核心板
- [NXP](https://www.nxp.com/) - i.MX RT1064 MCU

## License

- `libraries/` : 遵循各组件原有 License（逐飞科技 / NXP / FatFS 等）
- `project 2.0/` : MIT License
