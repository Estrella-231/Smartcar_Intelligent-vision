# PID 控制移植计划

## 目标

把队友工程中的新版速度 PID 控制思路移植到当前工程，同时暂时不新增源文件、不新增头文件路径、不要求在 Keil 中添加文件。

当前策略是只改已有文件：

- `code/uer_algorithm/pid/speed_pid.h`
- `code/uer_algorithm/pid/speed_pid.c`

暂不直接覆盖或移植：

- 队友版 `motor_driver.c/h`
- 队友版 `angle_pid.c/h`
- 队友版 `chassis/`
- 队友版 `car_system/`
- 队友版 `mecanum/odometry/turn/p2p` 新目录结构

## 已知结论

- 队友版速度环从当前的“增量式 PID”改成了“位置式 PI + 前馈 + 目标斜坡 + 条件积分抗饱和 + 最小有效 PWM”。
- 队友版 `speed_pid.h` 依赖 `chassis.h`，当前工程没有采用这个分层，不能直接复制。
- 当前工程 `position_pid.c` 仍依赖 `PID_Pram_t`，因此不能直接把 `PID_Pram_t` 改名为 `PID_Param_t`，否则会影响旧位置环编译。
- 队友版 `motor_driver.c` 直接把 `abs(pwm)` 传给 `pwm_set_duty()`，而逐飞库 `PWM_DUTY_MAX` 是 `10000`。当前工程已有 `1000 -> PWM_DUTY_MAX` 映射，不能覆盖。
- 队友版 `angle_pid` 删除了旧旋转接口，当前 `robot_control.c` 仍调用 `Rotate_Finish_Judge()`、`angle_pid_set_target()` 等旧接口，暂不移植。

## 移植清单

- [x] 1. 备份当前 PID 文件
  - 备份 `speed_pid.c`
  - 备份 `speed_pid.h`
  - 只作为人工回退点，不纳入 Keil 工程

- [x] 2. 调整 `speed_pid.h` 的结构体，保持旧工程兼容
  - 保留 `PID_Pram_t` 名称，避免影响 `position_pid`
  - 在结构体里补充新版速度环需要的字段：
    - `kp`
    - `ki`
    - `kd`
    - `kf`
    - `i_limit`
    - `output_limit`
    - `min_effect_pwm_fwd`
    - `min_effect_pwm_rev`
    - `integral_full_error`
    - `integral_half_error`
    - `target_step_limit`
    - `target_ramped`
    - `zero_reset_mode`
    - `zero_decay_factor`
    - `enable_d`
    - `d_filter_alpha`
    - `d_state`
    - `last_measurement`
  - 不引入 `chassis.h`
  - 继续使用当前工程已有 include 风格

- [x] 3. 在 `speed_pid.h` 中补充新版速度环宏和枚举
  - 增加 `MAX_WHEEL_SPEED`
  - 增加 `PID_ZERO_RESET_IMMEDIATE`
  - 增加 `PID_ZERO_RESET_DECAY`
  - 优先把参数局限在速度环文件里，避免大改 `robot_param.h`

- [x] 4. 改造 `speed_pid.c` 的全局参数初始化
  - 按队友版四个轮子的参数初始化 `g_speed_pid[MOTOR_MAX]`
  - 初始建议沿用队友参数：
    - LF/RF: `kp=1.5f, ki=0.05f, kd=0.0f, kf=1.8f`
    - LB/RB: `kp=1.6f, ki=0.05f, kd=0.0f, kf=1.9f`
    - `output_limit` 分别参考队友版 `850/850/900/870`
    - `min_effect_pwm_fwd/rev` 参考队友版 `35/38`、`36/40`
  - 保留后续现场调参空间

- [x] 5. 移植目标斜坡逻辑
  - 添加 `speed_pid_ramp_target()`
  - 用 `target_step_limit` 限制目标速度每周期变化量
  - 更新 `target_ramped`
  - 确认 `change_speed_target()` 写入的是斜坡后的目标值，方便调试查看

- [x] 6. 移植分段积分和抗积分饱和
  - 添加 `speed_pid_get_integral_scale()`
  - 小误差全积分，中误差半积分，大误差不积分
  - 输出接近饱和且误差继续推向饱和时禁止积分
  - 误差有助于退出饱和时允许积分

- [x] 7. 移植前馈 + 位置式 PI 输出
  - 输出公式改为：
    - `output = kf * target + kp * error + ki * integral + d_term`
  - 默认关闭 D 项，保留字段和逻辑
  - 输出使用每个轮子的 `output_limit` 限幅

- [x] 8. 移植最小有效 PWM 补偿
  - 正向小输出补到 `min_effect_pwm_fwd`
  - 反向小输出补到 `min_effect_pwm_rev`
  - 仅在输出非零时补偿，避免目标附近强制抖动

- [x] 9. 保留当前工程停止行为
  - 目标斜坡后为 0 时清积分或衰减积分
  - 清 `output/last_output/d_state/last_measurement`
  - `g_motor[motor_id].pwm_out = 0`
  - 返回 0

- [x] 10. 检查旧接口兼容性
  - `speed_pid_reset()`
  - `speed_pid_reset_all()`
  - `speed_pid_calc()`
  - `g_speed_pid`
  - `position_pid.c` 对 `PID_Pram_t` 的使用不应编译断裂

- [x] 11. 做文本级编译风险检查
  - 搜索是否还有不存在的字段名
  - 搜索是否误引入 `chassis.h`
  - 搜索是否误使用 `PID_Param_t`
  - 搜索是否删除了 `derivative_lpf` 后影响 `position_pid`

- [x] 12. 本地可运行检查
  - 如果能用现有 PC 测试，更新或暂时跳过旧 `speed_pid_test`
  - 如果不能跑 Keil 编译，至少完成静态搜索检查
  - 记录需要你在 Keil 里实际编译确认的点

- [ ] 13. 硬件验证建议
  - 单轮悬空测试：低目标速度能否起转
  - 四轮同向测试：前进时四轮速度方向是否一致
  - 横移测试：检查是否出现明显斜漂
  - 急停测试：目标为 0 后 PWM 是否立即归零
  - 堵转/释放测试：观察积分是否不会持续冲到不可控

## 暂缓事项

- [ ] 队友版 `angle_pid` 与 `turn.c` 的拆分设计
- [ ] 队友版 `car_system` 控制层
- [ ] 队友版 `chassis` BSP 封装
- [ ] 队友版 `robot_param.h` 全局参数重排
- [ ] 队友版 BFS 调度结构变更

这些内容涉及新增文件和 Keil 路径，先不动。

## 疑问记录

- [ ] 新速度环的 `output_limit` 是否按队友版 `850/850/900/870` 直接使用，还是先降低到更保守的值？
- [ ] 当前车四个轮子的正反向最小有效 PWM 是否已经测过？如果没有，先用队友版参数。
- [ ] 控制周期实际是否稳定为 `EXEC_CONTROL_PERIOD_MS = 20ms`？新版速度 PID 的目标斜坡按“每控制周期”计算。

## 执行记录

- 已按队友版参数直接使用 `output_limit = 850/850/900/870`。如果 Keil 编译后实车起步过猛，优先降低这四个值或降低 `kf`。
- 已先使用队友版最小有效 PWM：LF/RF 为 `35/38`，LB/RB 为 `36/40`。后续需要悬空单轮实测确认。
- 当前测试按 `20ms` 控制周期覆盖目标斜坡；Keil 工程中仍需要确认实际调用 `speed_pid_calc()` 的周期参数。
- 本地 PC 测试已覆盖目标斜坡、最小有效 PWM、目标归零清输出。
- 文本检查已确认没有引入 `chassis.h`，没有使用 `PID_Param_t`，并保留 `PID_Pram_t` 与 `derivative_lpf` 兼容旧 `position_pid`。
