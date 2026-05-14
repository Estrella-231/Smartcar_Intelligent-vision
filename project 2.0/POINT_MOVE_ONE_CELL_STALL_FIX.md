# 点移动差一格停住问题修复记录

## 现象

栅格路径执行时，小车有时会在距离下一个目标点大约一格的位置停住。此时任务调度并没有真正结束，但底盘不再移动。手动把车推到下一个目标点附近后，状态机又会继续执行后续路径。

## 根因

点到点控制器在接近目标点时仍然会输出一个非零的低速指令。这个指令从 `mm/s` 换算成“每个控制周期的轮速脉冲”后比较小，速度 PID 计算出的 PWM 也可能低于电机的最小有效 PWM。

原来的速度 PID 死区处理把这种“小但必要”的非零 PWM 直接清成 `0`，形成了控制死区：

- 调度器仍然认为当前路段没有完成
- 轮速目标仍然是非零
- 速度 PID 输出被清成零
- 里程计不再前进
- 手动推车后，里程计到达目标附近，状态机才继续

## 修复方案

在 `code/uer_algorithm/pid/speed_pid.c` 中修改速度 PID 的最小有效 PWM 处理逻辑。

现在非零 PWM 指令如果低于电机最小有效 PWM，不再清零，而是补到最小有效值：

- 正向输出：补到 `min_effect_pwm_fwd`
- 反向输出：补到 `-min_effect_pwm_rev`
- 目标为零时仍然清空 PID 状态并输出零

这样可以保证“还需要继续移动”的低速阶段不会被底层速度环静默吃掉。

## 回归测试

更新了 `code/tests/speed_pid_test.c`：

- 保留低速目标应使用最小有效 PWM 的测试
- 新增 `test_point_move_minimum_speed_is_not_silently_zeroed`
- 验证点移动最小有效速度经过速度 PID 后仍然有实际 PWM 输出

同时补齐了 `robot_control_exec_test` 使用的速度 PID 测试桩，让 PC 侧测试能覆盖当前堵转保护 API。

## 验证结果

重新编译并运行了相关 PC 侧测试：

```text
speed_pid_test passed
robot_control_exec_test passed
odometry_slip_test passed
motion_debug_plan_test passed
```

## 硬件复测关注点

烧录后，重点观察小车接近下一个栅格目标点时的遥测：

- 当前路段未完成时，轮速目标应保持非零
- PWM 通道不应因为指令较小而掉到零
- 如果仍然停住，下一步检查堵转保护状态、编码器方向、编码器比例和里程计尺度
