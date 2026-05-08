# 当前运动链路排查说明

## 主链路总览

当前主运动链路是：

```text
main.c 主循环
  -> motion_exec_tick(EXEC_CONTROL_PERIOD_MS)
      -> motion_exec_update_feedback()
          -> encoder_read_data()
          -> imu_deal_data_period()
          -> odometry_update()
      -> odometry_update_point_move_command()
      -> angle_pid_calc_output()
      -> Mecanum_mmps_to_pulse_per_period()
      -> Mecanum_inverse_kinematics()
      -> speed_pid_calc()
      -> motor_set_pwm()
```

当前不应优先排查的旧链路：

```text
position_pid
angle_pid_calc()
rotate_pid_calc()
Rotate_PWM_Calc()
```

这些不是当前主运动执行路径。

## 1. 主循环入口

文件：

- `user/src/main.c`

重点确认：

```c
motion_exec_tick(EXEC_CONTROL_PERIOD_MS);
```

以及：

```c
#define EXEC_CONTROL_PERIOD_MS (20)
```

这里决定运动执行器多久运行一次。当前速度 PID 的目标斜坡也是按这个周期计算。

排查项：

- [ ] `motion_exec_tick()` 是否确实在主循环里周期调用
- [ ] 实际周期是否接近 `EXEC_CONTROL_PERIOD_MS`
- [ ] 是否存在额外 `system_delay_ms()` 导致周期变慢
- [ ] 是否有调试模式绕过了 `motion_exec_tick()`

## 2. 运动调度器

文件：

- `code/user_business/robot_control.c`

重点函数：

```c
void motion_exec_tick(uint32_t control_period_ms)
```

它决定当前车处于什么执行状态：

```text
空闲
等待开始
旋转
直线/点到点移动
等待
错误
完成
```

重点调用：

```c
motion_exec_apply_body_command(...)
motion_exec_stop_chassis()
```

排查项：

- [ ] 是否进入了预期状态
- [ ] 是否误进入 `CAR_STATE_ERROR`
- [ ] 是否反复调用 `motion_exec_stop_chassis()`
- [ ] 是否正确加载路径或手动测试路径
- [ ] 目标段切换是否正常

## 3. 反馈更新

文件：

- `code/user_business/robot_control.c`

重点函数：

```c
static void motion_exec_update_feedback(uint32_t control_period_ms)
{
    encoder_read_data();
    imu_deal_data_period(control_period_ms);
    odometry_update(control_period_ms);
}
```

顺序：

```text
先读编码器
再更新 IMU
再融合里程计
```

排查项：

- [ ] `encoder_read_data()` 是否每个控制周期调用一次
- [ ] `imu_deal_data_period()` 是否使用真实周期
- [ ] `odometry_update()` 是否在新反馈之后调用
- [ ] 反馈更新是否早于控制输出计算

## 4. 编码器读取

文件：

- `code/user_driver/encoder.c`
- `code/user_driver/encoder.h`

重点函数：

```c
encoder_read_data()
get_encoder_data(MotorID)
get_encoder_position_delta(MotorID)
```

排查项：

- [ ] 四个编码器通道是否和 `MOTOR_LF/RF/LB/RB` 对应
- [ ] 静止时四个编码器反馈是否接近 0
- [ ] 单轮正转时对应编码器是否为正
- [ ] 前进时四个轮子的反馈符号是否一致
- [ ] 横移时对角轮反馈符号是否符合麦轮运动学
- [ ] 编码器方向归一化是否和电机正方向一致

## 5. IMU 更新

文件：

- `code/user_driver/gyroscope.c`
- `code/user_driver/gyroscope.h`

重点函数：

```c
imu_deal_data_period(period_ms)
imu_get_yaw_cd()
imu_get_gyro_z_cdps()
```

内部链路：

```text
gyro_read_data()
  -> IMU_calib_data()
  -> IMU_Convert_Physical()
  -> IMU_Calc_Rotate_Angle_Period()
  -> IMU_Calc_Rotate_Acc_Period()
```

排查项：

- [ ] `imu_init_and_calibrate()` 是否在启动时成功调用
- [ ] 车静止时 `imu_get_gyro_z_cdps()` 是否接近 0
- [ ] 顺时针/逆时针旋转时 yaw 符号是否符合控制约定
- [ ] `imu_get_yaw_cd()` 是否随旋转持续变化
- [ ] `period_ms` 是否和真实 IMU 更新周期一致
- [ ] 启动校准时车是否保持静止

## 6. 里程计

文件：

- `code/uer_algorithm/kinematics/odometry.c`
- `code/uer_algorithm/kinematics/odometry.h`

重点函数：

```c
odometry_update(period_ms)
odometry_update_point_move_command(&vx_cmd_mmps, &vy_cmd_mmps)
```

负责：

```text
编码器脉冲 -> 轮速 mm/s
轮速 -> 车体 vx/vy
结合 IMU yaw -> 全局 x/y
当前位置和目标点误差 -> vx/vy 指令
```

排查项：

- [ ] `x_mm/y_mm` 是否随实际移动方向正确变化
- [ ] `target_x_mm/target_y_mm` 是否设置正确
- [ ] `dx/dy` 误差方向是否正确
- [ ] `vx_cmd_mmps/vy_cmd_mmps` 是否朝目标方向输出
- [ ] 接近目标时是否进入完成状态
- [ ] `POINT_MOVE_FINISH_TOL_MM` 是否过大或过小
- [ ] `ODOM_SCALE_X/Y` 是否需要实车标定

## 7. 航向保持

文件：

- `code/user_business/robot_control.c`
- `code/uer_algorithm/pid/angle_pid.c`
- `code/uer_algorithm/pid/angle_pid.h`

重点调用：

```c
vz_cmd = angle_pid_calc_output(...)
```

作用：

```text
yaw 误差 -> 旋转修正 vz_cmd
```

排查项：

- [ ] 直线运动时 yaw 目标是否固定
- [ ] 偏航后 `vz_cmd` 方向是否能把车修回来
- [ ] `EXEC_YAW_VZ_LIMIT` 是否过大或过小
- [ ] `heading_hold_kp` 是否导致抖动
- [ ] `heading_hold_gyro_damping` 是否能压住旋转过冲

## 8. 麦轮逆解

文件：

- `code/uer_algorithm/kinematics/mecanum.c`
- `code/uer_algorithm/kinematics/mecanum.h`

重点函数：

```c
Mecanum_inverse_kinematics(vx_pulse, vy_pulse, vz_cmd);
Mecanum_mmps_to_pulse_per_period(velocity_mmps, period_ms);
```

当前目标分配关系：

```text
LF = vy + vx + vz
RF = vy - vx - vz
LB = vy - vx + vz
RB = vy + vx - vz
```

排查项：

- [ ] 前进时四轮目标是否同号
- [ ] 后退时四轮目标是否反向同号
- [ ] 横移时对角轮是否同号
- [ ] 原地旋转时左右轮是否反号
- [ ] `vx`、`vy` 的方向定义是否和场地坐标一致
- [ ] `PULSE_PER_MM` 是否和实际轮子/编码器匹配

## 9. 速度 PID

文件：

- `code/uer_algorithm/pid/speed_pid.c`
- `code/uer_algorithm/pid/speed_pid.h`

重点函数：

```c
speed_pid_calc(motor_id, target_speed, current_speed, period_ms)
```

当前速度 PID 链路：

```text
轮目标速度
  -> 目标斜坡 target_ramped
  -> 前馈 + PI
  -> 最小有效 PWM
  -> pwm_command
```

排查项：

- [ ] `target_speed` 是否来自对应轮的麦轮逆解结果
- [ ] `current_speed` 是否来自同一个轮子的编码器
- [ ] 目标为 0 时 PWM 是否立即归零
- [ ] 低速目标时是否能起转
- [ ] 目标斜坡是否太慢或太快
- [ ] 堵转时积分是否不会无限累积
- [ ] 四个轮子的 `kp/ki/kf/output_limit` 是否需要单独调

## 10. PWM 输出

文件：

- `code/user_driver/motor_driver.c`
- `code/user_driver/motor_driver.h`

重点函数：

```c
motor_set_pwm(motor_id, pwm_command)
```

排查项：

- [ ] PWM 正负号是否对应正确方向
- [ ] `PWM_MAX_VALUE = 1000` 是否正确映射到 `PWM_DUTY_MAX`
- [ ] 四个电机方向引脚正转电平是否正确
- [ ] 四个电机 PWM 引脚是否对应实际接线
- [ ] `MOTOR_LF/RF/LB/RB` 和实际车轮位置是否一致
- [ ] `motor_stop_all()` 是否能可靠停下四轮

## 建议最小排查顺序

不要一开始就跑完整 BFS 或完整路径。建议按以下顺序逐步闭环：

```text
1. 单独 motor_set_pwm() 看四轮方向
2. encoder_read_data() 看四轮反馈符号
3. Mecanum_inverse_kinematics() 看四轮目标分配
4. speed_pid_calc() 看 PWM 输出是否合理
5. odometry_update() 看 x/y/yaw 是否合理
6. motion_exec_tick() 看完整任务状态机
```

对应检查表：

- [ ] 单轮 PWM 测试通过
- [ ] 四轮编码器方向测试通过
- [ ] 前进/后退/横移/旋转目标分配通过
- [ ] 单轮速度闭环测试通过
- [ ] 四轮速度闭环测试通过
- [ ] 里程计直线移动方向正确
- [ ] 里程计横移方向正确
- [ ] 航向保持方向正确
- [ ] 点到点控制能收敛到目标
- [ ] 完整 `motion_exec_tick()` 状态切换正确

## PULSE_PER_MM 标定

`PULSE_PER_MM` 是点到点距离精度的基础参数。它不是编码器计数范围，也不是固定库参数，而是实车“每移动 1mm 对应多少编码器脉冲”的标定值。

当前代码中的：

```c
#define PULSE_PER_MM (7.7f)
```

是临时值。它会直接影响：

- `Mecanum_mmps_to_pulse_per_period()`：上层 mm/s 指令转换成轮目标脉冲
- `Mecanum_pulse_per_period_to_mmps()`：编码器反馈转换成 mm/s
- `odometry_update()`：编码器反馈积分成 x/y 位移
- 点到点路径是否走到正确距离

建议先做直线标定：

```text
1. 让车沿 X 或 Y 方向低速直线移动一段可测距离，例如 1000mm。
2. 记录四个轮子的有效编码器累计脉冲，去掉明显打滑或异常轮。
3. 求平均脉冲数 average_encoder_pulses。
4. 计算：
   PULSE_PER_MM = average_encoder_pulses / measured_travel_mm
5. 更新 robot_param.h 中的 PULSE_PER_MM。
6. 再检查 ODOM_SCALE_X / ODOM_SCALE_Y 是否还需要细调。
```

不要先靠调大 `POINT_MOVE_KP` 来补距离误差。`POINT_MOVE_KP` 只影响收敛速度，不能修复距离比例错误。

## 暂不优先排查的旧接口

以下接口不在当前主运动链路中，除非 Keil 编译报错或旧测试需要，否则不应优先排查：

- `position_pid_reset()`
- `position_pid_reset_all()`
- `pos_pid_calc()`
- `check_pos_arrived()`
- `angle_pid_calc()`
- `rotate_pid_calc()`
- `Rotate_PWM_Calc()`
- `IMU_Calc_Rotate_Angle()`
- `IMU_Calc_Rotate_Acc()`
- `imu_deal_data()`
