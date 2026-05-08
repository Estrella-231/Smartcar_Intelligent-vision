# Code Review 报告

**检查项目：** `project 2.0`  
**检查日期：** 2026-03-31  
**检查范围：** `code/uer_algorithm/kinematics`

---

## 结论摘要

本轮复核后，仅保留两项已确认值得跟踪的问题：

- `R-04`：编码器处理逻辑存在设计冲突，属于潜伏风险
- `R-07`：速度滤波存在真实 bug

其余先前条目暂不保留，原因是：

- 部分属于误报
- 部分严重度偏高
- 部分没有进入当前工程主链路

---

## 问题列表

| 编号 | 级别 | 文件 | 位置 | 结论 |
|---|---|---|---|---|
| R-04 | 中 | `mecanum.c` | `Calc_SpeedNow()` L243-L252 | `pos_now` 的数据语义与回绕修正逻辑冲突 |
| R-07 | 高 | `mecanum.c` | `Calc_SpeedNow()` L255-L259 | `speed_last` 参与滤波但未更新，属于真实 bug |

---

## R-04: 编码器处理逻辑冲突

**文件：** `mecanum.c`  
**位置：** `Calc_SpeedNow()`，约 L243-L252

### 现象

`Calc_SpeedNow()` 里先计算：

```c
pos_diff = g_motor[motor_id].pos_now - g_motor[motor_id].pos_last;
```

随后又把 `pos_diff` 按“硬件计数器回绕”方式做修正：

```c
if(pos_diff > ENCODER_PULSE_MAX)
{
    pos_diff -= (ENCODER_PULSE_MAX - ENCODER_PULSE_MIN + 1);
}
else if(pos_diff < ENCODER_PULSE_MIN)
{
    pos_diff += (ENCODER_PULSE_MAX - ENCODER_PULSE_MIN + 1);
}
```

### 问题原因

当前工程中，`encoder.c` 已经通过软件累加更新 `pos_now`：

```c
change_pos_now(motor_id, get_pos_now(motor_id) + encoder_data[i]);
```

这说明 `pos_now` 现在是**软件累计位移**，而不是一次次读取出来的**原始硬件计数器值**。  
在这种前提下，再套一层“硬件 16 位计数器回绕修正”，数据语义就冲突了。

### 风险判断

这不是当前主链路上的高频实战 bug，因为当前主链路并未使用 `Calc_SpeedNow()`。  
但如果后续重新启用这条兼容路径，或者中途重置 `pos_now` / 出现大跳变，这段逻辑可能计算出错误速度。

### 建议

二选一统一设计：

1. 如果 `pos_now` 继续表示软件累计位移，则这里应直接做差，不再做回绕修正。
2. 如果这里必须保留回绕修正，则输入应改回原始硬件计数器快照，而不是软件累计值。

---

## R-07: 速度滤波未更新上一拍值

**文件：** `mecanum.c`  
**位置：** `Calc_SpeedNow()`，约 L255-L259

### 现象

当前滤波写法为：

```c
g_motor[motor_id].speed_now =
    (SPEED_FILTER_FACTOR * g_motor[motor_id].speed_last +
     (10 - SPEED_FILTER_FACTOR) * speed_calc) / 10;
```

但函数末尾只更新了：

```c
g_motor[motor_id].pos_last = g_motor[motor_id].pos_now;
```

没有更新：

```c
g_motor[motor_id].speed_last
```

### 结论

这是一个真实 bug。  
`speed_last` 既然参与滤波，就必须在本次计算结束后回写为新的上一拍速度，否则滤波器状态不完整。

### 影响

如果这条兼容速度路径被重新启用，滤波结果会失真，速度反馈会长期偏向错误值。

### 建议修复

在 `Calc_SpeedNow()` 末尾补上：

```c
g_motor[motor_id].speed_last = g_motor[motor_id].speed_now;
```

---

## 备注

- 本文件只保留本轮复核后确认成立的两项。
- `R-07` 属于真实 bug。
- `R-04` 属于设计冲突与潜伏风险，应在后续清理兼容旧路径时一并处理。
