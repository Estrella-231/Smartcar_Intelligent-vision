# 手工矩形绕圈排障实施计划

## 目标

先把 BFS、地图识别、推箱子逻辑从问题链路里拿掉，只验证底盘运动、里程计、航向保持和段完成判据是否正常。默认运行入口改为手工矩形绕圈模式，便于先把“斜着走、跑得慢、距离不对、提前停”这几个问题拆开定位。

## 当前方案

1. 新增独立调试模式，不依赖 BFS。
2. 新增一条固定手工运动计划，起点仍为 `(1, 6)`。
3. 运行一条场地内圈矩形路线：
   - `U4`
   - `R13`
   - `D9`
   - `L13`
   - `U5`
4. 全段使用 `SEG_WALK`，不插入箱体、等待、旋转识别等业务逻辑。
5. 保持当前 yaw hold 链路，继续用 `motion_exec -> odometry -> angle_pid -> mecanum -> speed_pid -> motor` 这条主控制路径。

## 代码改动摘要

### 1. 运行模式

- 在 `robot_param.h` 增加统一运行模式宏：
  - `SMARTCAR_MODE_WHEEL_SIGN_CHECK`
  - `SMARTCAR_MODE_MANUAL_RECT_LAP`
  - `SMARTCAR_MODE_BFS_FIXED_MAP`
  - `SMARTCAR_MODE_OPENART_BFS`
- 默认模式切到 `SMARTCAR_MODE_MANUAL_RECT_LAP`。

### 2. 手工计划入口

- 在 `robot_control` 中新增：
  - `motion_exec_load_manual_plan(const MotionPlan *plan, uint8_t start_grid_x, uint8_t start_grid_y)`
  - `motion_exec_set_segment_accept_tolerance_mm(int32_t tolerance_mm)`
- 让执行器可以绕开 BFS，直接装载 `MotionPlan`。

### 3. 调试计划与遥测

- 新增 `debug_runtime.c/.h`：
  - 负责生成内圈矩形 `MotionPlan`
  - 负责定义紧凑四数遥测页
- 紧凑遥测改成自动轮播五页：
  - `POSE`
  - `TARGET`
  - `STATE`
  - `WHEEL_F`
  - `WHEEL_R`

### 4. 完成判据可调

- 在 `odometry` 中新增：
  - `odometry_set_finish_tolerance_mm(int32_t tolerance_mm)`
  - `odometry_get_finish_tolerance_mm(void)`
- 手工绕圈模式下收紧阈值：
  - `DEBUG_EXEC_SEGMENT_ACCEPT_TOL_MM = 20`
  - `DEBUG_POINT_MOVE_FINISH_TOL_MM = 20`

### 5. 四轮映射检查

- 把原先“只看后轮”的检查模式扩成“四轮正反转逐个检查”。
- 顺序为：
  - `LF+`
  - `RF+`
  - `LB+`
  - `RB+`
  - `LF-`
  - `RF-`
  - `LB-`
  - `RB-`
- 每步之间插入 `STOP` 段，避免惯性和编码器残留影响判断。

## 预期收益

### 1. 排除 BFS 干扰

- 如果车在手工矩形模式下仍然斜走、提前停、尺度不对，说明问题在底盘、里程计、轮向映射、完成判据或 PID，不在 BFS。
- 如果手工矩形模式正常，而 BFS 模式异常，后续就重点查地图、计划和段切换逻辑。

### 2. 提高现场可观测性

- 原来四数遥测只有 `phase / segment_index / x_mm / y_mm`，不够定位根因。
- 现在能直接看：
  - 当前目标点
  - 剩余误差
  - 车体速度
  - yaw 与 yaw 误差
  - 四轮目标与反馈

### 3. 提前收敛完成判据问题

- 当前默认阈值较松，确实可能导致“离目标一格左右就停”。
- 调试模式下先收紧阈值，便于判断是控制能力不够，还是业务层过早宣布完成。

## 验证顺序

1. 先跑四轮正反转映射检查。
2. 再跑手工矩形绕圈。
3. 用遥测页判断问题属于哪一层：
   - 轮向/编码器
   - 里程计尺度
   - 点到点完成判据
   - heading hold 干扰
   - 速度环跟踪能力
4. 底盘问题稳定后，再回到 BFS 模式继续联调。

## 已完成的实现点

- 默认入口已切到手工矩形绕圈模式。
- 手工计划注入接口已接入执行器。
- 紧凑遥测页已接入主循环。
- 四轮正反转检查模式已扩展。
- 主机侧回归测试已覆盖：
  - 手工计划装载
  - 调试矩形计划生成
  - 原有 rotate 执行路径

## 未覆盖项

- 还没有跑 MDK/IAR 全工程编译验证。
- 还没有上板做实车验证。
- IAR 工程文件未同步添加 `debug_runtime.c`，目前只确认了 MDK 工程文件已添加该源文件。
