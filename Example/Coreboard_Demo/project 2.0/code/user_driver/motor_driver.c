/**
 * @file motor_driver.c
 * @brief 电机驱动程序 - 控制四轮智能小车的四个DC电机
 * 
 * 本文件实现了电机驱动层的核心功能：
 * - 电机的初始化和硬件参数配置
 * - PWM信号控制（速度）和GPIO控制方向
 * - 支持独立控制每个电机或同时控制所有电机
 * - 提供速度、位置等运动参数的读写接口
 * 
 * 电机布局（四轮驱动）：
 *   左前(LF-电机1)    左后(LB-电机3)
 *        |                  |
 *   [小车俯视图]
 *        |                  |
 *   右前(RF-电机2)    右后(RB-电机4)
 */

#include "motor_driver.h"

#include "gpio.h"

/* ==================== 全局变量定义 ==================== */
/**
 * @brief 四个电机的控制结构体数组
 * 
 * 包含每个电机的：
 * - 硬件参数：GPIO方向引脚、PWM引脚
 * - 运动参数：目标速度、当前速度、上一速度、目标位置、当前位置、上一位置
 * - 输出参数：实际PWM输出值
 */
Motor_Control_t g_motor[MOTOR_MAX] = {0};

/**
 * @brief 整车控制结构体
 * 
 * 存储整车的全局控制信息（如运动模式、速度限制等）
 */
Car_Control_t g_car = {0};

/* ==================== 静态函数定义 ==================== */

/**
 * @brief 获取指定电机正转时方向引脚的GPIO电平
 * 
 * 由于不同电机驱动芯片的接线方式可能不同（有些是高电平正转，有些是低电平正转），
 * 本函数返回每个电机实际的"正转"对应的GPIO电平值，由硬件配置决定。
 * 
 * @param motor_id 电机ID (MOTOR_LF/MOTOR_RF/MOTOR_LB/MOTOR_RB)
 * @return GPIO电平值 (GPIO_HIGH=1 或 GPIO_LOW=0)
 * 
 * 例如：
 *   - 如果MOTOR1_FORWARD_LEVEL=GPIO_HIGH，则MOTOR_LF(左前轮)设高电平时正转
 *   - 如果MOTOR2_FORWARD_LEVEL=GPIO_LOW，则MOTOR_RF(右前轮)设低电平时正转
 */
static gpio_level_enum motor_forward_level(MotorID motor_id)
{
    switch(motor_id)
    {
        /* 左前轮电机 */
        case MOTOR_LF: return MOTOR1_FORWARD_LEVEL;    // 返回左前轮电机的正转电平
        /* 右前轮电机 */
        case MOTOR_RF: return MOTOR2_FORWARD_LEVEL;    // 返回右前轮电机的正转电平
        /* 左后轮电机 */
        case MOTOR_LB: return MOTOR3_FORWARD_LEVEL;    // 返回左后轮电机的正转电平
        /* 右后轮电机 */
        case MOTOR_RB: return MOTOR4_FORWARD_LEVEL;    // 返回右后轮电机的正转电平
        /* 默认返回高电平（容错处理） */
        default:       return GPIO_HIGH;
    }
}

/**
 * @brief 将有符号的PWM值转换为无符号的占空比值（0-PWM_MAX_VALUE）
 * 
 * 该函数处理PWM值的幅度（绝对值）并进行饱和限制。
 * 正值表示正向转动，负值表示反向转动，本函数只取其绝对值。
 * 
 * 工作流程：
 * 1. 计算PWM值的绝对值（取符号无关的幅度）
 * 2. 如果超过最大值PWM_MAX_VALUE，则限制为最大值（防止超出范围）
 * 3. 返回限制后的占空比
 * 
 * @param pwm 有符号PWM值，取值范围：[-PWM_MAX_VALUE, +PWM_MAX_VALUE]
 *            正值：正转，负值：反转
 * @return 无符号占空比值，范围：[0, PWM_MAX_VALUE]
 * 
 * 例如：
 *   motor_abs_pwm(100) → 100   (正向100)
 *   motor_abs_pwm(-100) → 100  (反向100，绝对值同样是100)
 *   motor_abs_pwm(10000) → PWM_MAX_VALUE (超限保护)
 */
static uint32_t motor_abs_pwm(int32_t pwm)
{
    /* 取绝对值：pwm>=0时为pwm，pwm<0时为-pwm */
    uint32_t duty = (uint32_t)((pwm >= 0) ? pwm : -pwm);

    /* 饱和限制：防止占空比超过最大值 */
    if(duty > PWM_MAX_VALUE)
    {
        duty = PWM_MAX_VALUE;   // 超限则设为最大值
    }

    return duty;  // 返回有效的占空比值
}

/**
 * @brief 初始化单个电机的硬件配置
 * 
 * 该函数为指定的电机配置GPIO引脚、PWM引脚，并初始化电机的所有运动状态参数。
 * 包括：方向控制、PWM速度控制、运动参数清零等。
 * 
 * 初始化步骤：
 * 1. 参数验证：检查motor_id的有效性
 * 2. GPIO配置：根据电机ID分配对应的方向引脚和PWM引脚
 * 3. 方向设置：获取该电机的正转电平并初始化方向引脚
 * 4. PWM初始化：初始化PWM模块（设置频率和初始占空比）
 * 5. 参数清零：初始化速度、位置等运动参数（全部置0）
 * 
 * @param motor_id 要初始化的电机ID (MOTOR_LF/MOTOR_RF/MOTOR_LB/MOTOR_RB)
 * @return void
 * 
 * 注意：
 *   - 无效的motor_id会导致函数直接返回（不进行任何操作）
 *   - 该函数需要在motor_driver_init()中被调用
 *   - 初始化后电机处于停止状态（PWM占空比为0）
 */
void motor_hw_init(MotorID motor_id)
{
    /* ============ 参数验证 ============ */
    /* 检查motor_id是否有效（不超出范围） */
    if(motor_id >= MOTOR_MAX)
    {
        return;  // 无效的电机ID，直接返回
    }

    /* ============ GPIO引脚配置 ============ */
    /* 根据不同的电机ID，分配对应的方向控制引脚(DIR)和脉宽调制引脚(PWM) */
    switch(motor_id)
    {
        /* 左前轮电机硬件参数配置 */
        case MOTOR_LF:
            g_motor[motor_id].hw.dir_pin = MOTOR1_DIR;    // 方向控制引脚
            g_motor[motor_id].hw.pwm_pin = MOTOR1_PWM;    // PWM速度控制引脚
            break;
        /* 右前轮电机硬件参数配置 */
        case MOTOR_RF:
            g_motor[motor_id].hw.dir_pin = MOTOR2_DIR;    // 方向控制引脚
            g_motor[motor_id].hw.pwm_pin = MOTOR2_PWM;    // PWM速度控制引脚
            break;
        /* 左后轮电机硬件参数配置 */
        case MOTOR_LB:
            g_motor[motor_id].hw.dir_pin = MOTOR3_DIR;    // 方向控制引脚
            g_motor[motor_id].hw.pwm_pin = MOTOR3_PWM;    // PWM速度控制引脚
            break;
        /* 右后轮电机硬件参数配置 */
        case MOTOR_RB:
            g_motor[motor_id].hw.dir_pin = MOTOR4_DIR;    // 方向控制引脚
            g_motor[motor_id].hw.pwm_pin = MOTOR4_PWM;    // PWM速度控制引脚
            break;
        default:
            return;  // 无效的case，直接返回
    }

    /* ============ 方向引脚初始化 ============ */
    /* 获取该电机的"正转"时对应的GPIO电平值 */
    g_motor[motor_id].hw.dir_level = motor_forward_level(motor_id);
    /* 将方向引脚初始化为正转电平 */
    gpio_set_level(g_motor[motor_id].hw.dir_pin, g_motor[motor_id].hw.dir_level);

    /* ============ PWM模块初始化 ============ */
    /* 初始化PWM：设置频率、初始占空比为0（电机初始不动） */
    pwm_init(g_motor[motor_id].hw.pwm_pin, PWM_FREQUENCY, 0);

    /* ============ 运动参数初始化 ============ */
    /* 目标速度清零（期望速度为0） */
    g_motor[motor_id].speed_target = 0;
    /* 当前速度清零（实际速度为0） */
    g_motor[motor_id].speed_now = 0;
    /* 上一时刻速度清零（用于速度变化率计算） */
    g_motor[motor_id].speed_last = 0;
    /* 目标位置清零（期望位置为0） */
    g_motor[motor_id].pos_target = 0;
    /* 当前位置清零（实际位置为0，编码器零点） */
    g_motor[motor_id].pos_now = 0;
    /* 上一时刻位置清零（用于位置变化率计算） */
    g_motor[motor_id].pos_last = 0;
    /* PWM输出值清零（初始不输出PWM） */
    g_motor[motor_id].pwm_out = 0;
}

/**
 * @brief 电机驱动模块总体初始化函数
 * 
 * 这是电机驱动层的入口函数，负责整个电机系统的初始化。
 * 必须在程序开始时调用，是使用任何电机功能的前置条件。n * 
 * 初始化流程：
 * 1. 初始化GPIO模块（为所有GPIO操作做准备）
 * 2. 逐个初始化四个电机的硬件和参数
 * 3. 停止所有电机（确保系统启动时电机不动作）
 * 
 * @param void
 * @return void
 * 
 * 建议调用时机：
 *   - 系统启动阶段，在main()开始时调用
 *   - 在任何电机控制函数（如motor_set_pwm）调用之前
 * 
 * 例如在main函数中：
 *   void main(void)
 *   {
 *       motor_driver_init();  // 初始化电机系统
 *       // ... 其他初始化代码 ...
 *       // 之后可以使用电机控制函数
 *   }
 */
void motor_driver_init(void)
{
    /* ============ 初始化GPIO模块 ============ */
    /* 配置所有GPIO相关的硬件（时钟使能、引脚复用等） */
    GPIO_Init();

    /* ============ 逐个初始化四个电机 ============ */
    /* 循环初始化所有电机（左前、右前、左后、右后） */
    for(int i = 0; i < MOTOR_MAX; i++)
    {
        /* 调用单电机初始化函数，配置硬件IO和参数 */
        motor_hw_init((MotorID)i);
    }

    /* ============ 停止所有电机 ============ */
    /* 确保系统初始化完成时，所有电机都处于停止状态 */
    motor_stop_all();
}

/**
 * @brief 设置指定电机的转动方向
 * 
 * 通过改变方向控制引脚的GPIO电平来控制电机的正反转。
 * 该函数仅改变方向，不改变速度（PWM占空比）。
 * 
 * 工作原理：
 * - forward=true：将方向引脚设为"正转"电平（由硬件配置决定）
 * - forward=false：将方向引脚设为"反转"电平（取反）
 * 
 * @param motor_id 电机ID (MOTOR_LF/MOTOR_RF/MOTOR_LB/MOTOR_RB)
 * @param forward 方向标志
 *               true：正转（前进）
 *               false：反转（后退）
 * @return void
 * 
 * 注意点：
 *   - 无效的motor_id会导致函数直接返回（无操作）
 *   - 一般由motor_set_pwm()内部自动调用，无需手动调用
 *   - 改变方向时，如果PWM还在输出，可能产生反向冲击
 * 
 * 示例：
 *   motor_set_direction(MOTOR_LF, true);   // 左前轮正转
 *   motor_set_direction(MOTOR_LF, false);  // 左前轮反转
 */
void motor_set_direction(MotorID motor_id, bool forward)
{
    /* 要设置的GPIO电平值 */
    gpio_level_enum level;

    /* ============ 参数验证 ============ */
    /* 检查电机ID是否有效 */
    if(motor_id >= MOTOR_MAX)
    {
        return;  // 无效ID，直接返回
    }

    /* ============ 计算目标电平 ============ */
    /* 
     * 根据forward参数确定要输出的GPIO电平：
     * - forward=true：设为"正转"电平
     * - forward=false：设为"反转"电平（正转电平的反向）
     * 
     * 例如，如果MOTOR1_FORWARD_LEVEL=GPIO_HIGH：
     *   forward=true  → level = GPIO_HIGH（正转）
     *   forward=false → level = GPIO_LOW（反转）
     */
    level = forward ? g_motor[motor_id].hw.dir_level :
                      (g_motor[motor_id].hw.dir_level == GPIO_HIGH ? GPIO_LOW : GPIO_HIGH);

    /* ============ 执行方向改变 ============ */
    /* 将计算出的电平值写入方向控制引脚 */
    gpio_set_level(g_motor[motor_id].hw.dir_pin, level);
}

/**
 * @brief 设置单个电机的PWM速度值
 * 
 * 这是电机速度控制的核心函数。通过PWM占空比控制电机的转速，
 * 通过方向标志控制电机的旋转方向。
 * 
 * PWM值含义：
 *   pwm > 0：正向转动（前进），值越大转速越快
 *   pwm < 0：反向转动（后退），绝对值越大转速越快
 *   pwm = 0：电机停止
 * 
 * 工作流程：
 * 1. 参数验证：检查motor_id有效性
 * 2. 范围限制：确保PWM值在[-PWM_MAX_VALUE, +PWM_MAX_VALUE]范围内
 * 3. 方向控制：根据PWM值的正负号设置电机转向
 * 4. 速度控制：根据PWM绝对值设置PWM占空比（影响转速）
 * 5. 记录输出：保存实际输出的PWM值
 * 
 * @param motor_id 电机ID (MOTOR_LF/MOTOR_RF/MOTOR_LB/MOTOR_RB)
 * @param pwm 目标PWM值
 *           范围：[-PWM_MAX_VALUE, +PWM_MAX_VALUE]
 *           正值：正转，负值：反转，0：停止
 * @return void
 * 
 * 工作例子：
 *   motor_set_pwm(MOTOR_LF, 100);   // 左前轮正转，速度为100
 *   motor_set_pwm(MOTOR_LF, -80);   // 左前轮反转，速度为80
 *   motor_set_pwm(MOTOR_LF, 0);     // 左前轮停止
 *   motor_set_pwm(MOTOR_LF, 10000); // 超限自动按PWM_MAX_VALUE限制
 */
void motor_set_pwm(MotorID motor_id, int32_t pwm)
{
    /* 限制后的PWM值（经过范围检查） */
    int32_t limited_pwm = pwm;

    /* ============ 参数验证 ============ */
    /* 检查电机ID的有效性 */
    if(motor_id >= MOTOR_MAX)
    {
        return;  // 无效ID，直接返回
    }

    /* ============ PWM范围限制 ============ */
    /* 确保PWM值不超出硬件和软件定义的范围 */
    if(limited_pwm > PWM_MAX_VALUE)
    {
        /* 正值超限：限制为最大正值，防止硬件损伤 */
        limited_pwm = PWM_MAX_VALUE;
    }
    else if(limited_pwm < -PWM_MAX_VALUE)
    {
        /* 负值超限：限制为最大负值，防止硬件损伤 */
        limited_pwm = -PWM_MAX_VALUE;
    }

    /* ============ 方向控制 ============ */
    /* 根据PWM值的正负号设置电机转向 */
    if(limited_pwm > 0)
    {
        /* PWM为正：设置电机正转 */
        motor_set_direction(motor_id, true);
    }
    else if(limited_pwm < 0)
    {
        /* PWM为负：设置电机反转 */
        motor_set_direction(motor_id, false);
    }
    /* 注意：pwm=0时，不改变方向，仅停止速度输出 */

    /* ============ 速度控制 ============ */
    /* 将有符号PWM值转换为无符号占空比，并输出到PWM模块 */
    pwm_set_duty(g_motor[motor_id].hw.pwm_pin, motor_abs_pwm(limited_pwm));

    /* ============ 记录当前输出值 ============ */
    /* 保存实际输出的PWM值（用于故障检测、调试等） */
    g_motor[motor_id].pwm_out = limited_pwm;
}

/**
 * @brief 重新应用所有电机存储的PWM输出值
 * 
 * 该函数用于恢复之前设置的所有电机速度。
 * 适用于以下场景：
 * - 电机被外部中断后的恢复
 * - 电源管理后重新启用电机
 * - 紧急停止后需要恢复之前的运动状态
 * 
 * @param void
 * @return void
 * 
 * 工作原理：
 * - 遍历所有4个电机
 * - 对每个电机重新调用motor_set_pwm()，输入值为g_motor[i].pwm_out
 * - g_motor[i].pwm_out存储了该电机上一次设置的PWM值
 * 
 * 典型用法：
 *   motor_set_pwm(MOTOR_LF, 100);   // 设置左前轮速度
 *   // ... 发生中断、异常等 ...
 *   motor_set_pwm_all();            // 恢复所有电机速度
 * 
 * 注意：
 *   - 该函数会覆盖当前的PWM设置
 *   - 如果没有事先调用过motor_set_pwm()，则pwm_out为0（停止状态）
 */
void motor_set_pwm_all(void)
{
    /* ============ 遍历所有电机 ============ */
    /* 逐个重新设置四个电机的PWM值 */
    for(int i = 0; i < MOTOR_MAX; i++)
    {
        /* 
         * 重新调用motor_set_pwm()，输入为保存的pwm_out值
         * 这会重新执行方向控制和PWM输出
         */
        motor_set_pwm((MotorID)i, g_motor[i].pwm_out);
    }
}

/**
 * @brief 停止指定电机
 * 
 * 该函数在物理上停止电机的运转。
 * 除了停止PWM输出，还会清除目标速度标志，表示电机不再受控制指令驱动。
 * 
 * 执行步骤：
 * 1. 参数验证：检查motor_id有效性
 * 2. 停止PWM输出：调用motor_set_pwm()设置PWM为0
 * 3. 清除目标速度：表示没有速度指令了
 * 4. 确保pwm_out为0：记录停止状态
 * 
 * @param motor_id 要停止的电机ID (MOTOR_LF/MOTOR_RF/MOTOR_LB/MOTOR_RB)
 * @return void
 * 
 * 使用场景：
 *   motor_stop(MOTOR_LF);  // 停止左前轮
 *   motor_stop(MOTOR_RF);  // 停止右前轮
 *   // 停止单个轮子可能导致小车转弯
 * 
 * 与motor_stop_all()的区别：
 *   - motor_stop()：停止单个电机
 *   - motor_stop_all()：停止所有电机
 * 
 * 注意：
 *   - 该函数会保留运动学信息（speed_now, pos_now）不变
 *   - 只清除控制指令（speed_target, pwm_out）
 */
void motor_stop(MotorID motor_id)
{
    /* ============ 参数验证 ============ */
    /* 检查电机ID有效性 */
    if(motor_id >= MOTOR_MAX)
    {
        return;  // 无效ID，直接返回
    }

    /* ============ 停止PWM输出 ============ */
    /* 调用motor_set_pwm()，设置PWM为0，电机停转（同时处理方向等细节） */
    motor_set_pwm(motor_id, 0);

    /* ============ 清除目标速度 ============ */
    /* 设目标速度为0，表示没有新的速度指令 */
    g_motor[motor_id].speed_target = 0;

    /* ============ 确保输出記錄為0 ============ */
    /* pwm_out应该在motor_set_pwm()中已经设为0，这里再确认一次 */
    g_motor[motor_id].pwm_out = 0;
}

/**
 * @brief 停止所有四个电机
 * 
 * 这是最常用的紧急停止函数。一次性停止整个四轮驱动系统。
 * 小车会立即停止运动。
 * 
 * @param void
 * @return void
 * 
 * 工作原理：
 * - 依次调用motor_stop()，停止四个电机（MOTOR_LF/MOTOR_RF/MOTOR_LB/MOTOR_RB）
 * 
 * 使用场景：
 *   motor_stop_all();  // 紧急制动，小车停止
 * 
 * 典型应用：
 * - main()初始化时：确保小车不会意外启动
 * - 任务结束时：安全停止小车
 * - 异常处理时：紧急刹停
 * - 遥控器释放时：停止小车运动
 * 
 * 执行时间：
 *   极短（四个GPIO操作 + PWM设置），不会产生明显延迟
 */
void motor_stop_all(void)
{
    /* ============ 遍历所有电机并逐个停止 ============ */
    /* 循环停止四个电机，确保小车完全停止 */
    for(int i = 0; i < MOTOR_MAX; i++)
    {
        /* 调用motor_stop()停止第i个电机 */
        motor_stop((MotorID)i);
    }
}

/* ==================== 运动状态读写函数 ==================== */
/* 下列函数用于读取和修改电机的运动参数（速度、位置等） */

/**
 * @brief 获取指定电机的当前位置
 * 
 * 读取电机的实时位置值。位置通常由编码器测量，单位为脉冲计数。
 * 
 * @param motor_id 电机ID (MOTOR_LF/MOTOR_RF/MOTOR_LB/MOTOR_RB)
 * @return 当前位置值（编码器脉冲计数）
 * 
 * 注意：
 *   - 返回值为有符号32位整数，可以表示正反两个方向的位移
 *   - 正值：正向累计位移
 *   - 负值：反向累计位移
 *   - 绝对值：移动距离的衡量
 */
int32_t get_pos_now(MotorID motor_id)
{
    /* 直接返回电机的当前位置值 */
    return g_motor[motor_id].pos_now;
}

/**
 * @brief 设置指定电机的当前位置
 * 
 * 用于手动更新电机的位置值。通常在以下情况使用：
 * - 初始化位置（如位置回零）
 * - 根据编码器中断更新位置
 * - 位置融合运算（多传感器）
 * 
 * @param motor_id 电机ID (MOTOR_LF/MOTOR_RF/MOTOR_LB/MOTOR_RB)
 * @param position 新的位置值（编码器脉冲计数）
 * @return void
 */
void change_pos_now(MotorID motor_id, int32_t position)
{
    /* 将电机位置更新为新值 */
    g_motor[motor_id].pos_now = position;
}

/**
 * @brief 获取指定电机的当前速度
 * 
 * 读取电机的实时转速。速度通常由编码器和定时器测量，单位为脉冲数/时间。
 * 
 * @param motor_id 电机ID (MOTOR_LF/MOTOR_RF/MOTOR_LB/MOTOR_RB)
 * @return 当前速度值（脉冲/采样周期）
 * 
 * 说明：
 *   - 正值：正向转动
 *   - 负值：反向转动
 *   - 绝对值：转速的大小
 *   - 0：电机停止
 */
int32_t get_speed_now(MotorID motor_id)
{
    /* 直接返回电机的当前速度值 */
    return g_motor[motor_id].speed_now;
}

/**
 * @brief 设置指定电机的当前速度
 * 
 * 用于更新电机的速度反馈值，通常由ISR（编码器中断处理程序）调用。
 * 该函数与motor_set_pwm()不同：
 * - change_speed_now()：更新当前速度状态（反馈）
 * - motor_set_pwm()：设置目标速度命令（控制）
 * 
 * @param motor_id 电机ID (MOTOR_LF/MOTOR_RF/MOTOR_LB/MOTOR_RB)
 * @param speed 新的速度值（脉冲/采样周期）
 * @return void
 */
void change_speed_now(MotorID motor_id, int32_t speed)
{
    /* 将电机速度更新为新值 */
    g_motor[motor_id].speed_now = speed;
}

/**
 * @brief 设置指定电机的目标速度
 * 
 * 设置电机期望达到的速度值。这个值通常用于PID控制器。
 * 控制系统会尝试让speed_now逼近speed_target。
 * 
 * @param motor_id 电机ID (MOTOR_LF/MOTOR_RF/MOTOR_LB/MOTOR_RB)
 * @param speed 目标速度值（脉冲/采样周期）
 * @return void
 * 
 * 工作流程：
 *   用户设置speed_target
 *     ↓
 *   PID控制器计算偏差 (speed_target - speed_now)
 *     ↓
 *   PID输出调整PWM值
 *     ↓
 *   电机加速/减速逼近目标速度
 */
void change_speed_target(MotorID motor_id, int32_t speed)
{
    /* 将电机目标速度设为新值 */
    g_motor[motor_id].speed_target = speed;
}

/**
 * @brief 设置指定电机的目标位置
 * 
 * 设置电机期望到达的位置。该值通常用于位置控制算法。
 * 运动控制算法会尝试让pos_now逼近pos_target。
 * 
 * 应用场景：
 *   - 精确转动：设置目标位置为100，让轮子转到第100个脉冲点
 *   - 轨迹规划：逐步更新pos_target来实现曲线轨迹
 *   - 路径跟踪：根据GPS或其他定位设备更新目标位置
 * 
 * @param motor_id 电机ID (MOTOR_LF/MOTOR_RF/MOTOR_LB/MOTOR_RB)
 * @param position 目标位置值（编码器脉冲计数）
 * @return void
 * 
 * 典型用法：
 *   change_pos_target(MOTOR_LF, 500);  // 让左前轮移动到第500个脉冲点
 */
void change_pos_target(MotorID motor_id, int32_t position)
{
    /* 将电机目标位置设为新值 */
    g_motor[motor_id].pos_target = position;
}

/*
 * Read back the stored speed target for one wheel.
 *
 * This accessor is part of the task-4 refactor. It lets upper layers query the
 * current target without reaching into g_motor[] directly.
 */
int32_t get_speed_target(MotorID motor_id)
{
    return g_motor[motor_id].speed_target;
}

/*
 * Read back the stored position target for one wheel.
 */
int32_t get_pos_target(MotorID motor_id)
{
    return g_motor[motor_id].pos_target;
}

/*
 * Car-level aggregate state helpers.
 *
 * These functions do not introduce new behavior. Their only purpose is to make
 * cross-module access explicit so later refactors can replace the g_car global
 * more easily.
 */
int32_t car_get_speed_now(void)
{
    return g_car.speed_now;
}

void car_set_speed_now(int32_t speed)
{
    g_car.speed_now = speed;
}

int32_t car_get_angle_now(void)
{
    return g_car.angle_now;
}

void car_set_angle_now(int32_t angle)
{
    g_car.angle_now = angle;
}

int32_t car_get_x_now(void)
{
    return g_car.x_now;
}

int32_t car_get_y_now(void)
{
    return g_car.y_now;
}

void car_set_position_now(int32_t x, int32_t y)
{
    g_car.x_now = x;
    g_car.y_now = y;
}

int32_t car_get_x_target(void)
{
    return g_car.x_target;
}

int32_t car_get_y_target(void)
{
    return g_car.y_target;
}

void car_set_position_target(int32_t x, int32_t y)
{
    g_car.x_target = x;
    g_car.y_target = y;
}
