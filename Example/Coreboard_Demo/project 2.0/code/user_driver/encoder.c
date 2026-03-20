/**
 * @file encoder.c
 * @brief 编码器读取驱动 - 获取四个电机的转速和位置反馈
 * 
 * 本文件实现了编码器数据采集和处理的核心功能：
 * - 初始化四个编码器（脉冲+方向模式）
 * - 周期性读取编码器计数值
 * - 对编码器数据进行符号归一化处理（消除硬件接线影响）
 * - 计算电机位置和速度用于反馈控制
 * - 提供编码器数据的读取接口
 * 
 * 工作模式：编码器采用"脉冲+方向"模式
 * - 脉冲信号(LSB)：计数脉冲数，决定位移大小
 * - 方向信号(DIR)：决定转向（正/反）
 * - 由SeekFree库处理底层采集和方向判断
 */

#include "encoder.h"

/* ==================== 全局变量定义 ==================== */
/**
 * @brief 四个编码器的计数数据
 * 
 * 存储从最近一个采样周期内，每个电机的编码器增量值
 * volatile关键字表示该变量可能被中断处理程序修改
 */
volatile int16_t encoder_data[MOTOR_MAX] = {0};

/* ==================== 静态辅助函数定义 ==================== */

/**
 * @brief 获取指定电机编码器的"正转"符号
 * 
 * 由于编码器安装方向可能与电机接线方向不一致，导致编码器反馈的正负号
 * 与实际硬件方向不符。本函数返回每个电机需要的符号调整因子。
 * 
 * 符号值含义：
 *   1：不需要符号反转（编码器正转与电机转向一致）
 *  -1：需要符号反转（编码器正转与电机转向相反）
 * 
 * @param motor_id 电机ID (MOTOR_LF/MOTOR_RF/MOTOR_LB/MOTOR_RB)
 * @return 符号调整因子 (1 或 -1)
 * 
 * 例如：
 *   - ENCODER1_FORWARD_SIGN=1：左前轮编码器正增量表示正转
 *   - ENCODER2_FORWARD_SIGN=-1：右前轮编码器负增量表示正转
 */
static int16_t encoder_forward_sign(MotorID motor_id)
{
    switch(motor_id)
    {
        /* 左前轮：返回编码器1的正转符号因子 */
        case MOTOR_LF: return ENCODER1_FORWARD_SIGN;
        /* 右前轮：返回编码器2的正转符号因子 */
        case MOTOR_RF: return ENCODER2_FORWARD_SIGN;
        /* 左后轮：返回编码器3的正转符号因子 */
        case MOTOR_LB: return ENCODER3_FORWARD_SIGN;
        /* 右后轮：返回编码器4的正转符号因子 */
        case MOTOR_RB: return ENCODER4_FORWARD_SIGN;
        /* 默认：返回1（容错处理） */
        default:       return 1;
    }
}

/**
 * @brief 根据电机ID获取对应的编码器索引
 * 
 * 建立电机编号与编码器硬件索引的映射关系。每个电机配备一个编码器。
 * 
 * 映射关系：
 *   MOTOR_LF (电机0) ←→ ENCODER_1 (编码器1)
 *   MOTOR_RF (电机1) ←→ ENCODER_2 (编码器2)
 *   MOTOR_LB (电机2) ←→ ENCODER_3 (编码器3)
 *   MOTOR_RB (电机3) ←→ ENCODER_4 (编码器4)
 * 
 * @param motor_id 电机ID (MOTOR_LF/MOTOR_RF/MOTOR_LB/MOTOR_RB)
 * @return 对应的编码器索引 (ENCODER_1/ENCODER_2/ENCODER_3/ENCODER_4)
 */
static encoder_index_enum encoder_channel_of(MotorID motor_id)
{
    switch(motor_id)
    {
        /* 左前轮对应编码器1 */
        case MOTOR_LF: return ENCODER_1;
        /* 右前轮对应编码器2 */
        case MOTOR_RF: return ENCODER_2;
        /* 左后轮对应编码器3 */
        case MOTOR_LB: return ENCODER_3;
        /* 右后轮对应编码器4 */
        case MOTOR_RB: return ENCODER_4;
        /* 默认：返回编码器1（容错处理） */
        default:       return ENCODER_1;
    }
}

/**
 * @brief 对编码器原始计数进行归一化处理
 * 
 * 将硬件编码器的原始计数值处理成标准化的增量值。包括两个处理步骤：
 * 1. 符号调整：根据编码器安装方向修正符号，使所有电机的"正命令"都对应"正反馈"
 * 2. 零点过滤：消除噪音导致的微小抖动，避免速度计算时的不稳定
 * 
 * 处理效果：
 * - 经过处理后，正向PWM指令与正向编码器增量总是对应的物理方向相同
 * - 消除零附近的高频抖动，使速度环控制更稳定
 * 
 * @param motor_id 电机ID (MOTOR_LF/MOTOR_RF/MOTOR_LB/MOTOR_RB)
 * @param raw_count 编码器的原始计数值（来自硬件）
 * @return 归一化后的增量值
 *         - 范围：[-32767, 32767]（int16_t）
 *         - 正值：电机正向旋转
 *         - 负值：电机反向旋转
 *         - 接近0的小值：被过滤为0
 * 
 * 工作流程：
 *   1. 以整数形式扩展raw_count为int32_t（防止乘法溢出）
 *   2. 使用符号因子调整编码器数据的正负号
 *   3. 对绝对值小于SPEED_ZERO_OFFSET的值清零（消除噪音）
 *   4. 转换回int16_t并返回
 * 
 * 例子：
 *   假设ENCODER1_FORWARD_SIGN=1, SPEED_ZERO_OFFSET=2
 *   - raw_count=10  → normalized = 1*10 = 10 (保留)
 *   - raw_count=-1  → normalized = 1*(-1) = -1, abs(-1)<=2 → 0 (过滤)
 *   
 *   假设ENCODER2_FORWARD_SIGN=-1
 *   - raw_count=10  → normalized = -1*10 = -10 (反向)
 *   - raw_count=-10 → normalized = -1*(-10) = 10 (反向)
 */
static int16_t encoder_normalize_delta(MotorID motor_id, int16_t raw_count)
{
    /* 扩展为int32_t避免后续乘法溢出 */
    int32_t normalized = raw_count;

    /* ============ 符号调整 ============ */
    /* 
     * 根据编码器安装方向修正符号。每个车轮的编码器可能有不同的安装方向，
     * 需要通过符号因子将其统一。处理后的效果：
     * - 正向PWM指令 → 正向编码器增量
     * - 反向PWM指令 → 反向编码器增量
     */
    normalized *= encoder_forward_sign(motor_id);

    /* ============ 零点过滤 ============ */
    /* 
     * 消除零附近的微小抖动，防止编码器噪声导致速度计算不稳定。
     * 策略：
     *   - 如果增量的绝对值 <= SPEED_ZERO_OFFSET，则认为是噪声，清零
     *   - 保留绝对值 > SPEED_ZERO_OFFSET 的有效数据
     * 
     * SPEED_ZERO_OFFSET的设置要点：
     *   - 太大：丢失有效的低速数据，卡死
     *   - 太小：噪声过多，速度波动
     *   - 推荐：比编码器典型噪声稍大一点
     */
    if(abs(normalized) <= SPEED_ZERO_OFFSET)
    {
        normalized = 0;  /* 清零噪声脉冲 */
    }

    return (int16_t)normalized;  /* 缩回int16_t并返回 */
}

/**
 * @brief 重置所有编码器和电机的位置、速度数据
 * 
 * 这是一个复合重置函数，同时清除硬件编码器计数器和软件端的运动状态，
 * 确保系统从干净的初始状态开始运行。适用于：
 * - 系统启动后的初始化
 * - 软件重新初始化（如任务重启）
 * - 位置归零操作
 * 
 * 重置范围：
 * - 硬件编码器计数器：清零（所有脉冲计数器复位）
 * - 编码器数据缓冲：清零（encoder_data[i] = 0）
 * - 电机位置：清零（pos_now, pos_last）
 * - 电机速度：清零（speed_now, speed_last）
 * 
 * 工作流程：
 *   对每个电机（i=0到3）执行：
 *   1. 调用encoder_clear_count()清空硬件编码器计数
 *   2. 清空速度缓冲（encoder_data[i] = 0）
 *   3. 同时重置电机端的软件累加器（确保起点一致）
 * 
 * 重要性：
 *   - 硬件和软件的起点必须同步，否则位置累计会产生偏差
 *   - 系统启动时的第一个控制周期才能从脏状态启动
 * 
 * @param void
 * @return void
 */
void encoder_reset_all(void)
{
    /* ============ 逐个重置四个编码器和相关软件状态 ============ */
    for(int i = 0; i < MOTOR_MAX; i++)
    {
        /* 
         * 清空硬件编码器计数器。
         * 硬件会停止累计脉冲，计数值返回0
         */
        encoder_clear_count(encoder_channel_of((MotorID)i));
        
        /* 清空速度增量缓冲 */
        encoder_data[i] = 0;

        /* ============ 同时重置电机的软件端累加器 ============ */
        /* 
         * 硬件编码器和软件电机控制是"生产者-消费者"关系：
         *   硬件编码器（生产） → encoder_data → 电机控制（消费）
         * 
         * 两端必须同时从零起始，避免位置漂移。
         * 如果只清编码器不清电机，或反之，会导致位置误差累计。
         */
        
        /* 当前位置清零（编码器零点） */
        g_motor[i].pos_now = 0;
        /* 上一时刻位置清零（用于速度计算） */
        g_motor[i].pos_last = 0;
        /* 当前速度清零 */
        g_motor[i].speed_now = 0;
        /* 上一时刻速度清零（用于加速度计算） */
        g_motor[i].speed_last = 0;
    }
}

/**
 * @brief 初始化四个编码器
 * 
 * 这是编码器驱动的入口函数，必须在任何编码器操作之前调用。
 * 初始化所有硬件编码器的脉冲和方向输入引脚，并重置软件状态。
 * 
 * 硬件配置：
 *   编码器1 (左前轮)：LSB引脚=ENCODER_1_LSB, DIR引脚=ENCODER_1_DIR
 *   编码器2 (右前轮)：LSB引脚=ENCODER_2_LSB, DIR引脚=ENCODER_2_DIR
 *   编码器3 (左后轮)：LSB引脚=ENCODER_3_LSB, DIR引脚=ENCODER_3_DIR
 *   编码器4 (右后轮)：LSB引脚=ENCODER_4_LSB, DIR引脚=ENCODER_4_DIR
 * 
 * 编码器类型：\"脉冲+方向\"模式（Pulse + Direction）
 *   - LSB（最低有效位/脉冲）：每转过一个标记产生一个脉冲
 *   - DIR（方向）：高电平表示正转，低电平表示反转
 * 
 * 符号处理说明：
 *   本驱动不直接信任硬件的符号（DIR引脚状态）。
 *   符号归一化由encoder_read_data()中的encoder_normalize_delta()完成。
 *   这样的好处是：
 *   - 可以通过改变ENCODER*_FORWARD_SIGN宏值来调整接线，无需改逻辑代码
 *   - 实现了硬件和软件的解耦，提高灵活性
 * 
 * @param void
 * @return void
 * 
 * 调用时机：
 *   在motor_driver_init()之后、encoder_read_data()之前调用
 *   通常在main()初始化阶段
 * 
 * 典型调用顺序：
 *   void main(void) {
 *       // ... 其他初始化 ...
 *       motor_driver_init();   // 初始化电机驱动
 *       Encoder_Init();        // 初始化编码器
 *       // 现在可以调用encoder_read_data()和其他编码器函数
 *   }
 */
void Encoder_Init(void)
{
    /* ============ 初始化四个编码器的硬件接口 ============ */
    /* 
     * 每个编码器使用SeekFree库提供的\"脉冲+方向\"初始化函数。
     * 参数：
     *   第1个：编码器硬件索引
     *   第2个：脉冲计数引脚(LSB)
     *   第3个：方向判断引脚(DIR)
     */
    
    /* 编码器1（左前轮）：初始化脉冲和方向输入 */
    encoder_dir_init(ENCODER_1, ENCODER_1_LSB, ENCODER_1_DIR);
    /* 编码器2（右前轮）：初始化脉冲和方向输入 */
    encoder_dir_init(ENCODER_2, ENCODER_2_LSB, ENCODER_2_DIR);
    /* 编码器3（左后轮）：初始化脉冲和方向输入 */
    encoder_dir_init(ENCODER_3, ENCODER_3_LSB, ENCODER_3_DIR);
    /* 编码器4（右后轮）：初始化脉冲和方向输入 */
    encoder_dir_init(ENCODER_4, ENCODER_4_LSB, ENCODER_4_DIR);

    /* ============ 重置所有编码器和电机状态 ============ */
    /* 
     * 初始化完硬件后，清空所有计数器和软件状态。
     * 确保从干净状态开始，避免初始化过程中的干扰。
     */
    encoder_reset_all();
}

/**
 * @brief 读取所有编码器数据并更新电机位置信息
 * 
 * 这是编码器驱动的数据采样函数，通常由上层定时任务周期性调用。
 * 获取每个电机在采样周期内的位移增量，并累计到电机的位置变量中。
 * 
 * 工作流程：
 *   对每个电机（i=0到3）执行：
 *   1. 从硬件编码器读取本采样周期内的计数值（脉冲数）
 *   2. 立即清零硬件计数器（为下一个周期做准备）
 *   3. 对原始计数进行归一化处理（符号调整 + 零点过滤）
 *   4. 保存在encoder_data[i]中（供上层读取）
 *   5. 累加到电机累计位置(pos_now)，形成绝对位置
 * 
 * 关键设计点：
 *   - \"先读后清\"策略：确保不丢失脉冲数据
 *   - 归一化处理：屏蔽编码器安装和接线的不一致性
 *   - 浮动基准：每个周期相对上一时刻的增量，避免绝对位置漂移
 * 
 * 数据一致性保证：
 *   - encoder_data[i]：本采样周期的增量（用于速度计算）
 *   - pos_now：累计位置（从系统启动或上次复位以来的总位移）
 *   - 两者保持同步，确保上层闭环控制的稳定性
 * 
 * @param void
 * @return void
 * 
 * 调用要求：
 *   - 必须在Encoder_Init()之后调用
 *   - 建议由定时任务（如10ms周期）周期性调用
 *   - 调用周期越快，编码器数据更新越及时，控制更精确
 * 
 * 调用频率建议：
 *   - 高精度控制：1~5ms一次
 *   - 普通控制：10ms左右
 *   - 低速控制：≤20ms（太慢会影响速度反馈精度）
 * 
 * 典型应用（中断中周期调用）：
 *   // 10ms定时中断处理函数
 *   void Timer_ISR(void) {
 *       encoder_read_data();      // 读取编码器
 *       // ... 可能的速度计算、PID控制 ...
 *       motor_set_pwm_all();      // 应用PWM输出
 *   }
 */
void encoder_read_data(void)
{
    /* ============ 逐个读取和处理四个编码器 ============ */
    for(int i = 0; i < MOTOR_MAX; i++)
    {
        /* 本次采样的原始脉冲计数值 */
        int16_t raw_count;
        /* 电机ID（从数组索引转换） */
        MotorID motor_id = (MotorID)i;
        /* 对应的编码器硬件索引 */
        encoder_index_enum encoder_index = encoder_channel_of(motor_id);

        /* ============ 从硬件读取计数值 ============ */
        /* 
         * 获取自上次清零以来的脉冲计数。
         * 返回值是有符号的整数（包含方向信息）：
         *   - 正值：正向转动的脉冲数
         *   - 负值：反向转动的脉冲数
         */
        raw_count = encoder_get_count(encoder_index);
        
        /* ============ 立即清零硬件计数器 ============ */
        /* 
         * \"先读后清\"时序保证：
         * 1. 该指令执行后，硬件将重新开始累计脉冲
         * 2. 在下一次encoder_read_data()调用前，脉冲都会累计到硬件计数器
         * 3. 这样确保每个采样周期的数据不会丢失
         */
        encoder_clear_count(encoder_index);

        /* ============ 对原始计数进行符号和噪声处理 ============ */
        /* 
         * encoder_normalize_delta()的工作：
         *   1. 符号调整（乘以ENCODER*_FORWARD_SIGN）
         *      - 使所有电机的正命令都对应正反馈
         *      - 不同编码器接线差异对上层透明
         *   2. 零点过滤（消除±SPEED_ZERO_OFFSET内的噪声）
         *      - 避免噪声导致的速度振荡
         *      - 保留有效的转速信息
         */
        encoder_data[i] = encoder_normalize_delta(motor_id, raw_count);
        
        /* ============ 累加到电机累计位置 ============ */
        /* 
         * 电机的总位移 = 上一时刻位置 + 本采样周期的增量
         * 
         * 精度考虑：
         *   - 每个采样周期都累加，不会丢失整数脉冲
         *   - 系统复位后位置重置（由encoder_reset_all()保证）
         *   - 绝对位置可能会因长时间运行溢出，需定期归零
         */
        change_pos_now(motor_id, get_pos_now(motor_id) + encoder_data[i]);
    }
}

/**
 * @brief 获取指定电机最近一个采样周期内的编码器数据
 * 
 * 提供上层应用对编码器反馈的读取接口。返回两次encoder_read_data()调用
 * 之间的位移增量（不是累计位置）。
 * 
 * 返回值含义：
 *   - 正值：电机在本采样周期内的正向脉冲数
 *   - 负值：电机在本采样周期内的反向脉冲数
 *   - 范围：[-32767, 32767]（int16_t）
 *   - 0：电机停止或噪声被过滤
 * 
 * @param motor_id 电机ID (MOTOR_LF/MOTOR_RF/MOTOR_LB/MOTOR_RB)
 * @return 本采样周期的编码器增量值（已进行符号归一化和噪声过滤）
 * 
 * 常见用途：
 *   - 速度计算：speed = encoder_data / 采样周期
 *   - PID反馈项：e = speed_target - speed_now
 *   - 故障检测：长时间speed_now=0可能表示电机卡死
 *   - 调试信息：打印编码器原始数据用于问题排查
 * 
 * 数据特点：
 *   - encoder_data[i]在每次encoder_read_data()后更新
 *   - 是增量值（相对量），不是绝对位置
 *   - 已消除接线反向的影响（符号归一化）
 *   - 已消除芯片噪声（零点过滤）
 * 
 * 精确性注意：
 *   - 脉冲计数采样于硬件整数值，有±1计数的量化误差
 *   - 极低速时（<1脉冲/周期），可能被零点过滤器清除
 *   - 过滤阈值由SPEED_ZERO_OFFSET宏定义，可根据噪声特性调整
 * 
 * 例子：
 *   int16_t feedback = get_encoder_data(MOTOR_LF);  // 获取左前轮反馈
 *   float speed = feedback * 100.0 / 10;            // 10ms周期时的mHz速度
 */
int16_t get_encoder_data(MotorID motor_id)
{
    /* 直接返回该电机的编码器增量值 */
    return encoder_data[motor_id];
}
