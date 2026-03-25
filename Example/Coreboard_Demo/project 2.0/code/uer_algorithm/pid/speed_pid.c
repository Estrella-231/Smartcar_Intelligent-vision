#include "speed_pid.h"

PID_Pram_t g_speed_pid[MOTOR_MAX] = {0};

/*
 * 重置单个车轮速度环的历史状态。
 *
 * 详细说明：
 * 任务 5 是一个台架测试，我们会反复对单个车轮执行启动、停止、手动阻挡和释放。
 * 如果不在每轮测试之间清空控制器状态，下一轮测试可能会带着残留积分项或过期的 PWM 基线开始，
 * 从而导致观察到的现象失真。
 */
void speed_pid_reset(MotorID motor_id)
{
    if(motor_id >= MOTOR_MAX)
    {
        return;
    }

    g_speed_pid[motor_id].integral = 0;
    g_speed_pid[motor_id].err = 0;
    g_speed_pid[motor_id].last_error = 0;
    g_speed_pid[motor_id].output = 0;
    g_speed_pid[motor_id].last_output = 0;
}

/*
 * 重置全部车轮速度环状态。
 */
void speed_pid_reset_all(void)
{
    for(int i = 0; i < MOTOR_MAX; i++)
    {
        speed_pid_reset((MotorID)i);
    }
}

/*
 * 增量式速度 PID。
 *
 * 这里采用增量式的原因：
 * 用户明确希望把控制器输出视为一个调整量，
 * 然后叠加到上一次 PWM 指令上：
 *
 *   pwm_adjust = PID(error)
 *   pwm_cmd    = last_pwm_cmd + pwm_adjust
 *
 * 这意味着当车轮被阻挡时会产生正的速度误差，
 * 进而得到正的 pwm_adjust，再继续抬高 PWM 指令，
 * 直到车轮恢复转速或者达到配置的输出上限。
 */
int32_t speed_pid_calc(MotorID motor_id,
                       int32_t target_speed,
                       int32_t current_speed,
                       uint32_t period_ms)
{
    int32_t pwm_adjust;
    int32_t pwm_command;
    float derivative;

    if(motor_id >= MOTOR_MAX || period_ms == 0)
    {
        return 0;
    }

    /*
     * 将最新的目标值和反馈值写回共享车轮状态，
     * 这样其他调试代码无需再维护自己的副本，也能直接查看同一组数据。
     */
    change_speed_target(motor_id, target_speed);
    change_speed_now(motor_id, current_speed);

    /*
     * 如果目标本身就是 0，则强制车轮停止并清空全部控制器记忆状态。
     * 这样可以避免车轮本应空闲时发生积分累积。
     */
    if(target_speed == 0)
    {
        speed_pid_reset(motor_id);
        g_motor[motor_id].pwm_out = 0;
        return 0;
    }

    /*
     * 速度误差仍按任务 5 中最直接的形式定义：
     * 当前控制周期内统计到的编码器脉冲数差值。
     */
    g_speed_pid[motor_id].err = target_speed - current_speed;

    /*
     * 接近 0 的小误差按噪声处理。
     * 对任务 5 来说，这可以避免车轮因编码器抖动而在目标附近来回摆动。
     */
    if(abs(g_speed_pid[motor_id].err) < SPEED_DEAD_ZONE)
    {
        g_speed_pid[motor_id].err = 0;
    }

    /*
     * 积分项按真实时间累加，
     * 这样后续即使调整控制周期，也不会悄悄改变控制器的等效强度。
     */
    g_speed_pid[motor_id].integral +=
        (int32_t)((float)g_speed_pid[motor_id].err * (float)period_ms / 1000.0f);
    g_speed_pid[motor_id].integral =
        LIMIT_ABS(g_speed_pid[motor_id].integral, SPEED_PID_I_LIMIT);

    /*
     * 任务 5 中的反馈单位本来就是“每个控制周期内的编码器脉冲数”，
     * 而不是 mm/s 这类连续物理速度单位。
     *
     * 因此微分项刻意采用相邻两次采样之间的误差变化量，
     * 而不是再换算成“每秒变化率”。
     * 否则会过度放大量化噪声，使单轮台架测试更难稳定。
     */
    derivative = (float)(g_speed_pid[motor_id].err - g_speed_pid[motor_id].last_error);

    /*
     * 这里得到的是本周期的 PWM 增量调整值，
     * 还不是最终的 PWM 指令。
     */
    pwm_adjust =
        (int32_t)(SPEED_PID_KP * (float)g_speed_pid[motor_id].err +
                  SPEED_PID_KI * (float)g_speed_pid[motor_id].integral +
                  SPEED_PID_KD * derivative);

    /*
     * 这是用户为任务 5 指定的行为：
     * “将 PID 输出直接叠加到前一次 PWM 值上”。
     */
    pwm_command = g_speed_pid[motor_id].last_output + pwm_adjust;

    /*
     * 遵守当前配置的有符号 PWM 限幅，
     * 防止控制器输出超过当前台架测试允许的安全范围。
     */
    if(pwm_command > SPEED_PWM_MAX_OUTPUT)
    {
        pwm_command = SPEED_PWM_MAX_OUTPUT;
    }
    else if(pwm_command < SPEED_PWM_MIN_OUTPUT)
    {
        pwm_command = SPEED_PWM_MIN_OUTPUT;
    }

    /*
     * 当车轮被要求运动，但累计后的 PWM 指令仍不足以克服电机死区时，
     * 注入一个最小有符号驱动力。
     * 这样可以让台架测试从静止启动时更可靠。
     */
    if(pwm_command == 0 && g_speed_pid[motor_id].err != 0)
    {
        pwm_command = (g_speed_pid[motor_id].err > 0) ? SPEED_DEAD_ZONE : -SPEED_DEAD_ZONE;
    }

    /*
     * Do not force a per-wheel anti-stall floor here.
     *
     * We tried that approach during ground testing, but it made the four wheels
     * react differently when one wheel momentarily lagged. The result was
     * diagonal drift and chassis sway.
     *
     * Ground anti-stall is now handled at the higher execution layer by:
     * - keeping a moderate chassis-level minimum move speed
     * - accepting a segment once the car is close enough to the blue target point
     *
     * That keeps all four wheels on the same chassis command instead of letting
     * one wheel get an extra private boost.
     */

    g_speed_pid[motor_id].output = pwm_adjust;
    g_speed_pid[motor_id].last_error = g_speed_pid[motor_id].err;
    g_speed_pid[motor_id].last_output = pwm_command;
    g_motor[motor_id].pwm_out = pwm_command;

    return pwm_command;
}
