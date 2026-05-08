#include "mecanum.h"

/*
 * =============================================================================
 * 麦克纳姆轮运动学模块
 * =============================================================================
 *
 * 概述:
 * 本模块实现四轮麦克纳姆机器人的运动学模型，提供逆运动学（车体速度 -> 车轮速度）
 * 和正运动学（车轮速度 -> 车体速度）的坐标变换。
 *
 * 车轮布局（俯视图）:
 *     +------------------+
 *     |   LF      RF    |   <-- 左前, 右前
 *     |   LB      RB    |   <-- 左后, 右后
 *     +------------------+
 *
 * 坐标约定:
 * - vx: 横向速度（正 = 向右）
 * - vy: 纵向速度（正 = 前进）
 * - vz: 偏航角速度（正 = 顺时针）
 *
 * 单位约定:
 * - 速度单位: mm/s（毫米/秒）
 * - 脉冲单位: PPP（每控制周期脉冲数）
 *
 * 设计说明:
 * 1. 速度环在编码器脉冲空间运行；几何比例常数尚未调校，因此车轮指令直接写入
 *    脉冲目标值。
 * 2. 正运动学在此阶段仅返回车体坐标系下的线速度(vx, vy)；全局坐标系的映射
 *    由IMU偏航角完成。
 * 3. 保留旧的方向枚举接口作为兼容封装层。
 *
 * 依赖函数/变量:
 * - change_speed_target() : 设置车轮速度目标（脉冲/周期）
 * - get_speed_now()       : 读取当前车轮速度（脉冲/周期）
 * - g_motor[]             : 电机状态结构体（pos_now, pos_last, speed_now）
 * - car_set_speed_now()   : 更新旧版车速状态
 *
 * =============================================================================
 */

/*
 * Task 7 general inverse kinematics.
 *
 * Wheel order:
 * - LF = left front
 * - RF = right front
 * - LB = left back
 * - RB = right back
 *
 * Command convention used here:
 * - vx_cmd > 0: move right
 * - vy_cmd > 0: move forward
 * - vz_cmd > 0: rotate clockwise
 *
 * Standard wheel target composition under this convention:
 * - LF =  vy + vx + vz
 * - RF =  vy - vx - vz
 * - LB =  vy - vx + vz
 * - RB =  vy + vx - vz
 *
 * This mapping satisfies the already validated project conventions:
 * - forward  => four wheels all positive
 * - right    => LF/RB positive, RF/LB negative
 * - clockwise rotation => left wheels positive, right wheels negative
 *
 * At the current project stage these values are written directly as wheel speed
 * targets in encoder-pulse space, because Task 6 has already verified the speed
 * loop in that space and the physical geometry constants are not yet tuned.
 */
void Mecanum_inverse_kinematics(int32_t vx_cmd, int32_t vy_cmd, int32_t vz_cmd)
{
    change_speed_target(MOTOR_LF, vy_cmd + vx_cmd + vz_cmd);
    change_speed_target(MOTOR_RF, vy_cmd - vx_cmd - vz_cmd);
    change_speed_target(MOTOR_LB, vy_cmd - vx_cmd + vz_cmd);
    change_speed_target(MOTOR_RB, vy_cmd + vx_cmd - vz_cmd);
}

/*
 * Round-2 body forward kinematics.
 *
 * This is the direct inverse of the wheel-composition rule already validated in
 * Task 7:
 * - LF = vy + vx + vz
 * - RF = vy - vx - vz
 * - LB = vy - vx + vz
 * - RB = vy + vx - vz
 *
 * Solving that system for the chassis linear velocity gives:
 * - vx = (LF - RF - LB + RB) / 4
 * - vy = (LF + RF + LB + RB) / 4
 *
 * The function only returns body-frame linear velocity because round two uses
 * IMU yaw for global-frame mapping and does not yet depend on wheel-derived wz.
 */
void Mecanum_forward_kinematics_mmps(int32_t lf_mmps,
                                     int32_t rf_mmps,
                                     int32_t lb_mmps,
                                     int32_t rb_mmps,
                                     int32_t *vx_body_mmps,
                                     int32_t *vy_body_mmps)
{
    if(vx_body_mmps != NULL)
    {
        *vx_body_mmps = (lf_mmps - rf_mmps - lb_mmps + rb_mmps) / 4;
    }

    if(vy_body_mmps != NULL)
    {
        *vy_body_mmps = (lf_mmps + rf_mmps + lb_mmps + rb_mmps) / 4;
    }
}

/*
 * Convert encoder pulse feedback from one control period into wheel linear
 * speed in mm/s.
 */
int32_t Mecanum_pulse_per_period_to_mmps(int32_t pulse_per_period, uint32_t period_ms)
{
    if(period_ms == 0)
    {
        return 0;
    }

    return (int32_t)(((float)pulse_per_period / PULSE_PER_MM) * 1000.0f / (float)period_ms);
}

/*
 * Convert a desired body linear speed in mm/s into the wheel-command unit used
 * by the already tuned speed loop: encoder pulses per control period.
 */
int32_t Mecanum_mmps_to_pulse_per_period(int32_t velocity_mmps, uint32_t period_ms)
{
    return (int32_t)(((float)velocity_mmps * PULSE_PER_MM * (float)period_ms) / 1000.0f);
}

/*
 * Compatibility wrapper for the older direction-based motion interface.
 *
 * The older project code described motion as:
 * - a coarse direction enum
 * - one scalar speed magnitude
 *
 * Task 7 needs the more general (vx, vy, vz) form, so this wrapper now maps the
 * old interface into the new inverse-kinematics function instead of duplicating
 * wheel formulas in two places.
 */
void Mecanum_entirety2part(MoveDirection car_direction, int32_t car_target_speed)
{
    switch(car_direction)
    {
        case MOVE_STOP:
            Mecanum_inverse_kinematics(0, 0, 0);
            break;

        case MOVE_FORWARD:
            Mecanum_inverse_kinematics(0, car_target_speed, 0);
            break;

        case MOVE_BACKWARD:
            Mecanum_inverse_kinematics(0, -car_target_speed, 0);
            break;

        case MOVE_LEFT:
            Mecanum_inverse_kinematics(-car_target_speed, 0, 0);
            break;

        case MOVE_RIGHT:
            Mecanum_inverse_kinematics(car_target_speed, 0, 0);
            break;

        case MOVE_ROTATE_CW:
            Mecanum_inverse_kinematics(0, 0, car_target_speed);
            break;

        case MOVE_ROTATE_CCW:
            Mecanum_inverse_kinematics(0, 0, -car_target_speed);
            break;

        default:
            Mecanum_inverse_kinematics(0, 0, 0);
            break;
    }
}

/*
 * Rough legacy forward-kinematics helper.
 *
 * The current project only uses this function to synthesize one scalar vehicle
 * quantity for older code paths. It is not a full pose/velocity estimator.
 */
void Mecanum_part2entirety(MoveDirection car_direction)
{
    int32_t lf = get_speed_now(MOTOR_LF);
    int32_t rf = get_speed_now(MOTOR_RF);
    int32_t lb = get_speed_now(MOTOR_LB);
    int32_t rb = get_speed_now(MOTOR_RB);

    int32_t vx_cmd = (lf - rf - lb + rb) / 4;
    int32_t vy_cmd = (lf + rf + lb + rb) / 4;
    int32_t vz_cmd = (lf - rf + lb - rb) / 4;

    if(lf == 0 && rf == 0 && lb == 0 && rb == 0)
    {
        car_set_speed_now(0);
        return;
    }

    switch(car_direction)
    {
        case MOVE_FORWARD:
        case MOVE_BACKWARD:
            car_set_speed_now(vy_cmd);
            break;

        case MOVE_LEFT:
        case MOVE_RIGHT:
            car_set_speed_now(vx_cmd);
            break;

        case MOVE_ROTATE_CW:
        case MOVE_ROTATE_CCW:
            car_set_speed_now(vz_cmd);
            break;

        default:
            car_set_speed_now(0);
            break;
    }
}

/*
 * Estimate wheel speed from encoder position change.
 *
 * This helper is kept for compatibility with the current project. The wheel
 * state still lives inside g_motor[] at this stage, so this function continues
 * to update the speed-related fields there.
 */
void Calc_SpeedNow(MotorID motor_id)
{
    int32_t speed_calc;
    int32_t pos_diff = g_motor[motor_id].pos_now - g_motor[motor_id].pos_last;

    speed_calc = (pos_diff * 1000) / (PULSE_PER_MM * PID_CONTROL_PERIOD);
    g_motor[motor_id].speed_now =
        (SPEED_FILTER_FACTOR * g_motor[motor_id].speed_last +
         (10 - SPEED_FILTER_FACTOR) * speed_calc) / 10;

    /*
     * Preserve the previous filtered speed sample for the next call.
     * Without this assignment the filter keeps mixing against a stale zero and
     * never behaves like a real first-order speed filter.
     */
    g_motor[motor_id].speed_last = g_motor[motor_id].speed_now;
    g_motor[motor_id].pos_last = g_motor[motor_id].pos_now;
}
