/**
 * @file    monitor.c
 * @brief   监控模块 - 适配 project+2.0 麦克纳姆轮四轮车
 * @details
 *   读取 project+2.0 的传感器数据，通过蓝牙发送到PC上位机。
 *   接收PC下发的PID参数，写入 g_speed_pid[]。
 *
 *   使用方法：
 *   1. main() 中调用 monitor_init()
 *   2. 主循环中调用 monitor_tick()（20ms周期）
 *
 * @version 2.0
 */

#include "monitor.h"
#include "comm_protocol.h"
#include "data_reporter.h"

/* project+2.0 的头文件 */
#include "user_driver/motor_driver.h"
#include "user_driver/gyroscope.h"
#include "uer_algorithm/kinematics/odometry.h"
#include "uer_algorithm/pid/speed_pid.h"
#include "uer_algorithm/pid/angle_pid.h"
#include "user_config/robot_param.h"
#include "zf_device_ble6a20.h"

/*============================================================================
 *  内部变量
 *============================================================================*/
static proto_ctx_t     g_proto;       /* 协议解析上下文 */
static data_reporter_t g_reporter;    /* 数据上报器 */
static uint32_t        g_tick_ms;     /* 系统tick计数器 */

/*============================================================================
 *  初始化
 *============================================================================*/

void monitor_init(void)
{
    protocol_init(&g_proto);
    reporter_init(&g_reporter, 0);
    reporter_enable(&g_reporter, 1, 20);  /* 20ms上报间隔 = 50Hz */
    g_tick_ms = 0;
}

/*============================================================================
 *  系统tick获取
 *============================================================================*/

uint32_t monitor_get_tick_ms(void)
{
    return g_tick_ms;
}

/*============================================================================
 *  处理PC下发的命令
 *============================================================================*/

static void handle_pc_command(const proto_frame_t *frame)
{
    if (frame == (void *)0 || !frame->valid) return;

    switch (frame->cmd) {

    case CMD_SET_PID_PARAMS:
        /* PC下发PID参数: [ch:u8][P:f32][I:f32][D:f32][out_max:f32] */
        if (frame->data_len >= 17) {
            uint8_t ch = frame->data[0];
            float p       = proto_bytes_to_float(&frame->data[1]);
            float i_val   = proto_bytes_to_float(&frame->data[5]);
            float d       = proto_bytes_to_float(&frame->data[9]);
            float out_max = proto_bytes_to_float(&frame->data[13]);

            if (ch < MOTOR_MAX) {
                g_speed_pid[ch].kp = p;
                g_speed_pid[ch].ki = i_val;
                g_speed_pid[ch].kd = d;
                g_speed_pid[ch].output_limit = (int32_t)out_max;
            }
            /* ch=4: 航向环参数 — angle_pid暂不支持在线修改 */
        }
        break;

    case CMD_SET_TARGET:
        /* PC设置目标值: [ch:u8][target:f32] */
        if (frame->data_len >= 5) {
            uint8_t ch = frame->data[0];
            float target = proto_bytes_to_float(&frame->data[1]);
            if (ch < MOTOR_MAX) {
                change_speed_target((MotorID)ch, (int32_t)target);
            }
        }
        break;

    case CMD_REQ_PID_PARAMS:
        /* PC请求PID参数: [ch:u8] */
        if (frame->data_len >= 1) {
            uint8_t ch = frame->data[0];
            uint8_t rsp_data[17];
            uint8_t rsp_buf[PROTO_MAX_FRAME_LEN];

            rsp_data[0] = ch;
            if (ch < MOTOR_MAX) {
                proto_float_to_bytes(g_speed_pid[ch].kp,           &rsp_data[1]);
                proto_float_to_bytes(g_speed_pid[ch].ki,           &rsp_data[5]);
                proto_float_to_bytes(g_speed_pid[ch].kd,           &rsp_data[9]);
                proto_float_to_bytes((float)g_speed_pid[ch].output_limit, &rsp_data[13]);
            } else {
                proto_float_to_bytes(0, &rsp_data[1]);
                proto_float_to_bytes(0, &rsp_data[5]);
                proto_float_to_bytes(0, &rsp_data[9]);
                proto_float_to_bytes(0, &rsp_data[13]);
            }

            uint16_t len = protocol_pack(&g_proto, CMD_RSP_PID_PARAMS,
                                         rsp_data, 17, rsp_buf);
            if (len > 0) {
                ble6a20_send_buffer(rsp_buf, len);
            }
        }
        break;

    case CMD_SET_REPORT_CFG:
        /* PC设置上报配置: [enable:u8][interval_ms:u16] */
        if (frame->data_len >= 3) {
            uint8_t  enable   = frame->data[0];
            uint16_t interval = (uint16_t)frame->data[1] | ((uint16_t)frame->data[2] << 8);
            reporter_enable(&g_reporter, enable, interval);
        }
        break;

    default:
        break;
    }
}

/*============================================================================
 *  主循环调用（每20ms调用一次）
 *============================================================================*/

void monitor_tick(void)
{
    uint8_t  rx_buf[64];
    uint32_t rx_len;
    uint32_t i;
    proto_frame_t frame;
    uint16_t tx_len;

    /* 更新系统tick */
    g_tick_ms += EXEC_CONTROL_PERIOD_MS;

    /* ---- 1. 接收PC命令 ---- */
    rx_len = ble6a20_read_buffer(rx_buf, sizeof(rx_buf));
    for (i = 0; i < rx_len; i++) {
        if (protocol_parse(&g_proto, rx_buf[i], &frame)) {
            handle_pc_command(&frame);
        }
    }

    /* ---- 2. 填充18通道数据 ---- */
    /* CH0-3: 四轮PWM */
    reporter_set_channel_f32(&g_reporter, 0,  (float)g_motor[MOTOR_LF].pwm_out);
    reporter_set_channel_f32(&g_reporter, 1,  (float)g_motor[MOTOR_RF].pwm_out);
    reporter_set_channel_f32(&g_reporter, 2,  (float)g_motor[MOTOR_LB].pwm_out);
    reporter_set_channel_f32(&g_reporter, 3,  (float)g_motor[MOTOR_RB].pwm_out);

    /* CH4-7: 四轮速度 */
    reporter_set_channel_f32(&g_reporter, 4,  (float)get_speed_now(MOTOR_LF));
    reporter_set_channel_f32(&g_reporter, 5,  (float)get_speed_now(MOTOR_RF));
    reporter_set_channel_f32(&g_reporter, 6,  (float)get_speed_now(MOTOR_LB));
    reporter_set_channel_f32(&g_reporter, 7,  (float)get_speed_now(MOTOR_RB));

    /* CH8: 角速度 (°/s), CH9: 偏航角 (°) — 原始单位0.01°，需除以100 */
    reporter_set_channel_f32(&g_reporter, 8,  (float)imu_get_gyro_z_cdps() / 100.0f);
    reporter_set_channel_f32(&g_reporter, 9,  (float)imu_get_yaw_cd() / 100.0f);

    /* CH10-11: 里程计坐标 (mm) */
    reporter_set_channel_f32(&g_reporter, 10, (float)odometry_get_x_mm());
    reporter_set_channel_f32(&g_reporter, 11, (float)odometry_get_y_mm());

    /* CH12: 航向误差 (°), CH13: 航向PID输出 */
    reporter_set_channel_f32(&g_reporter, 12, (float)angle_pid_get_error() / 100.0f);
    reporter_set_channel_f32(&g_reporter, 13, (float)angle_pid_get_output());

    /* CH14-15: 全局速度 (mm/s) */
    reporter_set_channel_f32(&g_reporter, 14, (float)odometry_get_vx_global_mmps());
    reporter_set_channel_f32(&g_reporter, 15, (float)odometry_get_vy_global_mmps());

    /* CH16: 运动状态, CH17: 赛道标志 */
    reporter_set_channel_f32(&g_reporter, 16, (float)g_rot_state);
    reporter_set_channel_f32(&g_reporter, 17, 0.0f);

    /* ---- 3. 打包并发送 ---- */
    tx_len = reporter_get_frame(&g_reporter, &g_proto, g_tick_ms);
    if (tx_len > 0) {
        ble6a20_send_buffer(reporter_get_frame_data(&g_reporter), tx_len);
    }
}
