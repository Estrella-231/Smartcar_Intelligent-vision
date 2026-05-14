/**
 * @file    monitor.c
 * @brief   鐩戞帶妯″潡 - 閫傞厤 project+2.0 楹﹀厠绾冲杞洓杞溅
 * @details
 *   璇诲彇 project+2.0 鐨勪紶鎰熷櫒鏁版嵁锛岄€氳繃钃濈墮鍙戦€佸埌PC涓婁綅鏈恒€? *   鎺ユ敹PC涓嬪彂鐨凱ID鍙傛暟锛屽啓鍏?g_speed_pid[]銆? *
 *   浣跨敤鏂规硶锛? *   1. main() 涓皟鐢?monitor_init()
 *   2. 涓诲惊鐜腑璋冪敤 monitor_tick()锛?0ms鍛ㄦ湡锛? *
 * @version 2.0
 */

#include "monitor.h"
#include "comm_protocol.h"
#include "data_reporter.h"

/* project+2.0 鐨勫ご鏂囦欢 */
#include "user_driver/motor_driver.h"
#include "user_driver/gyroscope.h"
#include "uer_algorithm/kinematics/odometry.h"
#include "uer_algorithm/pid/speed_pid.h"
#include "uer_algorithm/pid/angle_pid.h"
#include "user_config/robot_param.h"
#include "zf_device_ble6a20.h"
#include "zf_device_ips200.h"

/*============================================================================
 *  鍐呴儴鍙橀噺
 *============================================================================*/
static proto_ctx_t     g_proto;       /* 鍗忚瑙ｆ瀽涓婁笅鏂?*/
static data_reporter_t g_reporter;    /* 鏁版嵁涓婃姤鍣?*/
static uint32_t        g_tick_ms;     /* 绯荤粺tick璁℃暟鍣?*/
static uint32_t        g_pid_req_count;
static uint32_t        g_rx_byte_count;
static uint32_t        g_cmd_count;

/*============================================================================
 *  鍒濆鍖? *============================================================================*/

void monitor_init(void)
{
    protocol_init(&g_proto);
    reporter_init(&g_reporter, 0);
    reporter_enable(&g_reporter, 1, 20);  /* 20ms涓婃姤闂撮殧 = 50Hz */
    g_tick_ms = 0;
}

/*============================================================================
 *  绯荤粺tick鑾峰彇
 *============================================================================*/

uint32_t monitor_get_tick_ms(void)
{
    return g_tick_ms;
}

/*============================================================================
 *  澶勭悊PC涓嬪彂鐨勫懡浠? *============================================================================*/

static void handle_pc_command(const proto_frame_t *frame)
{
    if (frame == (void *)0 || !frame->valid) return;

    g_cmd_count++;
    ips200_show_string(0, 16, "CMD ");
    ips200_show_uint(24, 16, frame->cmd, 3);
    ips200_show_string(54, 16, "N ");
    ips200_show_uint(66, 16, g_cmd_count, 5);

    switch (frame->cmd) {

    case CMD_SET_PID_PARAMS:
        /* PC涓嬪彂PID鍙傛暟: [ch:u8][P:f32][I:f32][D:f32][out_max:f32] */
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
            } else if (ch == MOTOR_MAX) {
                angle_pid_set_params(p, 0.0f, d, out_max);
            }
        }
        break;

    case CMD_SET_TARGET:
        /* PC璁剧疆鐩爣鍊? [ch:u8][target:f32] */
        if (frame->data_len >= 5) {
            uint8_t ch = frame->data[0];
            float target = proto_bytes_to_float(&frame->data[1]);
            if (ch < MOTOR_MAX) {
                change_speed_target((MotorID)ch, (int32_t)target);
            }
        }
        break;

    case CMD_REQ_PID_PARAMS:
        /* PC璇锋眰PID鍙傛暟: [ch:u8] */
        if (frame->data_len >= 1) {
            uint8_t ch = frame->data[0];
            uint8_t rsp_data[17];
            uint8_t rsp_buf[PROTO_MAX_FRAME_LEN];

            g_pid_req_count++;
            ips200_show_string(0, 0, "PID REQ ");
            ips200_show_uint(48, 0, g_pid_req_count, 5);
            ips200_show_string(0, 8, "CH ");
            ips200_show_uint(18, 8, ch, 2);

            rsp_data[0] = ch;
            if (ch < MOTOR_MAX) {
                proto_float_to_bytes(g_speed_pid[ch].kp,           &rsp_data[1]);
                proto_float_to_bytes(g_speed_pid[ch].ki,           &rsp_data[5]);
                proto_float_to_bytes(g_speed_pid[ch].kd,           &rsp_data[9]);
                proto_float_to_bytes((float)g_speed_pid[ch].output_limit, &rsp_data[13]);
            } else if (ch == MOTOR_MAX) {
                float p;
                float i_val;
                float d;
                float out_max;
                angle_pid_get_params(&p, &i_val, &d, &out_max);
                proto_float_to_bytes(p,       &rsp_data[1]);
                proto_float_to_bytes(i_val,   &rsp_data[5]);
                proto_float_to_bytes(d,       &rsp_data[9]);
                proto_float_to_bytes(out_max, &rsp_data[13]);
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
        /* PC璁剧疆涓婃姤閰嶇疆: [enable:u8][interval_ms:u16] */
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
 *  涓诲惊鐜皟鐢紙姣?0ms璋冪敤涓€娆★級
 *============================================================================*/

void monitor_tick(void)
{
    uint8_t  rx_buf[64];
    uint32_t rx_len;
    uint32_t i;
    proto_frame_t frame;
    uint16_t tx_len;

    /* 鏇存柊绯荤粺tick */
    g_tick_ms += EXEC_CONTROL_PERIOD_MS;

    /* ---- 1. 鎺ユ敹PC鍛戒护 ---- */
    rx_len = ble6a20_read_buffer(rx_buf, sizeof(rx_buf));
    if (rx_len > 0U) {
        g_rx_byte_count += rx_len;
        ips200_show_string(0, 24, "RX ");
        ips200_show_uint(18, 24, g_rx_byte_count, 6);
    }
    for (i = 0; i < rx_len; i++) {
        if (protocol_parse(&g_proto, rx_buf[i], &frame)) {
            handle_pc_command(&frame);
        }
    }

    /* ---- 2. 濉厖22閫氶亾鏁版嵁 ---- */
    /* CH0-3: 鍥涜疆PWM */
    reporter_set_channel_f32(&g_reporter, 0,  (float)g_motor[MOTOR_LF].pwm_out);
    reporter_set_channel_f32(&g_reporter, 1,  (float)g_motor[MOTOR_RF].pwm_out);
    reporter_set_channel_f32(&g_reporter, 2,  (float)g_motor[MOTOR_LB].pwm_out);
    reporter_set_channel_f32(&g_reporter, 3,  (float)g_motor[MOTOR_RB].pwm_out);

    /* CH4-7: 鍥涜疆閫熷害 */
    reporter_set_channel_f32(&g_reporter, 4,  (float)get_speed_now(MOTOR_LF));
    reporter_set_channel_f32(&g_reporter, 5,  (float)get_speed_now(MOTOR_RF));
    reporter_set_channel_f32(&g_reporter, 6,  (float)get_speed_now(MOTOR_LB));
    reporter_set_channel_f32(&g_reporter, 7,  (float)get_speed_now(MOTOR_RB));

    /* CH8: 瑙掗€熷害 (掳/s), CH9: 鍋忚埅瑙?(掳) 鈥?鍘熷鍗曚綅0.01掳锛岄渶闄や互100 */
    reporter_set_channel_f32(&g_reporter, 8,  (float)imu_get_gyro_z_cdps() / 100.0f);
    reporter_set_channel_f32(&g_reporter, 9,  (float)imu_get_yaw_cd() / 100.0f);

    /* CH10-11: 閲岀▼璁″潗鏍?(mm) */
    reporter_set_channel_f32(&g_reporter, 10, (float)odometry_get_x_mm());
    reporter_set_channel_f32(&g_reporter, 11, (float)odometry_get_y_mm());

    /* CH12: 鑸悜璇樊 (掳), CH13: 鑸悜PID杈撳嚭 */
    reporter_set_channel_f32(&g_reporter, 12, (float)angle_pid_get_error() / 100.0f);
    reporter_set_channel_f32(&g_reporter, 13, (float)angle_pid_get_output());

    /* CH14-15: 鍏ㄥ眬閫熷害 (mm/s) */
    reporter_set_channel_f32(&g_reporter, 14, (float)odometry_get_vx_global_mmps());
    reporter_set_channel_f32(&g_reporter, 15, (float)odometry_get_vy_global_mmps());

    /* CH16: 杩愬姩鐘舵€? CH17: 璧涢亾鏍囧織 */
    reporter_set_channel_f32(&g_reporter, 16, (float)g_rot_state);
    reporter_set_channel_f32(&g_reporter, 17, 0.0f);

    /* CH18-21: 鍥涜疆鐩爣閫熷害锛坮amped锛?*/
    reporter_set_channel_f32(&g_reporter, 18, (float)speed_pid_get_target_ramped(MOTOR_LF));
    reporter_set_channel_f32(&g_reporter, 19, (float)speed_pid_get_target_ramped(MOTOR_RF));
    reporter_set_channel_f32(&g_reporter, 20, (float)speed_pid_get_target_ramped(MOTOR_LB));
    reporter_set_channel_f32(&g_reporter, 21, (float)speed_pid_get_target_ramped(MOTOR_RB));

    /* ---- 3. 鎵撳寘骞跺彂閫?---- */
    tx_len = reporter_get_frame(&g_reporter, &g_proto, g_tick_ms);
    if (tx_len > 0) {
        ble6a20_send_buffer(reporter_get_frame_data(&g_reporter), tx_len);
    }
}
