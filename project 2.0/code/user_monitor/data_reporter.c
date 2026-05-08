/**
 * @file    data_reporter.c
 * @brief   数据上报器实现 - 周期性打包并上报18通道浮点数据
 * @details 序列化格式: [ch_count:u8][ch0_f32][ch1_f32]...[ch17_f32]，小端序
 *          通过 protocol_pack 打包为完整通信帧
 * @version 1.0
 */

#include "data_reporter.h"
#include <string.h>

/* 默认上报间隔（毫秒） */
#define REPORTER_DEFAULT_INTERVAL_MS  20

void reporter_init(data_reporter_t *reporter, uint32_t sys_tick_ms)
{
    uint8_t i;

    if (reporter == (void *)0) return;

    /* 清零所有通道 */
    for (i = 0; i < REPORTER_CHANNELS; i++) {
        reporter->channels[i].f32 = 0.0f;
    }

    reporter->enabled     = 0;
    reporter->interval_ms = REPORTER_DEFAULT_INTERVAL_MS;
    reporter->last_tick   = sys_tick_ms;
    reporter->timestamp   = 0;
    reporter->frame_len   = 0;
    memset(reporter->frame_buf, 0, sizeof(reporter->frame_buf));
}

void reporter_set_channel_f32(data_reporter_t *reporter, uint8_t ch, float value)
{
    if (reporter == (void *)0) return;
    if (ch >= REPORTER_CHANNELS) return;

    reporter->channels[ch].f32 = value;
}

void reporter_enable(data_reporter_t *reporter, uint8_t enable, uint16_t interval_ms)
{
    if (reporter == (void *)0) return;

    reporter->enabled = enable;
    if (interval_ms > 0) {
        reporter->interval_ms = interval_ms;
    }
}

uint16_t reporter_get_frame(data_reporter_t *reporter, proto_ctx_t *proto_ctx, uint32_t sys_tick_ms)
{
    uint8_t  data_buf[1 + REPORTER_CHANNELS * 4];  /* 1字节通道数 + 18*4字节float */
    uint16_t pos = 0;
    uint8_t  i;

    if (reporter == (void *)0 || proto_ctx == (void *)0) return 0;

    /* 未使能则不上报 */
    if (!reporter->enabled) return 0;

    /* 检查是否到达上报时间 */
    if ((sys_tick_ms - reporter->last_tick) < reporter->interval_ms) {
        return 0;
    }

    /* 更新时间戳和上次上报时刻 */
    reporter->timestamp = sys_tick_ms;
    reporter->last_tick = sys_tick_ms;

    /* 序列化数据：先写通道数，再逐通道写float值（小端序） */
    data_buf[pos++] = REPORTER_CHANNELS;

    for (i = 0; i < REPORTER_CHANNELS; i++) {
        proto_float_to_bytes(reporter->channels[i].f32, &data_buf[pos]);
        pos += 4;
    }

    /* 使用协议打包 */
    reporter->frame_len = protocol_pack(proto_ctx, CMD_REPORT_STATUS,
                                        data_buf, (uint8_t)pos,
                                        reporter->frame_buf);

    return reporter->frame_len;
}

const uint8_t *reporter_get_frame_data(const data_reporter_t *reporter)
{
    if (reporter == (void *)0) return (void *)0;
    return reporter->frame_buf;
}

uint16_t reporter_get_frame_len(const data_reporter_t *reporter)
{
    if (reporter == (void *)0) return 0;
    return reporter->frame_len;
}
