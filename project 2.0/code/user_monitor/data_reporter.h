#ifndef __DATA_REPORTER_H
#define __DATA_REPORTER_H

#include <stdint.h>
#include "comm_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

#define REPORTER_CHANNELS  18

typedef union {
    float    f32;
    int16_t  i16;
    uint8_t  u8;
} channel_value_t;

typedef struct {
    channel_value_t channels[REPORTER_CHANNELS];
    uint8_t         enabled;
    uint16_t        interval_ms;
    uint32_t        last_tick;
    uint32_t        timestamp;
    uint8_t         frame_buf[PROTO_MAX_FRAME_LEN];
    uint16_t        frame_len;
} data_reporter_t;

void reporter_init(data_reporter_t *reporter, uint32_t sys_tick_ms);
void reporter_set_channel_f32(data_reporter_t *reporter, uint8_t ch, float value);
void reporter_enable(data_reporter_t *reporter, uint8_t enable, uint16_t interval_ms);
uint16_t reporter_get_frame(data_reporter_t *reporter, proto_ctx_t *proto_ctx, uint32_t sys_tick_ms);
const uint8_t *reporter_get_frame_data(const data_reporter_t *reporter);
uint16_t reporter_get_frame_len(const data_reporter_t *reporter);

#ifdef __cplusplus
}
#endif

#endif
