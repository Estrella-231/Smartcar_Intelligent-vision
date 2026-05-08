/**
 * @file    comm_protocol.c
 * @brief   通信协议实现 - PC与MCU之间的帧打包与解析
 * @details 帧格式: [0xAA][0x55][LEN][CMD][SEQ][DATA...][CRC16_LO][CRC16_HI]
 *          CRC采用CRC-CCITT多项式0x1021，初值0xFFFF
 * @version 1.0
 */

#include "comm_protocol.h"
#include <string.h>

/*============================================================================
 *  CRC-CCITT 查找表（多项式0x1021，初值0xFFFF）
 *  为节省MCU RAM，采用逐字节计算而非完整256项查找表
 *============================================================================*/

/**
 * @brief  更新CRC-CCITT（逐字节）
 * @param  crc  当前CRC值
 * @param  byte 输入字节
 * @return 更新后的CRC值
 */
static uint16_t crc16_update(uint16_t crc, uint8_t byte)
{
    uint8_t i;
    crc ^= ((uint16_t)byte << 8);
    for (i = 0; i < 8; i++) {
        if (crc & 0x8000) {
            crc = (crc << 1) ^ 0x1021;
        } else {
            crc = (crc << 1);
        }
    }
    return crc;
}

/*============================================================================
 *  接口函数实现
 *============================================================================*/

uint16_t crc16_calculate(const uint8_t *data, uint8_t len)
{
    uint16_t crc = 0xFFFF;  /* CRC-CCITT初值 */
    uint8_t  i;

    if (data == (void *)0 || len == 0) {
        return crc;
    }

    for (i = 0; i < len; i++) {
        crc = crc16_update(crc, data[i]);
    }

    return crc;
}

void protocol_init(proto_ctx_t *ctx)
{
    if (ctx == (void *)0) return;

    ctx->state  = PARSE_STATE_IDLE;
    ctx->rx_len = 0;
    ctx->rx_cmd = 0;
    ctx->rx_seq = 0;
    ctx->rx_idx = 0;
    ctx->crc_lo = 0;
    ctx->crc_calc = 0xFFFF;
    ctx->tx_seq = 0;
    memset(ctx->rx_buf, 0, sizeof(ctx->rx_buf));
}

uint16_t protocol_pack(proto_ctx_t *ctx, uint8_t cmd,
                       const uint8_t *data, uint8_t data_len,
                       uint8_t *out_buf)
{
    uint16_t pos = 0;
    uint16_t crc;
    uint8_t  seq;

    if (out_buf == (void *)0) return 0;
    if (data_len > PROTO_MAX_DATA_LEN) return 0;
    if (data == (void *)0 && data_len > 0) return 0;

    /* 获取并递增序列号 */
    seq = ctx->tx_seq++;

    /* 组装帧头 */
    out_buf[pos++] = PROTO_HEADER_0;    /* [0] 帧头 0xAA */
    out_buf[pos++] = PROTO_HEADER_1;    /* [1] 帧头 0x55 */
    out_buf[pos++] = data_len;          /* [2] 数据域长度 */
    out_buf[pos++] = cmd;               /* [3] 命令字 */
    out_buf[pos++] = seq;               /* [4] 序列号 */

    /* 拷贝数据域 */
    if (data_len > 0 && data != (void *)0) {
        memcpy(&out_buf[pos], data, data_len);
    }
    pos += data_len;

    /* 计算CRC（覆盖LEN+CMD+SEQ+DATA） */
    crc = crc16_calculate(&out_buf[2], data_len + 3);
    out_buf[pos++] = (uint8_t)(crc & 0xFF);        /* CRC低字节 */
    out_buf[pos++] = (uint8_t)((crc >> 8) & 0xFF); /* CRC高字节 */

    return pos;  /* 返回总帧长 */
}

uint8_t protocol_parse(proto_ctx_t *ctx, uint8_t byte, proto_frame_t *frame)
{
    if (ctx == (void *)0 || frame == (void *)0) return 0;

    switch (ctx->state) {

    case PARSE_STATE_IDLE:
        /* 等待帧头第1字节 0xAA */
        if (byte == PROTO_HEADER_0) {
            ctx->state = PARSE_STATE_H1;
        }
        break;

    case PARSE_STATE_H1:
        /* 等待帧头第2字节 0x55 */
        if (byte == PROTO_HEADER_1) {
            ctx->state    = PARSE_STATE_LEN;
            ctx->crc_calc = 0xFFFF;  /* 重置CRC计算（从LEN开始计算） */
        } else if (byte == PROTO_HEADER_0) {
            /* 可能新帧头的0xAA，保持在H1状态 */
        } else {
            ctx->state = PARSE_STATE_IDLE;
        }
        break;

    case PARSE_STATE_LEN:
        /* 接收数据域长度 */
        ctx->rx_len = byte;
        ctx->crc_calc = crc16_update(ctx->crc_calc, byte);
        if (ctx->rx_len > PROTO_MAX_DATA_LEN) {
            ctx->state = PARSE_STATE_IDLE;  /* 长度非法，丢弃 */
        } else {
            ctx->state = PARSE_STATE_CMD;
        }
        break;

    case PARSE_STATE_CMD:
        /* 接收命令字 */
        ctx->rx_cmd = byte;
        ctx->crc_calc = crc16_update(ctx->crc_calc, byte);
        ctx->state = PARSE_STATE_SEQ;
        break;

    case PARSE_STATE_SEQ:
        /* 接收序列号 */
        ctx->rx_seq = byte;
        ctx->crc_calc = crc16_update(ctx->crc_calc, byte);
        ctx->rx_idx = 0;
        if (ctx->rx_len > 0) {
            ctx->state = PARSE_STATE_DATA;
        } else {
            ctx->state = PARSE_STATE_CRC_LO;  /* 无数据域，直接跳到CRC */
        }
        break;

    case PARSE_STATE_DATA:
        /* 逐字节接收数据域 */
        ctx->rx_buf[ctx->rx_idx++] = byte;
        ctx->crc_calc = crc16_update(ctx->crc_calc, byte);
        if (ctx->rx_idx >= ctx->rx_len) {
            ctx->state = PARSE_STATE_CRC_LO;
        }
        break;

    case PARSE_STATE_CRC_LO:
        /* 接收CRC低字节 */
        ctx->crc_lo = byte;
        ctx->state  = PARSE_STATE_CRC_HI;
        break;

    case PARSE_STATE_CRC_HI:
        {
            /* 接收CRC高字节，组装完整CRC并校验 */
            uint16_t rx_crc = ((uint16_t)byte << 8) | ctx->crc_lo;

            if (rx_crc == ctx->crc_calc) {
                /* CRC校验通过，输出帧 */
                frame->cmd      = ctx->rx_cmd;
                frame->seq      = ctx->rx_seq;
                frame->data_len = ctx->rx_len;
                memcpy(frame->data, ctx->rx_buf, ctx->rx_len);
                frame->valid    = 1;
                ctx->state      = PARSE_STATE_IDLE;
                return 1;  /* 成功解析一帧 */
            } else {
                /* CRC校验失败，丢弃 */
                frame->valid = 0;
                ctx->state   = PARSE_STATE_IDLE;
            }
        }
        break;

    default:
        ctx->state = PARSE_STATE_IDLE;
        break;
    }

    return 0;  /* 尚未解析完成 */
}

/*============================================================================
 *  辅助函数：数据类型与字节数组互转（小端字节序）
 *============================================================================*/

void proto_float_to_bytes(float val, uint8_t *buf)
{
    /* 强制将float的内存表示按字节拷贝，小端序平台直接对应 */
    uint8_t *p = (uint8_t *)&val;
    buf[0] = p[0];
    buf[1] = p[1];
    buf[2] = p[2];
    buf[3] = p[3];
}

float proto_bytes_to_float(const uint8_t *buf)
{
    float val;
    uint8_t *p = (uint8_t *)&val;
    p[0] = buf[0];
    p[1] = buf[1];
    p[2] = buf[2];
    p[3] = buf[3];
    return val;
}

void proto_u32_to_bytes(uint32_t val, uint8_t *buf)
{
    buf[0] = (uint8_t)(val & 0xFF);
    buf[1] = (uint8_t)((val >> 8) & 0xFF);
    buf[2] = (uint8_t)((val >> 16) & 0xFF);
    buf[3] = (uint8_t)((val >> 24) & 0xFF);
}

uint32_t proto_bytes_to_u32(const uint8_t *buf)
{
    return (uint32_t)buf[0] |
           ((uint32_t)buf[1] << 8) |
           ((uint32_t)buf[2] << 16) |
           ((uint32_t)buf[3] << 24);
}

void proto_i16_to_bytes(int16_t val, uint8_t *buf)
{
    uint16_t u = (uint16_t)val;
    buf[0] = (uint8_t)(u & 0xFF);
    buf[1] = (uint8_t)((u >> 8) & 0xFF);
}

int16_t proto_bytes_to_i16(const uint8_t *buf)
{
    uint16_t u = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    return (int16_t)u;
}
