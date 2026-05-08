/**
 * @file    comm_protocol.h
 * @brief   通信协议头文件 - PC与MCU之间的串口通信协议
 * @details 帧格式: [0xAA][0x55][LEN][CMD][SEQ][DATA...][CRC16_LO][CRC16_HI]
 *          CRC-CCITT校验，小端字节序
 * @version 2.0 - 18通道 + 5PID通道
 */

#ifndef __COMM_PROTOCOL_H
#define __COMM_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 帧格式常量 */
#define PROTO_HEADER_0        0xAA
#define PROTO_HEADER_1        0x55
#define PROTO_MAX_DATA_LEN    128
#define PROTO_MAX_FRAME_LEN   (PROTO_MAX_DATA_LEN + 8)

/* 命令字 */
#define CMD_SET_PID_PARAMS    0x01    /* PC→MCU：设置PID参数 */
#define CMD_REPORT_STATUS     0x02    /* MCU→PC：上报状态数据 */
#define CMD_SET_TARGET        0x03    /* PC→MCU：设置目标值 */
#define CMD_SET_REPORT_CFG    0x04    /* PC→MCU：启停上报 */
#define CMD_REQ_PID_PARAMS    0x05    /* PC→MCU：请求PID参数 */
#define CMD_RSP_PID_PARAMS    0x06    /* MCU→PC：返回PID参数 */

/* 数据通道数和PID通道数 */
#define DATA_CHANNEL_COUNT    18
#define PID_CHANNEL_COUNT     5

/* 解析状态机 */
typedef enum {
    PARSE_STATE_IDLE = 0,
    PARSE_STATE_H1,
    PARSE_STATE_LEN,
    PARSE_STATE_CMD,
    PARSE_STATE_SEQ,
    PARSE_STATE_DATA,
    PARSE_STATE_CRC_LO,
    PARSE_STATE_CRC_HI
} parse_state_t;

/* 解析结果 */
typedef struct {
    uint8_t   cmd;
    uint8_t   seq;
    uint8_t   data[PROTO_MAX_DATA_LEN];
    uint8_t   data_len;
    uint8_t   valid;
} proto_frame_t;

/* 协议上下文 */
typedef struct {
    parse_state_t state;
    uint8_t       rx_len;
    uint8_t       rx_cmd;
    uint8_t       rx_seq;
    uint8_t       rx_buf[PROTO_MAX_DATA_LEN];
    uint8_t       rx_idx;
    uint8_t       crc_lo;
    uint16_t      crc_calc;
    uint8_t       tx_seq;
} proto_ctx_t;

/* 接口函数 */
void protocol_init(proto_ctx_t *ctx);
uint16_t protocol_pack(proto_ctx_t *ctx, uint8_t cmd,
                       const uint8_t *data, uint8_t data_len,
                       uint8_t *out_buf);
uint8_t protocol_parse(proto_ctx_t *ctx, uint8_t byte, proto_frame_t *frame);
uint16_t crc16_calculate(const uint8_t *data, uint8_t len);

/* 辅助函数：float/uint32/int16 与字节互转（小端序） */
void proto_float_to_bytes(float val, uint8_t *buf);
float proto_bytes_to_float(const uint8_t *buf);
void proto_u32_to_bytes(uint32_t val, uint8_t *buf);
uint32_t proto_bytes_to_u32(const uint8_t *buf);

#ifdef __cplusplus
}
#endif

#endif /* __COMM_PROTOCOL_H */
