#ifndef VISION_CONFIG_H
#define VISION_CONFIG_H

#include "zf_common_headfile.h"

/*
 * OpenART -> RT1064 link configuration.
 *
 * The user has fixed the hardware routing for this project:
 * - RT1064 uses UART_4
 * - TX = C16
 * - RX = C17
 * - baud = 115200
 *
 * The current OpenART firmware only sends data in one direction, from
 * OpenART to RT1064. RT1064 therefore only needs the RX path for now, but the
 * TX pin is still initialized so the UART block is configured normally.
 */
#define OPENART_UART_INDEX               (UART_4)
#define OPENART_UART_TX_PIN              (UART4_TX_C16)
#define OPENART_UART_RX_PIN              (UART4_RX_C17)
#define OPENART_UART_BAUDRATE            (115200U)

/*
 * OpenART packet layout.
 *
 * Byte 0   : frame header  0xDF
 * Byte 1   : payload len   195
 * Byte 2   : status
 * Byte 3-194   : 16x12 ASCII map data, row-major
 * Byte 195     : player x
 * Byte 196     : player y
 * Byte 197     : checksum over byte 0..196
 * Byte 198     : frame tail  0xD0
 */
#define OPENART_FRAME_HEADER             (0xDFU)
#define OPENART_FRAME_TAIL               (0xD0U)
#define OPENART_STATUS_INVALID           (0x00U)
#define OPENART_STATUS_VALID             (0x01U)
#define OPENART_PAYLOAD_LEN              (195U)
#define OPENART_PACKET_LEN               (199U)
#define OPENART_MAP_WIDTH                (16U)
#define OPENART_MAP_HEIGHT               (12U)
#define OPENART_MAP_SIZE                 (OPENART_MAP_WIDTH * OPENART_MAP_HEIGHT)
#define OPENART_MAP_OFFSET               (3U)
#define OPENART_PLAYER_X_OFFSET          (195U)
#define OPENART_PLAYER_Y_OFFSET          (196U)
#define OPENART_CHECKSUM_OFFSET          (197U)
#define OPENART_TAIL_OFFSET              (198U)

#endif
