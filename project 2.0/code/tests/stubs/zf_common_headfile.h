#ifndef ZF_COMMON_HEADFILE_H
#define ZF_COMMON_HEADFILE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define UART_4          (4)
#define UART4_TX_C16    (0)
#define UART4_RX_C17    (0)

#define LIMIT(x, min, max)    ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#define LIMIT_ABS(x, limit)   LIMIT(x, -(limit), (limit))

void system_delay_ms(uint32_t ms);

#endif
