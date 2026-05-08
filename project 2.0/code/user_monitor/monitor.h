#ifndef __MONITOR_H
#define __MONITOR_H

#include <stdint.h>

void monitor_init(void);
void monitor_tick(void);
uint32_t monitor_get_tick_ms(void);

#endif
