#ifndef ZF_DEVICE_IPS200_H
#define ZF_DEVICE_IPS200_H

#include <stdint.h>

#define RGB565_GRAY         (0x8410)
#define RGB565_WHITE        (0xFFFF)
#define RGB565_RED          (0xF800)
#define RGB565_BLUE         (0x001F)
#define RGB565_BLACK        (0x0000)
#define RGB565_GREEN        (0x07E0)

#define IPS200_CROSSWISE    (0)
#define IPS200_TYPE_SPI     (0)
#define IPS200_6X8_FONT     (0)

void ips200_set_dir(int dir);
void ips200_init(int type);
void ips200_set_font(int font);
void ips200_set_color(uint16_t fg, uint16_t bg);
void ips200_full(uint16_t color);
void ips200_draw_point(uint16_t x, uint16_t y, uint16_t color);
void ips200_draw_line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);

#endif
