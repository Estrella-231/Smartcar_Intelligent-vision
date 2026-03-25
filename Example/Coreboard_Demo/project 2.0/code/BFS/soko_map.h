/*
 * 文件名称: soko_map.h
 * 创建时间: 2026-03-18
 * 创作者: 王宇铖
 * 说明: 推箱子地图解析、查询和更新接口。
 */

#ifndef _SOKO_MAP_H
#define _SOKO_MAP_H

#include "soko_types.h"

extern NumberTarget g_soko_number_targets[SOKO_NUMBER_COUNT];
extern const uint8_t g_soko_class_to_target_num[SOKO_NUMBER_COUNT];

void soko_number_targets_clear(void);

void soko_map_clear(GameMap *map);
void soko_map_copy(GameMap *dst, const GameMap *src);

SokoStatus soko_map_parse_ascii(const char raw[MAP_H][MAP_W + 1], GameMap *out_map);
SokoStatus soko_map_bind_number_target(GameMap *map, uint8_t target_number, Point pos);

bool soko_map_is_wall(const GameMap *map, Point pos);
int16_t soko_map_find_box(const GameMap *map, Point pos);
int16_t soko_map_find_target(const GameMap *map, Point pos);
int16_t soko_map_find_bomb(const GameMap *map, Point pos);

bool soko_map_has_box(const GameMap *map, Point pos);
bool soko_map_has_target(const GameMap *map, Point pos);
bool soko_map_has_bomb(const GameMap *map, Point pos);

SokoStatus soko_map_remove_box_at(GameMap *map, Point pos);
SokoStatus soko_map_remove_target_at(GameMap *map, Point pos);
SokoStatus soko_map_remove_bomb_at(GameMap *map, Point pos);

int8_t soko_map_get_target_number(const GameMap *map, Point pos);
void soko_map_clear_explosion(GameMap *map, Point center);

#endif
