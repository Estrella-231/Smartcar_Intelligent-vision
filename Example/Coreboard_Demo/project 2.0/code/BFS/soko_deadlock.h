/*
 * 文件名称: soko_deadlock.h
 * 创建时间: 2026-03-18
 * 创作者: 王宇铖
 * 说明: 推箱子基础死锁判定接口。
 */

#ifndef _SOKO_DEADLOCK_H
#define _SOKO_DEADLOCK_H

#include "soko_types.h"

bool soko_deadlock_is_corner(const GameMap *map,
							 Point box_pos,
							 Point ignore_box_pos,
							 const uint8_t target_mask[MAP_H][MAP_W]);

#endif
