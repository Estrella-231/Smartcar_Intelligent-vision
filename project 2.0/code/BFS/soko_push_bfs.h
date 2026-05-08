/*
 * 文件名称: soko_push_bfs.h
 * 创建时间: 2026-03-18
 * 创作者: 王宇铖
 * 说明: 单箱推箱 BFS 接口。
 */

#ifndef _SOKO_PUSH_BFS_H
#define _SOKO_PUSH_BFS_H

#include "soko_types.h"

SokoStatus soko_bfs_push_box_to_any_target(const GameMap *map,
										   Point player_start,
										   Point box_start,
										   const uint8_t candidate_target_mask[MAP_H][MAP_W],
										   SolveResult *out_result);

#endif
