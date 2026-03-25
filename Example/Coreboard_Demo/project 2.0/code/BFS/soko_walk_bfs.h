/*
 * 文件名称: soko_walk_bfs.h
 * 创建时间: 2026-03-18
 * 创作者: 王宇铖
 * 说明: 玩家自由移动 BFS 接口。
 */

#ifndef _SOKO_WALK_BFS_H
#define _SOKO_WALK_BFS_H

#include "soko_types.h"

SokoStatus soko_bfs_walk_to_any(const GameMap *map,
								Point start,
								const Point *goals,
								uint8_t goal_count,
								ActionSeq *out_seq,
								Point *out_end,
								int16_t *out_goal_index);

#endif
