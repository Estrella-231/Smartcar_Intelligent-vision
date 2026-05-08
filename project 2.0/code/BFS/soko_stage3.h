/*
 * 文件名称: soko_stage3.h
 * 创建时间: 2026-03-18
 * 创作者: 王宇铖
 * 说明: 第三阶段炸弹策略与单箱恢复求解接口。
 */

#ifndef _SOKO_STAGE3_H
#define _SOKO_STAGE3_H

#include "soko_types.h"

SokoStatus soko_bfs_push_bomb_to_wall(const GameMap *map,
									  Point player_start,
									  Point bomb_start,
									  Point wall_target,
									  SolveResult *out_result);

SokoStatus soko_plan_stage3_with_bomb(GameMap *map,
									  Point box_pos,
									  const uint8_t target_mask[MAP_H][MAP_W],
									  ActionSeq *out_seq);

#endif
