/*
 * 文件名称: soko_utils.h
 * 创建时间: 2026-03-18
 * 创作者: 王宇铖
 * 说明: 推箱子 BFS 系统通用工具函数声明。
 */

#ifndef _SOKO_UTILS_H
#define _SOKO_UTILS_H

#include "soko_types.h"

Point soko_point_make(uint8_t x, uint8_t y);
bool soko_point_equal(Point a, Point b);

bool soko_in_bounds_xy(int16_t x, int16_t y);
bool soko_in_bounds_point(Point p);

uint8_t soko_point_to_cell(Point p);
Point soko_cell_to_point(uint8_t cell_id);

int8_t soko_dir_dx(Dir dir);
int8_t soko_dir_dy(Dir dir);
Point soko_step_point(Point p, Dir dir);

Action soko_action_from_dir(Dir dir, bool is_push);
bool soko_action_is_move(Action action);
bool soko_action_is_push(Action action);
bool soko_action_is_rotate(Action action);
SokoStatus soko_action_to_dir(Action action, Dir *out_dir);

void soko_action_seq_reset(ActionSeq *seq);
SokoStatus soko_action_seq_push(ActionSeq *seq, Action action);
SokoStatus soko_action_seq_append(ActionSeq *dst, const ActionSeq *src);

#endif
