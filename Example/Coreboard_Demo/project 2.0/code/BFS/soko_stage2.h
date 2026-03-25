/*
 * 文件名称: soko_stage2.h
 * 创建时间: 2026-03-18
 * 创作者: 王宇铖
 * 说明: 第二阶段稳定版观察、识别绑定与推箱调度接口。
 */

#ifndef _SOKO_STAGE2_H
#define _SOKO_STAGE2_H

#include "soko_types.h"

uint8_t soko_stage2_build_observe_points(const GameMap *map,
										 uint8_t box_index,
										 Point *out_points,
										 Action *out_face_actions,
										 uint8_t max_points);

SokoStatus soko_stage2_plan_observe_box(const GameMap *map,
										uint8_t box_index,
										ActionSeq *out_seq,
										Point *out_end);

SokoStatus soko_stage2_bind_box_target(GameMap *map, uint8_t box_index, uint8_t class_id);

SokoStatus soko_stage2_plan_bound_boxes(GameMap *map, ActionSeq *out_seq);

#endif
