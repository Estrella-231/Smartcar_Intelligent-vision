/*
 * 文件名称: soko_stage1.h
 * 创建时间: 2026-03-18
 * 创作者: 王宇铖
 * 说明: 第一阶段贪心调度接口。
 */

#ifndef _SOKO_STAGE1_H
#define _SOKO_STAGE1_H

#include "soko_types.h"

SokoStatus soko_plan_stage1_greedy(GameMap *map, ActionSeq *out_seq);

#endif
