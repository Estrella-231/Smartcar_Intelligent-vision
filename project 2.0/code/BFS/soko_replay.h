/*
 * 文件名称: soko_replay.h
 * 创建时间: 2026-03-18
 * 创作者: 王宇铖
 * 说明: 推箱子动作回放与合法性校验接口。
 */

#ifndef _SOKO_REPLAY_H
#define _SOKO_REPLAY_H

#include "soko_types.h"

SokoStatus soko_replay_apply_action(GameMap *map, Action action);
SokoStatus soko_replay_apply_sequence(GameMap *map, const ActionSeq *seq, SokoReplayResult *out_result);

#endif
