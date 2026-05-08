/*
 * 文件名称: soko_stage1.c
 * 创建时间: 2026-03-18
 * 创作者: 王宇铖
 * 说明: 第一阶段贪心调度实现。
 */

#include "soko_stage1.h"

#include "soko_push_bfs.h"
#include "soko_replay.h"
#include "soko_utils.h"

static bool soko_stage1_result_better(const SolveResult *lhs, const SolveResult *rhs)
{
	if(lhs->steps != rhs->steps)
	{
		return (lhs->steps < rhs->steps);
	}

	if(lhs->pushes != rhs->pushes)
	{
		return (lhs->pushes < rhs->pushes);
	}

	return (lhs->seq.count < rhs->seq.count);
}

SokoStatus soko_plan_stage1_greedy(GameMap *map, ActionSeq *out_seq)
{
	if((0 == map) || (0 == out_seq))
	{
		return SOKO_INVALID_MAP;
	}

	soko_action_seq_reset(out_seq);

	while(map->box_count > 0)
	{
		uint8_t i;
		bool best_found = false;
		SolveResult best_result;

		for(i = 0; i < map->box_count; i++)
		{
			SolveResult current_result;
			SokoStatus status;

			if(!map->boxes[i].active)
			{
				continue;
			}

			status = soko_bfs_push_box_to_any_target(map,
													 map->player,
													 map->boxes[i].pos,
													 map->generic_target_mask,
													 &current_result);
			if(SOKO_OK != status)
			{
				continue;
			}

			current_result.chosen_box = (int16_t)i;

			if(!best_found || soko_stage1_result_better(&current_result, &best_result))
			{
				best_result = current_result;
				best_found = true;
			}
		}

		if(!best_found)
		{
			return SOKO_NO_PATH;
		}

		if(SOKO_OK != soko_action_seq_append(out_seq, &best_result.seq))
		{
			return SOKO_OVERFLOW_ACTIONS;
		}

		if(SOKO_OK != soko_replay_apply_sequence(map, &best_result.seq, 0))
		{
			return SOKO_INTERNAL_ERROR;
		}
	}

	return SOKO_OK;
}
