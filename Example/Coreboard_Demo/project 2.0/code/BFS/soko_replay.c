/*
 * 文件名称: soko_replay.c
 * 创建时间: 2026-03-18
 * 创作者: 王宇铖
 * 说明: 推箱子动作回放与合法性校验实现。
 */

#include "soko_replay.h"
#include "soko_map.h"
#include "soko_utils.h"

static bool soko_replay_is_walk_blocked(const GameMap *map, Point pos)
{
	return (soko_map_is_wall(map, pos) || soko_map_has_box(map, pos) || soko_map_has_bomb(map, pos));
}

static bool soko_replay_is_push_blocked(const GameMap *map, Point pos)
{
	return (soko_map_is_wall(map, pos) || soko_map_has_box(map, pos) || soko_map_has_bomb(map, pos));
}

static SokoStatus soko_replay_move_box(GameMap *map, int16_t box_index, Point next_pos)
{
	if((0 == map) || (box_index < 0) || (box_index >= map->box_count))
	{
		return SOKO_INVALID_MAP;
	}

	map->boxes[box_index].pos = next_pos;

	if(soko_map_has_target(map, next_pos))
	{
		SokoStatus status;

		status = soko_map_remove_box_at(map, next_pos);
		if(SOKO_OK != status)
		{
			return status;
		}

		status = soko_map_remove_target_at(map, next_pos);
		if(SOKO_OK != status)
		{
			return status;
		}
	}

	return SOKO_OK;
}

SokoStatus soko_replay_apply_action(GameMap *map, Action action)
{
	Dir dir;
	Point next_player;

	if(0 == map)
	{
		return SOKO_INVALID_MAP;
	}

	if(ACT_WAIT_RECOG == action || soko_action_is_rotate(action))
	{
		return SOKO_OK;
	}

	if(SOKO_OK != soko_action_to_dir(action, &dir))
	{
		return SOKO_INVALID_MAP;
	}

	next_player = soko_step_point(map->player, dir);
	if(!soko_in_bounds_point(next_player))
	{
		return SOKO_INVALID_MAP;
	}

	if(soko_action_is_move(action))
	{
		if(soko_replay_is_walk_blocked(map, next_player))
		{
			return SOKO_INVALID_MAP;
		}

		map->player = next_player;
		return SOKO_OK;
	}

	if(soko_action_is_push(action))
	{
		int16_t box_index;
		Point next_box;
		SokoStatus status;

		box_index = soko_map_find_box(map, next_player);
		if(box_index < 0)
		{
			return SOKO_INVALID_MAP;
		}

		next_box = soko_step_point(next_player, dir);
		if(!soko_in_bounds_point(next_box))
		{
			return SOKO_INVALID_MAP;
		}

		if(soko_replay_is_push_blocked(map, next_box))
		{
			return SOKO_INVALID_MAP;
		}

		map->player = next_player;
		status = soko_replay_move_box(map, box_index, next_box);
		if(SOKO_OK != status)
		{
			return status;
		}

		return SOKO_OK;
	}

	return SOKO_INVALID_MAP;
}

SokoStatus soko_replay_apply_sequence(GameMap *map, const ActionSeq *seq, SokoReplayResult *out_result)
{
	uint16_t i;
	SokoReplayResult result;

	if((0 == map) || (0 == seq))
	{
		return SOKO_INVALID_MAP;
	}

	result.status = SOKO_OK;
	result.end_player = map->player;
	result.steps = 0;
	result.pushes = 0;
	result.actions = 0;

	for(i = 0; i < seq->count; i++)
	{
		SokoStatus status = soko_replay_apply_action(map, seq->data[i]);
		if(SOKO_OK != status)
		{
			result.status = status;
			if(0 != out_result)
			{
				*out_result = result;
			}
			return status;
		}

		if(soko_action_is_move(seq->data[i]) || soko_action_is_push(seq->data[i]))
		{
			result.steps++;
		}

		if(soko_action_is_push(seq->data[i]))
		{
			result.pushes++;
		}

		result.actions++;
		result.end_player = map->player;
	}

	if(0 != out_result)
	{
		*out_result = result;
	}

	return SOKO_OK;
}
