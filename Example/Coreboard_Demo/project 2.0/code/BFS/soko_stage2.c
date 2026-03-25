/*
 * 文件名称: soko_stage2.c
 * 创建时间: 2026-03-18
 * 创作者: 王宇铖
 * 说明: 第二阶段稳定版观察、识别绑定与推箱调度实现。
 */

#include <string.h>

#include "soko_stage2.h"

#include "soko_map.h"
#include "soko_push_bfs.h"
#include "soko_replay.h"
#include "soko_utils.h"
#include "soko_walk_bfs.h"

static Action soko_stage2_face_action(Point observe_point, Point box_point)
{
	if((observe_point.x == box_point.x) && (observe_point.y + 1 == box_point.y))
	{
		return ACT_ROTATE_180;
	}

	if((observe_point.x == box_point.x) && (observe_point.y == box_point.y + 1))
	{
		return ACT_ROTATE_0;
	}

	if((observe_point.x + 1 == box_point.x) && (observe_point.y == box_point.y))
	{
		return ACT_ROTATE_90;
	}

	return ACT_ROTATE_270;
}

static bool soko_stage2_is_observe_point_valid(const GameMap *map, Point point)
{
	if(!soko_in_bounds_point(point))
	{
		return false;
	}

	return !(soko_map_is_wall(map, point) || soko_map_has_box(map, point) || soko_map_has_bomb(map, point));
}

static void soko_stage2_build_single_target_mask(const GameMap *map,
												 uint8_t target_number,
												 uint8_t out_mask[MAP_H][MAP_W])
{
	uint8_t y;
	uint8_t x;

	for(y = 0; y < MAP_H; y++)
	{
		for(x = 0; x < MAP_W; x++)
		{
			out_mask[y][x] = 0;
		}
	}

	if((target_number < SOKO_NUMBER_COUNT) && g_soko_number_targets[target_number].active)
	{
		Point pos = g_soko_number_targets[target_number].pos;
		if(!soko_map_is_wall(map, pos))
		{
			out_mask[pos.y][pos.x] = 1;
		}
	}
}

static bool soko_stage2_result_better(const SolveResult *lhs, const SolveResult *rhs)
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

uint8_t soko_stage2_build_observe_points(const GameMap *map,
										 uint8_t box_index,
										 Point *out_points,
										 Action *out_face_actions,
										 uint8_t max_points)
{
	static const Dir k_dirs[4] = {DIR_UP, DIR_DOWN, DIR_LEFT, DIR_RIGHT};
	uint8_t count = 0;
	uint8_t i;
	Point box_pos;

	if((0 == map) || (0 == out_points) || (0 == out_face_actions) || (box_index >= map->box_count))
	{
		return 0;
	}

	box_pos = map->boxes[box_index].pos;

	for(i = 0; i < 4; i++)
	{
		Point observe_point = soko_step_point(box_pos, k_dirs[i]);
		if(count >= max_points)
		{
			break;
		}

		if(!soko_stage2_is_observe_point_valid(map, observe_point))
		{
			continue;
		}

		out_points[count] = observe_point;
		out_face_actions[count] = soko_stage2_face_action(observe_point, box_pos);
		count++;
	}

	return count;
}

SokoStatus soko_stage2_plan_observe_box(const GameMap *map,
										uint8_t box_index,
										ActionSeq *out_seq,
										Point *out_end)
{
	Point observe_points[4];
	Action face_actions[4];
	uint8_t goal_count;
	int16_t goal_index;
	ActionSeq walk_seq;
	SokoStatus status;

	if((0 == map) || (0 == out_seq) || (0 == out_end) || (box_index >= map->box_count))
	{
		return SOKO_INVALID_MAP;
	}

	goal_count = soko_stage2_build_observe_points(map, box_index, observe_points, face_actions, 4);
	if(0 == goal_count)
	{
		return SOKO_NO_PATH;
	}

	status = soko_bfs_walk_to_any(map, map->player, observe_points, goal_count, &walk_seq, out_end, &goal_index);
	if(SOKO_OK != status)
	{
		return status;
	}

	soko_action_seq_reset(out_seq);
	status = soko_action_seq_append(out_seq, &walk_seq);
	if(SOKO_OK != status)
	{
		return status;
	}

	status = soko_action_seq_push(out_seq, face_actions[goal_index]);
	if(SOKO_OK != status)
	{
		return status;
	}

	return soko_action_seq_push(out_seq, ACT_WAIT_RECOG);
}

SokoStatus soko_stage2_bind_box_target(GameMap *map, uint8_t box_index, uint8_t class_id)
{
	uint8_t target_number;
	Point target_pos;
	int16_t target_index;

	if((0 == map) || (box_index >= map->box_count) || (class_id >= SOKO_NUMBER_COUNT))
	{
		return SOKO_INVALID_MAP;
	}

	target_number = g_soko_class_to_target_num[class_id];
	if((target_number >= SOKO_NUMBER_COUNT) || !g_soko_number_targets[target_number].active)
	{
		return SOKO_INVALID_MAP;
	}

	target_pos = g_soko_number_targets[target_number].pos;
	target_index = soko_map_find_target(map, target_pos);
	if(target_index < 0)
	{
		return SOKO_INVALID_MAP;
	}

	map->boxes[box_index].recognized = 1;
	map->boxes[box_index].class_id = class_id;
	map->boxes[box_index].target_number = (int8_t)target_number;
	map->boxes[box_index].target_index = (int8_t)target_index;
	return SOKO_OK;
}

SokoStatus soko_stage2_plan_bound_boxes(GameMap *map, ActionSeq *out_seq)
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
			uint8_t target_mask[MAP_H][MAP_W];
			SolveResult current_result;
			SokoStatus status;

			if(!map->boxes[i].active || !map->boxes[i].recognized || (map->boxes[i].target_number < 0))
			{
				continue;
			}

			if(!g_soko_number_targets[(uint8_t)map->boxes[i].target_number].active)
			{
				continue;
			}

			soko_stage2_build_single_target_mask(map, (uint8_t)map->boxes[i].target_number, target_mask);
			status = soko_bfs_push_box_to_any_target(map,
													 map->player,
													 map->boxes[i].pos,
													 target_mask,
													 &current_result);
			if(SOKO_OK != status)
			{
				continue;
			}

			current_result.chosen_box = (int16_t)i;
			current_result.chosen_target = map->boxes[i].target_number;

			if(!best_found || soko_stage2_result_better(&current_result, &best_result))
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
