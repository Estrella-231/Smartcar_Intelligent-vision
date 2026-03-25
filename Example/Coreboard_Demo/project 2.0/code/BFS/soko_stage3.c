/*
 * 文件名称: soko_stage3.c
 * 创建时间: 2026-03-18
 * 创作者: 王宇铖
 * 说明: 第三阶段炸弹策略与单箱恢复求解实现。
 */

#include "soko_stage3.h"

#include "soko_map.h"
#include "soko_push_bfs.h"
#include "soko_replay.h"
#include "soko_utils.h"

static uint16_t g_bomb_queue[MAX_STATES];
static uint16_t g_bomb_parent[MAX_STATES];
static uint8_t g_bomb_visited[(MAX_STATES + 7) / 8];
static Action g_bomb_action[MAX_STATES];

static uint16_t soko_bomb_encode_state(Point bomb, Point player)
{
	return (uint16_t)(soko_point_to_cell(bomb) * CELL_COUNT + soko_point_to_cell(player));
}

static void soko_bomb_decode_state(uint16_t state, Point *bomb, Point *player)
{
	uint8_t bomb_cell = (uint8_t)(state / CELL_COUNT);
	uint8_t player_cell = (uint8_t)(state % CELL_COUNT);

	*bomb = soko_cell_to_point(bomb_cell);
	*player = soko_cell_to_point(player_cell);
}

static void soko_bomb_clear_buffers(void)
{
	uint16_t i;

	for(i = 0; i < MAX_STATES; i++)
	{
		g_bomb_parent[i] = INVALID_ID;
		g_bomb_action[i] = ACT_MOVE_U;
	}

	for(i = 0; i < (uint16_t)sizeof(g_bomb_visited); i++)
	{
		g_bomb_visited[i] = 0;
	}
}

static bool soko_bomb_is_visited(uint16_t state)
{
	return ((g_bomb_visited[state >> 3] >> (state & 7)) & 0x01U) ? true : false;
}

static void soko_bomb_mark_visited(uint16_t state)
{
	g_bomb_visited[state >> 3] |= (uint8_t)(1U << (state & 7));
}

static bool soko_bomb_player_blocked(const GameMap *map, Point pos, Point active_bomb)
{
	if(!soko_in_bounds_point(pos))
	{
		return true;
	}

	if(soko_map_is_wall(map, pos))
	{
		return true;
	}

	if(soko_point_equal(pos, active_bomb))
	{
		return false;
	}

	return (soko_map_has_box(map, pos) || soko_map_has_bomb(map, pos));
}

static bool soko_bomb_target_allowed(const GameMap *map, Point pos, Point active_bomb, Point wall_target)
{
	if(!soko_in_bounds_point(pos))
	{
		return false;
	}

	if(soko_point_equal(pos, wall_target))
	{
		return soko_map_is_wall(map, pos);
	}

	if(soko_map_is_wall(map, pos))
	{
		return false;
	}

	if(soko_point_equal(pos, active_bomb))
	{
		return true;
	}

	return !(soko_map_has_box(map, pos) || soko_map_has_bomb(map, pos));
}

static void soko_stage3_fill_result_defaults(SolveResult *result)
{
	result->status = SOKO_NO_PATH;
	result->steps = 0;
	result->pushes = 0;
	result->chosen_box = -1;
	result->chosen_target = -1;
	result->chosen_bomb = -1;
	result->end_player = soko_point_make(0, 0);
	soko_action_seq_reset(&result->seq);
}

static SokoStatus soko_bomb_reconstruct(uint16_t start_state, uint16_t goal_state, ActionSeq *out_seq)
{
	uint16_t cur = goal_state;

	soko_action_seq_reset(out_seq);

	while(cur != start_state)
	{
		SokoStatus status;

		if(INVALID_ID == g_bomb_parent[cur])
		{
			return SOKO_INTERNAL_ERROR;
		}

		status = soko_action_seq_push(out_seq, g_bomb_action[cur]);
		if(SOKO_OK != status)
		{
			return status;
		}

		cur = g_bomb_parent[cur];
	}

	if(out_seq->count > 1)
	{
		uint16_t left = 0;
		uint16_t right = (uint16_t)(out_seq->count - 1);

		while(left < right)
		{
			Action temp = out_seq->data[left];
			out_seq->data[left] = out_seq->data[right];
			out_seq->data[right] = temp;
			left++;
			right--;
		}
	}

	return SOKO_OK;
}

static SokoStatus soko_stage3_apply_bomb_sequence(GameMap *map, const ActionSeq *seq, Point wall_target)
{
	uint16_t i;

	if((0 == map) || (0 == seq))
	{
		return SOKO_INVALID_MAP;
	}

	for(i = 0; i < seq->count; i++)
	{
		Action action = seq->data[i];
		Dir dir;
		Point next_player;

		if(ACT_WAIT_RECOG == action || soko_action_is_rotate(action))
		{
			continue;
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
			if(soko_map_is_wall(map, next_player) || soko_map_has_box(map, next_player) || soko_map_has_bomb(map, next_player))
			{
				return SOKO_INVALID_MAP;
			}

			map->player = next_player;
			continue;
		}

		if(soko_action_is_push(action))
		{
			int16_t bomb_index = soko_map_find_bomb(map, next_player);
			Point next_bomb = soko_step_point(next_player, dir);

			if(bomb_index < 0)
			{
				return SOKO_INVALID_MAP;
			}

			if(soko_point_equal(next_bomb, wall_target))
			{
				map->player = next_player;
				if(SOKO_OK != soko_map_remove_bomb_at(map, next_player))
				{
					return SOKO_INTERNAL_ERROR;
				}

				map->base[wall_target.y][wall_target.x] = SOKO_BASE_EMPTY;
				soko_map_clear_explosion(map, wall_target);
				continue;
			}

			if(soko_map_is_wall(map, next_bomb) || soko_map_has_box(map, next_bomb) || soko_map_has_bomb(map, next_bomb))
			{
				return SOKO_INVALID_MAP;
			}

			map->player = next_player;
			map->bombs[bomb_index] = next_bomb;
			continue;
		}

		return SOKO_INVALID_MAP;
	}

	return SOKO_OK;
}

static bool soko_stage3_is_inner_wall(const GameMap *map, Point pos)
{
	if((0 == map) || !soko_in_bounds_point(pos))
	{
		return false;
	}

	if((0 == pos.x) || (0 == pos.y) || ((MAP_W - 1) == pos.x) || ((MAP_H - 1) == pos.y))
	{
		return false;
	}

	return soko_map_is_wall(map, pos);
}

SokoStatus soko_bfs_push_bomb_to_wall(const GameMap *map,
									  Point player_start,
									  Point bomb_start,
									  Point wall_target,
									  SolveResult *out_result)
{
	uint16_t head = 0;
	uint16_t tail = 0;
	uint16_t start_state;

	if((0 == map) || (0 == out_result))
	{
		return SOKO_INVALID_MAP;
	}

	soko_stage3_fill_result_defaults(out_result);

	if(!soko_in_bounds_point(player_start) || !soko_in_bounds_point(bomb_start) || !soko_stage3_is_inner_wall(map, wall_target))
	{
		return SOKO_INVALID_MAP;
	}

	if(!soko_map_has_bomb(map, bomb_start))
	{
		return SOKO_INVALID_MAP;
	}

	soko_bomb_clear_buffers();
	start_state = soko_bomb_encode_state(bomb_start, player_start);
	g_bomb_queue[tail++] = start_state;
	soko_bomb_mark_visited(start_state);

	while(head < tail)
	{
		uint16_t cur_state = g_bomb_queue[head++];
		Point cur_bomb;
		Point cur_player;
		uint8_t dir_index;

		soko_bomb_decode_state(cur_state, &cur_bomb, &cur_player);

		if(soko_point_equal(cur_bomb, wall_target))
		{
			SokoStatus status = soko_bomb_reconstruct(start_state, cur_state, &out_result->seq);
			uint16_t i;

			if(SOKO_OK != status)
			{
				return status;
			}

			out_result->status = SOKO_OK;
			out_result->end_player = cur_player;
			for(i = 0; i < out_result->seq.count; i++)
			{
				out_result->steps++;
				if(soko_action_is_push(out_result->seq.data[i]))
				{
					out_result->pushes++;
				}
			}
			return SOKO_OK;
		}

		for(dir_index = 0; dir_index < 4; dir_index++)
		{
			Dir dir = (Dir)dir_index;
			Point next_player = soko_step_point(cur_player, dir);
			uint16_t next_state;

			if(!soko_in_bounds_point(next_player))
			{
				continue;
			}

			if(!soko_point_equal(next_player, cur_bomb))
			{
				if(soko_bomb_player_blocked(map, next_player, cur_bomb))
				{
					continue;
				}

				next_state = soko_bomb_encode_state(cur_bomb, next_player);
				if(soko_bomb_is_visited(next_state))
				{
					continue;
				}

				if(tail >= MAX_STATES)
				{
					return SOKO_OVERFLOW_QUEUE;
				}

				soko_bomb_mark_visited(next_state);
				g_bomb_parent[next_state] = cur_state;
				g_bomb_action[next_state] = soko_action_from_dir(dir, false);
				g_bomb_queue[tail++] = next_state;
			}
			else
			{
				Point next_bomb = soko_step_point(cur_bomb, dir);

				if(!soko_bomb_target_allowed(map, next_bomb, cur_bomb, wall_target))
				{
					continue;
				}

				next_state = soko_bomb_encode_state(next_bomb, cur_bomb);
				if(soko_bomb_is_visited(next_state))
				{
					continue;
				}

				if(tail >= MAX_STATES)
				{
					return SOKO_OVERFLOW_QUEUE;
				}

				soko_bomb_mark_visited(next_state);
				g_bomb_parent[next_state] = cur_state;
				g_bomb_action[next_state] = soko_action_from_dir(dir, true);
				g_bomb_queue[tail++] = next_state;
			}
		}
	}

	return SOKO_NO_PATH;
}

SokoStatus soko_plan_stage3_with_bomb(GameMap *map,
									  Point box_pos,
									  const uint8_t target_mask[MAP_H][MAP_W],
									  ActionSeq *out_seq)
{
	GameMap trial_map;
	SolveResult direct_result;
	uint8_t y;
	uint8_t x;

	if((0 == map) || (0 == target_mask) || (0 == out_seq))
	{
		return SOKO_INVALID_MAP;
	}

	soko_action_seq_reset(out_seq);

	if(SOKO_OK == soko_bfs_push_box_to_any_target(map, map->player, box_pos, target_mask, &direct_result))
	{
		if(SOKO_OK != soko_action_seq_append(out_seq, &direct_result.seq))
		{
			return SOKO_OVERFLOW_ACTIONS;
		}

		if(SOKO_OK != soko_replay_apply_sequence(map, &direct_result.seq, 0))
		{
			return SOKO_INTERNAL_ERROR;
		}

		return SOKO_OK;
	}

	{
		bool best_found = false;
		ActionSeq best_bomb_seq;
		ActionSeq best_box_seq;
		Point best_wall = soko_point_make(0, 0);
		uint16_t best_cost = 0xFFFF;

		for(y = 1; y < (MAP_H - 1); y++)
		{
			for(x = 1; x < (MAP_W - 1); x++)
			{
				Point wall_target = soko_point_make(x, y);
				uint8_t bomb_index;

				if(!soko_stage3_is_inner_wall(map, wall_target))
				{
					continue;
				}

				for(bomb_index = 0; bomb_index < map->bomb_count; bomb_index++)
				{
					SolveResult bomb_result;
					SolveResult box_result;
					uint16_t total_cost;
					SokoStatus status;

					status = soko_bfs_push_bomb_to_wall(map,
														 map->player,
														 map->bombs[bomb_index],
														 wall_target,
														 &bomb_result);
					if(SOKO_OK != status)
					{
						continue;
					}

					soko_map_copy(&trial_map, map);
					status = soko_stage3_apply_bomb_sequence(&trial_map, &bomb_result.seq, wall_target);
					if(SOKO_OK != status)
					{
						continue;
					}

					status = soko_bfs_push_box_to_any_target(&trial_map,
															 trial_map.player,
															 box_pos,
															 target_mask,
															 &box_result);
					if(SOKO_OK != status)
					{
						continue;
					}

					total_cost = (uint16_t)(bomb_result.steps + box_result.steps);
					if(!best_found || (total_cost < best_cost))
					{
						best_found = true;
						best_cost = total_cost;
						best_bomb_seq = bomb_result.seq;
						best_box_seq = box_result.seq;
						best_wall = wall_target;
					}
				}
			}
		}

		if(!best_found)
		{
			return SOKO_NO_PATH;
		}

		if(SOKO_OK != soko_action_seq_append(out_seq, &best_bomb_seq))
		{
			return SOKO_OVERFLOW_ACTIONS;
		}

		if(SOKO_OK != soko_stage3_apply_bomb_sequence(map, &best_bomb_seq, best_wall))
		{
			return SOKO_INTERNAL_ERROR;
		}

		if(SOKO_OK != soko_action_seq_append(out_seq, &best_box_seq))
		{
			return SOKO_OVERFLOW_ACTIONS;
		}

		if(SOKO_OK != soko_replay_apply_sequence(map, &best_box_seq, 0))
		{
			return SOKO_INTERNAL_ERROR;
		}
	}

	return SOKO_OK;
}
