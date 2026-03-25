/*
 * 文件名称: soko_push_bfs.c
 * 创建时间: 2026-03-18
 * 创作者: 王宇铖
 * 说明: 单箱推箱 BFS 实现。
 */

#include "soko_push_bfs.h"

#include "soko_deadlock.h"
#include "soko_map.h"
#include "soko_utils.h"

static uint16_t g_push_queue[MAX_STATES];
static uint16_t g_push_parent[MAX_STATES];
static uint8_t g_push_visited[(MAX_STATES + 7) / 8];
static Action g_push_action[MAX_STATES];

static uint16_t soko_push_encode_state(Point box, Point player)
{
	return (uint16_t)(soko_point_to_cell(box) * CELL_COUNT + soko_point_to_cell(player));
}

static void soko_push_decode_state(uint16_t state, Point *box, Point *player)
{
	uint8_t box_cell;
	uint8_t player_cell;

	box_cell = (uint8_t)(state / CELL_COUNT);
	player_cell = (uint8_t)(state % CELL_COUNT);

	*box = soko_cell_to_point(box_cell);
	*player = soko_cell_to_point(player_cell);
}

static void soko_push_clear_buffers(void)
{
	uint16_t i;

	for(i = 0; i < MAX_STATES; i++)
	{
		g_push_parent[i] = INVALID_ID;
		g_push_action[i] = ACT_MOVE_U;
	}

	for(i = 0; i < (uint16_t)sizeof(g_push_visited); i++)
	{
		g_push_visited[i] = 0;
	}
}

static bool soko_push_is_visited(uint16_t state)
{
	return ((g_push_visited[state >> 3] >> (state & 7)) & 0x01U) ? true : false;
}

static void soko_push_mark_visited(uint16_t state)
{
	g_push_visited[state >> 3] |= (uint8_t)(1U << (state & 7));
}

static bool soko_push_player_blocked(const GameMap *map, Point pos, Point active_box)
{
	if(!soko_in_bounds_point(pos))
	{
		return true;
	}

	if(soko_map_is_wall(map, pos) || soko_map_has_bomb(map, pos))
	{
		return true;
	}

	if(soko_point_equal(pos, active_box))
	{
		return false;
	}

	return soko_map_has_box(map, pos);
}

static bool soko_push_box_blocked(const GameMap *map, Point pos, Point active_box)
{
	if(!soko_in_bounds_point(pos))
	{
		return true;
	}

	if(soko_map_is_wall(map, pos) || soko_map_has_bomb(map, pos))
	{
		return true;
	}

	if(soko_point_equal(pos, active_box))
	{
		return false;
	}

	return soko_map_has_box(map, pos);
}

static void soko_push_fill_result_defaults(SolveResult *result)
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

static SokoStatus soko_push_reconstruct(uint16_t start_state, uint16_t goal_state, ActionSeq *out_seq)
{
	uint16_t cur = goal_state;

	soko_action_seq_reset(out_seq);

	while(cur != start_state)
	{
		SokoStatus status;

		if(INVALID_ID == g_push_parent[cur])
		{
			return SOKO_INTERNAL_ERROR;
		}

		status = soko_action_seq_push(out_seq, g_push_action[cur]);
		if(SOKO_OK != status)
		{
			return status;
		}

		cur = g_push_parent[cur];
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

SokoStatus soko_bfs_push_box_to_any_target(const GameMap *map,
										   Point player_start,
										   Point box_start,
										   const uint8_t candidate_target_mask[MAP_H][MAP_W],
										   SolveResult *out_result)
{
	uint16_t head = 0;
	uint16_t tail = 0;
	uint16_t start_state;

	if((0 == map) || (0 == candidate_target_mask) || (0 == out_result))
	{
		return SOKO_INVALID_MAP;
	}

	soko_push_fill_result_defaults(out_result);

	if(!soko_in_bounds_point(player_start) || !soko_in_bounds_point(box_start))
	{
		return SOKO_INVALID_MAP;
	}

	if(!soko_map_has_box(map, box_start))
	{
		return SOKO_INVALID_MAP;
	}

	if(candidate_target_mask[box_start.y][box_start.x])
	{
		out_result->status = SOKO_OK;
		out_result->chosen_target = soko_map_find_target(map, box_start);
		out_result->end_player = player_start;
		return SOKO_OK;
	}

	soko_push_clear_buffers();
	start_state = soko_push_encode_state(box_start, player_start);

	g_push_queue[tail++] = start_state;
	soko_push_mark_visited(start_state);

	while(head < tail)
	{
		uint16_t cur_state = g_push_queue[head++];
		Point cur_box;
		Point cur_player;
		uint8_t dir_index;

		soko_push_decode_state(cur_state, &cur_box, &cur_player);

		if(candidate_target_mask[cur_box.y][cur_box.x])
		{
			SokoStatus status = soko_push_reconstruct(start_state, cur_state, &out_result->seq);
			uint16_t i;

			if(SOKO_OK != status)
			{
				return status;
			}

			out_result->status = SOKO_OK;
			out_result->chosen_target = soko_map_find_target(map, cur_box);
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

			if(!soko_point_equal(next_player, cur_box))
			{
				if(soko_push_player_blocked(map, next_player, cur_box))
				{
					continue;
				}

				next_state = soko_push_encode_state(cur_box, next_player);
				if(soko_push_is_visited(next_state))
				{
					continue;
				}

				if(tail >= MAX_STATES)
				{
					return SOKO_OVERFLOW_QUEUE;
				}

				soko_push_mark_visited(next_state);
				g_push_parent[next_state] = cur_state;
				g_push_action[next_state] = soko_action_from_dir(dir, false);
				g_push_queue[tail++] = next_state;
			}
			else
			{
				Point next_box = soko_step_point(cur_box, dir);

				if(!soko_in_bounds_point(next_box))
				{
					continue;
				}

				if(soko_push_box_blocked(map, next_box, cur_box))
				{
					continue;
				}

				if(soko_deadlock_is_corner(map, next_box, cur_box, candidate_target_mask))
				{
					continue;
				}

				next_state = soko_push_encode_state(next_box, cur_box);
				if(soko_push_is_visited(next_state))
				{
					continue;
				}

				if(tail >= MAX_STATES)
				{
					return SOKO_OVERFLOW_QUEUE;
				}

				soko_push_mark_visited(next_state);
				g_push_parent[next_state] = cur_state;
				g_push_action[next_state] = soko_action_from_dir(dir, true);
				g_push_queue[tail++] = next_state;
			}
		}
	}

	return SOKO_NO_PATH;
}
