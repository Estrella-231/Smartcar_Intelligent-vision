/*
 * 文件名称: soko_walk_bfs.c
 * 创建时间: 2026-03-18
 * 创作者: 王宇铖
 * 说明: 玩家自由移动 BFS 实现。
 */

#include "soko_walk_bfs.h"

#include "soko_map.h"
#include "soko_utils.h"

static uint16_t g_walk_parent[CELL_COUNT];
static uint8_t g_walk_visited[(CELL_COUNT + 7) / 8];
static uint8_t g_walk_queue[CELL_COUNT];
static Action g_walk_action[CELL_COUNT];
static Action g_walk_reverse_action[CELL_COUNT];

static void soko_walk_clear_buffers(void)
{
	uint16_t i;

	for(i = 0; i < CELL_COUNT; i++)
	{
		g_walk_parent[i] = INVALID_ID;
		g_walk_action[i] = ACT_MOVE_U;
		g_walk_reverse_action[i] = ACT_MOVE_U;
	}

	for(i = 0; i < (uint16_t)sizeof(g_walk_visited); i++)
	{
		g_walk_visited[i] = 0;
	}
}

static bool soko_walk_is_visited(uint8_t cell)
{
	return ((g_walk_visited[cell >> 3] >> (cell & 7)) & 0x01U) ? true : false;
}

static void soko_walk_mark_visited(uint8_t cell)
{
	g_walk_visited[cell >> 3] |= (uint8_t)(1U << (cell & 7));
}

static bool soko_walk_is_goal(Point point, const Point *goals, uint8_t goal_count, int16_t *goal_index)
{
	uint8_t i;

	for(i = 0; i < goal_count; i++)
	{
		if(soko_point_equal(point, goals[i]))
		{
			if(0 != goal_index)
			{
				*goal_index = (int16_t)i;
			}
			return true;
		}
	}

	return false;
}

static bool soko_walk_is_blocked(const GameMap *map, Point pos)
{
	return (soko_map_is_wall(map, pos) || soko_map_has_box(map, pos) || soko_map_has_bomb(map, pos));
}

static SokoStatus soko_walk_reconstruct(uint8_t start_cell, uint8_t goal_cell, ActionSeq *out_seq)
{
	uint16_t cur = goal_cell;

	soko_action_seq_reset(out_seq);

	while(cur != start_cell)
	{
		SokoStatus status;

		if(INVALID_ID == g_walk_parent[cur])
		{
			return SOKO_INTERNAL_ERROR;
		}

		status = soko_action_seq_push(out_seq, g_walk_reverse_action[cur]);
		if(SOKO_OK != status)
		{
			return status;
		}

		cur = g_walk_parent[cur];
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

SokoStatus soko_bfs_walk_to_any(const GameMap *map,
								Point start,
								const Point *goals,
								uint8_t goal_count,
								ActionSeq *out_seq,
								Point *out_end,
								int16_t *out_goal_index)
{
	uint16_t head = 0;
	uint16_t tail = 0;
	uint8_t start_cell;
	int16_t hit_goal_index = -1;

	if((0 == map) || (0 == goals) || (0 == out_seq) || (0 == out_end) || (0 == out_goal_index))
	{
		return SOKO_INVALID_MAP;
	}

	if((0 == goal_count) || !soko_in_bounds_point(start))
	{
		return SOKO_INVALID_MAP;
	}

	if(soko_walk_is_goal(start, goals, goal_count, &hit_goal_index))
	{
		soko_action_seq_reset(out_seq);
		*out_end = start;
		*out_goal_index = hit_goal_index;
		return SOKO_OK;
	}

	soko_walk_clear_buffers();
	start_cell = soko_point_to_cell(start);

	g_walk_queue[tail++] = start_cell;
	soko_walk_mark_visited(start_cell);

	while(head < tail)
	{
		uint8_t cur_cell = g_walk_queue[head++];
		Point cur_point = soko_cell_to_point(cur_cell);
		uint8_t dir_index;

		if(soko_walk_is_goal(cur_point, goals, goal_count, &hit_goal_index))
		{
			SokoStatus status = soko_walk_reconstruct(start_cell, cur_cell, out_seq);
			if(SOKO_OK != status)
			{
				return status;
			}

			*out_end = cur_point;
			*out_goal_index = hit_goal_index;
			return SOKO_OK;
		}

		for(dir_index = 0; dir_index < 4; dir_index++)
		{
			Dir dir = (Dir)dir_index;
			Point next_point = soko_step_point(cur_point, dir);
			uint8_t next_cell;

			if(!soko_in_bounds_point(next_point))
			{
				continue;
			}

			if(soko_walk_is_blocked(map, next_point))
			{
				continue;
			}

			next_cell = soko_point_to_cell(next_point);
			if(soko_walk_is_visited(next_cell))
			{
				continue;
			}

			if(tail >= CELL_COUNT)
			{
				return SOKO_OVERFLOW_QUEUE;
			}

			soko_walk_mark_visited(next_cell);
			g_walk_parent[next_cell] = cur_cell;
			g_walk_action[next_cell] = soko_action_from_dir(dir, false);
			g_walk_reverse_action[next_cell] = g_walk_action[next_cell];
			g_walk_queue[tail++] = next_cell;
		}
	}

	soko_action_seq_reset(out_seq);
	*out_end = start;
	*out_goal_index = -1;
	return SOKO_NO_PATH;
}
