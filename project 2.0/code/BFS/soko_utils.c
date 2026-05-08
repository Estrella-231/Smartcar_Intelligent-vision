/*
 * 文件名称: soko_utils.c
 * 创建时间: 2026-03-18
 * 创作者: 王宇铖
 * 说明: 推箱子 BFS 系统通用工具函数实现。
 */

#include "soko_utils.h"

static const int8_t g_soko_dir_dx[4] = {0, 0, -1, 1};
static const int8_t g_soko_dir_dy[4] = {-1, 1, 0, 0};

Point soko_point_make(uint8_t x, uint8_t y)
{
	Point point;

	point.x = x;
	point.y = y;
	return point;
}

bool soko_point_equal(Point a, Point b)
{
	return ((a.x == b.x) && (a.y == b.y));
}

bool soko_in_bounds_xy(int16_t x, int16_t y)
{
	return ((x >= 0) && (x < MAP_W) && (y >= 0) && (y < MAP_H));
}

bool soko_in_bounds_point(Point p)
{
	return soko_in_bounds_xy(p.x, p.y);
}

uint8_t soko_point_to_cell(Point p)
{
	return (uint8_t)(p.y * MAP_W + p.x);
}

Point soko_cell_to_point(uint8_t cell_id)
{
	Point point;

	point.x = (uint8_t)(cell_id % MAP_W);
	point.y = (uint8_t)(cell_id / MAP_W);
	return point;
}

int8_t soko_dir_dx(Dir dir)
{
	return g_soko_dir_dx[(uint8_t)dir];
}

int8_t soko_dir_dy(Dir dir)
{
	return g_soko_dir_dy[(uint8_t)dir];
}

Point soko_step_point(Point p, Dir dir)
{
	Point next;

	next.x = (uint8_t)((int16_t)p.x + soko_dir_dx(dir));
	next.y = (uint8_t)((int16_t)p.y + soko_dir_dy(dir));
	return next;
}

Action soko_action_from_dir(Dir dir, bool is_push)
{
	if(is_push)
	{
		return (Action)(ACT_PUSH_U + (uint8_t)dir);
	}

	return (Action)(ACT_MOVE_U + (uint8_t)dir);
}

bool soko_action_is_move(Action action)
{
	return ((action >= ACT_MOVE_U) && (action <= ACT_MOVE_R));
}

bool soko_action_is_push(Action action)
{
	return ((action >= ACT_PUSH_U) && (action <= ACT_PUSH_R));
}

bool soko_action_is_rotate(Action action)
{
	return ((action >= ACT_ROTATE_0) && (action <= ACT_ROTATE_270));
}

SokoStatus soko_action_to_dir(Action action, Dir *out_dir)
{
	if(0 == out_dir)
	{
		return SOKO_INVALID_MAP;
	}

	if(soko_action_is_move(action))
	{
		*out_dir = (Dir)(action - ACT_MOVE_U);
		return SOKO_OK;
	}

	if(soko_action_is_push(action))
	{
		*out_dir = (Dir)(action - ACT_PUSH_U);
		return SOKO_OK;
	}

	return SOKO_INVALID_MAP;
}

void soko_action_seq_reset(ActionSeq *seq)
{
	if(0 == seq)
	{
		return;
	}

	seq->count = 0;
}

SokoStatus soko_action_seq_push(ActionSeq *seq, Action action)
{
	if(0 == seq)
	{
		return SOKO_INVALID_MAP;
	}

	if(seq->count >= MAX_ACTIONS)
	{
		return SOKO_OVERFLOW_ACTIONS;
	}

	seq->data[seq->count++] = action;
	return SOKO_OK;
}

SokoStatus soko_action_seq_append(ActionSeq *dst, const ActionSeq *src)
{
	uint16_t i;

	if((0 == dst) || (0 == src))
	{
		return SOKO_INVALID_MAP;
	}

	for(i = 0; i < src->count; i++)
	{
		SokoStatus status = soko_action_seq_push(dst, src->data[i]);
		if(SOKO_OK != status)
		{
			return status;
		}
	}

	return SOKO_OK;
}
