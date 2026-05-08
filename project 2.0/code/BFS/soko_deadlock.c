/*
 * 文件名称: soko_deadlock.c
 * 创建时间: 2026-03-18
 * 创作者: 王宇铖
 * 说明: 推箱子基础死锁判定实现。
 */

#include "soko_deadlock.h"

#include "soko_map.h"
#include "soko_utils.h"

static bool soko_deadlock_is_blocked(const GameMap *map, Point pos, Point ignore_box_pos)
{
	if(!soko_in_bounds_point(pos))
	{
		return true;
	}

	if(soko_map_is_wall(map, pos))
	{
		return true;
	}

	if(soko_point_equal(pos, ignore_box_pos))
	{
		return false;
	}

	return soko_map_has_box(map, pos);
}

bool soko_deadlock_is_corner(const GameMap *map,
							 Point box_pos,
							 Point ignore_box_pos,
							 const uint8_t target_mask[MAP_H][MAP_W])
{
	bool up;
	bool down;
	bool left;
	bool right;

	if((0 == map) || (0 == target_mask) || !soko_in_bounds_point(box_pos))
	{
		return false;
	}

	if(target_mask[box_pos.y][box_pos.x])
	{
		return false;
	}

	up = soko_deadlock_is_blocked(map, soko_step_point(box_pos, DIR_UP), ignore_box_pos);
	down = soko_deadlock_is_blocked(map, soko_step_point(box_pos, DIR_DOWN), ignore_box_pos);
	left = soko_deadlock_is_blocked(map, soko_step_point(box_pos, DIR_LEFT), ignore_box_pos);
	right = soko_deadlock_is_blocked(map, soko_step_point(box_pos, DIR_RIGHT), ignore_box_pos);

	return ((up && left) || (up && right) || (down && left) || (down && right));
}
