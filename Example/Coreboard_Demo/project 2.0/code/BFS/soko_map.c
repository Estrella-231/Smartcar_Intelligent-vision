/*
 * 文件名称: soko_map.c
 * 创建时间: 2026-03-18
 * 创作者: 王宇铖
 * 说明: 推箱子地图解析、查询和更新实现。
 */

#include <string.h>

#include "soko_map.h"
#include "soko_utils.h"

NumberTarget g_soko_number_targets[SOKO_NUMBER_COUNT];

const uint8_t g_soko_class_to_target_num[SOKO_NUMBER_COUNT] =
{
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9
};

static SokoStatus soko_map_add_box(GameMap *map, Point pos)
{
	BoxInfo *box;

	if(map->box_count >= MAX_BOXES)
	{
		return SOKO_OVERFLOW_OBJECTS;
	}

	box = &map->boxes[map->box_count++];
	box->pos = pos;
	box->active = 1;
	box->recognized = 0;
	box->class_id = 0xFF;
	box->target_number = -1;
	box->target_index = -1;
	return SOKO_OK;
}

static SokoStatus soko_map_add_target(GameMap *map, Point pos)
{
	NumberTarget *target;

	if(map->target_count >= MAX_TARGETS)
	{
		return SOKO_OVERFLOW_OBJECTS;
	}

	target = &map->targets[map->target_count++];
	target->pos = pos;
	target->active = 1;
	map->generic_target_mask[pos.y][pos.x] = 1;
	return SOKO_OK;
}

static SokoStatus soko_map_add_bomb(GameMap *map, Point pos)
{
	if(map->bomb_count >= MAX_BOMBS)
	{
		return SOKO_OVERFLOW_OBJECTS;
	}

	map->bombs[map->bomb_count++] = pos;
	return SOKO_OK;
}

void soko_number_targets_clear(void)
{
	memset(g_soko_number_targets, 0, sizeof(g_soko_number_targets));
}

void soko_map_clear(GameMap *map)
{
	uint8_t y;
	uint8_t x;

	if(0 == map)
	{
		return;
	}

	memset(map, 0, sizeof(*map));

	for(y = 0; y < MAP_H; y++)
	{
		for(x = 0; x < MAP_W; x++)
		{
			map->number_target_map[y][x] = -1;
		}
	}
}

void soko_map_copy(GameMap *dst, const GameMap *src)
{
	if((0 == dst) || (0 == src))
	{
		return;
	}

	memcpy(dst, src, sizeof(*dst));
}

SokoStatus soko_map_parse_ascii(const char raw[MAP_H][MAP_W + 1], GameMap *out_map)
{
	uint8_t y;
	uint8_t x;
	uint8_t player_count = 0;

	if((0 == raw) || (0 == out_map))
	{
		return SOKO_INVALID_MAP;
	}

	soko_map_clear(out_map);

	for(y = 0; y < MAP_H; y++)
	{
		for(x = 0; x < MAP_W; x++)
		{
			Point pos = soko_point_make(x, y);
			char cell = raw[y][x];
			SokoStatus status = SOKO_OK;

			switch(cell)
			{
				case SOKO_CHAR_EMPTY:
					out_map->base[y][x] = SOKO_BASE_EMPTY;
					break;

				case SOKO_CHAR_WALL:
					out_map->base[y][x] = SOKO_BASE_WALL;
					break;

				case SOKO_CHAR_TARGET:
					out_map->base[y][x] = SOKO_BASE_EMPTY;
					status = soko_map_add_target(out_map, pos);
					break;

				case SOKO_CHAR_BOX:
					out_map->base[y][x] = SOKO_BASE_EMPTY;
					status = soko_map_add_box(out_map, pos);
					break;

				case SOKO_CHAR_BOMB:
					out_map->base[y][x] = SOKO_BASE_EMPTY;
					status = soko_map_add_bomb(out_map, pos);
					break;

				case SOKO_CHAR_PLAYER:
					out_map->base[y][x] = SOKO_BASE_EMPTY;
					out_map->player = pos;
					player_count++;
					break;

				default:
					return SOKO_INVALID_MAP;
			}

			if(SOKO_OK != status)
			{
				return status;
			}
		}
	}

	if(1 != player_count)
	{
		return SOKO_INVALID_MAP;
	}

	return SOKO_OK;
}

SokoStatus soko_map_bind_number_target(GameMap *map, uint8_t target_number, Point pos)
{
	int16_t target_index;

	if((0 == map) || (target_number >= SOKO_NUMBER_COUNT) || !soko_in_bounds_point(pos))
	{
		return SOKO_INVALID_MAP;
	}

	if(SOKO_BASE_WALL == map->base[pos.y][pos.x])
	{
		return SOKO_INVALID_MAP;
	}

	g_soko_number_targets[target_number].active = 1;
	g_soko_number_targets[target_number].pos = pos;
	map->number_target_map[pos.y][pos.x] = (int8_t)target_number;
	map->generic_target_mask[pos.y][pos.x] = 1;

	target_index = soko_map_find_target(map, pos);
	if(target_index < 0)
	{
		return soko_map_add_target(map, pos);
	}

	return SOKO_OK;
}

bool soko_map_is_wall(const GameMap *map, Point pos)
{
	if((0 == map) || !soko_in_bounds_point(pos))
	{
		return true;
	}

	return (SOKO_BASE_WALL == map->base[pos.y][pos.x]);
}

int16_t soko_map_find_box(const GameMap *map, Point pos)
{
	uint8_t i;

	if(0 == map)
	{
		return -1;
	}

	for(i = 0; i < map->box_count; i++)
	{
		if(map->boxes[i].active && soko_point_equal(map->boxes[i].pos, pos))
		{
			return (int16_t)i;
		}
	}

	return -1;
}

int16_t soko_map_find_target(const GameMap *map, Point pos)
{
	uint8_t i;

	if(0 == map)
	{
		return -1;
	}

	for(i = 0; i < map->target_count; i++)
	{
		if(map->targets[i].active && soko_point_equal(map->targets[i].pos, pos))
		{
			return (int16_t)i;
		}
	}

	return -1;
}

int16_t soko_map_find_bomb(const GameMap *map, Point pos)
{
	uint8_t i;

	if(0 == map)
	{
		return -1;
	}

	for(i = 0; i < map->bomb_count; i++)
	{
		if(soko_point_equal(map->bombs[i], pos))
		{
			return (int16_t)i;
		}
	}

	return -1;
}

bool soko_map_has_box(const GameMap *map, Point pos)
{
	return (soko_map_find_box(map, pos) >= 0);
}

bool soko_map_has_target(const GameMap *map, Point pos)
{
	return (soko_map_find_target(map, pos) >= 0);
}

bool soko_map_has_bomb(const GameMap *map, Point pos)
{
	return (soko_map_find_bomb(map, pos) >= 0);
}

SokoStatus soko_map_remove_box_at(GameMap *map, Point pos)
{
	int16_t index;
	uint8_t i;

	if(0 == map)
	{
		return SOKO_INVALID_MAP;
	}

	index = soko_map_find_box(map, pos);
	if(index < 0)
	{
		return SOKO_INVALID_MAP;
	}

	for(i = (uint8_t)index; (i + 1) < map->box_count; i++)
	{
		map->boxes[i] = map->boxes[i + 1];
	}

	if(map->box_count > 0)
	{
		map->box_count--;
	}

	return SOKO_OK;
}

SokoStatus soko_map_remove_target_at(GameMap *map, Point pos)
{
	int16_t index;
	uint8_t i;
	int8_t target_number;

	if(0 == map)
	{
		return SOKO_INVALID_MAP;
	}

	index = soko_map_find_target(map, pos);
	if(index < 0)
	{
		return SOKO_INVALID_MAP;
	}

	for(i = (uint8_t)index; (i + 1) < map->target_count; i++)
	{
		map->targets[i] = map->targets[i + 1];
	}

	if(map->target_count > 0)
	{
		map->target_count--;
	}

	map->generic_target_mask[pos.y][pos.x] = 0;

	target_number = map->number_target_map[pos.y][pos.x];
	if(target_number >= 0)
	{
		g_soko_number_targets[(uint8_t)target_number].active = 0;
		map->number_target_map[pos.y][pos.x] = -1;
	}

	return SOKO_OK;
}

SokoStatus soko_map_remove_bomb_at(GameMap *map, Point pos)
{
	int16_t index;
	uint8_t i;

	if(0 == map)
	{
		return SOKO_INVALID_MAP;
	}

	index = soko_map_find_bomb(map, pos);
	if(index < 0)
	{
		return SOKO_INVALID_MAP;
	}

	for(i = (uint8_t)index; (i + 1) < map->bomb_count; i++)
	{
		map->bombs[i] = map->bombs[i + 1];
	}

	if(map->bomb_count > 0)
	{
		map->bomb_count--;
	}

	return SOKO_OK;
}

int8_t soko_map_get_target_number(const GameMap *map, Point pos)
{
	if((0 == map) || !soko_in_bounds_point(pos))
	{
		return -1;
	}

	return map->number_target_map[pos.y][pos.x];
}

void soko_map_clear_explosion(GameMap *map, Point center)
{
	int16_t y;
	int16_t x;

	if((0 == map) || !soko_in_bounds_point(center))
	{
		return;
	}

	for(y = (int16_t)center.y - 1; y <= (int16_t)center.y + 1; y++)
	{
		for(x = (int16_t)center.x - 1; x <= (int16_t)center.x + 1; x++)
		{
			if(!soko_in_bounds_xy(x, y))
			{
				continue;
			}

			if((0 == x) || (0 == y) || (MAP_W - 1 == x) || (MAP_H - 1 == y))
			{
				continue;
			}

			if(SOKO_BASE_WALL == map->base[y][x])
			{
				map->base[y][x] = SOKO_BASE_EMPTY;
			}
		}
	}
}
