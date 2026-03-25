/*
 * 文件名称: soko_types.h
 * 创建时间: 2026-03-18
 * 创作者: 王宇铖
 * 说明: 推箱子 BFS 系统通用类型、常量和状态码定义。
 */

#ifndef _SOKO_TYPES_H
#define _SOKO_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#define MAP_W 16
#define MAP_H 12
#define CELL_COUNT (MAP_W * MAP_H)

#define MAX_BOXES 20
#define MAX_TARGETS 20
#define MAX_BOMBS 20

#define MAX_ACTIONS 2048
#define MAX_SEGMENTS 1024
#define MAX_STATES (CELL_COUNT * CELL_COUNT)

#define INVALID_ID 0xFFFF
#define SOKO_NUMBER_COUNT 10

#define SOKO_CHAR_EMPTY '-'
#define SOKO_CHAR_WALL '#'
#define SOKO_CHAR_TARGET '.'
#define SOKO_CHAR_BOX '$'
#define SOKO_CHAR_BOMB 'X'
#define SOKO_CHAR_PLAYER '@'

#define SOKO_BASE_EMPTY 0
#define SOKO_BASE_WALL 1

typedef struct
{
	uint8_t x;
	uint8_t y;
} Point;

typedef enum
{
	DIR_UP = 0,
	DIR_DOWN,
	DIR_LEFT,
	DIR_RIGHT
} Dir;

typedef enum
{
	ACT_MOVE_U = 0,
	ACT_MOVE_D,
	ACT_MOVE_L,
	ACT_MOVE_R,
	ACT_PUSH_U,
	ACT_PUSH_D,
	ACT_PUSH_L,
	ACT_PUSH_R,
	ACT_ROTATE_0,
	ACT_ROTATE_90,
	ACT_ROTATE_180,
	ACT_ROTATE_270,
	ACT_WAIT_RECOG
} Action;

typedef enum
{
	SOKO_OK = 0,
	SOKO_NO_PATH,
	SOKO_INVALID_MAP,
	SOKO_OVERFLOW_ACTIONS,
	SOKO_OVERFLOW_OBJECTS,
	SOKO_OVERFLOW_QUEUE,
	SOKO_INTERNAL_ERROR
} SokoStatus;

typedef struct
{
	uint8_t active;
	Point pos;
} NumberTarget;

typedef struct
{
	Point pos;
	uint8_t active;
	uint8_t recognized;
	uint8_t class_id;
	int8_t target_number;
	int8_t target_index;
} BoxInfo;

typedef struct
{
	uint8_t box_index;
	uint8_t class_id;
	uint8_t confidence;
} VisionResult;

typedef struct
{
	uint8_t base[MAP_H][MAP_W];
	uint8_t generic_target_mask[MAP_H][MAP_W];
	int8_t number_target_map[MAP_H][MAP_W];

	Point player;

	BoxInfo boxes[MAX_BOXES];
	uint8_t box_count;

	NumberTarget targets[MAX_TARGETS];
	uint8_t target_count;

	Point bombs[MAX_BOMBS];
	uint8_t bomb_count;
} GameMap;

typedef struct
{
	Action data[MAX_ACTIONS];
	uint16_t count;
} ActionSeq;

typedef enum
{
	SEG_WALK = 0,
	SEG_PUSH,
	SEG_ROTATE,
	SEG_WAIT
} SegmentType;

typedef struct
{
	SegmentType type;
	Dir dir;
	uint16_t cells;
	uint16_t angle_deg;
	uint16_t time_ms;
} MotionSegment;

typedef struct
{
	MotionSegment data[MAX_SEGMENTS];
	uint16_t count;
} MotionPlan;

typedef struct
{
	float walk_speed;
	float push_speed;
	float rotate_speed_deg_s;
	float yaw_target_deg;
	uint16_t recog_wait_ms;
} MotionParam;

typedef struct
{
	SokoStatus status;
	uint16_t steps;
	uint16_t pushes;
	int16_t chosen_box;
	int16_t chosen_target;
	int16_t chosen_bomb;
	Point end_player;
	ActionSeq seq;
} SolveResult;

typedef struct
{
	SokoStatus status;
	Point end_player;
	uint16_t steps;
	uint16_t pushes;
	uint16_t actions;
} SokoReplayResult;

#endif
