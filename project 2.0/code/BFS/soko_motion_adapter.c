/*
 * 文件名称: soko_motion_adapter.c
 * 创建时间: 2026-03-18
 * 创作者: 王宇铖
 * 说明: 推箱子动作压缩与执行层适配实现。
 */

#include "soko_motion_adapter.h"

#include "soko_utils.h"

void soko_motion_plan_reset(MotionPlan *plan)
{
	if(0 == plan)
	{
		return;
	}

	plan->count = 0;
}

SokoStatus soko_motion_plan_push(MotionPlan *plan, MotionSegment segment)
{
	if(0 == plan)
	{
		return SOKO_INVALID_MAP;
	}

	if(plan->count >= MAX_SEGMENTS)
	{
		return SOKO_OVERFLOW_ACTIONS;
	}

	plan->data[plan->count++] = segment;
	return SOKO_OK;
}

void soko_motion_param_set_default(MotionParam *param)
{
	if(0 == param)
	{
		return;
	}

	param->walk_speed = 20.0f;
	param->push_speed = 12.0f;
	param->rotate_speed_deg_s = 90.0f;
	param->yaw_target_deg = 0.0f;
	param->recog_wait_ms = 300;
}

static uint16_t soko_motion_action_angle(Action action)
{
	switch(action)
	{
		case ACT_ROTATE_0: return 0;
		case ACT_ROTATE_90: return 90;
		case ACT_ROTATE_180: return 180;
		case ACT_ROTATE_270: return 270;
		default: return 0;
	}
}

SokoStatus soko_motion_compress_actions(const ActionSeq *seq,
										MotionPlan *out_plan,
										const MotionParam *param)
{
	uint16_t i;
	MotionParam local_param;

	if((0 == seq) || (0 == out_plan))
	{
		return SOKO_INVALID_MAP;
	}

	if(0 == param)
	{
		soko_motion_param_set_default(&local_param);
		param = &local_param;
	}

	soko_motion_plan_reset(out_plan);

	for(i = 0; i < seq->count; i++)
	{
		Action action = seq->data[i];
		MotionSegment segment;
		SokoStatus status;

		if((out_plan->count > 0) && (soko_action_is_move(action) || soko_action_is_push(action)))
		{
			Dir dir;
			MotionSegment *tail = &out_plan->data[out_plan->count - 1];

			status = soko_action_to_dir(action, &dir);
			if(SOKO_OK != status)
			{
				return status;
			}

			if(((soko_action_is_move(action) && (SEG_WALK == tail->type)) ||
				(soko_action_is_push(action) && (SEG_PUSH == tail->type))) &&
			   (tail->dir == dir))
			{
				tail->cells++;
				continue;
			}
		}

		segment.type = SEG_WAIT;
		segment.dir = DIR_UP;
		segment.cells = 0;
		segment.angle_deg = 0;
		segment.time_ms = 0;

		if(soko_action_is_move(action) || soko_action_is_push(action))
		{
			Dir dir;

			status = soko_action_to_dir(action, &dir);
			if(SOKO_OK != status)
			{
				return status;
			}

			segment.type = soko_action_is_push(action) ? SEG_PUSH : SEG_WALK;
			segment.dir = dir;
			segment.cells = 1;
		}
		else if(soko_action_is_rotate(action))
		{
			segment.type = SEG_ROTATE;
			segment.angle_deg = soko_motion_action_angle(action);
		}
		else if(ACT_WAIT_RECOG == action)
		{
			segment.type = SEG_WAIT;
			segment.time_ms = param->recog_wait_ms;
		}
		else
		{
			return SOKO_INVALID_MAP;
		}

		status = soko_motion_plan_push(out_plan, segment);
		if(SOKO_OK != status)
		{
			return status;
		}
	}

	return SOKO_OK;
}

SokoStatus soko_motion_segment_to_velocity(const MotionSegment *segment,
										   const MotionParam *param,
										   float *vx,
										   float *vy)
{
	float speed;

	if((0 == segment) || (0 == param) || (0 == vx) || (0 == vy))
	{
		return SOKO_INVALID_MAP;
	}

	*vx = 0.0f;
	*vy = 0.0f;

	if((SEG_ROTATE == segment->type) || (SEG_WAIT == segment->type))
	{
		return SOKO_OK;
	}

	speed = (SEG_PUSH == segment->type) ? param->push_speed : param->walk_speed;

	switch(segment->dir)
	{
		case DIR_UP:
			*vx = 0.0f;
			*vy = speed;
			break;

		case DIR_DOWN:
			*vx = 0.0f;
			*vy = -speed;
			break;

		case DIR_LEFT:
			*vx = -speed;
			*vy = 0.0f;
			break;

		case DIR_RIGHT:
			*vx = speed;
			*vy = 0.0f;
			break;

		default:
			return SOKO_INVALID_MAP;
	}

	return SOKO_OK;
}
