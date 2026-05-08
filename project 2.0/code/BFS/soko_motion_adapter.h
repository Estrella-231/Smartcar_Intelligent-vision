/*
 * 文件名称: soko_motion_adapter.h
 * 创建时间: 2026-03-18
 * 创作者: 王宇铖
 * 说明: 推箱子动作压缩与执行层适配接口。
 */

#ifndef _SOKO_MOTION_ADAPTER_H
#define _SOKO_MOTION_ADAPTER_H

#include "soko_types.h"

void soko_motion_plan_reset(MotionPlan *plan);
SokoStatus soko_motion_plan_push(MotionPlan *plan, MotionSegment segment);

void soko_motion_param_set_default(MotionParam *param);

SokoStatus soko_motion_compress_actions(const ActionSeq *seq,
										MotionPlan *out_plan,
										const MotionParam *param);

SokoStatus soko_motion_segment_to_velocity(const MotionSegment *segment,
										   const MotionParam *param,
										   float *vx,
										   float *vy);

#endif
