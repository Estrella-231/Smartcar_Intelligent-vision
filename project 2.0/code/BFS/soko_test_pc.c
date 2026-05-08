/*
 * 文件名称: soko_test_pc.c
 * 创建时间: 2026-03-18
 * 创作者: 王宇铖
 * 说明: 推箱子 BFS PC 端基础测试入口。
 */

#include <stdio.h>
#include <string.h>

#include "soko_deadlock.h"
#include "soko_map.h"
#include "soko_motion_adapter.h"
#include "soko_push_bfs.h"
#include "soko_replay.h"
#include "soko_stage1.h"
#include "soko_stage2.h"
#include "soko_stage3.h"
#include "soko_utils.h"
#include "soko_walk_bfs.h"

static const char g_walk_map[MAP_H][MAP_W + 1] =
{
	"################",
	"#@-------------#",
	"#------#-------#",
	"#------#-------#",
	"#--------------#",
	"#--------------#",
	"#--------------#",
	"#--------------#",
	"#--------------#",
	"#--------------#",
	"#--------------#",
	"################"
};

static const char g_push_map[MAP_H][MAP_W + 1] =
{
	"################",
	"#--------------#",
	"#@$.-----------#",
	"#--------------#",
	"#--------------#",
	"#--------------#",
	"#--------------#",
	"#--------------#",
	"#--------------#",
	"#--------------#",
	"#--------------#",
	"################"
};

static const char g_stage1_map[MAP_H][MAP_W + 1] =
{
	"################",
	"#@-------------#",
	"#-$.-----------#",
	"#-$.-----------#",
	"#--------------#",
	"#--------------#",
	"#--------------#",
	"#--------------#",
	"#--------------#",
	"#--------------#",
	"#--------------#",
	"################"
};

static const char g_deadlock_map[MAP_H][MAP_W + 1] =
{
	"################",
	"#$-------------#",
	"#@-------------#",
	"#------------.-#",
	"#--------------#",
	"#--------------#",
	"#--------------#",
	"#--------------#",
	"#--------------#",
	"#--------------#",
	"#--------------#",
	"################"
};

static const char g_stage2_map[MAP_H][MAP_W + 1] =
{
	"################",
	"#@-------------#",
	"#-$.-----------#",
	"#-$.-----------#",
	"#--------------#",
	"#--------------#",
	"#--------------#",
	"#--------------#",
	"#--------------#",
	"#--------------#",
	"#--------------#",
	"################"
};

static const char g_stage3_map[MAP_H][MAP_W + 1] =
{
	"################",
	"################",
	"#@X#$.##########",
	"################",
	"#--------------#",
	"#--------------#",
	"#--------------#",
	"#--------------#",
	"#--------------#",
	"#--------------#",
	"#--------------#",
	"################"
};

static const char g_stage1_complex_map[MAP_H][MAP_W + 1] =
{
	"################",
	"#-#------------#",
	"#-.------#####-#",
	"##$###---#---#-#",
	"#----#---#.#-#-#",
	"#@---#####.#-#-#",
	"#-------$--$-#-#",
	"#-----------##-#",
	"#--------------#",
	"#-----####-----#",
	"#--------------#",
	"################"
};

static void soko_test_print_actions(const ActionSeq *seq)
{
	uint16_t i;

	for(i = 0; i < seq->count; i++)
	{
		switch(seq->data[i])
		{
			case ACT_MOVE_U: printf("MU"); break;
			case ACT_MOVE_D: printf("MD"); break;
			case ACT_MOVE_L: printf("ML"); break;
			case ACT_MOVE_R: printf("MR"); break;
			case ACT_PUSH_U: printf("PU"); break;
			case ACT_PUSH_D: printf("PD"); break;
			case ACT_PUSH_L: printf("PL"); break;
			case ACT_PUSH_R: printf("PR"); break;
			case ACT_ROTATE_0: printf("R0"); break;
			case ACT_ROTATE_90: printf("R90"); break;
			case ACT_ROTATE_180: printf("R180"); break;
			case ACT_ROTATE_270: printf("R270"); break;
			case ACT_WAIT_RECOG: printf("WAIT"); break;
			default: printf("??"); break;
		}

		if(i + 1 < seq->count)
		{
			printf(",");
		}
	}

	printf("\n");
}

static int soko_test_walk_case(void)
{
	GameMap map;
	ActionSeq seq;
	Point goals[1];
	Point end_point;
	int16_t goal_index;
	SokoStatus status;

	status = soko_map_parse_ascii(g_walk_map, &map);
	if(SOKO_OK != status)
	{
		printf("[FAIL] walk parse status=%d\n", status);
		return 1;
	}

	goals[0] = soko_point_make(14, 10);
	status = soko_bfs_walk_to_any(&map, map.player, goals, 1, &seq, &end_point, &goal_index);
	if(SOKO_OK != status)
	{
		printf("[FAIL] walk bfs status=%d\n", status);
		return 1;
	}

	if(!soko_point_equal(end_point, goals[0]) || (0 != goal_index))
	{
		printf("[FAIL] walk end mismatch\n");
		return 1;
	}

	printf("[PASS] walk bfs steps=%u actions=", seq.count);
	soko_test_print_actions(&seq);
	return 0;
}

static int soko_test_push_case(void)
{
	GameMap map;
	GameMap replay_map;
	SolveResult result;
	SokoReplayResult replay_result;
	SokoStatus status;

	status = soko_map_parse_ascii(g_push_map, &map);
	if(SOKO_OK != status)
	{
		printf("[FAIL] push parse status=%d\n", status);
		return 1;
	}

	soko_map_copy(&replay_map, &map);

	status = soko_bfs_push_box_to_any_target(&map,
											 map.player,
											 soko_point_make(2, 2),
											 map.generic_target_mask,
											 &result);
	if(SOKO_OK != status)
	{
		printf("[FAIL] push bfs status=%d\n", status);
		return 1;
	}

	status = soko_replay_apply_sequence(&replay_map, &result.seq, &replay_result);
	if(SOKO_OK != status)
	{
		printf("[FAIL] push replay status=%d\n", status);
		return 1;
	}

	if((0 != replay_map.box_count) || (0 != replay_map.target_count))
	{
		printf("[FAIL] push final map mismatch\n");
		return 1;
	}

	printf("[PASS] push bfs steps=%u pushes=%u actions=", result.steps, result.pushes);
	soko_test_print_actions(&result.seq);
	return 0;
}

static int soko_test_deadlock_case(void)
{
	GameMap map;
	SolveResult result;
	SokoStatus status;
	bool deadlock_flag;

	status = soko_map_parse_ascii(g_deadlock_map, &map);
	if(SOKO_OK != status)
	{
		printf("[FAIL] deadlock parse status=%d\n", status);
		return 1;
	}

	deadlock_flag = soko_deadlock_is_corner(&map,
											 soko_point_make(1, 1),
											 soko_point_make(1, 1),
											 map.generic_target_mask);
	if(!deadlock_flag)
	{
		printf("[FAIL] deadlock corner not detected\n");
		return 1;
	}

	status = soko_bfs_push_box_to_any_target(&map,
											 map.player,
											 soko_point_make(1, 1),
											 map.generic_target_mask,
											 &result);
	if(SOKO_NO_PATH != status)
	{
		printf("[FAIL] deadlock bfs expected no path status=%d\n", status);
		return 1;
	}

	printf("[PASS] deadlock prune baseline confirmed\n");
	return 0;
}

static int soko_test_stage1_case(void)
{
	GameMap map;
	GameMap replay_map;
	ActionSeq seq;
	SokoReplayResult replay_result;
	SokoStatus status;

	status = soko_map_parse_ascii(g_stage1_map, &map);
	if(SOKO_OK != status)
	{
		printf("[FAIL] stage1 parse status=%d\n", status);
		return 1;
	}

	soko_map_copy(&replay_map, &map);
	status = soko_plan_stage1_greedy(&map, &seq);
	if(SOKO_OK != status)
	{
		printf("[FAIL] stage1 plan status=%d\n", status);
		return 1;
	}

	status = soko_replay_apply_sequence(&replay_map, &seq, &replay_result);
	if(SOKO_OK != status)
	{
		printf("[FAIL] stage1 replay status=%d\n", status);
		return 1;
	}

	if((0 != replay_map.box_count) || (0 != replay_map.target_count))
	{
		printf("[FAIL] stage1 final map mismatch\n");
		return 1;
	}

	printf("[PASS] stage1 greedy actions=%u replay_steps=%u\n", seq.count, replay_result.steps);
	printf("       sequence=");
	soko_test_print_actions(&seq);
	return 0;
}

static int soko_test_stage1_complex_case(void)
{
	GameMap map;
	GameMap replay_map;
	ActionSeq seq;
	SokoReplayResult replay_result;
	SokoStatus status;

	status = soko_map_parse_ascii(g_stage1_complex_map, &map);
	if(SOKO_OK != status)
	{
		printf("[FAIL] stage1 complex parse status=%d\n", status);
		return 1;
	}

	soko_map_copy(&replay_map, &map);
	status = soko_plan_stage1_greedy(&map, &seq);
	if(SOKO_OK != status)
	{
		printf("[FAIL] stage1 complex plan status=%d\n", status);
		return 1;
	}

	status = soko_replay_apply_sequence(&replay_map, &seq, &replay_result);
	if(SOKO_OK != status)
	{
		printf("[FAIL] stage1 complex replay status=%d\n", status);
		return 1;
	}

	if((0 != replay_map.box_count) || (0 != replay_map.target_count))
	{
		printf("[FAIL] stage1 complex final map mismatch\n");
		return 1;
	}

	printf("[PASS] stage1 complex greedy actions=%u replay_steps=%u\n", seq.count, replay_result.steps);
	printf("       sequence=");
	soko_test_print_actions(&seq);
	return 0;
}

static int soko_test_stage2_case(void)
{
	GameMap map;
	GameMap replay_map;
	ActionSeq observe_seq;
	ActionSeq push_seq;
	Point observe_end;
	SokoReplayResult replay_result;
	SokoStatus status;

	status = soko_map_parse_ascii(g_stage2_map, &map);
	if(SOKO_OK != status)
	{
		printf("[FAIL] stage2 parse status=%d\n", status);
		return 1;
	}

	soko_number_targets_clear();
	status = soko_map_bind_number_target(&map, 1, soko_point_make(3, 2));
	if(SOKO_OK != status)
	{
		printf("[FAIL] stage2 bind target1 status=%d\n", status);
		return 1;
	}

	status = soko_map_bind_number_target(&map, 2, soko_point_make(3, 3));
	if(SOKO_OK != status)
	{
		printf("[FAIL] stage2 bind target2 status=%d\n", status);
		return 1;
	}

	status = soko_stage2_plan_observe_box(&map, 0, &observe_seq, &observe_end);
	if(SOKO_OK != status)
	{
		printf("[FAIL] stage2 observe status=%d\n", status);
		return 1;
	}

	if((observe_seq.count < 2) || (ACT_WAIT_RECOG != observe_seq.data[observe_seq.count - 1]))
	{
		printf("[FAIL] stage2 observe sequence invalid\n");
		return 1;
	}

	status = soko_stage2_bind_box_target(&map, 0, 1);
	if(SOKO_OK != status)
	{
		printf("[FAIL] stage2 bind box0 status=%d\n", status);
		return 1;
	}

	status = soko_stage2_bind_box_target(&map, 1, 2);
	if(SOKO_OK != status)
	{
		printf("[FAIL] stage2 bind box1 status=%d\n", status);
		return 1;
	}

	soko_map_copy(&replay_map, &map);
	status = soko_stage2_plan_bound_boxes(&map, &push_seq);
	if(SOKO_OK != status)
	{
		printf("[FAIL] stage2 bound plan status=%d\n", status);
		return 1;
	}

	status = soko_replay_apply_sequence(&replay_map, &push_seq, &replay_result);
	if(SOKO_OK != status)
	{
		printf("[FAIL] stage2 replay status=%d\n", status);
		return 1;
	}

	if((0 != replay_map.box_count) || (0 != replay_map.target_count))
	{
		printf("[FAIL] stage2 final map mismatch\n");
		return 1;
	}

	printf("[PASS] stage2 observe_actions=%u push_actions=%u\n", observe_seq.count, push_seq.count);
	return 0;
}

static int soko_test_stage3_case(void)
{
	GameMap map;
	GameMap replay_map;
	ActionSeq seq;
	SokoReplayResult replay_result;
	SokoStatus status;

	status = soko_map_parse_ascii(g_stage3_map, &map);
	if(SOKO_OK != status)
	{
		printf("[FAIL] stage3 parse status=%d\n", status);
		return 1;
	}

	soko_map_copy(&replay_map, &map);
	status = soko_plan_stage3_with_bomb(&map, soko_point_make(4, 2), map.generic_target_mask, &seq);
	if(SOKO_OK != status)
	{
		printf("[FAIL] stage3 plan status=%d\n", status);
		return 1;
	}

	if(seq.count < 2)
	{
		printf("[FAIL] stage3 sequence too short\n");
		return 1;
	}

	if((0 != map.box_count) || (0 != map.target_count))
	{
		printf("[FAIL] stage3 final map mismatch after planner apply\n");
		return 1;
	}

	status = soko_replay_apply_sequence(&replay_map, &seq, &replay_result);
	if(SOKO_OK == status)
	{
		printf("[FAIL] stage3 generic replay should not validate bomb sequence directly\n");
		return 1;
	}

	printf("[PASS] stage3 bomb-assisted plan actions=%u\n", seq.count);
	return 0;
}

static int soko_test_motion_adapter_case(void)
{
	ActionSeq seq;
	MotionPlan plan;
	MotionParam param;
	float vx;
	float vy;
	SokoStatus status;

	soko_action_seq_reset(&seq);
	soko_motion_param_set_default(&param);

	if(SOKO_OK != soko_action_seq_push(&seq, ACT_MOVE_D) ||
	   SOKO_OK != soko_action_seq_push(&seq, ACT_MOVE_D) ||
	   SOKO_OK != soko_action_seq_push(&seq, ACT_PUSH_R) ||
	   SOKO_OK != soko_action_seq_push(&seq, ACT_PUSH_R) ||
	   SOKO_OK != soko_action_seq_push(&seq, ACT_ROTATE_90) ||
	   SOKO_OK != soko_action_seq_push(&seq, ACT_WAIT_RECOG))
	{
		printf("[FAIL] motion setup action push failed\n");
		return 1;
	}

	status = soko_motion_compress_actions(&seq, &plan, &param);
	if(SOKO_OK != status)
	{
		printf("[FAIL] motion compress status=%d\n", status);
		return 1;
	}

	if(4 != plan.count)
	{
		printf("[FAIL] motion segment count mismatch count=%u\n", plan.count);
		return 1;
	}

	if((SEG_WALK != plan.data[0].type) || (2 != plan.data[0].cells) || (DIR_DOWN != plan.data[0].dir))
	{
		printf("[FAIL] motion walk segment mismatch\n");
		return 1;
	}

	if((SEG_PUSH != plan.data[1].type) || (2 != plan.data[1].cells) || (DIR_RIGHT != plan.data[1].dir))
	{
		printf("[FAIL] motion push segment mismatch\n");
		return 1;
	}

	if((SEG_ROTATE != plan.data[2].type) || (90 != plan.data[2].angle_deg))
	{
		printf("[FAIL] motion rotate segment mismatch\n");
		return 1;
	}

	if((SEG_WAIT != plan.data[3].type) || (param.recog_wait_ms != plan.data[3].time_ms))
	{
		printf("[FAIL] motion wait segment mismatch\n");
		return 1;
	}

	status = soko_motion_segment_to_velocity(&plan.data[1], &param, &vx, &vy);
	if(SOKO_OK != status)
	{
		printf("[FAIL] motion velocity status=%d\n", status);
		return 1;
	}

	if((vx <= 0.0f) || (0.0f != vy))
	{
		printf("[FAIL] motion velocity mismatch vx=%.2f vy=%.2f\n", vx, vy);
		return 1;
	}

	printf("[PASS] motion adapter segments=%u push_vx=%.2f push_vy=%.2f\n", plan.count, vx, vy);
	return 0;
}

int main(void)
{
	int fail_count = 0;

	fail_count += soko_test_walk_case();
	fail_count += soko_test_push_case();
	fail_count += soko_test_deadlock_case();
	fail_count += soko_test_stage1_case();
	fail_count += soko_test_stage1_complex_case();
	fail_count += soko_test_stage2_case();
	fail_count += soko_test_stage3_case();
	fail_count += soko_test_motion_adapter_case();

	if(0 == fail_count)
	{
		printf("[PASS] all pc tests passed\n");
		return 0;
	}

	printf("[FAIL] pc tests failed count=%d\n", fail_count);
	return 1;
}
