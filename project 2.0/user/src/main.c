#include "zf_common_headfile.h"

#include <stdarg.h>
#include <string.h>

#include "angle_pid.h"
#include "bluetooth.h"
#include "encoder.h"
#include "gyroscope.h"
#include "motor_driver.h"
#include "odometry.h"
#include "robot_control.h"
#include "speed_pid.h"
#include "state_machine.h"
#include "debug_runtime.h"

#include "soko_map.h"
#include "soko_motion_adapter.h"
#include "soko_replay.h"
#include "soko_stage1.h"
#include "user_monitor/monitor.h"

/*
 * Runtime entry policy:
 * - keep the BFS/OpenART bring-up path available for later integration
 * - default to a manual rectangle-lap mode so chassis + odometry can be tuned
 *   without map parsing or planner noise
 * - keep a dedicated wheel-sign check mode for one-wheel-at-a-time harness
 *   validation
 */
#define BFS_STEP2_STATUS_WAIT_FRAME         (100)
#define BFS_STEP2_STATUS_BRIDGE_OK          (110)
#define BFS_STEP3_STATUS_PLAN_OK            (120)
#define BFS_STEP2_ERROR_BRIDGE_BASE         (-600)
#define BFS_STEP3_ERROR_PLAN_BASE           (-700)
#define BFS_STEP3_ERROR_REPLAY_BASE         (-800)
#define BFS_STEP3_ERROR_MOTION_BASE         (-900)
#define BFS_STEP3_ERROR_FINAL_MAP           (-1000)

#define BFS_STEP2_FILTER_DEPTH              (3)
#define BFS_STEP3_TRIGGER_STABLE_COUNT      (3)
#define BFS_USE_FIXED_STAGE1_MAP_TEST       ((SMARTCAR_RUNTIME_MODE) == SMARTCAR_MODE_BFS_FIXED_MAP)
#define BFS_FIXED_PLAYER_X                  (1)
#define BFS_FIXED_PLAYER_Y                  (6)

#define WHEEL_SIGN_CHECK_STOP_MS            (1000U)
#define WHEEL_SIGN_CHECK_DRIVE_MS           (1500U)
#define WHEEL_SIGN_CHECK_PWM_VALUE          (300)

static volatile int32_t g_debug_a = BFS_STEP2_STATUS_WAIT_FRAME;
static volatile int32_t g_debug_b = 0;
static volatile int32_t g_debug_c = 0;
static volatile int32_t g_debug_d = 0;

static uint32_t g_last_processed_valid_count = 0;
static uint32_t g_last_map_signature = 0;
static uint32_t g_stable_frame_count = 0;
static uint32_t g_last_planned_signature = 0;
static uint8_t g_has_planned_signature = 0;
static uint8_t g_history_count = 0;
static uint8_t g_history_write_index = 0;
static char g_raw_map_history[BFS_STEP2_FILTER_DEPTH][MAP_H][MAP_W + 1];
static uint32_t g_fixed_frame_counter = 0;
static motion_debug_page_t g_motion_debug_page = MOTION_DEBUG_PAGE_POSE;
static uint32_t g_motion_debug_page_elapsed_ms = 0U;
static MotionPlan g_manual_rect_lap_plan = {0};

typedef struct
{
    MotorID motor_id;
    int32_t pwm;
    uint32_t duration_ms;
    const char *label;
} wheel_sign_check_step_t;

static const wheel_sign_check_step_t g_wheel_sign_check_steps[] =
{
    {MOTOR_MAX, 0, WHEEL_SIGN_CHECK_STOP_MS, "STOP"},
    {MOTOR_LF,  WHEEL_SIGN_CHECK_PWM_VALUE, WHEEL_SIGN_CHECK_DRIVE_MS, "LF+"},
    {MOTOR_MAX, 0, WHEEL_SIGN_CHECK_STOP_MS, "STOP"},
    {MOTOR_RF,  WHEEL_SIGN_CHECK_PWM_VALUE, WHEEL_SIGN_CHECK_DRIVE_MS, "RF+"},
    {MOTOR_MAX, 0, WHEEL_SIGN_CHECK_STOP_MS, "STOP"},
    {MOTOR_LB,  WHEEL_SIGN_CHECK_PWM_VALUE, WHEEL_SIGN_CHECK_DRIVE_MS, "LB+"},
    {MOTOR_MAX, 0, WHEEL_SIGN_CHECK_STOP_MS, "STOP"},
    {MOTOR_RB,  WHEEL_SIGN_CHECK_PWM_VALUE, WHEEL_SIGN_CHECK_DRIVE_MS, "RB+"},
    {MOTOR_MAX, 0, WHEEL_SIGN_CHECK_STOP_MS, "STOP"},
    {MOTOR_LF, -WHEEL_SIGN_CHECK_PWM_VALUE, WHEEL_SIGN_CHECK_DRIVE_MS, "LF-"},
    {MOTOR_MAX, 0, WHEEL_SIGN_CHECK_STOP_MS, "STOP"},
    {MOTOR_RF, -WHEEL_SIGN_CHECK_PWM_VALUE, WHEEL_SIGN_CHECK_DRIVE_MS, "RF-"},
    {MOTOR_MAX, 0, WHEEL_SIGN_CHECK_STOP_MS, "STOP"},
    {MOTOR_LB, -WHEEL_SIGN_CHECK_PWM_VALUE, WHEEL_SIGN_CHECK_DRIVE_MS, "LB-"},
    {MOTOR_MAX, 0, WHEEL_SIGN_CHECK_STOP_MS, "STOP"},
    {MOTOR_RB, -WHEEL_SIGN_CHECK_PWM_VALUE, WHEEL_SIGN_CHECK_DRIVE_MS, "RB-"}
};

static uint32_t g_wheel_sign_step_index = 0U;
static uint32_t g_wheel_sign_step_elapsed_ms = 0U;
static uint32_t g_wheel_sign_last_reported_index = 0xFFFFFFFFUL;

#if BFS_USE_FIXED_STAGE1_MAP_TEST
/*
 * Fixed first-level map used for real-ground motion bring-up before live
 * OpenART mounting is ready.
 *
 * This map intentionally contains no '@'. The player position is injected from
 * BFS_FIXED_PLAYER_X / BFS_FIXED_PLAYER_Y so the same downstream bridge path is
 * used as the real OpenART case where the car always starts from a known cell.
 */
static const char g_fixed_stage1_map[MAP_H][MAP_W + 1] =
{
    "################",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#------$-..----#",
    "#--------------#",
    "#------$-------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "#--------------#",
    "################"
};
#endif

/*
 * Keep formatted BLE logging local to this entry file.
 *
 * The helper is reused by both the bridge summary and the raw map dump, while
 * keeping the current test mode independent from other business modules.
 */
static void bfs_send_text_line(const char *format, ...)
{
    char line_buffer[128];
    va_list args;

    memset(line_buffer, 0, sizeof(line_buffer));
    va_start(args, format);
    vsnprintf(line_buffer, sizeof(line_buffer), format, args);
    va_end(args);
    ble6a20_send_string(line_buffer);
}

static const char *bfs_action_name(Action action)
{
    switch(action)
    {
        case ACT_MOVE_U: return "MU";
        case ACT_MOVE_D: return "MD";
        case ACT_MOVE_L: return "ML";
        case ACT_MOVE_R: return "MR";
        case ACT_PUSH_U: return "PU";
        case ACT_PUSH_D: return "PD";
        case ACT_PUSH_L: return "PL";
        case ACT_PUSH_R: return "PR";
        case ACT_ROTATE_0: return "R0";
        case ACT_ROTATE_90: return "R90";
        case ACT_ROTATE_180: return "R180";
        case ACT_ROTATE_270: return "R270";
        case ACT_WAIT_RECOG: return "WAIT";
        default: return "UNK";
    }
}

static const char *bfs_segment_type_name(SegmentType type)
{
    switch(type)
    {
        case SEG_WALK: return "WALK";
        case SEG_PUSH: return "PUSH";
        case SEG_ROTATE: return "ROT";
        case SEG_WAIT: return "WAIT";
        default: return "UNK";
    }
}

static const char *bfs_dir_name(Dir dir)
{
    switch(dir)
    {
        case DIR_UP: return "U";
        case DIR_DOWN: return "D";
        case DIR_LEFT: return "L";
        case DIR_RIGHT: return "R";
        default: return "?";
    }
}

/*
 * Build a lightweight map signature for stability tracking.
 *
 * The next step will trigger planning only after several identical valid maps.
 * Step 2 does not plan yet, but it already computes the signature and stable
 * count so we can verify the bridge behaves predictably on real sensor input.
 */
static uint32_t bfs_calc_raw_map_signature(const char raw_map[MAP_H][MAP_W + 1])
{
    uint32_t hash = 2166136261UL;

    for(uint32_t row = 0; row < OPENART_MAP_HEIGHT; row++)
    {
        for(uint32_t col = 0; col < OPENART_MAP_WIDTH; col++)
        {
            hash ^= (uint8_t)raw_map[row][col];
            hash *= 16777619UL;
        }
    }

    return hash;
}

/*
 * Convert the OpenART frame into the exact ASCII shape expected by
 * soko_map_parse_ascii():
 * - 12 rows
 * - 16 visible cells per row
 * - one trailing '\0' per row
 *
 * The player coordinate pair from OpenART is treated as the source of truth.
 * The bridge therefore clears any old '@' markers first and then writes the
 * player marker back to the specified coordinate.
 */
static void bfs_openart_frame_to_raw_map(const openart_frame_t *frame,
                                         char raw_map[MAP_H][MAP_W + 1])
{
    uint32_t row;
    uint32_t col;

    for(row = 0; row < MAP_H; row++)
    {
        for(col = 0; col < MAP_W; col++)
        {
            char cell = frame->map[row][col];

            if(SOKO_CHAR_PLAYER == cell)
            {
                cell = SOKO_CHAR_EMPTY;
            }

            raw_map[row][col] = cell;
        }

        raw_map[row][MAP_W] = '\0';
    }

    if((frame->player_x < MAP_W) && (frame->player_y < MAP_H))
    {
        raw_map[frame->player_y][frame->player_x] = SOKO_CHAR_PLAYER;
    }
}

/*
 * Provide one bridge input frame for the current control cycle.
 *
 * In fixed-map test mode, a synthetic valid frame is generated every cycle so
 * the stability filter can still reach the "stable for 3 frames" threshold and
 * trigger the normal BFS planning path.
 */
static uint8_t bfs_get_next_bridge_frame(openart_frame_t *out_frame,
                                         uint32_t *out_valid_count)
{
    if((0 == out_frame) || (0 == out_valid_count))
    {
        return 0U;
    }

#if BFS_USE_FIXED_STAGE1_MAP_TEST
    memset(out_frame, 0, sizeof(*out_frame));
    out_frame->status = OPENART_STATUS_VALID;
    out_frame->player_x = BFS_FIXED_PLAYER_X;
    out_frame->player_y = BFS_FIXED_PLAYER_Y;

    for(uint32_t row = 0; row < MAP_H; row++)
    {
        memcpy(out_frame->map[row], g_fixed_stage1_map[row], MAP_W);
    }

    g_fixed_frame_counter++;
    *out_valid_count = g_fixed_frame_counter;
    return 1U;
#else
    *out_valid_count = openart_get_valid_frame_count();
    if(*out_valid_count == g_last_processed_valid_count)
    {
        return 0U;
    }

    if(!openart_copy_latest_frame(out_frame))
    {
        return 0U;
    }

    return 1U;
#endif
}

/*
 * Store the newest raw map into a small fixed-depth history buffer.
 *
 * Step 2 uses a 3-frame temporal window because the current visual map jitter
 * mostly appears as single-frame edge flicker. Keeping the last three maps is
 * enough to run a cheap majority filter without adding large memory cost.
 */
static void bfs_push_raw_map_history(const char raw_map[MAP_H][MAP_W + 1])
{
    memcpy(g_raw_map_history[g_history_write_index], raw_map, sizeof(g_raw_map_history[0]));

    g_history_write_index++;
    if(g_history_write_index >= BFS_STEP2_FILTER_DEPTH)
    {
        g_history_write_index = 0;
    }

    if(g_history_count < BFS_STEP2_FILTER_DEPTH)
    {
        g_history_count++;
    }
}

/*
 * Return the most common cell among the last three raw maps.
 *
 * Tie handling:
 * - if two or three frames agree, keep the agreed cell
 * - if all cells differ, fall back to the newest sample
 *
 * The fallback keeps the bridge responsive when a real map change happens.
 */
static char bfs_majority_cell(char sample0, char sample1, char sample2, char newest)
{
    if((sample0 == sample1) || (sample0 == sample2))
    {
        return sample0;
    }

    if(sample1 == sample2)
    {
        return sample1;
    }

    return newest;
}

/*
 * Build one temporally filtered map from the last three raw observations.
 *
 * Before the history buffer is full, the newest raw map is passed through
 * unchanged so the bridge can still be inspected immediately after boot.
 */
static void bfs_build_filtered_raw_map(char filtered_map[MAP_H][MAP_W + 1])
{
    uint8_t newest_index;

    newest_index = (uint8_t)((g_history_write_index + BFS_STEP2_FILTER_DEPTH - 1U) %
                             BFS_STEP2_FILTER_DEPTH);

    if(g_history_count < BFS_STEP2_FILTER_DEPTH)
    {
        memcpy(filtered_map, g_raw_map_history[newest_index], sizeof(g_raw_map_history[0]));
        return;
    }

    for(uint32_t row = 0; row < MAP_H; row++)
    {
        for(uint32_t col = 0; col < MAP_W; col++)
        {
            filtered_map[row][col] = bfs_majority_cell(
                g_raw_map_history[0][row][col],
                g_raw_map_history[1][row][col],
                g_raw_map_history[2][row][col],
                g_raw_map_history[newest_index][row][col]);
        }

        filtered_map[row][MAP_W] = '\0';
    }
}

/*
 * Print the current bridge input map once whenever the map signature changes.
 *
 * This makes it possible to compare:
 * - the OpenART terminal view
 * - the exact raw rows passed into soko_map_parse_ascii()
 *
 * without flooding BLE on every identical frame.
 */
static void bfs_print_bridge_raw_map(const char raw_map[MAP_H][MAP_W + 1])
{
    bfs_send_text_line("BFS bridge filtered map\r\n");

    for(uint32_t row = 0; row < MAP_H; row++)
    {
        bfs_send_text_line("%s\r\n", raw_map[row]);
    }
}

/*
 * Print the atomic planner result in readable short tokens.
 *
 * The compressed motion plan is the execution-facing view, but keeping the
 * original action list in the log makes it much easier to compare MCU results
 * with the already-verified PC-side BFS outputs.
 */
static void bfs_print_action_sequence(const ActionSeq *seq)
{
    char line_buffer[128];
    uint16_t index;
    size_t used;

    if(0 == seq)
    {
        return;
    }

    bfs_send_text_line("BFS actions count=%u\r\n", seq->count);

    memset(line_buffer, 0, sizeof(line_buffer));
    used = (size_t)snprintf(line_buffer, sizeof(line_buffer), "ACT ");

    for(index = 0; index < seq->count; index++)
    {
        const char *token = bfs_action_name(seq->data[index]);
        int written;

        written = snprintf(&line_buffer[used],
                           sizeof(line_buffer) - used,
                           "%s%s",
                           token,
                           (index + 1 < seq->count) ? "," : "");

        if((written < 0) || ((size_t)written >= (sizeof(line_buffer) - used)))
        {
            line_buffer[used] = '\0';
            ble6a20_send_string(line_buffer);
            ble6a20_send_string("\r\n");

            memset(line_buffer, 0, sizeof(line_buffer));
            used = (size_t)snprintf(line_buffer, sizeof(line_buffer), "ACT ");
            index--;
            continue;
        }

        used += (size_t)written;
    }

    ble6a20_send_string(line_buffer);
    ble6a20_send_string("\r\n");
}

static void bfs_print_motion_plan(const MotionPlan *plan)
{
    uint16_t index;

    if(0 == plan)
    {
        return;
    }

    bfs_send_text_line("BFS segments count=%u\r\n", plan->count);

    for(index = 0; index < plan->count; index++)
    {
        const MotionSegment *segment = &plan->data[index];

        bfs_send_text_line("SEG %u type=%s dir=%s cells=%u angle=%u wait=%u\r\n",
                           index,
                           bfs_segment_type_name(segment->type),
                           bfs_dir_name(segment->dir),
                           segment->cells,
                           segment->angle_deg,
                           segment->time_ms);
    }
}

/*
 * Execute stage1 once for one stable filtered map.
 *
 * The planner mutates its input GameMap, so this function keeps:
 * - one parsed source snapshot
 * - one working copy for stage1
 * - one replay copy for post-plan validation
 *
 * Compact numeric debug output after a successful plan:
 * - a = latest valid frame count
 * - b = stable frame count
 * - c = action count
 * - d = segment count
 */
static void bfs_plan_once_for_stable_map(const GameMap *source_map,
                                         uint32_t current_valid_count,
                                         uint32_t current_signature)
{
    GameMap working_map;
    GameMap replay_map;
    ActionSeq action_seq;
    MotionPlan motion_plan;
    MotionParam motion_param;
    SokoReplayResult replay_result;
    SokoStatus status;

    if(0 == source_map)
    {
        return;
    }

    memset(&working_map, 0, sizeof(working_map));
    memset(&replay_map, 0, sizeof(replay_map));
    memset(&action_seq, 0, sizeof(action_seq));
    memset(&motion_plan, 0, sizeof(motion_plan));
    memset(&motion_param, 0, sizeof(motion_param));
    memset(&replay_result, 0, sizeof(replay_result));

    soko_map_copy(&working_map, source_map);
    status = soko_plan_stage1_greedy(&working_map, &action_seq);
    if(SOKO_OK != status)
    {
        g_debug_a = BFS_STEP3_ERROR_PLAN_BASE - status;
        g_debug_b = (int32_t)g_stable_frame_count;
        g_debug_c = 0;
        g_debug_d = 0;
        g_last_planned_signature = current_signature;
        g_has_planned_signature = 1U;
        bfs_runtime_state_update_plan(current_signature, status, 0, 0);
        bfs_send_text_line("BFS stage1 failed cnt=%lu stable=%lu status=%d\r\n",
                           (unsigned long)current_valid_count,
                           (unsigned long)g_stable_frame_count,
                           status);
        return;
    }

    soko_map_copy(&replay_map, source_map);
    status = soko_replay_apply_sequence(&replay_map, &action_seq, &replay_result);
    if(SOKO_OK != status)
    {
        g_debug_a = BFS_STEP3_ERROR_REPLAY_BASE - status;
        g_debug_b = (int32_t)g_stable_frame_count;
        g_debug_c = 0;
        g_debug_d = 0;
        g_last_planned_signature = current_signature;
        g_has_planned_signature = 1U;
        bfs_runtime_state_update_plan(current_signature, status, 0, 0);
        bfs_send_text_line("BFS replay failed cnt=%lu stable=%lu status=%d\r\n",
                           (unsigned long)current_valid_count,
                           (unsigned long)g_stable_frame_count,
                           status);
        return;
    }

    if((0 != replay_map.box_count) || (0 != replay_map.target_count))
    {
        g_debug_a = BFS_STEP3_ERROR_FINAL_MAP;
        g_debug_b = (int32_t)g_stable_frame_count;
        g_debug_c = 0;
        g_debug_d = 0;
        g_last_planned_signature = current_signature;
        g_has_planned_signature = 1U;
        bfs_runtime_state_update_plan(current_signature, SOKO_INTERNAL_ERROR, 0, 0);
        bfs_send_text_line("BFS replay final map mismatch box=%u target=%u\r\n",
                           replay_map.box_count,
                           replay_map.target_count);
        return;
    }

    soko_motion_param_set_default(&motion_param);
    status = soko_motion_compress_actions(&action_seq, &motion_plan, &motion_param);
    if(SOKO_OK != status)
    {
        g_debug_a = BFS_STEP3_ERROR_MOTION_BASE - status;
        g_debug_b = (int32_t)g_stable_frame_count;
        g_debug_c = 0;
        g_debug_d = 0;
        g_last_planned_signature = current_signature;
        g_has_planned_signature = 1U;
        bfs_runtime_state_update_plan(current_signature, status, 0, 0);
        bfs_send_text_line("BFS motion compress failed cnt=%lu stable=%lu status=%d\r\n",
                           (unsigned long)current_valid_count,
                           (unsigned long)g_stable_frame_count,
                           status);
        return;
    }

    g_debug_a = (int32_t)current_valid_count;
    g_debug_b = (int32_t)g_stable_frame_count;
    g_debug_c = (int32_t)action_seq.count;
    g_debug_d = (int32_t)motion_plan.count;
    g_last_planned_signature = current_signature;
    g_has_planned_signature = 1U;
    bfs_runtime_state_update_plan(current_signature, SOKO_OK, &action_seq, &motion_plan);

    bfs_send_text_line("BFS stage1 ok cnt=%lu stable=%lu actions=%u segments=%u pushes=%u steps=%u\r\n",
                       (unsigned long)current_valid_count,
                       (unsigned long)g_stable_frame_count,
                       action_seq.count,
                       motion_plan.count,
                       replay_result.pushes,
                       replay_result.steps);
    bfs_print_action_sequence(&action_seq);
    bfs_print_motion_plan(&motion_plan);
}

/*
 * Handle one newly arrived valid OpenART frame.
 *
 * Compact numeric debug output:
 * - a = latest valid frame count, or negative bridge error
 * - b = stable identical-frame count
 * - c = parsed box count
 * - d = parsed target count
 *
 * The BLE text summary adds player and bomb counts because only four compact
 * debug slots are available in send_data().
 */
static void bfs_process_new_valid_frame(void)
{
    openart_frame_t latest_frame;
    char raw_map[MAP_H][MAP_W + 1];
    char filtered_raw_map[MAP_H][MAP_W + 1];
    GameMap parsed_map;
    SokoStatus status;
    uint32_t current_valid_count;
    uint32_t current_signature;
    uint8_t map_changed = 0;

    if(!bfs_get_next_bridge_frame(&latest_frame, &current_valid_count))
    {
        return;
    }

    bfs_openart_frame_to_raw_map(&latest_frame, raw_map);
    bfs_push_raw_map_history(raw_map);
    bfs_build_filtered_raw_map(filtered_raw_map);

    status = soko_map_parse_ascii(filtered_raw_map, &parsed_map);
    if(SOKO_OK != status)
    {
        g_debug_a = BFS_STEP2_ERROR_BRIDGE_BASE - status;
        g_debug_b = 0;
        g_debug_c = 0;
        g_debug_d = 0;
        g_last_processed_valid_count = current_valid_count;
        bfs_send_text_line("BFS bridge parse failed cnt=%lu status=%d p=(%u,%u)\r\n",
                           (unsigned long)current_valid_count,
                           status,
                           latest_frame.player_x,
                           latest_frame.player_y);
        return;
    }

    current_signature = bfs_calc_raw_map_signature(filtered_raw_map);
    if(current_signature == g_last_map_signature)
    {
        g_stable_frame_count++;
    }
    else
    {
        g_last_map_signature = current_signature;
        g_stable_frame_count = 1;
        map_changed = 1;
    }

    g_last_processed_valid_count = current_valid_count;
    g_debug_a = (int32_t)current_valid_count;
    g_debug_b = (int32_t)g_stable_frame_count;
    g_debug_c = (int32_t)parsed_map.box_count;
    g_debug_d = (int32_t)parsed_map.target_count;
    bfs_runtime_state_update_bridge(current_valid_count,
                                    g_stable_frame_count,
                                    current_signature,
                                    parsed_map.player.x,
                                    parsed_map.player.y,
                                    parsed_map.box_count,
                                    parsed_map.target_count,
                                    parsed_map.bomb_count,
                                    (g_stable_frame_count >= BFS_STEP3_TRIGGER_STABLE_COUNT));

    if(map_changed)
    {
        bfs_send_text_line("BFS bridge ok cnt=%lu stable=%lu p=(%u,%u) box=%u target=%u bomb=%u\r\n",
                           (unsigned long)current_valid_count,
                           (unsigned long)g_stable_frame_count,
                           parsed_map.player.x,
                           parsed_map.player.y,
                           parsed_map.box_count,
                           parsed_map.target_count,
                           parsed_map.bomb_count);
        bfs_print_bridge_raw_map(filtered_raw_map);
    }
    else if(BFS_STEP3_TRIGGER_STABLE_COUNT == g_stable_frame_count)
    {
        /*
         * Print one extra summary exactly at the first "stable enough" point.
         * This is also the trigger point for the current stage1 one-shot plan.
         */
        bfs_send_text_line("BFS bridge stable cnt=%lu stable=%lu p=(%u,%u) box=%u target=%u bomb=%u\r\n",
                           (unsigned long)current_valid_count,
                           (unsigned long)g_stable_frame_count,
                           parsed_map.player.x,
                           parsed_map.player.y,
                           parsed_map.box_count,
                           parsed_map.target_count,
                           parsed_map.bomb_count);
    }

    if((g_stable_frame_count >= BFS_STEP3_TRIGGER_STABLE_COUNT) &&
       ((0U == g_has_planned_signature) || (current_signature != g_last_planned_signature)))
    {
        bfs_plan_once_for_stable_map(&parsed_map, current_valid_count, current_signature);
    }
}

/*
 * Refresh the compact four-number telemetry from the execution scheduler.
 *
 * The debug page rotates automatically every second:
 * - POSE    : phase / segment / x / y
 * - TARGET  : target_x / target_y / dx / dy
 * - STATE   : vx / vy / yaw / yaw_err
 * - WHEEL_F : LF target/fb, RF target/fb
 * - WHEEL_R : LB target/fb, RB target/fb
 * - PWM     : LF / RF / LB / RB signed PWM output
 */
static void motion_exec_refresh_debug_output(uint32_t period_ms)
{
    motion_exec_runtime_state_t exec_state;
    motion_debug_snapshot_t snapshot;
    int32_t a = 0;
    int32_t b = 0;
    int32_t c = 0;
    int32_t d = 0;

    if(!motion_exec_runtime_state_copy(&exec_state))
    {
        return;
    }

    memset(&snapshot, 0, sizeof(snapshot));
    snapshot.phase = (int32_t)exec_state.phase;
    snapshot.segment_index = (int32_t)exec_state.current_segment_index;
    snapshot.x_mm = odometry_get_x_mm();
    snapshot.y_mm = odometry_get_y_mm();
    snapshot.target_x_mm = exec_state.segment_target_x_mm;
    snapshot.target_y_mm = exec_state.segment_target_y_mm;
    snapshot.dx_mm = odometry_get_target_dx_mm();
    snapshot.dy_mm = odometry_get_target_dy_mm();
    snapshot.vx_body_mmps = odometry_get_vx_body_mmps();
    snapshot.vy_body_mmps = odometry_get_vy_body_mmps();
    snapshot.yaw_cd = imu_get_yaw_cd();
    snapshot.yaw_error_cd = angle_pid_get_error();
    snapshot.lf_target = get_speed_target(MOTOR_LF);
    snapshot.lf_feedback = get_encoder_data(MOTOR_LF);
    snapshot.rf_target = get_speed_target(MOTOR_RF);
    snapshot.rf_feedback = get_encoder_data(MOTOR_RF);
    snapshot.lb_target = get_speed_target(MOTOR_LB);
    snapshot.lb_feedback = get_encoder_data(MOTOR_LB);
    snapshot.rb_target = get_speed_target(MOTOR_RB);
    snapshot.rb_feedback = get_encoder_data(MOTOR_RB);
    snapshot.lf_pwm = g_motor[MOTOR_LF].pwm_out;
    snapshot.rf_pwm = g_motor[MOTOR_RF].pwm_out;
    snapshot.lb_pwm = g_motor[MOTOR_LB].pwm_out;
    snapshot.rb_pwm = g_motor[MOTOR_RB].pwm_out;
    snapshot.command_scale_pct = motion_exec_get_command_scale_pct();
    snapshot.slip_weight_pct = odometry_get_slip_weight_pct();
    snapshot.lf_target_ramped = speed_pid_get_target_ramped(MOTOR_LF);
    snapshot.rf_target_ramped = speed_pid_get_target_ramped(MOTOR_RF);

    motion_debug_fill_telemetry_page(g_motion_debug_page,
                                     &snapshot,
                                     &a,
                                     &b,
                                     &c,
                                     &d);
    g_debug_a = a;
    g_debug_b = b;
    g_debug_c = c;
    g_debug_d = d;

    g_motion_debug_page_elapsed_ms += period_ms;
    if(g_motion_debug_page_elapsed_ms >= DEBUG_TELEMETRY_PAGE_PERIOD_MS)
    {
        g_motion_debug_page_elapsed_ms = 0U;
        g_motion_debug_page =
            (motion_debug_page_t)(((uint32_t)g_motion_debug_page + 1U) % MOTION_DEBUG_PAGE_COUNT);
        bfs_send_text_line("DBG %s\r\n", motion_debug_telemetry_page_name(g_motion_debug_page));
    }
}

/*
 * One-wheel-at-a-time sign/mapping verification mode.
 *
 * Purpose:
 * - verify every wheel channel drives the intended wheel
 * - verify the matching encoder channel follows that same wheel
 * - verify positive motor command produces positive encoder feedback after
 *   normalization
 *
 * BLE compact output in this mode:
 * - a = LF encoder delta
 * - b = RF encoder delta
 * - c = LB encoder delta
 * - d = RB encoder delta
 *
 * The active wheel and PWM command are printed as BLE text at each step. Reading
 * all four encoder deltas directly makes motor/encoder cross-mapping visible.
 */
static void wheel_sign_check_tick(uint32_t period_ms)
{
    const wheel_sign_check_step_t *step =
        &g_wheel_sign_check_steps[g_wheel_sign_step_index];

    if(g_wheel_sign_last_reported_index != g_wheel_sign_step_index)
    {
        g_wheel_sign_last_reported_index = g_wheel_sign_step_index;
        bfs_send_text_line("Wheel check %s\r\n", step->label);
    }

    g_wheel_sign_step_elapsed_ms += period_ms;

    motor_stop_all();
    if(step->motor_id < MOTOR_MAX)
    {
        motor_set_pwm(step->motor_id, step->pwm);
    }

    encoder_read_data();

    g_debug_a = get_encoder_data(MOTOR_LF);
    g_debug_b = get_encoder_data(MOTOR_RF);
    g_debug_c = get_encoder_data(MOTOR_LB);
    g_debug_d = get_encoder_data(MOTOR_RB);

    if(g_wheel_sign_step_elapsed_ms >= step->duration_ms)
    {
        g_wheel_sign_step_elapsed_ms = 0U;
        g_wheel_sign_step_index =
            (g_wheel_sign_step_index + 1U) %
            (uint32_t)(sizeof(g_wheel_sign_check_steps) / sizeof(g_wheel_sign_check_steps[0]));
    }
}

static void motion_debug_reset_pages(void)
{
    g_motion_debug_page = MOTION_DEBUG_PAGE_POSE;
    g_motion_debug_page_elapsed_ms = 0U;
}

static void smartcar_runtime_init(void)
{
    BlueTooth_Init();
    motor_driver_init();
    Encoder_Init();

    g_debug_a = 0;
    g_debug_b = 0;
    g_debug_c = 0;
    g_debug_d = 0;

#if SMARTCAR_RUNTIME_MODE == SMARTCAR_MODE_WHEEL_SIGN_CHECK
    g_wheel_sign_step_index = 0U;
    g_wheel_sign_step_elapsed_ms = 0U;
    g_wheel_sign_last_reported_index = 0xFFFFFFFFUL;
    motor_stop_all();
    bfs_send_text_line("Wheel sign check mode\r\n");
    return;
#endif

    if(!imu_init_and_calibrate())
    {
        bfs_send_text_line("IMU init failed\r\n");
    }
    else
    {
        imu_reset_yaw();
    }

    motion_display_init();
    motion_debug_reset_pages();

    motion_exec_init();

    g_last_processed_valid_count = 0;
    g_last_map_signature = 0;
    g_stable_frame_count = 0;
    g_last_planned_signature = 0;
    g_has_planned_signature = 0;
    g_history_count = 0;
    g_history_write_index = 0;
    memset(g_raw_map_history, 0, sizeof(g_raw_map_history));
    bfs_runtime_state_reset();

    motion_exec_set_segment_accept_tolerance_mm(EXEC_SEGMENT_ACCEPT_TOL_MM);
    odometry_set_finish_tolerance_mm(POINT_MOVE_FINISH_TOL_MM);

#if SMARTCAR_RUNTIME_MODE == SMARTCAR_MODE_MANUAL_RECT_LAP
    motion_debug_build_inner_rect_lap_plan(&g_manual_rect_lap_plan);
    motion_exec_set_segment_accept_tolerance_mm(DEBUG_EXEC_SEGMENT_ACCEPT_TOL_MM);
    odometry_set_finish_tolerance_mm(DEBUG_POINT_MOVE_FINISH_TOL_MM);
    motion_exec_load_manual_plan(&g_manual_rect_lap_plan, ODOM_START_GRID_X, ODOM_START_GRID_Y);
    motion_exec_request_start();
    bfs_send_text_line("Manual rectangle lap mode\r\n");
    bfs_send_text_line("DBG %s\r\n", motion_debug_telemetry_page_name(g_motion_debug_page));
#else
    openart_protocol_init();
    if(EXEC_AUTO_START_FOR_TEST)
    {
        motion_exec_request_start();
    }
#if SMARTCAR_RUNTIME_MODE == SMARTCAR_MODE_BFS_FIXED_MAP
    bfs_send_text_line("BFS + execution scheduler mode (fixed stage1 map test)\r\n");
#else
    bfs_send_text_line("BFS + execution scheduler mode\r\n");
#endif
    bfs_send_text_line("DBG %s\r\n", motion_debug_telemetry_page_name(g_motion_debug_page));
#endif
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);
    debug_init();
    system_delay_ms(300);

    smartcar_runtime_init();
    interrupt_global_enable(0);
	  monitor_init();


    while(1)
    {
#if SMARTCAR_RUNTIME_MODE == SMARTCAR_MODE_WHEEL_SIGN_CHECK
        wheel_sign_check_tick(EXEC_CONTROL_PERIOD_MS);
#else
#if SMARTCAR_RUNTIME_MODE != SMARTCAR_MODE_MANUAL_RECT_LAP
        bfs_process_new_valid_frame();
#endif
        motion_exec_tick(EXEC_CONTROL_PERIOD_MS);
        motion_display_tick();
        motion_exec_refresh_debug_output(EXEC_CONTROL_PERIOD_MS);
				monitor_tick();
#endif
        send_data(g_debug_a, g_debug_b, g_debug_c, g_debug_d);
        system_delay_ms(EXEC_CONTROL_PERIOD_MS);
    }
}

/*
 * The shared ISR file still references these PIT hooks.
 * Step 2 does not use PIT, so keep explicit no-op stubs for linker stability.
 */
void pit_handler(void)
{
}

void pit1_handler(void)
{
}
