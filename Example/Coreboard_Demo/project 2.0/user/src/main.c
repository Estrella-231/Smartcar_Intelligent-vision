#include "zf_common_headfile.h"

#include <stdarg.h>
#include <string.h>

#include "bluetooth.h"
#include "encoder.h"
#include "gyroscope.h"
#include "odometry.h"
#include "robot_control.h"
#include "state_machine.h"

#include "soko_map.h"
#include "soko_motion_adapter.h"
#include "soko_replay.h"
#include "soko_stage1.h"

/*
 * Current entry mode:
 * - keep the OpenART parser code available
 * - allow one fixed 16 x 12 map to replace live OpenART frames during ground test
 * - inject the known competition start cell (1, 6) as the player position
 * - run the full BFS -> execution scheduler chain on that stable map
 *
 * The fixed-map path exists because the team does not yet have the final
 * OpenART bracket and competition screen fixture on the car. This lets the
 * chassis and push sequence be tuned on real ground first, while preserving the
 * same bridge / planner / execution interfaces for later live-vision tests.
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
#define BFS_USE_FIXED_STAGE1_MAP_TEST       (1)
#define BFS_FIXED_PLAYER_X                  (1)
#define BFS_FIXED_PLAYER_Y                  (6)

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
 * Output definition in the current movement-execution stage:
 * - a = execution state-machine phase
 * - b = current segment index
 * - c = odometry x_mm
 * - d = odometry y_mm
 *
 * More detailed planner and execution data is still printed as BLE text by the
 * BFS bridge and the scheduler themselves.
 */
static void motion_exec_refresh_debug_output(void)
{
    motion_exec_runtime_state_t exec_state;

    if(motion_exec_runtime_state_copy(&exec_state))
    {
        g_debug_a = (int32_t)exec_state.phase;
        g_debug_b = (int32_t)exec_state.current_segment_index;
        g_debug_c = odometry_get_x_mm();
        g_debug_d = odometry_get_y_mm();
    }
}

static void smartcar_runtime_init(void)
{
    BlueTooth_Init();
    motor_driver_init();
    Encoder_Init();

    if(!imu_init_and_calibrate())
    {
        bfs_send_text_line("IMU init failed\r\n");
    }
    else
    {
        imu_reset_yaw();
    }

    openart_protocol_init();
    motion_display_init();

    g_debug_a = BFS_STEP2_STATUS_WAIT_FRAME;
    g_debug_b = 0;
    g_debug_c = 0;
    g_debug_d = 0;

    g_last_processed_valid_count = 0;
    g_last_map_signature = 0;
    g_stable_frame_count = 0;
    g_last_planned_signature = 0;
    g_has_planned_signature = 0;
    g_history_count = 0;
    g_history_write_index = 0;
    memset(g_raw_map_history, 0, sizeof(g_raw_map_history));
    bfs_runtime_state_reset();
    motion_exec_init();

    if(EXEC_AUTO_START_FOR_TEST)
    {
        motion_exec_request_start();
    }

#if BFS_USE_FIXED_STAGE1_MAP_TEST
    bfs_send_text_line("BFS + execution scheduler mode (fixed stage1 map test)\r\n");
#else
    bfs_send_text_line("BFS + execution scheduler mode\r\n");
#endif
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);
    debug_init();
    system_delay_ms(300);

    smartcar_runtime_init();
    interrupt_global_enable(0);

    while(1)
    {
        bfs_process_new_valid_frame();
        motion_exec_tick(EXEC_CONTROL_PERIOD_MS);
        motion_display_tick();
        motion_exec_refresh_debug_output();
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
