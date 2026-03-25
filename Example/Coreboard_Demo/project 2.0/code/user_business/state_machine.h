#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "vision_config.h"
#include "soko_types.h"

/*
 * Public data shape for one parsed OpenART frame.
 *
 * The communication guide fixes a single valid frame to:
 * - one status byte
 * - one 16x12 ASCII map
 * - one player coordinate pair
 *
 * When RT1064 receives an invalid-status frame, this structure is NOT updated.
 * The caller therefore always sees the latest valid frame that passed:
 * - header check
 * - payload-length check
 * - tail check
 * - checksum check
 * - status == OPENART_STATUS_VALID
 */
typedef struct
{
    uint8_t status;
    char map[OPENART_MAP_HEIGHT][OPENART_MAP_WIDTH];
    uint8_t player_x;
    uint8_t player_y;
} openart_frame_t;

typedef enum
{
    OPENART_ERROR_NONE = 0,
    OPENART_ERROR_LENGTH = 1,
    OPENART_ERROR_TAIL = 2,
    OPENART_ERROR_CHECKSUM = 3,
    OPENART_ERROR_STATUS = 4,
    OPENART_ERROR_PLAYER_RANGE = 5,
} openart_error_t;

void openart_protocol_init(void);
void openart_uart_rx_handler(void);

uint8_t openart_has_valid_frame(void);
uint8_t openart_copy_latest_frame(openart_frame_t *out_frame);

uint32_t openart_get_valid_frame_count(void);
uint32_t openart_get_invalid_status_count(void);
uint32_t openart_get_parse_error_count(void);
openart_error_t openart_get_last_error(void);

int32_t openart_get_player_x(void);
int32_t openart_get_player_y(void);

/*
 * Public runtime snapshot for the BFS bridge / planner pipeline.
 *
 * This structure is the formal system-readable state that later business logic
 * can consume directly, instead of scraping BLE text logs.
 */
typedef enum
{
    BFS_RUNTIME_IDLE = 0,
    BFS_RUNTIME_WAIT_FRAME,
    BFS_RUNTIME_BRIDGE_OK,
    BFS_RUNTIME_STABLE_READY,
    BFS_RUNTIME_PLAN_OK,
    BFS_RUNTIME_PLAN_ERROR
} bfs_runtime_phase_t;

typedef struct
{
    uint8_t has_filtered_map;
    uint8_t has_plan;
    bfs_runtime_phase_t phase;

    uint32_t valid_frame_count;
    uint32_t stable_frame_count;
    uint32_t filtered_map_signature;
    uint32_t planned_map_signature;

    uint8_t player_x;
    uint8_t player_y;
    uint8_t box_count;
    uint8_t target_count;
    uint8_t bomb_count;

    SokoStatus last_plan_status;
    ActionSeq last_action_seq;
    MotionPlan last_motion_plan;
} bfs_runtime_state_t;

void bfs_runtime_state_reset(void);
void bfs_runtime_state_update_bridge(uint32_t valid_frame_count,
                                     uint32_t stable_frame_count,
                                     uint32_t filtered_map_signature,
                                     uint8_t player_x,
                                     uint8_t player_y,
                                     uint8_t box_count,
                                     uint8_t target_count,
                                     uint8_t bomb_count,
                                     uint8_t stable_ready);
void bfs_runtime_state_update_plan(uint32_t planned_map_signature,
                                   SokoStatus plan_status,
                                   const ActionSeq *action_seq,
                                   const MotionPlan *motion_plan);
uint8_t bfs_runtime_state_copy(bfs_runtime_state_t *out_state);

/*
 * Formal runtime snapshot for the motion execution scheduler.
 *
 * This sits one layer above BFS:
 * - BFS decides "what path to take"
 * - the execution scheduler decides "which segment is being executed now"
 */
typedef enum
{
    CAR_STATE_WAIT_START = 0,
    CAR_STATE_WAIT_MAP,
    CAR_STATE_WAIT_BFS,
    CAR_STATE_LOAD_PLAN,
    CAR_STATE_EXEC_SEGMENT,
    CAR_STATE_WAIT_SEGMENT_FINISH,
    CAR_STATE_PLAN_DONE,
    CAR_STATE_ERROR
} car_exec_phase_t;

typedef enum
{
    EXEC_ERROR_NONE = 0,
    EXEC_ERROR_NO_FILTERED_MAP,
    EXEC_ERROR_NO_BFS_PLAN,
    EXEC_ERROR_EMPTY_PLAN,
    EXEC_ERROR_SEGMENT_RANGE,
    EXEC_ERROR_UNSUPPORTED_SEGMENT
} exec_error_t;

typedef struct
{
    uint8_t start_requested;
    uint8_t plan_loaded;
    uint8_t plan_finished;
    car_exec_phase_t phase;
    exec_error_t error_code;

    uint32_t source_plan_signature;
    uint16_t current_segment_index;
    uint8_t current_grid_x;
    uint8_t current_grid_y;

    MotionSegment current_segment;
    int32_t segment_target_x_mm;
    int32_t segment_target_y_mm;
    int32_t segment_wait_remaining_ms;
} motion_exec_runtime_state_t;

void motion_exec_runtime_state_reset(void);
void motion_exec_runtime_state_store(const motion_exec_runtime_state_t *state);
uint8_t motion_exec_runtime_state_copy(motion_exec_runtime_state_t *out_state);

#endif
