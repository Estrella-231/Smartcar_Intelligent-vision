#include "state_machine.h"

#include <string.h>

/*
 * This source file hosts the OpenART UART receive path.
 *
 * Reason for placing the implementation here:
 * the current MDK/IAR project already compiles state_machine.c, while new .c
 * files would also require project-file edits. Using this compiled unit keeps
 * the first integration step small and buildable.
 */

static volatile uint8_t g_rx_packet[OPENART_PACKET_LEN];
static volatile uint16_t g_rx_index = 0;

static volatile uint8_t g_has_valid_frame = 0;
static volatile openart_frame_t g_latest_frame;

static volatile uint32_t g_valid_frame_count = 0;
static volatile uint32_t g_invalid_status_count = 0;
static volatile uint32_t g_parse_error_count = 0;
static volatile openart_error_t g_last_error = OPENART_ERROR_NONE;
static volatile bfs_runtime_state_t g_bfs_runtime_state;
static volatile motion_exec_runtime_state_t g_motion_exec_runtime_state;

/*
 * Compute the packet checksum exactly as defined by the OpenART document:
 * sum byte 0..196 and keep the low 8 bits.
 */
static uint8_t openart_calc_checksum(const uint8_t *data, uint16_t len)
{
    uint32_t sum = 0;

    for(uint16_t i = 0; i < len; i++)
    {
        sum += data[i];
    }

    return (uint8_t)(sum & 0xFFU);
}

/*
 * Convert one verified packet into the public frame structure.
 *
 * The map is stored in row-major order so later logic can index it directly as
 * map[row][col] with:
 * - row: 0..11
 * - col: 0..15
 */
static void openart_store_frame(const uint8_t *packet)
{
    openart_frame_t temp_frame;
    uint32_t primask;

    temp_frame.status = packet[2];
    temp_frame.player_x = packet[OPENART_PLAYER_X_OFFSET];
    temp_frame.player_y = packet[OPENART_PLAYER_Y_OFFSET];

    for(uint16_t i = 0; i < OPENART_MAP_SIZE; i++)
    {
        uint8_t row = (uint8_t)(i / OPENART_MAP_WIDTH);
        uint8_t col = (uint8_t)(i % OPENART_MAP_WIDTH);
        temp_frame.map[row][col] = (char)packet[OPENART_MAP_OFFSET + i];
    }

    /*
     * Main-loop readers and the UART interrupt share the same frame buffer.
     * Protect the whole structure copy so the caller never sees a torn map.
     */
    primask = interrupt_global_disable();
    memcpy((void *)&g_latest_frame, &temp_frame, sizeof(temp_frame));
    g_has_valid_frame = 1U;
    g_valid_frame_count++;
    g_last_error = OPENART_ERROR_NONE;
    interrupt_global_enable(primask);
}

/*
 * Validate the packet against the communication guide.
 *
 * Invalid-status frames are intentionally treated differently from malformed
 * frames:
 * - malformed frame: parser error count++, packet discarded
 * - status invalid : invalid-status count++, packet discarded, last valid frame kept
 */
static void openart_process_packet(const uint8_t *packet)
{
    if(packet[1] != OPENART_PAYLOAD_LEN)
    {
        g_parse_error_count++;
        g_last_error = OPENART_ERROR_LENGTH;
        return;
    }

    if(packet[OPENART_TAIL_OFFSET] != OPENART_FRAME_TAIL)
    {
        g_parse_error_count++;
        g_last_error = OPENART_ERROR_TAIL;
        return;
    }

    if(openart_calc_checksum(packet, OPENART_CHECKSUM_OFFSET) != packet[OPENART_CHECKSUM_OFFSET])
    {
        g_parse_error_count++;
        g_last_error = OPENART_ERROR_CHECKSUM;
        return;
    }

    if(packet[2] != OPENART_STATUS_VALID)
    {
        g_invalid_status_count++;
        g_last_error = OPENART_ERROR_STATUS;
        return;
    }

    if((packet[OPENART_PLAYER_X_OFFSET] >= OPENART_MAP_WIDTH) ||
       (packet[OPENART_PLAYER_Y_OFFSET] >= OPENART_MAP_HEIGHT))
    {
        g_parse_error_count++;
        g_last_error = OPENART_ERROR_PLAYER_RANGE;
        return;
    }

    openart_store_frame(packet);
}

/*
 * Reset the byte collector to the idle "wait for frame header" state.
 */
static void openart_reset_receiver(void)
{
    g_rx_index = 0U;
}

void openart_protocol_init(void)
{
    uint32_t primask;

    uart_init(OPENART_UART_INDEX,
              OPENART_UART_BAUDRATE,
              OPENART_UART_TX_PIN,
              OPENART_UART_RX_PIN);
    uart_rx_interrupt(OPENART_UART_INDEX, 1);

    primask = interrupt_global_disable();
    memset((void *)&g_latest_frame, 0, sizeof(g_latest_frame));
    g_has_valid_frame = 0U;
    g_valid_frame_count = 0U;
    g_invalid_status_count = 0U;
    g_parse_error_count = 0U;
    g_last_error = OPENART_ERROR_NONE;
    g_rx_index = 0U;
    interrupt_global_enable(primask);
}

/*
 * Drain all bytes currently queued in UART4.
 *
 * The receiver uses a simple fixed-length packet collector:
 * - wait for header 0xDF
 * - collect 199 bytes
 * - validate the complete packet
 * - on any failure, drop the packet and wait for the next header
 *
 * This is enough for the current one-way fixed-frame protocol and keeps the
 * first hardware integration easy to reason about.
 */
void openart_uart_rx_handler(void)
{
    uint8_t data;

    while(uart_query_byte(OPENART_UART_INDEX, &data))
    {
        if(0U == g_rx_index)
        {
            if(data == OPENART_FRAME_HEADER)
            {
                g_rx_packet[g_rx_index++] = data;
            }
            continue;
        }

        g_rx_packet[g_rx_index++] = data;

        if(g_rx_index >= OPENART_PACKET_LEN)
        {
            openart_process_packet((const uint8_t *)g_rx_packet);
            openart_reset_receiver();
        }
    }
}

uint8_t openart_has_valid_frame(void)
{
    return g_has_valid_frame;
}

uint8_t openart_copy_latest_frame(openart_frame_t *out_frame)
{
    uint32_t primask;

    if((NULL == out_frame) || (0U == g_has_valid_frame))
    {
        return 0U;
    }

    primask = interrupt_global_disable();
    memcpy(out_frame, (const void *)&g_latest_frame, sizeof(openart_frame_t));
    interrupt_global_enable(primask);
    return 1U;
}

uint32_t openart_get_valid_frame_count(void)
{
    return g_valid_frame_count;
}

uint32_t openart_get_invalid_status_count(void)
{
    return g_invalid_status_count;
}

uint32_t openart_get_parse_error_count(void)
{
    return g_parse_error_count;
}

openart_error_t openart_get_last_error(void)
{
    return g_last_error;
}

int32_t openart_get_player_x(void)
{
    int32_t player_x = -1;
    uint32_t primask;

    if(0U == g_has_valid_frame)
    {
        return -1;
    }

    primask = interrupt_global_disable();
    player_x = (int32_t)g_latest_frame.player_x;
    interrupt_global_enable(primask);
    return player_x;
}

int32_t openart_get_player_y(void)
{
    int32_t player_y = -1;
    uint32_t primask;

    if(0U == g_has_valid_frame)
    {
        return -1;
    }

    primask = interrupt_global_disable();
    player_y = (int32_t)g_latest_frame.player_y;
    interrupt_global_enable(primask);
    return player_y;
}

void bfs_runtime_state_reset(void)
{
    uint32_t primask;
    bfs_runtime_state_t reset_state;

    memset(&reset_state, 0, sizeof(reset_state));
    reset_state.phase = BFS_RUNTIME_WAIT_FRAME;
    reset_state.last_plan_status = SOKO_OK;

    primask = interrupt_global_disable();
    memcpy((void *)&g_bfs_runtime_state, &reset_state, sizeof(reset_state));
    interrupt_global_enable(primask);
}

void bfs_runtime_state_update_bridge(uint32_t valid_frame_count,
                                     uint32_t stable_frame_count,
                                     uint32_t filtered_map_signature,
                                     uint8_t player_x,
                                     uint8_t player_y,
                                     uint8_t box_count,
                                     uint8_t target_count,
                                     uint8_t bomb_count,
                                     uint8_t stable_ready)
{
    uint32_t primask;

    primask = interrupt_global_disable();

    g_bfs_runtime_state.has_filtered_map = 1U;
    g_bfs_runtime_state.valid_frame_count = valid_frame_count;
    g_bfs_runtime_state.stable_frame_count = stable_frame_count;
    g_bfs_runtime_state.filtered_map_signature = filtered_map_signature;
    g_bfs_runtime_state.player_x = player_x;
    g_bfs_runtime_state.player_y = player_y;
    g_bfs_runtime_state.box_count = box_count;
    g_bfs_runtime_state.target_count = target_count;
    g_bfs_runtime_state.bomb_count = bomb_count;

    /*
     * Preserve a completed plan state while the same filtered map remains in
     * view. Otherwise the periodic bridge refresh would overwrite PLAN_OK with
     * BRIDGE_OK or STABLE_READY even though the latest formal plan result is
     * still valid for this exact map signature.
     */
    if(g_bfs_runtime_state.has_plan &&
       (g_bfs_runtime_state.planned_map_signature == filtered_map_signature) &&
       (SOKO_OK == g_bfs_runtime_state.last_plan_status))
    {
        g_bfs_runtime_state.phase = BFS_RUNTIME_PLAN_OK;
    }
    else if(stable_ready)
    {
        g_bfs_runtime_state.phase = BFS_RUNTIME_STABLE_READY;
    }
    else
    {
        g_bfs_runtime_state.phase = BFS_RUNTIME_BRIDGE_OK;
    }

    interrupt_global_enable(primask);
}

void bfs_runtime_state_update_plan(uint32_t planned_map_signature,
                                   SokoStatus plan_status,
                                   const ActionSeq *action_seq,
                                   const MotionPlan *motion_plan)
{
    uint32_t primask;

    primask = interrupt_global_disable();

    g_bfs_runtime_state.planned_map_signature = planned_map_signature;
    g_bfs_runtime_state.last_plan_status = plan_status;

    if((SOKO_OK == plan_status) && (0 != action_seq) && (0 != motion_plan))
    {
        memcpy((void *)&g_bfs_runtime_state.last_action_seq, action_seq, sizeof(ActionSeq));
        memcpy((void *)&g_bfs_runtime_state.last_motion_plan, motion_plan, sizeof(MotionPlan));
        g_bfs_runtime_state.has_plan = 1U;
        g_bfs_runtime_state.phase = BFS_RUNTIME_PLAN_OK;
    }
    else
    {
        memset((void *)&g_bfs_runtime_state.last_action_seq, 0, sizeof(ActionSeq));
        memset((void *)&g_bfs_runtime_state.last_motion_plan, 0, sizeof(MotionPlan));
        g_bfs_runtime_state.has_plan = 0U;
        g_bfs_runtime_state.phase = BFS_RUNTIME_PLAN_ERROR;
    }

    interrupt_global_enable(primask);
}

uint8_t bfs_runtime_state_copy(bfs_runtime_state_t *out_state)
{
    uint32_t primask;

    if(0 == out_state)
    {
        return 0U;
    }

    primask = interrupt_global_disable();
    memcpy(out_state, (const void *)&g_bfs_runtime_state, sizeof(bfs_runtime_state_t));
    interrupt_global_enable(primask);
    return 1U;
}

void motion_exec_runtime_state_reset(void)
{
    uint32_t primask;
    motion_exec_runtime_state_t reset_state;

    memset(&reset_state, 0, sizeof(reset_state));
    reset_state.phase = CAR_STATE_WAIT_START;
    reset_state.error_code = EXEC_ERROR_NONE;

    primask = interrupt_global_disable();
    memcpy((void *)&g_motion_exec_runtime_state, &reset_state, sizeof(reset_state));
    interrupt_global_enable(primask);
}

void motion_exec_runtime_state_store(const motion_exec_runtime_state_t *state)
{
    uint32_t primask;

    if(0 == state)
    {
        return;
    }

    primask = interrupt_global_disable();
    memcpy((void *)&g_motion_exec_runtime_state, state, sizeof(motion_exec_runtime_state_t));
    interrupt_global_enable(primask);
}

uint8_t motion_exec_runtime_state_copy(motion_exec_runtime_state_t *out_state)
{
    uint32_t primask;

    if(0 == out_state)
    {
        return 0U;
    }

    primask = interrupt_global_disable();
    memcpy(out_state, (const void *)&g_motion_exec_runtime_state, sizeof(motion_exec_runtime_state_t));
    interrupt_global_enable(primask);
    return 1U;
}
