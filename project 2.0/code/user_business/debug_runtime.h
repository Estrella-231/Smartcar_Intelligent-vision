#ifndef DEBUG_RUNTIME_H
#define DEBUG_RUNTIME_H

#include "soko_types.h"

typedef enum
{
    MOTION_DEBUG_PAGE_POSE = 0,
    MOTION_DEBUG_PAGE_TARGET,
    MOTION_DEBUG_PAGE_STATE,
    MOTION_DEBUG_PAGE_WHEEL_FRONT,
    MOTION_DEBUG_PAGE_WHEEL_REAR,
    MOTION_DEBUG_PAGE_PWM,
    MOTION_DEBUG_PAGE_TRACK,
    MOTION_DEBUG_PAGE_COUNT
} motion_debug_page_t;

typedef struct
{
    int32_t phase;
    int32_t segment_index;
    int32_t x_mm;
    int32_t y_mm;

    int32_t target_x_mm;
    int32_t target_y_mm;
    int32_t dx_mm;
    int32_t dy_mm;

    int32_t vx_body_mmps;
    int32_t vy_body_mmps;
    int32_t yaw_cd;
    int32_t yaw_error_cd;

    int32_t lf_target;
    int32_t lf_feedback;
    int32_t rf_target;
    int32_t rf_feedback;
    int32_t lb_target;
    int32_t lb_feedback;
    int32_t rb_target;
    int32_t rb_feedback;

    int32_t lf_pwm;
    int32_t rf_pwm;
    int32_t lb_pwm;
    int32_t rb_pwm;

    int32_t command_scale_pct;
    int32_t slip_weight_pct;
    int32_t lf_target_ramped;
    int32_t rf_target_ramped;
} motion_debug_snapshot_t;

void motion_debug_build_inner_rect_lap_plan(MotionPlan *out_plan);
const char *motion_debug_telemetry_page_name(motion_debug_page_t page);
void motion_debug_fill_telemetry_page(motion_debug_page_t page,
                                      const motion_debug_snapshot_t *snapshot,
                                      int32_t *a,
                                      int32_t *b,
                                      int32_t *c,
                                      int32_t *d);

#endif
