#include "debug_runtime.h"

#include <string.h>

void motion_debug_build_inner_rect_lap_plan(MotionPlan *out_plan)
{
    if(out_plan == 0)
    {
        return;
    }

    memset(out_plan, 0, sizeof(*out_plan));

    out_plan->count = 5U;

    out_plan->data[0].type = SEG_WALK;
    out_plan->data[0].dir = DIR_UP;
    out_plan->data[0].cells = 4U;

    out_plan->data[1].type = SEG_WALK;
    out_plan->data[1].dir = DIR_RIGHT;
    out_plan->data[1].cells = 13U;

    out_plan->data[2].type = SEG_WALK;
    out_plan->data[2].dir = DIR_DOWN;
    out_plan->data[2].cells = 9U;

    out_plan->data[3].type = SEG_WALK;
    out_plan->data[3].dir = DIR_LEFT;
    out_plan->data[3].cells = 13U;

    out_plan->data[4].type = SEG_WALK;
    out_plan->data[4].dir = DIR_UP;
    out_plan->data[4].cells = 5U;
}

const char *motion_debug_telemetry_page_name(motion_debug_page_t page)
{
    switch(page)
    {
        case MOTION_DEBUG_PAGE_POSE:        return "POSE";
        case MOTION_DEBUG_PAGE_TARGET:      return "TARGET";
        case MOTION_DEBUG_PAGE_STATE:       return "STATE";
        case MOTION_DEBUG_PAGE_WHEEL_FRONT: return "WHEEL_F";
        case MOTION_DEBUG_PAGE_WHEEL_REAR:  return "WHEEL_R";
        case MOTION_DEBUG_PAGE_PWM:         return "PWM";
        case MOTION_DEBUG_PAGE_TRACK:       return "TRACK";
        default:                            return "UNKNOWN";
    }
}

void motion_debug_fill_telemetry_page(motion_debug_page_t page,
                                      const motion_debug_snapshot_t *snapshot,
                                      int32_t *a,
                                      int32_t *b,
                                      int32_t *c,
                                      int32_t *d)
{
    if((snapshot == 0) || (a == 0) || (b == 0) || (c == 0) || (d == 0))
    {
        return;
    }

    switch(page)
    {
        case MOTION_DEBUG_PAGE_TARGET:
            *a = snapshot->target_x_mm;
            *b = snapshot->target_y_mm;
            *c = snapshot->dx_mm;
            *d = snapshot->dy_mm;
            break;

        case MOTION_DEBUG_PAGE_STATE:
            *a = snapshot->vx_body_mmps;
            *b = snapshot->vy_body_mmps;
            *c = snapshot->yaw_cd;
            *d = snapshot->yaw_error_cd;
            break;

        case MOTION_DEBUG_PAGE_WHEEL_FRONT:
            *a = snapshot->lf_target;
            *b = snapshot->lf_feedback;
            *c = snapshot->rf_target;
            *d = snapshot->rf_feedback;
            break;

        case MOTION_DEBUG_PAGE_WHEEL_REAR:
            *a = snapshot->lb_target;
            *b = snapshot->lb_feedback;
            *c = snapshot->rb_target;
            *d = snapshot->rb_feedback;
            break;

        case MOTION_DEBUG_PAGE_PWM:
            *a = snapshot->lf_pwm;
            *b = snapshot->rf_pwm;
            *c = snapshot->lb_pwm;
            *d = snapshot->rb_pwm;
            break;

        case MOTION_DEBUG_PAGE_TRACK:
            *a = snapshot->command_scale_pct;
            *b = snapshot->slip_weight_pct;
            *c = snapshot->lf_target_ramped;
            *d = snapshot->rf_target_ramped;
            break;

        case MOTION_DEBUG_PAGE_POSE:
        default:
            *a = snapshot->phase;
            *b = snapshot->segment_index;
            *c = snapshot->x_mm;
            *d = snapshot->y_mm;
            break;
    }
}
