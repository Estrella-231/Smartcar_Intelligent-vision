#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../user_business/debug_runtime.h"

static void test_build_inner_rect_lap_plan_matches_expected_route(void)
{
    MotionPlan plan;

    memset(&plan, 0, sizeof(plan));
    motion_debug_build_inner_rect_lap_plan(&plan);

    assert(plan.count == 5U);

    assert(plan.data[0].type == SEG_WALK);
    assert(plan.data[0].dir == DIR_UP);
    assert(plan.data[0].cells == 4U);

    assert(plan.data[1].type == SEG_WALK);
    assert(plan.data[1].dir == DIR_RIGHT);
    assert(plan.data[1].cells == 13U);

    assert(plan.data[2].type == SEG_WALK);
    assert(plan.data[2].dir == DIR_DOWN);
    assert(plan.data[2].cells == 9U);

    assert(plan.data[3].type == SEG_WALK);
    assert(plan.data[3].dir == DIR_LEFT);
    assert(plan.data[3].cells == 13U);

    assert(plan.data[4].type == SEG_WALK);
    assert(plan.data[4].dir == DIR_UP);
    assert(plan.data[4].cells == 5U);
}

static void test_fill_telemetry_page_exposes_target_and_wheel_data(void)
{
    motion_debug_snapshot_t snapshot;
    int32_t a = 0;
    int32_t b = 0;
    int32_t c = 0;
    int32_t d = 0;

    memset(&snapshot, 0, sizeof(snapshot));
    snapshot.target_x_mm = 2800;
    snapshot.target_y_mm = 300;
    snapshot.dx_mm = -40;
    snapshot.dy_mm = 25;
    snapshot.lf_target = 120;
    snapshot.lf_feedback = 118;
    snapshot.rf_target = 122;
    snapshot.rf_feedback = 119;
    snapshot.command_scale_pct = 90;
    snapshot.slip_weight_pct = 75;
    snapshot.lf_target_ramped = 80;
    snapshot.rf_target_ramped = 82;
    snapshot.lf_pwm = 40;
    snapshot.rf_pwm = 41;
    snapshot.lb_pwm = 42;
    snapshot.rb_pwm = 43;

    motion_debug_fill_telemetry_page(MOTION_DEBUG_PAGE_TARGET, &snapshot, &a, &b, &c, &d);
    assert(a == 2800);
    assert(b == 300);
    assert(c == -40);
    assert(d == 25);

    motion_debug_fill_telemetry_page(MOTION_DEBUG_PAGE_WHEEL_FRONT, &snapshot, &a, &b, &c, &d);
    assert(a == 120);
    assert(b == 118);
    assert(c == 122);
    assert(d == 119);

    assert(strcmp(motion_debug_telemetry_page_name(MOTION_DEBUG_PAGE_WHEEL_FRONT), "WHEEL_F") == 0);

    motion_debug_fill_telemetry_page(MOTION_DEBUG_PAGE_PWM, &snapshot, &a, &b, &c, &d);
    assert(a == 40);
    assert(b == 41);
    assert(c == 42);
    assert(d == 43);
    assert(strcmp(motion_debug_telemetry_page_name(MOTION_DEBUG_PAGE_PWM), "PWM") == 0);

    motion_debug_fill_telemetry_page(MOTION_DEBUG_PAGE_TRACK, &snapshot, &a, &b, &c, &d);
    assert(a == 90);
    assert(b == 75);
    assert(c == 80);
    assert(d == 82);
    assert(strcmp(motion_debug_telemetry_page_name(MOTION_DEBUG_PAGE_TRACK), "TRACK") == 0);
}

int main(void)
{
    test_build_inner_rect_lap_plan_matches_expected_route();
    test_fill_telemetry_page_exposes_target_and_wheel_data();

    puts("motion_debug_plan_test passed");
    return 0;
}
