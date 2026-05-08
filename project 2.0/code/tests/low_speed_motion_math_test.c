#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "../user_utils/low_speed_motion_math.h"

#define TEST_PULSE_PER_MM   (7.7f)
#define TEST_ZERO_OFFSET    (20)

static void test_encoder_position_delta_keeps_low_speed_motion(void)
{
    EncoderDeltaSample_t sample;

    sample = low_speed_encoder_prepare_delta(1, 1, TEST_ZERO_OFFSET);
    assert(sample.feedback_delta == 0);
    assert(sample.position_delta == 1);

    sample = low_speed_encoder_prepare_delta(-2, -1, TEST_ZERO_OFFSET);
    assert(sample.feedback_delta == 0);
    assert(sample.position_delta == 2);
}

static void test_mecanum_body_delta_accumulates_sub_mm_motion(void)
{
    int32_t vx_body_um;
    int32_t vy_body_um;
    int32_t total_vy_um = 0;

    for(int i = 0; i < 10; i++)
    {
        low_speed_mecanum_body_delta_um(1,
                                        1,
                                        1,
                                        1,
                                        TEST_PULSE_PER_MM,
                                        &vx_body_um,
                                        &vy_body_um);
        assert(vx_body_um == 0);
        assert(vy_body_um > 0);
        total_vy_um += vy_body_um;
    }

    assert(total_vy_um >= 1000);
}

int main(void)
{
    test_encoder_position_delta_keeps_low_speed_motion();
    test_mecanum_body_delta_accumulates_sub_mm_motion();

    puts("low_speed_motion_math_test passed");
    return 0;
}
