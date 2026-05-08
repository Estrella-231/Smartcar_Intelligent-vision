#ifndef LOW_SPEED_MOTION_MATH_H
#define LOW_SPEED_MOTION_MATH_H

#include <stdint.h>

typedef struct {
    int16_t feedback_delta;
    int16_t position_delta;
} EncoderDeltaSample_t;

static inline EncoderDeltaSample_t low_speed_encoder_prepare_delta(int16_t raw_count,
                                                                   int16_t forward_sign,
                                                                   int16_t zero_offset)
{
    EncoderDeltaSample_t sample;
    int32_t signed_delta = (int32_t)raw_count * (int32_t)forward_sign;

    sample.position_delta = (int16_t)signed_delta;
    sample.feedback_delta =
        ((signed_delta <= zero_offset) && (signed_delta >= -zero_offset)) ?
        0 :
        (int16_t)signed_delta;

    return sample;
}

static inline void low_speed_mecanum_body_delta_um(int32_t lf_delta,
                                                   int32_t rf_delta,
                                                   int32_t lb_delta,
                                                   int32_t rb_delta,
                                                   float pulse_per_mm,
                                                   int32_t *vx_body_um,
                                                   int32_t *vy_body_um)
{
    float denom = 4.0f * pulse_per_mm;

    if((vx_body_um != NULL) || (vy_body_um != NULL))
    {
        if(denom <= 0.0f)
        {
            if(vx_body_um != NULL)
            {
                *vx_body_um = 0;
            }

            if(vy_body_um != NULL)
            {
                *vy_body_um = 0;
            }

            return;
        }
    }

    if(vx_body_um != NULL)
    {
        *vx_body_um =
            (int32_t)(((float)(lf_delta - rf_delta - lb_delta + rb_delta) * 1000.0f) / denom);
    }

    if(vy_body_um != NULL)
    {
        *vy_body_um =
            (int32_t)(((float)(lf_delta + rf_delta + lb_delta + rb_delta) * 1000.0f) / denom);
    }
}

#endif
