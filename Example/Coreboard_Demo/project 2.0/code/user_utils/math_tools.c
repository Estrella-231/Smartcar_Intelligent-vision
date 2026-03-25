#include "math_tools.h"



int32_t speed2pwm(int32_t speed)
{ 
    int32_t pwm = speed;
    pwm = LIMIT(pwm, SPEED_PWM_MIN_OUTPUT, SPEED_PWM_MAX_OUTPUT);
    return pwm;
}