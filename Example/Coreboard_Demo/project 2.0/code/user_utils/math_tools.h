#ifndef MATH_TOOLS_H
#define MATH_TOOLS_H

#include "zf_common_headfile.h"
#include <math.h>
#include "robot_param.h"


/************************ 通用工具宏 ************************/
#define LIMIT(x, min, max)    ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))   // 数值限幅宏
#define LIMIT_ABS(x, limit)   LIMIT(x, -(limit), (limit))                           // 绝对值限幅宏
#define MM_TO_PULSE(mm)     ((int32_t)((mm) * PULSE_PER_MM))                        // 距离->编码器脉冲
#define PULSE_TO_MM(pulse)  ((int32_t)((pulse) / PULSE_PER_MM))                     // 编码器脉冲->距离
                                                                       














#endif
