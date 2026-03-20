#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include "zf_common_headfile.h"

/*
 * Business-layer control entry points should expose only the API that higher
 * layers need to call.
 *
 * Avoid turning this header into a catch-all include for drivers, algorithms,
 * Bluetooth, or IMU state. Those implementation details belong in
 * robot_control.c so module boundaries stay readable.
 */
void Control_Rotate(int32_t angle);

#endif // ROBOT_CONTROL_H
