# Task 4 Module Review

## Date

2026-03-21

## Current Conclusion

Task 4 is not fully complete yet.

The project already has directory-level layering:
- `code/user_driver`
- `code/uer_algorithm`
- `code/user_business`

But real encapsulation is still incomplete because several algorithm and business
files directly read or write shared globals such as `g_motor`, `g_car`, `g_imu`,
`g_angle`, and `g_rotate`.

## Cleanup Completed In This Round

- Narrowed `code/user_business/robot_control.h` so it no longer re-exports the whole driver and algorithm stack.
- Added explicit public declarations to:
  - `code/uer_algorithm/pid/speed_pid.h`
  - `code/uer_algorithm/pid/position_pid.h`
- Added accessor-style APIs for:
  - aggregate car state in `code/user_driver/motor_driver.h/.c`
  - IMU readout in `code/user_driver/gyroscope.h/.c`
  - angle PID target/reset in `code/uer_algorithm/pid/angle_pid.h/.c`
- Switched part of the call chain to those APIs:
  - `code/user_business/robot_control.c`
  - `code/uer_algorithm/kinematics/mecanum.c`
  - `code/uer_algorithm/kinematics/odometry.c`
  - `code/uer_algorithm/pid/position_pid.c`

## Remaining Problems

- `code/user_business/state_machine.c` is empty.
- `code/user_business/push_action.c` is empty.
- `angle_pid.c` still writes wheel PWM through `g_motor[i].pwm_out`.
- `Calc_SpeedNow()` still updates wheel state directly inside `g_motor[]`.
- Position/speed PID still depend on shared global storage, only with fewer direct cross-layer touches than before.

## Recommended Next Step

Before Task 5, keep the current structure but continue using the new access
functions instead of adding fresh direct writes to `g_motor`, `g_car`, or
`g_imu`.
