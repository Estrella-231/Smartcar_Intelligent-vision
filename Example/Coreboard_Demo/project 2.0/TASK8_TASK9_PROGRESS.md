# Task 8-9 Progress

## Date

2026-03-21

## Task 8 Status

Basic high-rate IMU integration completed and verified on hardware.

### Reported Result

Static test:

- IMU status stayed valid
- yaw output stayed at `0`
- no obvious self-drift was observed in the reported data

One-turn manual rotation test:

- final yaw was about `-348.06 deg`
- ideal result for one clockwise turn would be about `-360.00 deg`
- current accumulated error is about `11.94 deg`

### Current Conclusion

- the 5 ms IMU integration path is working
- static stability is good
- responsiveness is available for closed-loop heading control
- full-turn accuracy is improved enough to continue, but not drift-free

## Task 9 Status

In progress.

### This Round's Change

Files changed:

- `code/uer_algorithm/pid/angle_pid.h`
- `code/uer_algorithm/pid/angle_pid.c`
- `user/src/main.c`

### New Control Structure

- outer loop: angle PID
- inner loop: four-wheel speed PID

Current flow:

1. read current yaw from IMU
2. compute heading error against target yaw
3. convert angle error into `vz_correction`
4. feed `(base_vx, base_vy, vz_correction)` into inverse kinematics
5. execute the resulting wheel targets through the existing speed loop

### Current Test Mode

Default mode is stationary heading hold:

- `TASK9_TARGET_YAW_CD = 0`
- `TASK9_BASE_VX_CMD = 0`
- `TASK9_BASE_VY_CMD = 0`

Bluetooth output:

- `a = target yaw`
- `b = current yaw`
- `c = vz correction`
- `d = angle error`

### Next Validation

- keep the car still and manually twist the body
- check whether the correction term reacts immediately
- check whether the car actively counters the disturbance
- after that, test with non-zero base translation

### Static Disturbance Update

The user reported that the chassis actively twisted back when disturbed by hand.

Reported debug behavior:

- target yaw stayed near `0`
- current yaw changed under disturbance
- `vz_correction` rose quickly and hit the configured task-level limit of about `+/-180`
- after the body returned, correction dropped back toward zero

### Current Conclusion

- Task 9 basic attitude closed-loop behavior is working
- the "static anti-disturbance" part has passed
- the next step is the Task 10 moving heading-hold test

## 2026-03-21 Additional Tuning Update

The user reported that moving heading hold was basically correct, but with a
slight shake during right translation.

To reduce that shake, the following tuning-oriented changes were applied:

- the Task 9/10 heading outer loop was made more conservative
- the derivative contribution was removed from the heading-hold outer loop
- a slightly wider small-error deadband was added
- a small output deadband was added
- the task-level `Vz` limit was reduced from `180` to `120`

Goal of this change:

- keep heading-hold behavior
- reduce small alternating corrective yaw commands during translation

## 2026-03-21 Follow-up Tuning Update

The previous conservative PI-style attempt made the left-right shake worse.

The heading-hold outer loop has now been changed again for moving use:

- removed integral accumulation from the moving heading-hold path
- switched to proportional heading correction plus gyro-rate damping
- added output slew limiting
- reduced the task-level `Vz` limit again from `120` to `90`

Reason for this change:

- the reported oscillation pattern is more consistent with over-correction and
  integral carryover than with insufficient correction
- moving heading hold usually benefits from damping on measured yaw rate
  instead of stronger integral action

## 2026-03-21 Task 10 Validation Update

The user reported that after the latest tuning:

- right-translation heading hold worked very well
- heading remained controlled
- the previous left-right shake was greatly reduced

Reported debug data also shows a much calmer correction pattern than the
previous oscillatory version. The correction term now returns to `0` more often
instead of staying pinned at the task-level limit.

### Current Conclusion

- Task 10 moving heading-hold validation can be considered complete
- the current stage objective of motion control plus basic localization/heading
  modules has reached a usable debug milestone
