# Task 5-6 Progress

## Date

2026-03-21

## Task 5 Status

Completed on hardware.

### Conclusion

- The single-wheel speed PID bench test has passed.
- The user confirmed the wheel behavior was normal in practical testing.
- The current incremental speed-loop implementation is now the base for four-wheel expansion.

### Code Basis

- `code/uer_algorithm/pid/speed_pid.c`
- `code/uer_algorithm/pid/speed_pid.h`

Controller behavior:

- read encoder feedback every 20 ms
- compute `target - current`
- feed the error into speed PID
- add the PID adjustment directly onto the previous PWM command

## Task 6 Status

In progress.

### This Round's Change

`user/src/main.c` has been switched from Task 5 single-wheel bench mode to Task 6 four-wheel speed closed-loop mode.

### Current Test Setup

- all four wheels run the speed loop every 20 ms
- all four wheels currently use the same target speed
- default target:
  - `TASK6_TARGET_SPEED_PULSE = 120`
- Bluetooth output currently sends the measured wheel speeds:
  - `a = LF`
  - `b = RF`
  - `c = LB`
  - `d = RB`

### Debug Option

If PWM trend needs to be observed instead of speed consistency, change:

- `TASK6_DEBUG_SEND_PWM` from `0` to `1`

Then Bluetooth will send the four wheel PWM commands instead of wheel speeds.

### Air Test Result

Completed on hardware.

User-reported sample data:

- `136, 145, 143, 136`
- `108, 106, 121, 108`
- `130, 138, 127, 128`
- `134, 134, 130, 133`
- `131, 122, 133, 125`

Current conclusion:

- the four-wheel speed loop is active on all wheels
- suspended no-load operation is stable enough to continue
- no obvious violent oscillation was visible in the reported data

### Next Validation

Ground test:

- place the car on the ground
- keep a relatively low target speed
- observe whether straight motion is stable
- check whether wheel effort looks even
- check whether the car shows obvious drift or yaw
