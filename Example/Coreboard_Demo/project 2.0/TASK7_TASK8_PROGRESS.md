# Task 7-8 Progress

## Date

2026-03-21

## Task 7 Status

Completed on hardware.

### Reported Result

The user confirmed that:

- pure forward was normal
- pure clockwise rotation was normal
- combined motion was normal
- the remaining test item was also finished and passed

### Conclusion

Task 7 inverse kinematics can be considered complete for the current stage.

## Task 8 Status

In progress.

### This Round's Change

The codebase has been switched to a dedicated Task 8 IMU high-rate test mode.

Files changed:

- `code/user_config/hardware_config.h`
- `code/user_driver/gyroscope.h`
- `code/user_driver/gyroscope.c`
- `user/src/isr.c`
- `user/src/main.c`

### Key Technical Change

- PIT channels are now split into:
  - `PIT_CH_CONTROL = PIT_CH0`
  - `PIT_CH_IMU_FAST = PIT_CH1`
- IMU integration has been changed from fixed-period logic to period-aware logic
- a 5 ms IMU fast loop now runs through `pit1_handler()`
- the 20 ms control/debug rhythm is still kept through `pit_handler()`

### Current Test Output

Bluetooth now sends:

- `a = IMU init status`
- `b = raw gyro_z`
- `c = gyro_z in 0.01 deg/s`
- `d = yaw in 0.01 deg`

### Next Validation

- power on and keep the car still
- check whether yaw remains nearly stable
- rotate the car manually and observe whether yaw responds with less delay
- rotate about 360 degrees and record the final accumulated yaw error
