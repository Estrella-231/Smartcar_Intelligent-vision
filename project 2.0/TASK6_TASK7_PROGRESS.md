# Task 6-7 Progress

## Date

2026-03-21

## Task 6 Status

Completed on hardware.

### Ground Test Result

The user reported that after placing the car on the ground:

- straight motion was good
- wheel effort looked basically even
- startup was smooth
- no obvious shaking was observed

User-reported sample data:

- `114, 114, 108, 116`
- `128, 129, 127, 132`
- `130, 130, 134, 135`
- `128, 130, 130, 132`
- `114, 114, 86, 134`

### Conclusion

- the four-wheel speed closed loop is usable under load
- Task 6 can be considered complete
- the project can move to Task 7 inverse kinematics

## Task 7 Status

In progress.

### This Round's Change

Added a general inverse-kinematics entry in:

- `code/uer_algorithm/kinematics/mecanum.c`
- `code/uer_algorithm/kinematics/mecanum.h`

New interface:

- `Mecanum_inverse_kinematics(int32_t vx_cmd, int32_t vy_cmd, int32_t vz_cmd)`

### Coordinate Convention

- `vx_cmd > 0`: move right
- `vy_cmd > 0`: move forward
- `vz_cmd > 0`: rotate clockwise

### Wheel Target Mapping

- `LF = vy + vx + vz`
- `RF = vy - vx - vz`
- `LB = vy - vx + vz`
- `RB = vy + vx - vz`

### Test Entry

`user/src/main.c` has been switched to a Task 7 inverse-kinematics test mode.

Current test macros:

- `TASK7_TEST_VX_CMD`
- `TASK7_TEST_VY_CMD`
- `TASK7_TEST_VZ_CMD`

### Suggested Validation Order

- pure forward
- pure right translation
- pure clockwise rotation
- one combined motion case

### Validation Update

Hardware validation reported by the user:

- pure forward: passed
- pure clockwise rotation: passed
- one combined motion case: passed

Sample data reported for pure forward:

- `132, 140, 132, 126`
- `127, 120, 128, 132`
- `130, 129, 128, 128`
- `134, 128, 132, 134`

Sample data reported for pure clockwise rotation target output:

- `120, -120, 120, -120`

### Current Conclusion

- the inverse-kinematics mapping is working for forward motion
- the inverse-kinematics mapping is working for clockwise rotation
- the inverse-kinematics mapping is working for at least one combined-motion case
- one standalone pure-right-translation validation is still recommended before
  Task 7 is marked fully complete
