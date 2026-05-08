# Round 3 Execution Plan

## Summary

This stage connects the existing BFS output to the real chassis execution path.
The current implementation scope is:

- unify the field/grid coordinate system
- add a motion-execution state machine
- convert `MotionPlan` segments into physical target points
- drive the chassis through the existing odometry + point-move + heading-hold stack
- keep execution status available through formal runtime state, not only BLE text

## Fixed Conventions

- Field origin: bottom-left corner
- Field size: `3200 mm x 2400 mm`
- Grid size: `16 x 12`
- One grid cell: `200 mm x 200 mm`
- Cell target point: always the cell center
- Heading policy: keep `Yaw = 0` during execution
- Execution granularity: run one compressed `MotionSegment` at a time

Grid-center mapping:

- `x_mm = grid_x * 200 + 100`
- `y_mm = grid_y * 200 + 100`

## Implemented Modules

- `state_machine.h/.c`
  - `bfs_runtime_state_t` remains the formal BFS result source
  - `motion_exec_runtime_state_t` is the new formal execution-state snapshot
- `robot_control.h/.c`
  - `motion_exec_init()`
  - `motion_exec_request_start()`
  - `motion_exec_tick()`
- `odometry.h/.c`
  - point-move profiles for normal walk vs push segments
- `main.c`
  - current runtime entry now runs:
    - OpenART receive/parse
    - filtered-map BFS trigger
    - motion execution scheduler
    - compact BLE telemetry

## State Machine

Execution phases:

- `CAR_STATE_WAIT_START`
- `CAR_STATE_WAIT_MAP`
- `CAR_STATE_WAIT_BFS`
- `CAR_STATE_LOAD_PLAN`
- `CAR_STATE_EXEC_SEGMENT`
- `CAR_STATE_WAIT_SEGMENT_FINISH`
- `CAR_STATE_PLAN_DONE`
- `CAR_STATE_ERROR`

Current auto-start behavior:

- `EXEC_AUTO_START_FOR_TEST = 1`
- this is only for bench bring-up before a real start trigger exists

## Segment Execution Rules

- `SEG_WALK`: move to the target cell center with normal point-move profile
- `SEG_PUSH`: move to the target cell center with lower-speed push profile
- `SEG_WAIT`: hold stop and count down wait time
- unsupported segment types enter `CAR_STATE_ERROR`

## Current Telemetry

`send_data(a, b, c, d)` now means:

- `a = execution phase`
- `b = current segment index`
- `c = odometry x_mm`
- `d = odometry y_mm`

Detailed BFS and segment text logs are still printed over BLE.
