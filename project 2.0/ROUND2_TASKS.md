# Round 2 Tasks

This round is split into four independent tasks. They will be debugged one by one instead of being enabled at the same time.

## Task 1: Body-Frame Forward Kinematics

Goal:
- recover chassis body velocity from four wheel encoder speeds
- output `vx_body` and `vy_body`

Convention:
- `vx_body > 0`: move right
- `vy_body > 0`: move forward

Validation:
- pure forward: `vy_body > 0`, `vx_body ~= 0`
- pure right: `vx_body > 0`, `vy_body ~= 0`
- mixed motion: both values change with the command

Status:
- completed

## Task 2: Global Velocity Mapping

Goal:
- rotate body velocity into the field frame using IMU yaw
- output `vx_global` and `vy_global`

Validation:
- when `yaw = 0`, body/global velocity should match
- after rotating the car by about 90 degrees, forward body motion should mainly appear on the global side axis

Status:
- completed

## Task 3: Position Integration

Goal:
- integrate global velocity over time to obtain `X` and `Y`
- define a fixed start point, default `(0, 0)`

Validation:
- car static: `X/Y` should not drift quickly
- straight 0.4 m move: integrated distance should be close to measured distance

Status:
- completed

## Task 4: Calibration and Anti-Slip

Goal:
- correct model/field mismatch
- reduce odometry damage during wheel slip

Work items:
- tune `ODOM_SCALE_X`
- tune `ODOM_SCALE_Y`
- add anti-slip weighting only after basic `X/Y` is correct

Validation:
- repeated 0.4 m forward and right moves stay close to measured result
- quick start/stop should not cause large coordinate jumps

Status:
- completed

## Task 5: Point-to-Point Move

Goal:
- move from the current pose to one target point and stop
- keep chassis heading fixed during the move

Current scope:
- single target point only
- no path planning
- no obstacle avoidance
- default heading target stays at `0`

Validation:
- `(0, 0) -> (600, 0)` should stop near the target
- `(0, 0) -> (0, 600)` should stop near the target
- `(0, 0) -> (600, 600)` should reach the quadrant target without obvious spin

Status:
- completed

Residual risk:
- final 5 to 15 cm of convergence can still show visible small oscillation
- current version is accepted as a usable first point-to-point implementation
- if tighter stop quality is needed, open a separate optimization task for the
  final convergence phase
