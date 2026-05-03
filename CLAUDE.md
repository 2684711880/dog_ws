# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Workspace layout

ROS 2 (ament) workspace for a 12-DoF quadruped (RS02 over 4 CAN buses). Standard colcon layout under `src/`:

- `dog_msgs` — `LocomotionCommand`, `MotorFeedback` interfaces
- `dog_teleop` — `xbox_input_node`: `/joy` → `/locomotion_cmd`
- `dog_control` — core C++ package; locomotion controller, hardware driver, calibration / debug executables, URDF, RViz config
- `dog_bringup` — pure launch package (`locomotion_control.launch.py`, `display.launch.py`)
- `dog_tool` — Python debug/visualization (`lf_foot_monitor`)
- `dm_imu` — Python IMU driver

Project documentation is in Chinese under `docs/` — `节点说明.md` is authoritative for node parameters and topic contracts; `机械结构说明.md` is authoritative for mechanical dimensions and zero-pose; `重构v2.md` is the active refactor proposal (not yet implemented).

## Build / run / test

Build & source from the workspace root:

```bash
colcon build --symlink-install
source install/setup.bash
```

Build a single package: `colcon build --packages-select dog_control`. Force a clean rebuild of a package: `colcon build --packages-select dog_control --cmake-clean-cache`.

Run tests (gtest under `dog_control/test/`):

```bash
colcon test --packages-select dog_control
colcon test-result --verbose
```

Run a single gtest filter directly after build: `./build/dog_control/test_locomotion_core --gtest_filter=Name.*`.

Bring up CAN before launching real hardware: `sudo scripts/can.sh` (configures `can0..can3` at 1 Mbps, sample-point 0.75). `scripts/clean_logs.sh` wipes `log/`.

Standard launches:

```bash
# Xbox control + locomotion controller
ros2 run joy joy_node
ros2 launch dog_bringup locomotion_control.launch.py input_source:=xbox

# Real hardware controller (after CAN is up)
ros2 run dog_control real_motor_controller

# Visualization
ros2 launch dog_bringup display.launch.py use_gui:=true
```

Standalone hardware tools (CAN-only, do **not** use ROS topics): `dog_control/zero_calibrator` (single-leg mechanical zero) and `dog_control/dji_3508_spin_test` (single motor spin test).

## Architecture

Control pipeline (one direction, one node per stage):

```
/joy → xbox_input_node → /locomotion_cmd → locomotion_core_node → /joint_states + /gait_params → real_motor_controller → 12× RS02 (CAN)
                                                                                                  ↓
                                                                                          /motor_feedback
```

Both `locomotion_core_node` and `real_motor_controller` run **5 ms (200 Hz)** control loops.

`locomotion_core_node` filters `LocomotionCommand` by its `input_source` parameter (default `xbox`); messages whose `source` field doesn't match are silently dropped. Cmd timeout (`cmd_timeout_s`, default 0.25 s) automatically returns to `STAND`.

`locomotion_core_node` is split into a thin ROS shell + three libs (see `dog_control/CMakeLists.txt`):
- `locomotion_params` — declare/load all `rcl` parameters
- `locomotion_runtime` — phase clock, command filter (lowpass + slew), gait transition blend
- `locomotion_planner` — IK, foot trajectory, joint output

Hardware library (`motor_hardware_lib`) backs `real_motor_controller`, `zero_calibrator`, and `dji_3508_spin_test`.

### Single source of truth: `motor_ros2/leg_model.h`

`include/motor_ros2/leg_model.h` defines `LegId {LF,RF,LB,RB}`, `JointId {J1,J2,J3}`, `LegJoints<T>`, `QuadJoints<T>`, `flatten`/`unflatten`, joint names, signs, and limits. Do **not** introduce parallel enums, separate joint-name lists, or per-leg-id arrays elsewhere — use this header. The CAN-bus → leg mapping is `can0=LF, can1=RF, can2=RB, can3=LB`.

### Topic contracts (do not break)

- `/locomotion_cmd` (`dog_msgs/LocomotionCommand`): `source` must match `input_source`. `height` is **clearance offset**, not absolute z; convention is `axis7=+1` ⇒ body raised ⇒ `height` value decreases. `gait_mode` ∈ {`STAND=0, STEPPING=1, WALK=2, TROT=3}`.
- `/gait_params` (`std_msgs/String`): plain gait name string (`STAND`/`STEPPING`/`WALK`/`TROT`). `real_motor_controller` switches PID gains on this — do not rename or restructure.
- `/joint_states` (`sensor_msgs/JointState`): 12 entries, ordered per `lm::kLegOrder` × `J1..J3`. `Joint_3` carries an internal **2.0× gear ratio** transform — keep `joint3_cmd_min/max` in sync with the hardware mapping when changing IK output conventions.

### Hardware safety defaults

`real_motor_controller` has gait-adaptive PID (different `kp/kd` per gait — see `docs/节点说明.md` §2.4). Default firmware is configured to **only enable LB leg's 3 motors**; the other legs are commented out in source until hand-validated. Recommended on-robot bring-up order is single-leg → diagonal pair → all four.

### Mechanical facts that affect tuning

- ~20 mm vertical backlash in the foot — keep swing height ≥ 40 mm so legs don't drag.
- Knee (`Joint_3`) has 2:1 gear ratio and a 19° mechanical offset baked into the zero pose.
- "Zero pose" = standing with all 12 motor positions = 0; control commands are offsets from this.

## Conventions

- Coordinates: right-handed, X forward, Y left, Z up. IK runs in hip-local frame.
- Documentation is in Chinese; match existing tone if editing docs in `docs/`.
- After modifying `dog_control` source files, both `locomotion_core_lib` and the executables that link it (`locomotion_core_node`, plus tests) need to be rebuilt — `colcon build --packages-select dog_control` handles this.
