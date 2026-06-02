# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS 2 Humble workspace for a 12-DOF quadruped dog robot (ROBOCON 2026). Four legs × 3 joints (J1 hip, J2 thigh, J3 knee), RS02 motors over CAN bus. C++14, ament_cmake build system.

## Build Commands

```bash
# Full workspace build
cd /home/a/dog/dog_ws && colcon build --symlink-install

# Single package
colcon build --packages-select dog_control

# Build with tests
colcon build --cmake-args -DBUILD_TESTING=ON

# Run all tests
colcon test --ctest-args tests

# Run a single test binary
colcon test --ctest-args -R test_leg_kinematics
# Or directly: ./build/dog_control/test_leg_kinematics

# Source the workspace
source install/setup.bash
```

## Launch Modes

```bash
# Real hardware (xbox or keyboard input)
ros2 launch dog_bringup quad_locomotion.launch.py input_source:=xbox
ros2 launch dog_bringup quad_locomotion.launch.py input_source:=keyboard

# Gazebo simulation (keyboard only, opens Gazebo + controllers)
ros2 launch dog_bringup gazebo_sim.launch.py

# Real motor controller node (separate terminal)
ros2 run dog_control real_motor_controller

# Zero calibration tool
ros2 run dog_control zero_calibrator
```

## Architecture

### Package Layout

- **dog_control** — Core control library and nodes
- **dog_bringup** — Launch files (hardware + simulation)
- **dog_teleop** — Input nodes (Xbox gamepad, keyboard)
- **dog_msgs** — Custom ROS message definitions (`LocomotionCommand`, `MotorFeedback`)
- **dm_imu** — IMU driver (external dependency, not in this workspace)
- **dog_tool** — Python diagnostic tools (`js_range_extractor`, `lf_foot_monitor`)

### Control Pipeline (`quad` namespace)

The core control loop lives in `src/dog_control/src/quad/` with headers in `include/quad/`:

```
MotionCommand → CommandFilter → GaitScheduler → FootPlanner → LegKinematics(IK) → SafetyMonitor → JointCommand
```

- **CommandFilter** — Slew-rate limiting, exponential smoothing, timeout detection
- **GaitScheduler** — Phase tracking for 4 gait types: STAND, STEPPING, WALK, TROT
- **FootPlanner** — Bezier-curve swing trajectories, stance hold, per-gait params
- **LegKinematics** — 3-DOF IK/FK, workspace clamping, J1-to-body coordinate transforms
- **SafetyMonitor** — Joint limit enforcement, IK failure counting, force-stand on fault
- **LocomotionController** — Orchestrator that wires all modules together

### Key Data Types (`include/quad/types.h`)

- `Vec3` — 3D vector (X forward, Y left, Z up in body frame)
- `MotionCommand` — Filtered velocity command (vx, vy, yaw_rate, body_height)
- `GaitType` — Enum: STAND=0, STEPPING=1, WALK=2, TROT=3 (matches `dog_msgs` integers)
- `ContactSchedule` — Per-leg phase and stance/swing state
- `JointCommand` — Final 12-DOF position output

### Leg Model (`include/motor_ros2/leg_model.h`)

Defines the physical robot model: joint names, sign conventions, limits, and flat-index mapping. The `kJointSigns` array handles left/right mirror differences — signs apply to both commands and feedback.

Leg order: LF(0), RF(1), LB(2), RB(3). Joint order per leg: J1, J2, J3.

### Node Executables

| Node | Purpose |
|------|---------|
| `quad_locomotion_node` | Main control node — runs the full pipeline, accepts input from teleop |
| `sim_bridge_node` | Gazebo bridge — applies sign correction, forwards to `forward_position_controller` |
| `real_motor_controller` | Hardware node — CAN bus motor control for physical robot (RS02) |
| `zero_calibrator` | Single-leg zero-position calibration tool |

### Simulation vs Real Hardware

- **Simulation**: `gazebo_sim.launch.py` launches Gazebo + `ros2_control` + `sim_bridge_node` + `quad_locomotion_node`. The sim bridge applies `kJointSigns` correction before forwarding to Gazebo's `forward_position_controller`.
- **Real hardware**: `quad_locomotion.launch.py` launches `quad_locomotion_node` + teleop. Motor control runs separately via `real_motor_controller` on CAN bus (RS02 电机).

## Configuration

All locomotion parameters are in `config/quad_locomotion.yaml`. Per-gait params (frequency, duty_factor, step_height, max_step_length) are namespaced under `walk_*`, `trot_*`, `stepping_*`. The `input_source` parameter selects xbox vs keyboard.

## URDF

`urdf/dog.urdf` — Robot description. Meshes in `meshes/`. 用于 Gazebo 仿真和 RViz 可视化。**注意：URDF 中的尺寸数据不准确，机械参数以 `docs/机械结构说明.md` 和 `include/quad/leg_kinematics.h` 中的 `LegConfig` 为准。**

## Conventions

- All code in the `quad` namespace uses body-frame coordinates: X forward, Y left, Z up
- Joint commands are in "joint-cmd space" (not raw motor space) — sign conversion happens at the hardware/bridge boundary
- The `lm` namespace alias (`namespace lm = motor_ros2::leg_model`) is used throughout
- Gait phase offsets follow standard quadruped conventions: TROT diagonal (0, π, π, 0), WALK sequential (0, π/2, π, 3π/2)
