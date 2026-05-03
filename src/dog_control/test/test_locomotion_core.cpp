#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "motor_ros2/leg_model.h"
#include "motor_ros2/locomotion_core_types.h"
#include "motor_ros2/locomotion_planner.h"
#include "motor_ros2/locomotion_runtime.h"

namespace motor_ros2 {
namespace locomotion {

class LocomotionPlannerTestPeer
{
public:
  static std::array<LocomotionPlanner::FootTarget, leg_model::kLegCount> compute_foot_targets(
    const LocomotionPlanner &planner,
    GaitMode mode,
    double phase,
    double vx,
    double vy,
    double yaw,
    double clearance)
  {
    return planner.compute_foot_targets(mode, phase, vx, vy, yaw, clearance);
  }
};

}  // namespace locomotion
}  // namespace motor_ros2

namespace lc = motor_ros2::locomotion;
namespace lm = motor_ros2::leg_model;

namespace {

rclcpp::Time ros_time_from_seconds(double sec)
{
  return rclcpp::Time(static_cast<int64_t>(std::llround(sec * 1e9)), RCL_ROS_TIME);
}

double unit_to_phase(double u)
{
  return 2.0 * lc::kPI * u;
}

double walk_mid_swing_unit(const lc::LocomotionParams &params)
{
  const double swing_ratio = 1.0 - params.walk_duty_factor;
  return params.walk_pre_shift_ratio + 0.5 * swing_ratio;
}

}  // namespace

TEST(LocomotionCoreTest, TrajectoryHelpersAreStable)
{
  constexpr double kDuty = 0.80;
  constexpr double kPreShift = 0.05;
  constexpr double kStepLen = 0.2;
  constexpr double kStepHeight = 0.05;
  constexpr double kEps = 1e-12;

  EXPECT_NEAR(lc::LocomotionPlanner::stride_offset(0.0, kStepLen), 0.1, kEps);
  EXPECT_NEAR(lc::LocomotionPlanner::stride_offset(1.2 * lc::kPI, kStepLen), -0.1, kEps);

  EXPECT_NEAR(
    lc::LocomotionPlanner::walk_stride_offset(unit_to_phase(kPreShift), kStepLen, kDuty, kPreShift),
    -0.1, kEps);
  EXPECT_NEAR(
    lc::LocomotionPlanner::walk_stride_offset(unit_to_phase(0.25), kStepLen, kDuty, kPreShift),
    0.1, kEps);
  EXPECT_NEAR(
    lc::LocomotionPlanner::walk_stride_offset(unit_to_phase(0.65), kStepLen, kDuty, kPreShift),
    0.0, 1e-12);

  EXPECT_NEAR(lc::LocomotionPlanner::swing_lift(0.0, kStepHeight), 0.0, kEps);
  EXPECT_NEAR(lc::LocomotionPlanner::swing_lift(1.6 * lc::kPI, kStepHeight), 0.05, 1e-12);

  EXPECT_NEAR(
    lc::LocomotionPlanner::walk_swing_lift(unit_to_phase(kPreShift), kStepHeight, kDuty, kPreShift),
    0.0, kEps);
  EXPECT_NEAR(
    lc::LocomotionPlanner::walk_swing_lift(unit_to_phase(0.15), kStepHeight, kDuty, kPreShift),
    0.05, 1e-12);
  EXPECT_NEAR(
    lc::LocomotionPlanner::walk_swing_lift(unit_to_phase(0.25), kStepHeight, kDuty, kPreShift),
    0.0, kEps);
}

TEST(LocomotionCoreTest, FootIkReachabilityAndSymmetry)
{
  lc::LocomotionParams params;
  lc::LocomotionPlanner planner(params);

  double t1 = 0.0;
  double t2 = 0.0;
  double t3 = 0.0;
  EXPECT_TRUE(planner.foot_ik(0.10, 0.08, -0.29, t1, t2, t3));
  EXPECT_FALSE(planner.foot_ik(1.0, 0.0, -0.29, t1, t2, t3));

  double t1_l = 0.0;
  double t2_l = 0.0;
  double t3_l = 0.0;
  double t1_r = 0.0;
  double t2_r = 0.0;
  double t3_r = 0.0;

  ASSERT_TRUE(planner.foot_ik(0.12, 0.09, -0.29, t1_l, t2_l, t3_l));
  ASSERT_TRUE(planner.foot_ik(0.12, -0.09, -0.29, t1_r, t2_r, t3_r));

  EXPECT_NEAR(t1_l, -t1_r, 1e-12);
  EXPECT_NEAR(t2_l, t2_r, 1e-12);
  EXPECT_NEAR(t3_l, t3_r, 1e-12);
}

TEST(LocomotionCoreTest, RuntimeWalkTransitionsSkipJointBlendAndResetPhase)
{
  lc::LocomotionParams params;
  lc::LocomotionRuntime runtime(params);

  const rclcpp::Time t0 = ros_time_from_seconds(0.0);
  runtime.reset(t0);
  runtime.set_command("xbox", lc::GaitMode::WALK, 0.05, 0.0, 0.0, 0.0, t0);

  const lc::RuntimeSnapshot walk_enter = runtime.step(ros_time_from_seconds(0.005));
  EXPECT_FALSE(walk_enter.transition_active);
  EXPECT_EQ(walk_enter.mode_current, lc::GaitMode::WALK);
  EXPECT_EQ(walk_enter.mode_target, lc::GaitMode::WALK);
  EXPECT_LT(walk_enter.gait_phase, 0.2);

  runtime.set_command("xbox", lc::GaitMode::TROT, 0.20, 0.0, 0.0, 0.0, ros_time_from_seconds(0.105));
  const lc::RuntimeSnapshot trot_enter = runtime.step(ros_time_from_seconds(0.110));
  EXPECT_FALSE(trot_enter.transition_active);
  EXPECT_EQ(trot_enter.mode_current, lc::GaitMode::TROT);
  EXPECT_EQ(trot_enter.mode_target, lc::GaitMode::TROT);

  runtime.reset(t0);
  runtime.set_command("xbox", lc::GaitMode::TROT, 0.2, 0.0, 0.0, 0.0, t0);
  (void)runtime.step(ros_time_from_seconds(0.005));
  runtime.set_command("xbox", lc::GaitMode::TROT, 0.2, 0.0, 0.0, 0.0, ros_time_from_seconds(0.390));
  (void)runtime.step(ros_time_from_seconds(0.400));
  const lc::RuntimeSnapshot p0 = runtime.step(ros_time_from_seconds(0.420));
  const lc::RuntimeSnapshot p1 = runtime.step(ros_time_from_seconds(0.440));

  double phase_delta = p1.gait_phase - p0.gait_phase;
  if (phase_delta < 0.0) {
    phase_delta += 2.0 * lc::kPI;
  }
  EXPECT_NEAR(phase_delta, 2.0 * lc::kPI * params.step_freq_trot * 0.02, 1e-9);
}

TEST(LocomotionCoreTest, WalkUsesExpectedLegOrderWithSingleSwingLeg)
{
  lc::LocomotionParams params;
  lc::LocomotionPlanner planner(params);

  const double clearance = params.height_default;
  const double mid_swing = walk_mid_swing_unit(params);
  const std::vector<lm::LegId> expected_order = {
    lm::LegId::LF, lm::LegId::RB, lm::LegId::RF, lm::LegId::LB
  };

  for (const lm::LegId expected_leg : expected_order) {
    const double phase = lc::LocomotionPlanner::walk_phase_offset_for_leg(expected_leg) +
      unit_to_phase(mid_swing);
    const auto targets = lc::LocomotionPlannerTestPeer::compute_foot_targets(
      planner, lc::GaitMode::WALK, phase, 0.05, 0.0, 0.0, clearance);

    std::size_t swing_count = 0;
    std::size_t swing_idx = lm::kLegCount;
    for (std::size_t leg_idx = 0; leg_idx < lm::kLegCount; ++leg_idx) {
      if (targets[leg_idx].in_swing) {
        ++swing_count;
        swing_idx = leg_idx;
      }
    }

    ASSERT_EQ(swing_count, 1u);
    EXPECT_EQ(static_cast<lm::LegId>(swing_idx), expected_leg);
    EXPECT_GT(targets[swing_idx].z_up, -clearance);
  }
}

TEST(LocomotionCoreTest, WalkSwingProfilesMatchDesignWithoutLateralShift)
{
  lc::LocomotionParams params;
  lc::LocomotionPlanner planner(params);

  const double clearance = params.height_default;
  const auto early_walk_targets = lc::LocomotionPlannerTestPeer::compute_foot_targets(
    planner, lc::GaitMode::WALK, unit_to_phase(0.02), 0.0, 0.0, 0.0, clearance);

  EXPECT_FALSE(early_walk_targets[static_cast<std::size_t>(lm::LegId::LF)].in_swing);
  EXPECT_NEAR(
    early_walk_targets[static_cast<std::size_t>(lm::LegId::LF)].y,
    params.stand_foot_y, 1e-12);
  EXPECT_NEAR(
    early_walk_targets[static_cast<std::size_t>(lm::LegId::RF)].y,
    -params.stand_foot_y, 1e-12);

  const double eps_phase = 1e-4;
  const double swing_start = params.walk_pre_shift_ratio;
  const double swing_end = params.walk_pre_shift_ratio + (1.0 - params.walk_duty_factor);

  const double x_start = lc::LocomotionPlanner::walk_stride_offset(
    unit_to_phase(swing_start + eps_phase), 0.2, params.walk_duty_factor, params.walk_pre_shift_ratio);
  const double x_start_prev = lc::LocomotionPlanner::walk_stride_offset(
    unit_to_phase(swing_start + 2.0 * eps_phase), 0.2, params.walk_duty_factor, params.walk_pre_shift_ratio);
  EXPECT_NEAR(x_start_prev - x_start, 0.0, 2e-4);

  const double x_end_prev = lc::LocomotionPlanner::walk_stride_offset(
    unit_to_phase(swing_end - 2.0 * eps_phase), 0.2, params.walk_duty_factor, params.walk_pre_shift_ratio);
  const double x_end = lc::LocomotionPlanner::walk_stride_offset(
    unit_to_phase(swing_end - eps_phase), 0.2, params.walk_duty_factor, params.walk_pre_shift_ratio);
  EXPECT_NEAR(x_end - x_end_prev, 0.0, 2e-4);

  EXPECT_NEAR(
    lc::LocomotionPlanner::walk_swing_lift(
      unit_to_phase(swing_start), 0.05, params.walk_duty_factor, params.walk_pre_shift_ratio),
    0.0, 1e-12);
  EXPECT_NEAR(
    lc::LocomotionPlanner::walk_swing_lift(
      unit_to_phase(swing_end), 0.05, params.walk_duty_factor, params.walk_pre_shift_ratio),
    0.0, 1e-12);
}

TEST(LocomotionCoreTest, WalkStanceFeetSlideBackwardAndRemainReachable)
{
  lc::LocomotionParams params;
  lc::LocomotionPlanner planner(params);

  const auto targets_a = lc::LocomotionPlannerTestPeer::compute_foot_targets(
    planner, lc::GaitMode::WALK, unit_to_phase(0.30), 0.05, 0.0, 0.0, params.height_default);
  const auto targets_b = lc::LocomotionPlannerTestPeer::compute_foot_targets(
    planner, lc::GaitMode::WALK, unit_to_phase(0.60), 0.05, 0.0, 0.0, params.height_default);
  EXPECT_GT(
    targets_a[static_cast<std::size_t>(lm::LegId::LF)].x,
    targets_b[static_cast<std::size_t>(lm::LegId::LF)].x);

  constexpr int kSamples = 200;
  for (int i = 0; i < kSamples; ++i) {
    const double phase = unit_to_phase(static_cast<double>(i) / static_cast<double>(kSamples));
    const auto targets = lc::LocomotionPlannerTestPeer::compute_foot_targets(
      planner, lc::GaitMode::WALK, phase, 0.05, 0.0, 0.02, params.height_default);

    std::size_t swing_count = 0;
    for (std::size_t leg_idx = 0; leg_idx < lm::kLegCount; ++leg_idx) {
      double t1 = 0.0;
      double t2 = 0.0;
      double t3 = 0.0;
      EXPECT_TRUE(
        planner.foot_ik(targets[leg_idx].x, targets[leg_idx].y, targets[leg_idx].z_up, t1, t2, t3))
        << "phase_sample=" << i << " leg=" << leg_idx;
      if (targets[leg_idx].in_swing) {
        ++swing_count;
      }
    }
    EXPECT_LE(swing_count, 1u) << "phase_sample=" << i;
  }
}

TEST(LocomotionCoreTest, WalkKeepsJoint1Locked)
{
  lc::LocomotionParams params;
  lc::LocomotionPlanner planner(params);

  constexpr int kSamples = 200;
  for (int i = 0; i < kSamples; ++i) {
    const double phase = unit_to_phase(static_cast<double>(i) / static_cast<double>(kSamples));
    const auto joints = planner.compute(
      lc::GaitMode::WALK, phase, 0.05, 0.0, 0.02, params.height_default);

    for (std::size_t leg_idx = 0; leg_idx < lm::kLegCount; ++leg_idx) {
      EXPECT_NEAR(
        joints.at(static_cast<lm::LegId>(leg_idx)).j1,
        0.0, 1e-12) << "phase_sample=" << i << " leg=" << leg_idx;
    }
  }
}
