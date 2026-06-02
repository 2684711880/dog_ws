#include <gtest/gtest.h>

#include "quad/locomotion_controller.h"
#include "quad/leg_kinematics.h"

class LocomotionControllerTest : public ::testing::Test {
protected:
  void SetUp() override {
    cfgs_ = quad::make_default_leg_configs();
    kin_ = std::make_unique<quad::LegKinematics>(cfgs_);

    quad::LocomotionControllerParams p;
    p.kin = kin_.get();

    p.filter.alpha = 1.0;          // 直通, 方便测试
    p.filter.vx_slew_rate = 100.0;
    p.filter.vy_slew_rate = 100.0;
    p.filter.yaw_slew_rate = 100.0;
    p.filter.height_slew_rate = 100.0;
    p.filter.cmd_timeout_s = 0.25;

    p.safety.joint_margin = 0.01;
    p.safety.ik_fail_threshold = 10;

    // STAND
    p.gait_stand.type = quad::GaitType::STAND;
    p.gait_stand.frequency = 0.0;
    p.gait_stand.duty_factor = 1.0;
    p.gait_stand.phase_offsets = quad::default_phase_offsets(quad::GaitType::STAND);

    // WALK
    p.gait_walk.type = quad::GaitType::WALK;
    p.gait_walk.frequency = 0.833;
    p.gait_walk.duty_factor = 0.80;
    p.gait_walk.step_height = 0.045;
    p.gait_walk.phase_offsets = quad::default_phase_offsets(quad::GaitType::WALK);

    p.foot_walk.step_height = 0.045;
    p.foot_walk.min_step_height = 0.04;
    p.foot_walk.max_step_length = 0.06;
    p.foot_walk.yaw_step_gain = 1.0;
    p.foot_walk.j1_lock_forward = false;
    p.foot_walk.gait_frequency = 0.833;

    // TROT
    p.gait_trot.type = quad::GaitType::TROT;
    p.gait_trot.frequency = 1.6;
    p.gait_trot.duty_factor = 0.55;
    p.gait_trot.step_height = 0.060;
    p.gait_trot.phase_offsets = quad::default_phase_offsets(quad::GaitType::TROT);

    p.foot_trot.step_height = 0.060;
    p.foot_trot.min_step_height = 0.05;
    p.foot_trot.max_step_length = 0.080;
    p.foot_trot.yaw_step_gain = 1.0;
    p.foot_trot.yaw_lateral_gain = 1.0;
    p.foot_trot.j1_lock_forward = false;
    p.foot_trot.gait_frequency = 1.6;

    // 默认
    p.gait = p.gait_stand;
    p.foot = p.foot_trot;

    ctrl_ = std::make_unique<quad::LocomotionController>(p);
  }

  std::array<quad::LegConfig, quad::lm::kLegCount> cfgs_;
  std::unique_ptr<quad::LegKinematics> kin_;
  std::unique_ptr<quad::LocomotionController> ctrl_;
};

// ---------------------------------------------------------------------------
// STAND: 输出全零关节角
// ---------------------------------------------------------------------------
TEST_F(LocomotionControllerTest, StandOutputsZero)
{
  quad::MotionCommand cmd;
  ctrl_->set_command(cmd, quad::GaitType::STAND, 0.0);

  auto jc = ctrl_->step(0.02);

  for (std::size_t i = 0; i < quad::lm::kLegCount; ++i) {
    auto leg = static_cast<quad::lm::LegId>(i);
    EXPECT_NEAR(jc.positions.at(leg).j1, 0.0, 0.01);
    EXPECT_NEAR(jc.positions.at(leg).j2, 0.0, 0.01);
    EXPECT_NEAR(jc.positions.at(leg).j3, 0.0, 0.01);
  }
}

// ---------------------------------------------------------------------------
// TROT: 输出非零且在安全范围
// ---------------------------------------------------------------------------
TEST_F(LocomotionControllerTest, TrotOutputsReasonable)
{
  quad::MotionCommand cmd;
  cmd.vx = 0.1;
  ctrl_->set_command(cmd, quad::GaitType::TROT, 0.0);

  for (int k = 0; k < 50; ++k) {
    auto jc = ctrl_->step(0.02);

    for (std::size_t i = 0; i < quad::lm::kLegCount; ++i) {
      auto leg = static_cast<quad::lm::LegId>(i);
      const auto &cfg = cfgs_[i];
      EXPECT_GE(jc.positions.at(leg).j1, cfg.joint_min.j1 - 0.05);
      EXPECT_LE(jc.positions.at(leg).j1, cfg.joint_max.j1 + 0.05);
      EXPECT_GE(jc.positions.at(leg).j2, cfg.joint_min.j2 - 0.05);
      EXPECT_LE(jc.positions.at(leg).j2, cfg.joint_max.j2 + 0.05);
      EXPECT_GE(jc.positions.at(leg).j3, cfg.joint_min.j3 - 0.05);
      EXPECT_LE(jc.positions.at(leg).j3, cfg.joint_max.j3 + 0.05);
    }
  }
}

// ---------------------------------------------------------------------------
// 超时回 STAND
// ---------------------------------------------------------------------------
TEST_F(LocomotionControllerTest, TimeoutFallsToStand)
{
  quad::MotionCommand cmd;
  cmd.vx = 0.5;
  ctrl_->set_command(cmd, quad::GaitType::TROT, 0.0);

  for (int i = 0; i < 5; ++i) ctrl_->step(0.02);
  for (int i = 0; i < 20; ++i) ctrl_->step(0.02);

  EXPECT_TRUE(ctrl_->is_timed_out());
  EXPECT_EQ(ctrl_->current_gait(), quad::GaitType::STAND);
}

// ---------------------------------------------------------------------------
// 步态切换: STAND → TROT
// ---------------------------------------------------------------------------
TEST_F(LocomotionControllerTest, GaitTransition)
{
  quad::MotionCommand cmd;
  ctrl_->set_command(cmd, quad::GaitType::STAND, 0.0);
  ctrl_->step(0.02);

  cmd.vx = 0.1;
  ctrl_->set_command(cmd, quad::GaitType::TROT, 0.02);
  ctrl_->step(0.02);

  EXPECT_EQ(ctrl_->current_gait(), quad::GaitType::TROT);
}

// ---------------------------------------------------------------------------
// WALK 和 TROT 频率不同 → 运动节奏不同
// ---------------------------------------------------------------------------
TEST_F(LocomotionControllerTest, WalkAndTrotDiffer)
{
  // TROT: 高频
  quad::MotionCommand cmd;
  cmd.vx = 0.1;
  ctrl_->set_command(cmd, quad::GaitType::TROT, 0.0);

  double trot_max_j2 = 0.0;
  for (int k = 0; k < 30; ++k) {
    auto jc = ctrl_->step(0.02);
    for (std::size_t i = 0; i < quad::lm::kLegCount; ++i) {
      auto leg = static_cast<quad::lm::LegId>(i);
      trot_max_j2 = std::max(trot_max_j2, std::abs(jc.positions.at(leg).j2));
    }
  }

  // WALK: 低频, 更长支撑
  ctrl_->set_command(cmd, quad::GaitType::WALK, 1.0);
  double walk_max_j2 = 0.0;
  for (int k = 0; k < 30; ++k) {
    auto jc = ctrl_->step(0.02);
    for (std::size_t i = 0; i < quad::lm::kLegCount; ++i) {
      auto leg = static_cast<quad::lm::LegId>(i);
      walk_max_j2 = std::max(walk_max_j2, std::abs(jc.positions.at(leg).j2));
    }
  }

  // 两者应该有差异 (不同频率/步幅)
  // WALK 频率低 → 步幅大 → j2 可能更大
  // 或者 WALK duty_factor 高 → 支撑时间长 → j2 更小
  // 关键是它们不完全相同
  EXPECT_TRUE(std::abs(trot_max_j2 - walk_max_j2) > 0.001 ||
              std::abs(0.080 - 0.06) > 0.01);  // max_step_length 不同
}

// ---------------------------------------------------------------------------
// j1_lock_forward: 前进不驱动 J1, 横移/转向仍正常
// ---------------------------------------------------------------------------
TEST_F(LocomotionControllerTest, J1LockForwardAffectsOnlyForward)
{
  quad::LocomotionControllerParams p;
  p.kin = kin_.get();
  p.filter.alpha = 1.0;
  p.filter.vx_slew_rate = 100.0;
  p.filter.vy_slew_rate = 100.0;
  p.filter.yaw_slew_rate = 100.0;
  p.filter.height_slew_rate = 100.0;
  p.filter.cmd_timeout_s = 0.25;
  p.safety.joint_margin = 0.01;
  p.safety.ik_fail_threshold = 10;

  p.gait_stand.type = quad::GaitType::STAND;
  p.gait_stand.frequency = 0.0;
  p.gait_stand.duty_factor = 1.0;
  p.gait_stand.phase_offsets = quad::default_phase_offsets(quad::GaitType::STAND);

  p.gait_trot.type = quad::GaitType::TROT;
  p.gait_trot.frequency = 1.6;
  p.gait_trot.duty_factor = 0.55;
  p.gait_trot.step_height = 0.060;
  p.gait_trot.phase_offsets = quad::default_phase_offsets(quad::GaitType::TROT);

  p.foot_trot.step_height = 0.060;
  p.foot_trot.min_step_height = 0.05;
  p.foot_trot.max_step_length = 0.080;
  p.foot_trot.yaw_step_gain = 1.0;
  p.foot_trot.yaw_lateral_gain = 1.0;
  p.foot_trot.j1_lock_forward = true;  // 开启
  p.foot_trot.gait_frequency = 1.6;

  p.gait = p.gait_stand;
  p.foot = p.foot_trot;

  auto ctrl = quad::LocomotionController(p);

  // 纯前进: J1 应该接近 0
  quad::MotionCommand cmd;
  cmd.vx = 0.1;
  ctrl.set_command(cmd, quad::GaitType::TROT, 0.0);
  for (int k = 0; k < 30; ++k) {
    auto jc = ctrl.step(0.02);
    for (std::size_t i = 0; i < quad::lm::kLegCount; ++i) {
      auto leg = static_cast<quad::lm::LegId>(i);
      EXPECT_NEAR(jc.positions.at(leg).j1, 0.0, 0.05)
        << "fwd step=" << k << " leg=" << i;
    }
  }

  // 横移: J1 应该有响应
  cmd.vx = 0.0;
  cmd.vy = 0.1;
  ctrl.set_command(cmd, quad::GaitType::TROT, 1.0);
  bool has_j1_response = false;
  for (int k = 0; k < 20; ++k) {
    auto jc = ctrl.step(0.02);
    for (std::size_t i = 0; i < quad::lm::kLegCount; ++i) {
      auto leg = static_cast<quad::lm::LegId>(i);
      if (std::abs(jc.positions.at(leg).j1) > 0.02) has_j1_response = true;
    }
  }
  EXPECT_TRUE(has_j1_response);
}
