#include <gtest/gtest.h>

#include "quad/safety_monitor.h"
#include "quad/leg_kinematics.h"

class SafetyMonitorTest : public ::testing::Test {
protected:
  void SetUp() override {
    cfgs_ = quad::make_default_leg_configs();
    kin_ = std::make_unique<quad::LegKinematics>(cfgs_);

    quad::SafetyParams sp;
    sp.joint_margin = 0.01;
    sp.ik_fail_threshold = 5;
    monitor_ = std::make_unique<quad::SafetyMonitor>(sp);
  }

  std::array<quad::LegConfig, quad::lm::kLegCount> cfgs_;
  std::unique_ptr<quad::LegKinematics> kin_;
  std::unique_ptr<quad::SafetyMonitor> monitor_;
};

// ---------------------------------------------------------------------------
// 正常关节角不被限幅
// ---------------------------------------------------------------------------
TEST_F(SafetyMonitorTest, NormalJointsPassThrough)
{
  quad::lm::QuadJoints<double> joints{};
  // 全零 (站立姿态) 应该安全通过
  EXPECT_TRUE(monitor_->check_and_clamp(joints, *kin_));

  // 所有关节应保持 0
  for (std::size_t i = 0; i < quad::lm::kLegCount; ++i) {
    auto leg = static_cast<quad::lm::LegId>(i);
    EXPECT_NEAR(joints.at(leg).j1, 0.0, 1e-6);
    EXPECT_NEAR(joints.at(leg).j2, 0.0, 1e-6);
    EXPECT_NEAR(joints.at(leg).j3, 0.0, 1e-6);
  }
}

// ---------------------------------------------------------------------------
// 超限关节被限幅
// ---------------------------------------------------------------------------
TEST_F(SafetyMonitorTest, ExcessiveJointsClamped)
{
  quad::lm::QuadJoints<double> joints{};
  joints.at(quad::lm::LegId::LF).j1 = 5.0;  // 远超限
  joints.at(quad::lm::LegId::LF).j2 = -5.0;

  EXPECT_FALSE(monitor_->check_and_clamp(joints, *kin_));

  const auto &cfg = cfgs_[0];
  EXPECT_LE(joints.at(quad::lm::LegId::LF).j1, cfg.joint_max.j1 - 0.01);
  EXPECT_GE(joints.at(quad::lm::LegId::LF).j2, cfg.joint_min.j2 + 0.01);
}

// ---------------------------------------------------------------------------
// IK 失败计数 → 强制 STAND
// ---------------------------------------------------------------------------
TEST_F(SafetyMonitorTest, IkFailTriggersForceStand)
{
  EXPECT_FALSE(monitor_->should_force_stand());

  // 连续报告 IK 失败
  for (int i = 0; i < 4; ++i) {
    monitor_->report_ik_result(0, false);
    EXPECT_FALSE(monitor_->should_force_stand());
  }
  monitor_->report_ik_result(0, false);  // 第 5 次
  EXPECT_TRUE(monitor_->should_force_stand());
}

// ---------------------------------------------------------------------------
// IK 成功重置计数
// ---------------------------------------------------------------------------
TEST_F(SafetyMonitorTest, IkSuccessResetsCount)
{
  for (int i = 0; i < 4; ++i) {
    monitor_->report_ik_result(0, false);
  }
  EXPECT_EQ(monitor_->ik_fail_count(0), 4);

  monitor_->report_ik_result(0, true);
  EXPECT_EQ(monitor_->ik_fail_count(0), 0);
  EXPECT_FALSE(monitor_->should_force_stand());
}

// ---------------------------------------------------------------------------
// reset 清零
// ---------------------------------------------------------------------------
TEST_F(SafetyMonitorTest, ResetClearsAll)
{
  for (std::size_t i = 0; i < quad::lm::kLegCount; ++i) {
    for (int j = 0; j < 10; ++j) {
      monitor_->report_ik_result(i, false);
    }
  }
  EXPECT_TRUE(monitor_->should_force_stand());

  monitor_->reset();
  EXPECT_FALSE(monitor_->should_force_stand());
  for (std::size_t i = 0; i < quad::lm::kLegCount; ++i) {
    EXPECT_EQ(monitor_->ik_fail_count(i), 0);
  }
}

// ---------------------------------------------------------------------------
// 边界: 越界 leg_idx 不崩溃
// ---------------------------------------------------------------------------
TEST_F(SafetyMonitorTest, OutOfBoundsLegIdx)
{
  monitor_->report_ik_result(99, false);  // 不应崩溃
  EXPECT_EQ(monitor_->ik_fail_count(99), 0);
}
