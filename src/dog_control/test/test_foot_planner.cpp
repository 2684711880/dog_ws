#include <gtest/gtest.h>

#include "quad/foot_planner.h"
#include "quad/gait_scheduler.h"
#include "quad/leg_kinematics.h"

#include <cmath>

constexpr double kPI = 3.14159265358979323846;

class FootPlannerTest : public ::testing::Test {
protected:
  void SetUp() override {
    cfgs_ = quad::make_default_leg_configs();
    kin_ = std::make_unique<quad::LegKinematics>(cfgs_);

    quad::FootPlannerParams fp;
    fp.step_height = 0.05;
    fp.min_step_height = 0.05;
    fp.max_step_length = 0.12;
    fp.bezier_lift_ratio = 0.4;
    fp.yaw_step_gain = 1.0;
    fp.yaw_step_gain_in_place = 1.2;
    fp.gait_frequency = 2.0;
    planner_ = std::make_unique<quad::FootPlanner>(fp);
  }

  std::array<quad::LegConfig, quad::lm::kLegCount> cfgs_;
  std::unique_ptr<quad::LegKinematics> kin_;
  std::unique_ptr<quad::FootPlanner> planner_;
};

// ---------------------------------------------------------------------------
// Bezier 高度: t=0 和 t=1 时为 0, t=0.5 时最大
// ---------------------------------------------------------------------------
TEST_F(FootPlannerTest, BezierHeightEndpoints)
{
  EXPECT_NEAR(quad::FootPlanner::bezier_swing_height(0.0, 0.4), 0.0, 1e-6);
  EXPECT_NEAR(quad::FootPlanner::bezier_swing_height(1.0, 0.4), 0.0, 1e-6);
}

TEST_F(FootPlannerTest, BezierHeightPeakAtMiddle)
{
  double h_mid = quad::FootPlanner::bezier_swing_height(0.5, 0.4);
  EXPECT_GT(h_mid, 0.5);  // 应该接近 1.0
  EXPECT_LE(h_mid, 1.0);
}

TEST_F(FootPlannerTest, BezierHeightNonNegative)
{
  for (int i = 0; i <= 100; ++i) {
    double t = i / 100.0;
    double h = quad::FootPlanner::bezier_swing_height(t, 0.4);
    EXPECT_GE(h, -1e-6) << "t=" << t;
    EXPECT_LE(h, 1.0 + 1e-6) << "t=" << t;
  }
}

// ---------------------------------------------------------------------------
// stance_offset: t=0 → +step/2, t=1 → -step/2
// ---------------------------------------------------------------------------
TEST_F(FootPlannerTest, StancePushbackEndpoints)
{
  const double step = 0.1;
  EXPECT_NEAR(quad::FootPlanner::stance_offset(0.0, step), step / 2, 1e-6);
  EXPECT_NEAR(quad::FootPlanner::stance_offset(1.0, step), -step / 2, 1e-6);
}

TEST_F(FootPlannerTest, StancePushbackMonotonic)
{
  const double step = 0.1;
  double prev = quad::FootPlanner::stance_offset(0.0, step);
  for (int i = 1; i <= 10; ++i) {
    double t = i / 10.0;
    double cur = quad::FootPlanner::stance_offset(t, step);
    EXPECT_LT(cur, prev) << "t=" << t;
    prev = cur;
  }
}

// ---------------------------------------------------------------------------
// STAND 模式: 所有腿 IK 可达
// ---------------------------------------------------------------------------
TEST_F(FootPlannerTest, StandAllReachable)
{
  quad::GaitScheduler sched;
  quad::GaitParams gp;
  gp.type = quad::GaitType::STAND;
  gp.frequency = 0.0;
  gp.duty_factor = 1.0;
  sched.set_gait(gp);

  auto cs = sched.step(0.01);
  quad::MotionCommand cmd;

  auto joints = planner_->compute(cs, cmd, 0.0, *kin_);

  // 所有关节应该接近 0 (站立姿态)
  for (std::size_t i = 0; i < quad::lm::kLegCount; ++i) {
    auto leg = static_cast<quad::lm::LegId>(i);
    EXPECT_NEAR(joints.at(leg).j1, 0.0, 0.01) << "leg=" << i;
    EXPECT_NEAR(joints.at(leg).j2, 0.0, 0.01) << "leg=" << i;
    EXPECT_NEAR(joints.at(leg).j3, 0.0, 0.01) << "leg=" << i;
  }
}

// ---------------------------------------------------------------------------
// TROT 模式: IK 不崩溃, 输出在合理范围
// ---------------------------------------------------------------------------
TEST_F(FootPlannerTest, TrotOutputReasonable)
{
  quad::GaitScheduler sched;
  quad::GaitParams gp;
  gp.type = quad::GaitType::TROT;
  gp.frequency = 2.0;
  gp.duty_factor = 0.5;
  gp.phase_offsets = {0.0, kPI, kPI, 0.0};  // 对角步态弧度
  sched.set_gait(gp);

  quad::MotionCommand cmd;
  cmd.vx = 0.1;

  // 多步测试
  for (int k = 0; k < 50; ++k) {
    auto cs = sched.step(0.01);
    auto joints = planner_->compute(cs, cmd, 0.0, *kin_);

    for (std::size_t i = 0; i < quad::lm::kLegCount; ++i) {
      // 关节角应该在合理范围内
      auto leg = static_cast<quad::lm::LegId>(i);
      EXPECT_GT(joints.at(leg).j1, -2.0) << "k=" << k << " leg=" << i;
      EXPECT_LT(joints.at(leg).j1, 2.0) << "k=" << k << " leg=" << i;
      EXPECT_GT(joints.at(leg).j2, -2.0) << "k=" << k << " leg=" << i;
      EXPECT_LT(joints.at(leg).j2, 2.0) << "k=" << k << " leg=" << i;
    }
  }
}

// ---------------------------------------------------------------------------
// 零速度: 足端应该在 nominal 附近
// ---------------------------------------------------------------------------
TEST_F(FootPlannerTest, ZeroVelocityNearNominal)
{
  quad::GaitScheduler sched;
  quad::GaitParams gp;
  gp.type = quad::GaitType::TROT;
  gp.frequency = 2.0;
  gp.duty_factor = 0.5;
  gp.phase_offsets = {0.0, kPI, kPI, 0.0};  // 对角步态弧度
  sched.set_gait(gp);

  quad::MotionCommand cmd;  // vx=vy=yaw_rate=0

  // 支撑相中途
  auto cs = sched.step(0.125);  // 进入支撑相
  auto joints = planner_->compute(cs, cmd, 0.0, *kin_);

  // 关节角应该非常接近 0
  for (std::size_t i = 0; i < quad::lm::kLegCount; ++i) {
    if (cs.in_stance[i]) {
      auto leg = static_cast<quad::lm::LegId>(i);
      EXPECT_NEAR(joints.at(leg).j1, 0.0, 0.02) << "leg=" << i;
      EXPECT_NEAR(joints.at(leg).j2, 0.0, 0.02) << "leg=" << i;
      EXPECT_NEAR(joints.at(leg).j3, 0.0, 0.02) << "leg=" << i;
    }
  }
}

// ---------------------------------------------------------------------------
// vx 响应: vx=0 和 vx=0.1 应产生不同关节角
// ---------------------------------------------------------------------------
TEST_F(FootPlannerTest, VxAffectsJointAngles)
{
  quad::GaitScheduler sched;
  quad::GaitParams gp;
  gp.type = quad::GaitType::TROT;
  gp.frequency = 2.0;
  gp.duty_factor = 0.5;
  gp.phase_offsets = {0.0, kPI, kPI, 0.0};
  sched.set_gait(gp);

  // vx=0
  quad::GaitScheduler sched_zero;
  sched_zero.set_gait(gp);

  quad::MotionCommand cmd_zero;   // vx=0
  quad::MotionCommand cmd_fwd;
  cmd_fwd.vx = 0.1;

  bool found_diff = false;
  for (int k = 0; k < 30; ++k) {
    auto cs0 = sched_zero.step(0.02);
    auto cs1 = sched.step(0.02);

    auto j0 = planner_->compute(cs0, cmd_zero, 0.0, *kin_);
    auto j1 = planner_->compute(cs1, cmd_fwd, 0.0, *kin_);

    for (std::size_t i = 0; i < quad::lm::kLegCount; ++i) {
      auto leg = static_cast<quad::lm::LegId>(i);
      if (std::abs(j0.at(leg).j2 - j1.at(leg).j2) > 0.01) {
        found_diff = true;
      }
    }
  }
  EXPECT_TRUE(found_diff) << "vx=0 and vx=0.1 should produce different joint angles";
}

// ---------------------------------------------------------------------------
// vx 正负: +vx 和 -vx 都应产生不同于 vx=0 的 J2 振幅
// ---------------------------------------------------------------------------
TEST_F(FootPlannerTest, VxPositiveAndNegativeBothDifferFromZero)
{
  quad::GaitParams gp;
  gp.type = quad::GaitType::TROT;
  gp.frequency = 2.0;
  gp.duty_factor = 0.5;
  gp.phase_offsets = {0.0, kPI, kPI, 0.0};

  // 收集一个周期内 J2 的最大绝对值
  auto collect_max_j2 = [&](double vx) -> double {
    quad::GaitScheduler sched;
    sched.set_gait(gp);
    quad::MotionCommand cmd;
    cmd.vx = vx;
    double max_j2 = 0.0;
    for (int k = 0; k < 100; ++k) {
      auto cs = sched.step(0.002);
      auto j = planner_->compute(cs, cmd, 0.0, *kin_);
      for (std::size_t i = 0; i < quad::lm::kLegCount; ++i) {
        auto leg = static_cast<quad::lm::LegId>(i);
        max_j2 = std::max(max_j2, std::abs(j.at(leg).j2));
      }
    }
    return max_j2;
  };

  double j2_zero = collect_max_j2(0.0);
  double j2_fwd  = collect_max_j2(0.1);
  double j2_bwd  = collect_max_j2(-0.1);

  // vx ≠ 0 时 J2 振幅应大于 vx=0 (差异小但可测量, ~0.004 rad ≈ 0.25°)
  EXPECT_GT(j2_fwd - j2_zero, 0.001)
    << "vx=+0.1 J2 amplitude should exceed vx=0"
    << " fwd=" << j2_fwd << " zero=" << j2_zero;
  EXPECT_GT(j2_bwd - j2_zero, 0.001)
    << "vx=-0.1 J2 amplitude should exceed vx=0"
    << " bwd=" << j2_bwd << " zero=" << j2_zero;

  // 正负 vx 振幅应接近 (对称)
  EXPECT_NEAR(j2_fwd, j2_bwd, 0.01)
    << "vx=+0.1 and vx=-0.1 J2 amplitude should be symmetric"
    << " fwd=" << j2_fwd << " bwd=" << j2_bwd;
}

// ---------------------------------------------------------------------------
// TROT yaw: 原地转向按足端相对机身中心的切向方向同时产生 x/y 步幅
// ---------------------------------------------------------------------------
TEST_F(FootPlannerTest, YawUsesTangentialFootMotion)
{
  quad::FootPlannerParams fp;
  fp.step_height = 0.060;
  fp.min_step_height = 0.05;
  fp.max_step_length = 0.080;
  fp.yaw_step_gain = 1.0;
  fp.yaw_step_gain_in_place = 1.2;
  fp.yaw_lateral_gain = 1.0;
  fp.gait_frequency = 1.6;
  auto planner = quad::FootPlanner(fp);

  quad::ContactSchedule cs;
  for (std::size_t i = 0; i < quad::lm::kLegCount; ++i) {
    cs.in_stance[i] = true;
    cs.stance_progress[i] = 0.0;
    cs.swing_progress[i] = -1.0;
  }

  quad::MotionCommand cmd;
  cmd.yaw_rate = 0.25;

  std::array<quad::FootPlannerLegDebug, quad::lm::kLegCount> debug{};
  planner.compute(cs, cmd, 0.0, *kin_, &debug);

  const auto lf_nom = cfgs_[0].j1_to_j3_at_zero + cfgs_[0].stand_foot_from_j3;
  const auto rf_nom = cfgs_[1].j1_to_j3_at_zero + cfgs_[1].stand_foot_from_j3;
  const double lf_dx = debug[0].foot_pos_j1.x - lf_nom.x;
  const double rf_dx = debug[1].foot_pos_j1.x - rf_nom.x;
  EXPECT_LT(lf_dx, -1e-4);
  EXPECT_GT(rf_dx, 1e-4);

  const auto lb_nom = cfgs_[2].j1_to_j3_at_zero + cfgs_[2].stand_foot_from_j3;
  const double lf_dy = debug[0].foot_pos_j1.y - lf_nom.y;
  const double lb_dy = debug[2].foot_pos_j1.y - lb_nom.y;
  EXPECT_GT(lf_dy, 1e-4);
  EXPECT_LT(lb_dy, -1e-4);

  for (std::size_t i = 0; i < quad::lm::kLegCount; ++i) {
    EXPECT_TRUE(debug[i].ik_success) << "leg=" << i;
  }
}

// ---------------------------------------------------------------------------
// j1_lock_forward: vx 前进时 J1 应为 0, 横移时 J1 应有响应
// ---------------------------------------------------------------------------
TEST_F(FootPlannerTest, J1LockForwardOnlyLocksForward)
{
  quad::FootPlannerParams fp;
  fp.step_height = 0.05;
  fp.min_step_height = 0.05;
  fp.max_step_length = 0.12;
  fp.gait_frequency = 2.0;
  fp.j1_lock_forward = true;
  auto planner_locked = quad::FootPlanner(fp);

  quad::GaitScheduler sched;
  quad::GaitParams gp;
  gp.type = quad::GaitType::TROT;
  gp.frequency = 2.0;
  gp.duty_factor = 0.5;
  gp.phase_offsets = {0.0, kPI, kPI, 0.0};
  sched.set_gait(gp);

  // 纯前进: J1 应接近 0
  quad::MotionCommand cmd_fwd;
  cmd_fwd.vx = 0.1;
  double max_j1_fwd = 0.0;
  for (int k = 0; k < 30; ++k) {
    auto cs = sched.step(0.02);
    auto j = planner_locked.compute(cs, cmd_fwd, 0.0, *kin_);
    for (std::size_t i = 0; i < quad::lm::kLegCount; ++i) {
      auto leg = static_cast<quad::lm::LegId>(i);
      max_j1_fwd = std::max(max_j1_fwd, std::abs(j.at(leg).j1));
    }
  }
  EXPECT_LT(max_j1_fwd, 0.05) << "J1 should be near 0 for pure forward with lock";

  // 横移: J1 应该有响应
  sched.set_gait(gp);
  quad::MotionCommand cmd_lat;
  cmd_lat.vy = 0.1;
  bool has_j1 = false;
  for (int k = 0; k < 20; ++k) {
    auto cs = sched.step(0.02);
    auto j = planner_locked.compute(cs, cmd_lat, 0.0, *kin_);
    for (std::size_t i = 0; i < quad::lm::kLegCount; ++i) {
      auto leg = static_cast<quad::lm::LegId>(i);
      if (std::abs(j.at(leg).j1) > 0.02) has_j1 = true;
    }
  }
  EXPECT_TRUE(has_j1) << "J1 should respond to lateral motion even with lock";

  // 后退: J1 也应接近 0
  sched.set_gait(gp);
  quad::MotionCommand cmd_bwd;
  cmd_bwd.vx = -0.1;
  double max_j1_bwd = 0.0;
  for (int k = 0; k < 30; ++k) {
    auto cs = sched.step(0.02);
    auto j = planner_locked.compute(cs, cmd_bwd, 0.0, *kin_);
    for (std::size_t i = 0; i < quad::lm::kLegCount; ++i) {
      auto leg = static_cast<quad::lm::LegId>(i);
      max_j1_bwd = std::max(max_j1_bwd, std::abs(j.at(leg).j1));
    }
  }
  EXPECT_LT(max_j1_bwd, 0.05) << "J1 should be near 0 for pure backward with lock";
}
