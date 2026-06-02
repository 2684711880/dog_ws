#include <gtest/gtest.h>

#include "quad/leg_kinematics.h"

#include <cmath>

class LegKinematicsTest : public ::testing::Test {
protected:
  void SetUp() override {
    cfgs_ = quad::make_default_leg_configs();
    kin_ = std::make_unique<quad::LegKinematics>(cfgs_);
  }

  std::array<quad::LegConfig, quad::lm::kLegCount> cfgs_;
  std::unique_ptr<quad::LegKinematics> kin_;
};

// ---------------------------------------------------------------------------
// 自检: FK(0,0,0) ≈ 串联链零位足端
// ---------------------------------------------------------------------------
TEST_F(LegKinematicsTest, SelfCheckPasses)
{
  EXPECT_TRUE(kin_->self_check(5e-3));
}

// ---------------------------------------------------------------------------
// FK(0,0,0) 精确匹配 nominal
// ---------------------------------------------------------------------------
TEST_F(LegKinematicsTest, FKZeroMatchesNominal)
{
  for (std::size_t i = 0; i < quad::lm::kLegCount; ++i) {
    auto leg = static_cast<quad::lm::LegId>(i);
    quad::lm::LegJoints<double> zero{};
    auto fk = kin_->forward_kinematics(leg, zero);
    const auto &nom = cfgs_[i].j1_to_j3_at_zero + cfgs_[i].stand_foot_from_j3;
    EXPECT_NEAR(fk.x, nom.x, 1e-6) << "leg=" << i;
    EXPECT_NEAR(fk.y, nom.y, 1e-6) << "leg=" << i;
    EXPECT_NEAR(fk.z, nom.z, 1e-6) << "leg=" << i;
  }
}

// ---------------------------------------------------------------------------
// 新零位几何: 前后 J3 间距 420mm, 左右足端间距 290mm
// ---------------------------------------------------------------------------
TEST_F(LegKinematicsTest, ZeroGeometryMatchesSerialChain)
{
  const auto lf_j3 = cfgs_[0].j1_origin_in_body + cfgs_[0].j1_to_j3_at_zero;
  const auto rf_j3 = cfgs_[1].j1_origin_in_body + cfgs_[1].j1_to_j3_at_zero;
  const auto lb_j3 = cfgs_[2].j1_origin_in_body + cfgs_[2].j1_to_j3_at_zero;

  EXPECT_NEAR(lf_j3.x - lb_j3.x, 0.420, 1e-9);
  EXPECT_NEAR(lf_j3.y - rf_j3.y, 0.290, 1e-9);
}

// ---------------------------------------------------------------------------
// IK → FK 往返: FK(IK(p)) ≈ p
// ---------------------------------------------------------------------------
TEST_F(LegKinematicsTest, FKIKAroundTrip)
{
  // 测试多个可达点 (基于新的 nominal: 前腿 ±0.06, 后腿 ∓0.06)
  std::vector<quad::Vec3> test_points = {
    {0.06, 0.095, -0.290},    // LF stand
    {0.06, 0.095, -0.250},    // 抬高
    {0.10, 0.095, -0.280},    // 前伸
    {0.02, 0.095, -0.270},    // 后缩
    {-0.06, 0.095, -0.290},   // LB stand
    {0.06, -0.095, -0.290},   // RF stand
  };

  for (const auto &p : test_points) {
    for (std::size_t i = 0; i < quad::lm::kLegCount; ++i) {
      auto leg = static_cast<quad::lm::LegId>(i);

      // 只测试与该腿 y 方向匹配的点
      const auto &cfg = cfgs_[i];
      if (std::signbit(p.y) != std::signbit(cfg.j1_to_j3_at_zero.y)) {
        continue;
      }

      quad::lm::LegJoints<double> q;
      ASSERT_TRUE(kin_->inverse_kinematics(leg, p, q))
        << "IK failed for leg=" << i << " p=(" << p.x << "," << p.y << "," << p.z << ")";

      auto fk = kin_->forward_kinematics(leg, q);
      EXPECT_NEAR(fk.x, p.x, 1e-4) << "leg=" << i;
      EXPECT_NEAR(fk.y, p.y, 1e-4) << "leg=" << i;
      EXPECT_NEAR(fk.z, p.z, 1e-4) << "leg=" << i;
    }
  }
}

// ---------------------------------------------------------------------------
// 非零 J1 的 FK → IK → FK 往返
// ---------------------------------------------------------------------------
TEST_F(LegKinematicsTest, FKIKAroundTripWithNonZeroJ1)
{
  const std::array<quad::lm::LegJoints<double>, 3> poses = {{
    {+0.25, -0.10, +0.20},
    {-0.20, +0.08, -0.10},
    {+0.15, +0.12, +0.05},
  }};

  for (std::size_t i = 0; i < quad::lm::kLegCount; ++i) {
    const auto leg = static_cast<quad::lm::LegId>(i);
    for (const auto &pose : poses) {
      const auto p = kin_->forward_kinematics(leg, pose);
      quad::lm::LegJoints<double> recovered;
      ASSERT_TRUE(kin_->inverse_kinematics(leg, p, recovered))
        << "IK failed for leg=" << i;

      const auto round_trip = kin_->forward_kinematics(leg, recovered);
      EXPECT_NEAR(round_trip.x, p.x, 1e-6) << "leg=" << i;
      EXPECT_NEAR(round_trip.y, p.y, 1e-6) << "leg=" << i;
      EXPECT_NEAR(round_trip.z, p.z, 1e-6) << "leg=" << i;
    }
  }
}

// ---------------------------------------------------------------------------
// J1 旋转会带动 J1->J3 和末端链一起旋转
// ---------------------------------------------------------------------------
TEST_F(LegKinematicsTest, J1RotatesWholeDownstreamChain)
{
  auto leg = quad::lm::LegId::LF;
  quad::lm::LegJoints<double> zero{};
  quad::lm::LegJoints<double> rotated{0.30, 0.0, 0.0};

  const auto p0 = kin_->forward_kinematics(leg, zero);
  const auto p1 = kin_->forward_kinematics(leg, rotated);

  EXPECT_NEAR(p0.x, p1.x, 1e-9);
  EXPECT_GT(p1.y, p0.y);
  EXPECT_GT(p1.z, p0.z);
}

// ---------------------------------------------------------------------------
// IK 不可达点返回 false
// ---------------------------------------------------------------------------
TEST_F(LegKinematicsTest, IKUnreachableReturnsFalse)
{
  auto leg = quad::lm::LegId::LF;
  quad::lm::LegJoints<double> q;

  // 太远
  EXPECT_FALSE(kin_->inverse_kinematics(leg, {0.5, 0.0, -0.5}, q));
  // thigh=shank 时 min_reach=0, (0,0,-0.01) 可达。改用 z=-0.5 (D>max_reach)
  EXPECT_FALSE(kin_->inverse_kinematics(leg, {0.0, 0.0, -0.5}, q));
}

// ---------------------------------------------------------------------------
// is_reachable 一致性
// ---------------------------------------------------------------------------
TEST_F(LegKinematicsTest, IsReachableConsistent)
{
  auto leg = quad::lm::LegId::LF;

  EXPECT_TRUE(kin_->is_reachable(leg, {0.06, 0.095, -0.290}));
  EXPECT_FALSE(kin_->is_reachable(leg, {0.5, 0.0, -0.5}));
  EXPECT_FALSE(kin_->is_reachable(leg, {0.0, 0.0, -0.5}));
}

// ---------------------------------------------------------------------------
// clamp_to_workspace 输出可达
// ---------------------------------------------------------------------------
TEST_F(LegKinematicsTest, ClampProducesReachable)
{
  auto leg = quad::lm::LegId::LF;

  // 超远点
  auto clamped = kin_->clamp_to_workspace(leg, {0.5, 0.0, -0.5});
  EXPECT_TRUE(kin_->is_reachable(leg, clamped));

  // 超远点 (z方向)
  clamped = kin_->clamp_to_workspace(leg, {0.0, 0.0, -0.5});
  EXPECT_TRUE(kin_->is_reachable(leg, clamped));

  // 已在域内的点不变
  auto in = quad::Vec3(0.06, 0.095, -0.290);
  clamped = kin_->clamp_to_workspace(leg, in);
  EXPECT_NEAR(clamped.x, in.x, 1e-6);
  EXPECT_NEAR(clamped.y, in.y, 1e-6);
  EXPECT_NEAR(clamped.z, in.z, 1e-6);
}

// ---------------------------------------------------------------------------
// 非零 J1 下工作空间仍和 IK 一致
// ---------------------------------------------------------------------------
TEST_F(LegKinematicsTest, WorkspaceConsistentWithNonZeroJ1)
{
  auto leg = quad::lm::LegId::LF;
  quad::lm::LegJoints<double> pose{0.25, -0.10, 0.20};
  const auto p = kin_->forward_kinematics(leg, pose);

  EXPECT_TRUE(kin_->is_reachable(leg, p));

  const auto clamped = kin_->clamp_to_workspace(leg, p);
  EXPECT_NEAR(clamped.x, p.x, 1e-6);
  EXPECT_NEAR(clamped.y, p.y, 1e-6);
  EXPECT_NEAR(clamped.z, p.z, 1e-6);
}

// ---------------------------------------------------------------------------
// j1_to_body / body_to_j1 往返
// ---------------------------------------------------------------------------
TEST_F(LegKinematicsTest, J1BodyRoundTrip)
{
  for (std::size_t i = 0; i < quad::lm::kLegCount; ++i) {
    auto leg = static_cast<quad::lm::LegId>(i);
    quad::Vec3 p_j1(0.05, 0.03, -0.25);

    auto p_body = kin_->j1_to_body(leg, p_j1);
    auto p_back = kin_->body_to_j1(leg, p_body);

    EXPECT_NEAR(p_back.x, p_j1.x, 1e-9);
    EXPECT_NEAR(p_back.y, p_j1.y, 1e-9);
    EXPECT_NEAR(p_back.z, p_j1.z, 1e-9);
  }
}

// ---------------------------------------------------------------------------
// FK body 系一致性: FK_body = j1_to_body(FK_j1)
// ---------------------------------------------------------------------------
TEST_F(LegKinematicsTest, FKBodyConsistent)
{
  for (std::size_t i = 0; i < quad::lm::kLegCount; ++i) {
    auto leg = static_cast<quad::lm::LegId>(i);
    quad::lm::LegJoints<double> q{0.1, -0.2, 0.3};

    auto fk_j1 = kin_->forward_kinematics(leg, q);
    auto fk_body = kin_->forward_kinematics_body(leg, q);
    auto expected = kin_->j1_to_body(leg, fk_j1);

    EXPECT_NEAR(fk_body.x, expected.x, 1e-9);
    EXPECT_NEAR(fk_body.y, expected.y, 1e-9);
    EXPECT_NEAR(fk_body.z, expected.z, 1e-9);
  }
}

// ---------------------------------------------------------------------------
// IK body 系一致性: IK_body(p_body) == IK_j1(body_to_j1(p_body))
// ---------------------------------------------------------------------------
TEST_F(LegKinematicsTest, IKBodyConsistent)
{
  auto leg = quad::lm::LegId::LF;
  quad::Vec3 p_body(0.21, 0.145, -0.290);  // j1_origin + stand_foot

  quad::lm::LegJoints<double> q_body, q_j1;
  ASSERT_TRUE(kin_->inverse_kinematics_body(leg, p_body, q_body));
  ASSERT_TRUE(kin_->inverse_kinematics(leg, kin_->body_to_j1(leg, p_body), q_j1));

  EXPECT_NEAR(q_body.j1, q_j1.j1, 1e-9);
  EXPECT_NEAR(q_body.j2, q_j1.j2, 1e-9);
  EXPECT_NEAR(q_body.j3, q_j1.j3, 1e-9);
}

// ---------------------------------------------------------------------------
// 四条腿 FK(0) 的 y 值左右对称
// ---------------------------------------------------------------------------
TEST_F(LegKinematicsTest, LeftRightSymmetry)
{
  quad::lm::LegJoints<double> zero{};

  auto fk_lf = kin_->forward_kinematics(quad::lm::LegId::LF, zero);
  auto fk_rf = kin_->forward_kinematics(quad::lm::LegId::RF, zero);
  auto fk_lb = kin_->forward_kinematics(quad::lm::LegId::LB, zero);
  auto fk_rb = kin_->forward_kinematics(quad::lm::LegId::RB, zero);

  // LF/RF x 相同, y 相反
  EXPECT_NEAR(fk_lf.x, fk_rf.x, 1e-6);
  EXPECT_NEAR(fk_lf.y, -fk_rf.y, 1e-6);
  EXPECT_NEAR(fk_lf.z, fk_rf.z, 1e-6);

  // LB/RB x 相同, y 相反
  EXPECT_NEAR(fk_lb.x, fk_rb.x, 1e-6);
  EXPECT_NEAR(fk_lb.y, -fk_rb.y, 1e-6);
  EXPECT_NEAR(fk_lb.z, fk_rb.z, 1e-6);
}

// ---------------------------------------------------------------------------
// 关节命令限幅在 config 范围内
// ---------------------------------------------------------------------------
TEST_F(LegKinematicsTest, IKOutputWithinLimits)
{
  // 对每条腿, 用其站立足端位置附近的可达点测试
  for (std::size_t i = 0; i < quad::lm::kLegCount; ++i) {
    auto leg = static_cast<quad::lm::LegId>(i);
    const auto &cfg = cfgs_[i];
    const auto nom = cfg.j1_to_j3_at_zero + cfg.stand_foot_from_j3;

    // 在 nominal 附近取几个可达偏移
    std::vector<quad::Vec3> offsets = {
      {0, 0, 0},
      {0.02, 0, 0},
      {-0.02, 0, 0},
      {0, 0, 0.02},
    };

    for (const auto &off : offsets) {
      quad::Vec3 p = nom + off;
      quad::lm::LegJoints<double> q;
      if (!kin_->inverse_kinematics(leg, p, q)) continue;

      EXPECT_GE(q.j1, cfg.joint_min.j1 - 0.01) << "leg=" << i;
      EXPECT_LE(q.j1, cfg.joint_max.j1 + 0.01) << "leg=" << i;
      EXPECT_GE(q.j2, cfg.joint_min.j2 - 0.01) << "leg=" << i;
      EXPECT_LE(q.j2, cfg.joint_max.j2 + 0.01) << "leg=" << i;
      EXPECT_GE(q.j3, cfg.joint_min.j3 - 0.01) << "leg=" << i;
      EXPECT_LE(q.j3, cfg.joint_max.j3 + 0.01) << "leg=" << i;
    }
  }
}
