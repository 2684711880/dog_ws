#include <gtest/gtest.h>

#include "quad/gait_scheduler.h"

#include <cmath>

constexpr double kPI = 3.14159265358979323846;
constexpr double kHalfPI = kPI / 2.0;

// 辅助: 构造 TROT 步态参数 (弧度)
static quad::GaitParams make_trot_params()
{
  quad::GaitParams p;
  p.type = quad::GaitType::TROT;
  p.frequency = 2.0;           // 2 Hz
  p.duty_factor = 0.5;         // 50% 支撑
  p.phase_offsets = {0.0, kPI, kPI, 0.0};  // 对角腿同相 (LF/RB, RF/LB)
  return p;
}

// 辅助: 构造 WALK 步态参数 (弧度, 四腿顺序)
static quad::GaitParams make_walk_params()
{
  quad::GaitParams p;
  p.type = quad::GaitType::WALK;
  p.frequency = 0.833;
  p.duty_factor = 0.80;
  p.phase_offsets = {0.0, kHalfPI, kPI, kHalfPI + kPI};  // LF→RF→RB→LB 顺序
  return p;
}

// 辅助: 构造 STAND 步态参数
static quad::GaitParams make_stand_params()
{
  quad::GaitParams p;
  p.type = quad::GaitType::STAND;
  p.frequency = 0.0;
  p.duty_factor = 1.0;
  p.phase_offsets = {0.0, 0.0, 0.0, 0.0};
  return p;
}

// ---------------------------------------------------------------------------
// STAND: 所有腿都在支撑相
// ---------------------------------------------------------------------------
TEST(GaitSchedulerTest, StandAllStance)
{
  quad::GaitScheduler sched;
  sched.set_gait(make_stand_params());

  auto cs = sched.step(0.01);
  for (std::size_t i = 0; i < quad::lm::kLegCount; ++i) {
    EXPECT_TRUE(cs.in_stance[i]) << "leg=" << i;
    EXPECT_NEAR(cs.stance_progress[i], 0.5, 1e-6) << "leg=" << i;
    EXPECT_LT(cs.swing_progress[i], 0) << "leg=" << i;
  }
}

// ---------------------------------------------------------------------------
// STAND: 相位不推进
// ---------------------------------------------------------------------------
TEST(GaitSchedulerTest, StandPhaseDoesNotAdvance)
{
  quad::GaitScheduler sched;
  sched.set_gait(make_stand_params());

  auto cs1 = sched.step(0.01);
  auto cs2 = sched.step(0.01);
  auto cs3 = sched.step(0.1);

  EXPECT_NEAR(cs1.global_phase, 0.0, 1e-9);
  EXPECT_NEAR(cs2.global_phase, 0.0, 1e-9);
  EXPECT_NEAR(cs3.global_phase, 0.0, 1e-9);
}

// ---------------------------------------------------------------------------
// TROT: 相位正确推进
// ---------------------------------------------------------------------------
TEST(GaitSchedulerTest, TrotPhaseAdvances)
{
  quad::GaitScheduler sched;
  sched.set_gait(make_trot_params());

  auto cs = sched.step(0.01);
  // 2 Hz * 0.01s * 2π = 0.04π ≈ 0.1257
  EXPECT_NEAR(cs.global_phase, 2.0 * 0.01 * 2.0 * M_PI, 1e-6);
}

// ---------------------------------------------------------------------------
// TROT: 对角腿同相 (LF/RB, RF/LB)
// ---------------------------------------------------------------------------
TEST(GaitSchedulerTest, TrotDiagonalInPhase)
{
  quad::GaitScheduler sched;
  sched.set_gait(make_trot_params());

  // step 一次, 取 phase_in_cycle 比较
  auto cs = sched.step(0.001);

  // LF(0) 和 RB(3) phase_offset 都是 0 → 同相
  EXPECT_NEAR(cs.phase_in_cycle[0], cs.phase_in_cycle[3], 1e-6);
  // RF(1) 和 LB(2) phase_offset 都是 π → 同相
  EXPECT_NEAR(cs.phase_in_cycle[1], cs.phase_in_cycle[2], 1e-6);
}

// ---------------------------------------------------------------------------
// WALK: 四腿顺序, 每条间隔 1/4 周期
// ---------------------------------------------------------------------------
TEST(GaitSchedulerTest, WalkSequentialPhase)
{
  quad::GaitScheduler sched;
  sched.set_gait(make_walk_params());

  auto cs = sched.step(0.001);

  // LF=0, RF=π/2, RB=π, LB=3π/2 → phase_in_cycle 各差 0.25
  EXPECT_NEAR(cs.phase_in_cycle[0], 0.0, 0.01);          // LF ≈ 0
  EXPECT_NEAR(cs.phase_in_cycle[1], 0.25, 0.01);         // RF ≈ 0.25
  EXPECT_NEAR(cs.phase_in_cycle[2], 0.50, 0.01);         // RB ≈ 0.50
  EXPECT_NEAR(cs.phase_in_cycle[3], 0.75, 0.01);         // LB ≈ 0.75

  // 任意两条相邻腿不同时在摆动相 (duty=0.80, 只有 0.20 是摆动)
  // 在 t≈0 时: LF 刚开始支撑, RF/RB/LB 都在支撑 → 只有 LF 可能刚进入支撑
  // 关键验证: 四条腿的 phase_in_cycle 互不相同
  for (int i = 0; i < 4; ++i) {
    for (int j = i + 1; j < 4; ++j) {
      EXPECT_GT(std::abs(cs.phase_in_cycle[i] - cs.phase_in_cycle[j]), 0.1)
        << "legs " << i << " and " << j << " should have different phases";
    }
  }
}

// ---------------------------------------------------------------------------
// TROT: duty_factor=0.5 时, 半周期支撑半周期摆动
// ---------------------------------------------------------------------------
TEST(GaitSchedulerTest, TrotDutyFactorHalf)
{
  quad::GaitScheduler sched;
  sched.set_gait(make_trot_params());

  // 推进到刚好 u ≈ 0.25 (< 0.5 duty) → 应该在支撑相
  // dt 使得 global_phase ≈ 0.25 * 2π = π/2
  // dt = (π/2) / (2π * 2) = 0.125
  auto cs = sched.step(0.125);
  // LF phase_offset=0, u = global_phase / 2π ≈ 0.25
  EXPECT_TRUE(cs.in_stance[0]);
  EXPECT_GE(cs.stance_progress[0], 0.0);
  EXPECT_LE(cs.stance_progress[0], 1.0);
}

// ---------------------------------------------------------------------------
// set_gait 重置相位
// ---------------------------------------------------------------------------
TEST(GaitSchedulerTest, SetGaitResetsPhase)
{
  quad::GaitScheduler sched;
  sched.set_gait(make_trot_params());
  sched.step(0.1);  // 推进一些

  sched.set_gait(make_trot_params());
  auto cs = sched.step(0.001);
  // 相位应该从接近 0 开始
  EXPECT_LT(cs.global_phase, 0.1);
}

// ---------------------------------------------------------------------------
// reset_phase
// ---------------------------------------------------------------------------
TEST(GaitSchedulerTest, ResetPhase)
{
  quad::GaitScheduler sched;
  sched.set_gait(make_trot_params());
  sched.step(0.1);

  sched.reset_phase();
  auto cs = sched.step(0.001);
  EXPECT_LT(cs.global_phase, 0.1);
}

// ---------------------------------------------------------------------------
// 相位绕回 (wrap around)
// ---------------------------------------------------------------------------
TEST(GaitSchedulerTest, PhaseWrapsAround)
{
  quad::GaitScheduler sched;
  sched.set_gait(make_trot_params());

  // 推进超过 2π
  // dt 使得 global_phase > 2π: dt = 1.0 → phase = 2π * 2 * 1.0 = 4π
  auto cs = sched.step(1.0);
  EXPECT_LT(cs.global_phase, 2.0 * M_PI);
  EXPECT_GE(cs.global_phase, 0.0);
}

// ---------------------------------------------------------------------------
// stance_progress / swing_progress 范围
// ---------------------------------------------------------------------------
TEST(GaitSchedulerTest, ProgressInRange)
{
  quad::GaitScheduler sched;
  sched.set_gait(make_trot_params());

  // 多步测试
  for (int k = 0; k < 100; ++k) {
    auto cs = sched.step(0.002);
    for (std::size_t i = 0; i < quad::lm::kLegCount; ++i) {
      if (cs.in_stance[i]) {
        EXPECT_GE(cs.stance_progress[i], 0.0) << "k=" << k << " leg=" << i;
        EXPECT_LE(cs.stance_progress[i], 1.0) << "k=" << k << " leg=" << i;
        EXPECT_LT(cs.swing_progress[i], 0) << "k=" << k << " leg=" << i;
      } else {
        EXPECT_GE(cs.swing_progress[i], 0.0) << "k=" << k << " leg=" << i;
        EXPECT_LE(cs.swing_progress[i], 1.0) << "k=" << k << " leg=" << i;
        EXPECT_LT(cs.stance_progress[i], 0) << "k=" << k << " leg=" << i;
      }
    }
  }
}
