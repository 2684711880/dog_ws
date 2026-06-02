#include <gtest/gtest.h>

#include "quad/command_filter.h"

#include <cmath>

// ---------------------------------------------------------------------------
// lowpass 静态函数
// ---------------------------------------------------------------------------
TEST(CommandFilterTest, LowpassConverges)
{
  double val = 0.0;
  for (int i = 0; i < 100; ++i) {
    val = quad::CommandFilter::lowpass(val, 1.0, 0.3);
  }
  EXPECT_NEAR(val, 1.0, 0.01);
}

TEST(CommandFilterTest, LowpassAlpha0NoChange)
{
  double val = 5.0;
  val = quad::CommandFilter::lowpass(val, 10.0, 0.0);
  EXPECT_NEAR(val, 5.0, 1e-9);
}

TEST(CommandFilterTest, LowpassAlpha1Immediate)
{
  double val = 5.0;
  val = quad::CommandFilter::lowpass(val, 10.0, 1.0);
  EXPECT_NEAR(val, 10.0, 1e-9);
}

// ---------------------------------------------------------------------------
// slew 静态函数
// ---------------------------------------------------------------------------
TEST(CommandFilterTest, SlewsRateLimited)
{
  double val = 0.0;
  val = quad::CommandFilter::slew(val, 10.0, 1.0, 0.02);
  // max_delta = 1.0 * 0.02 = 0.02
  EXPECT_NEAR(val, 0.02, 1e-6);
}

TEST(CommandFilterTest, SlewNoLimitWhenClose)
{
  double val = 0.0;
  val = quad::CommandFilter::slew(val, 0.001, 1.0, 0.02);
  // max_delta = 0.02, target - current = 0.001 < 0.02
  EXPECT_NEAR(val, 0.001, 1e-6);
}

TEST(CommandFilterTest, SlewNegativeDirection)
{
  double val = 1.0;
  val = quad::CommandFilter::slew(val, 0.0, 1.0, 0.02);
  EXPECT_NEAR(val, 0.98, 1e-6);
}

// ---------------------------------------------------------------------------
// 完整滤波流程
// ---------------------------------------------------------------------------
TEST(CommandFilterTest, SetCommandAndStep)
{
  quad::CommandFilterParams params;
  params.alpha = 1.0;     // 低通直通
  params.vx_slew_rate = 100.0;  // 不限速
  params.vy_slew_rate = 100.0;
  params.yaw_slew_rate = 100.0;
  params.height_slew_rate = 100.0;
  params.cmd_timeout_s = 0.25;

  quad::CommandFilter filter(params);

  quad::MotionCommand cmd;
  cmd.vx = 0.5;
  cmd.vy = 0.3;
  cmd.yaw_rate = 0.1;
  cmd.body_height = 0.28;

  filter.set_command(cmd, 0.0);
  auto out = filter.step(0.02);

  // alpha=1.0 + slew_rate=100 → 一步到位
  EXPECT_NEAR(out.vx, 0.5, 0.01);
  EXPECT_NEAR(out.vy, 0.3, 0.01);
  EXPECT_NEAR(out.yaw_rate, 0.1, 0.01);
  EXPECT_NEAR(out.body_height, 0.28, 0.01);
}

// ---------------------------------------------------------------------------
// 超时: 回到 STAND
// ---------------------------------------------------------------------------
TEST(CommandFilterTest, TimeoutReturnsZero)
{
  quad::CommandFilterParams params;
  params.alpha = 1.0;
  params.vx_slew_rate = 100.0;
  params.vy_slew_rate = 100.0;
  params.yaw_slew_rate = 100.0;
  params.height_slew_rate = 100.0;
  params.cmd_timeout_s = 0.1;

  quad::CommandFilter filter(params);

  quad::MotionCommand cmd;
  cmd.vx = 1.0;
  filter.set_command(cmd, 0.0);

  // 先走一步
  filter.step(0.02);

  // 超时后
  auto out = filter.step(0.2);  // 累计 0.22s > 0.1s timeout

  EXPECT_TRUE(filter.is_timed_out());
  // vx 应该被低通拉向 0
  EXPECT_LT(out.vx, 0.5);
}

// ---------------------------------------------------------------------------
// 持续收指令不会超时
// ---------------------------------------------------------------------------
TEST(CommandFilterTest, ContinuousNoTimeout)
{
  quad::CommandFilterParams params;
  params.alpha = 1.0;
  params.vx_slew_rate = 100.0;
  params.vy_slew_rate = 100.0;
  params.yaw_slew_rate = 100.0;
  params.height_slew_rate = 100.0;
  params.cmd_timeout_s = 0.25;

  quad::CommandFilter filter(params);

  quad::MotionCommand cmd;
  cmd.vx = 0.3;

  for (int i = 0; i < 50; ++i) {
    filter.set_command(cmd, i * 0.02);
    filter.step(0.02);
    EXPECT_FALSE(filter.is_timed_out()) << "step=" << i;
  }
}

// ---------------------------------------------------------------------------
// height 限幅
// ---------------------------------------------------------------------------
TEST(CommandFilterTest, HeightClamped)
{
  quad::CommandFilterParams params;
  params.alpha = 1.0;
  params.vx_slew_rate = 100.0;
  params.vy_slew_rate = 100.0;
  params.yaw_slew_rate = 100.0;
  params.height_slew_rate = 100.0;
  params.height_min = 0.22;
  params.height_max = 0.32;

  quad::CommandFilter filter(params);

  // 超高
  quad::MotionCommand cmd;
  cmd.body_height = 0.50;
  filter.set_command(cmd, 0.0);
  auto out = filter.step(0.02);
  EXPECT_LE(out.body_height, 0.32);

  // 超低
  cmd.body_height = 0.10;
  filter.set_command(cmd, 0.1);
  out = filter.step(0.02);
  // 需要多步收敛到 0.22
  for (int i = 0; i < 50; ++i) {
    out = filter.step(0.02);
  }
  EXPECT_GE(out.body_height, 0.22 - 0.01);
}
