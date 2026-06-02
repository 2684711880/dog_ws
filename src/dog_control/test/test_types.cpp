#include <gtest/gtest.h>

#include "quad/types.h"

TEST(QuadTypes, Vec3Basic)
{
  quad::Vec3 a(1, 2, 3);
  quad::Vec3 b(4, 5, 6);

  auto c = a + b;
  EXPECT_DOUBLE_EQ(c.x, 5.0);
  EXPECT_DOUBLE_EQ(c.y, 7.0);
  EXPECT_DOUBLE_EQ(c.z, 9.0);

  auto d = b - a;
  EXPECT_DOUBLE_EQ(d.x, 3.0);
  EXPECT_DOUBLE_EQ(d.y, 3.0);
  EXPECT_DOUBLE_EQ(d.z, 3.0);

  auto e = a * 2.0;
  EXPECT_DOUBLE_EQ(e.x, 2.0);
  EXPECT_DOUBLE_EQ(e.y, 4.0);
  EXPECT_DOUBLE_EQ(e.z, 6.0);

  EXPECT_DOUBLE_EQ(quad::Vec3(3, 4, 0).norm(), 5.0);
}

TEST(QuadTypes, GaitTypeMapping)
{
  EXPECT_EQ(quad::to_gait_type(0), quad::GaitType::STAND);
  EXPECT_EQ(quad::to_gait_type(1), quad::GaitType::STEPPING);
  EXPECT_EQ(quad::to_gait_type(2), quad::GaitType::WALK);
  EXPECT_EQ(quad::to_gait_type(3), quad::GaitType::TROT);
  EXPECT_EQ(quad::to_gait_type(99), quad::GaitType::STAND);

  EXPECT_STREQ(quad::gait_type_name(quad::GaitType::STAND), "STAND");
  EXPECT_STREQ(quad::gait_type_name(quad::GaitType::TROT), "TROT");
}

TEST(QuadTypes, GaitTypeValuesMatchDogMsgs)
{
  // 验证枚举值与 dog_msgs 的常量一致
  EXPECT_EQ(static_cast<uint8_t>(quad::GaitType::STAND), 0);
  EXPECT_EQ(static_cast<uint8_t>(quad::GaitType::STEPPING), 1);
  EXPECT_EQ(static_cast<uint8_t>(quad::GaitType::WALK), 2);
  EXPECT_EQ(static_cast<uint8_t>(quad::GaitType::TROT), 3);
}

TEST(QuadTypes, JointCommandDefaultIsZero)
{
  quad::JointCommand cmd;
  for (std::size_t i = 0; i < quad::lm::kLegCount; ++i) {
    auto leg = static_cast<quad::lm::LegId>(i);
    EXPECT_DOUBLE_EQ(cmd.positions.at(leg).j1, 0.0);
    EXPECT_DOUBLE_EQ(cmd.positions.at(leg).j2, 0.0);
    EXPECT_DOUBLE_EQ(cmd.positions.at(leg).j3, 0.0);
  }
}

TEST(QuadTypes, ContactScheduleDefault)
{
  quad::ContactSchedule cs;
  EXPECT_DOUBLE_EQ(cs.global_phase, 0.0);
  for (std::size_t i = 0; i < quad::lm::kLegCount; ++i) {
    EXPECT_FALSE(cs.in_stance[i]);
    EXPECT_DOUBLE_EQ(cs.swing_progress[i], 0.0);
  }
}
