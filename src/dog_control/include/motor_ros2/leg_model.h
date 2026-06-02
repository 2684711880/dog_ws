#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace motor_ros2 {
namespace leg_model {

constexpr std::size_t kLegCount = 4;
constexpr std::size_t kJointsPerLeg = 3;
constexpr std::size_t kJointCount = kLegCount * kJointsPerLeg;

enum class LegId : std::size_t { LF = 0, RF = 1, LB = 2, RB = 3 };
enum class JointId : std::size_t { J1 = 0, J2 = 1, J3 = 2 };

template<typename T>
struct LegJoints {
  T j1{};
  T j2{};
  T j3{};

  T &at(JointId joint)
  {
    switch (joint) {
      case JointId::J1: return j1;
      case JointId::J2: return j2;
      default: return j3;
    }
  }

  const T &at(JointId joint) const
  {
    switch (joint) {
      case JointId::J1: return j1;
      case JointId::J2: return j2;
      default: return j3;
    }
  }
};

template<typename T>
struct QuadJoints {
  std::array<LegJoints<T>, kLegCount> legs{};

  LegJoints<T> &at(LegId leg)
  {
    return legs[static_cast<std::size_t>(leg)];
  }

  const LegJoints<T> &at(LegId leg) const
  {
    return legs[static_cast<std::size_t>(leg)];
  }
};

struct JointLimit {
  float min_rad;
  float max_rad;
};

struct JointMeta {
  const char *name;
  LegId leg;
  JointId joint;
  std::size_t flat_index;
  float sign;
  JointLimit limit;
};

constexpr std::array<LegId, kLegCount> kLegOrder = {
  LegId::LF, LegId::RF, LegId::LB, LegId::RB
};

constexpr std::size_t flat_index(LegId leg, JointId joint)
{
  return static_cast<std::size_t>(leg) * kJointsPerLeg + static_cast<std::size_t>(joint);
}

constexpr std::array<const char *, kJointCount> kJointNames = {
  "LF_Joint_1", "LF_Joint_2", "LF_Joint_3",
  "RF_Joint_1", "RF_Joint_2", "RF_Joint_3",
  "LB_Joint_1", "LB_Joint_2", "LB_Joint_3",
  "RB_Joint_1", "RB_Joint_2", "RB_Joint_3"
};

constexpr std::array<float, kJointCount> kJointSigns = {
  -1.0f,  1.0f, -1.0f,
  -1.0f, -1.0f,  1.0f,
  1.0f,  1.0f, -1.0f,
  1.0f, -1.0f,  1.0f
};
// J1 前后腿安装方向是镜像的：后腿 J1 需要反号，才能与前腿共享同一套关节命令口径。
// 这里的符号会同时作用于下发和反馈，因此要始终作为唯一真源维护。

// Joint limits in joint-command space (NOT motor space).
// Left/right legs share identical limits — no sign flipping needed.
// To update: test with manual_joint_pub and fill the value you sent directly.
constexpr std::array<JointLimit, kJointCount> kJointLimits = {
  JointLimit{-1.2f, +1.2f},   // LF_J1
  JointLimit{-1.2f, +1.2f},   // LF_J2
  JointLimit{-0.6f, +2.4f},   // LF_J3
  JointLimit{-1.2f, +1.2f},   // RF_J1
  JointLimit{-1.2f, +1.2f},   // RF_J2
  JointLimit{-0.6f, +2.4f},   // RF_J3
  JointLimit{-1.2f, +1.2f},   // LB_J1
  JointLimit{-1.2f, +1.2f},   // LB_J2
  JointLimit{-0.6f, +2.4f},   // LB_J3
  JointLimit{-1.2f, +1.2f},   // RB_J1
  JointLimit{-1.2f, +1.2f},   // RB_J2
  JointLimit{-0.6f, +2.4f},   // RB_J3
};

constexpr std::array<JointMeta, kJointCount> kJointMeta = {{
  {kJointNames[0], LegId::LF, JointId::J1, 0,  kJointSigns[0],  kJointLimits[0]},
  {kJointNames[1], LegId::LF, JointId::J2, 1,  kJointSigns[1],  kJointLimits[1]},
  {kJointNames[2], LegId::LF, JointId::J3, 2,  kJointSigns[2],  kJointLimits[2]},
  {kJointNames[3], LegId::RF, JointId::J1, 3,  kJointSigns[3],  kJointLimits[3]},
  {kJointNames[4], LegId::RF, JointId::J2, 4,  kJointSigns[4],  kJointLimits[4]},
  {kJointNames[5], LegId::RF, JointId::J3, 5,  kJointSigns[5],  kJointLimits[5]},
  {kJointNames[6], LegId::LB, JointId::J1, 6,  kJointSigns[6],  kJointLimits[6]},
  {kJointNames[7], LegId::LB, JointId::J2, 7,  kJointSigns[7],  kJointLimits[7]},
  {kJointNames[8], LegId::LB, JointId::J3, 8,  kJointSigns[8],  kJointLimits[8]},
  {kJointNames[9], LegId::RB, JointId::J1, 9,  kJointSigns[9],  kJointLimits[9]},
  {kJointNames[10], LegId::RB, JointId::J2, 10, kJointSigns[10], kJointLimits[10]},
  {kJointNames[11], LegId::RB, JointId::J3, 11, kJointSigns[11], kJointLimits[11]}
}};

template<typename T>
inline std::array<T, kJointCount> flatten(const QuadJoints<T> &quad)
{
  std::array<T, kJointCount> out{};
  for (std::size_t li = 0; li < kLegCount; ++li) {
    const LegId leg = static_cast<LegId>(li);
    const LegJoints<T> &src = quad.at(leg);
    out[flat_index(leg, JointId::J1)] = src.j1;
    out[flat_index(leg, JointId::J2)] = src.j2;
    out[flat_index(leg, JointId::J3)] = src.j3;
  }
  return out;
}
// ROS 的 sensor_msgs::msg::JointState 的 .position 字段是 std::vector<double>——一个一维连续数组，
// 没有"腿"和"关节"的层次概念。必须把逻辑上的二维结构按固定顺序排成一维。

template<typename T>
inline QuadJoints<T> unflatten(const std::array<T, kJointCount> &flat)
{
  QuadJoints<T> out{};
  for (std::size_t li = 0; li < kLegCount; ++li) {
    const LegId leg = static_cast<LegId>(li);
    LegJoints<T> &dst = out.at(leg);
    dst.j1 = flat[flat_index(leg, JointId::J1)];
    dst.j2 = flat[flat_index(leg, JointId::J2)];
    dst.j3 = flat[flat_index(leg, JointId::J3)];
  }
  return out;
}

template<typename T>
inline std::vector<T> flatten_to_vector(const QuadJoints<T> &quad)
{
  const auto flat = flatten(quad);
  return std::vector<T>(flat.begin(), flat.end());
}

inline std::vector<std::string> joint_names_vector()
{
  return std::vector<std::string>(kJointNames.begin(), kJointNames.end());
}
// ROS 2 的 JointState 消息的 name 字段类型是 std::vector<std::string>，
// 而我们的关节名称存储在 constexpr std::array<const char *, 12> 中。两者类型不匹配，需要一个桥接。
inline bool has_canonical_joint_order(const std::vector<std::string> &names)
{
  if (names.size() != kJointCount) return false;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    if (names[i] != kJointNames[i]) return false;
  }
  return true;
}

static_assert(kJointCount == 12, "Joint count must remain 12.");
static_assert(kJointCount == kLegCount * kJointsPerLeg, "Invalid leg/joint dimensions.");
static_assert(flat_index(LegId::LF, JointId::J1) == 0, "LF_J1 index mismatch");
static_assert(flat_index(LegId::RF, JointId::J1) == 3, "RF_J1 index mismatch");
static_assert(flat_index(LegId::LB, JointId::J1) == 6, "LB_J1 index mismatch");
static_assert(flat_index(LegId::RB, JointId::J1) == 9, "RB_J1 index mismatch");

}  // namespace leg_model
}  // namespace motor_ros2
