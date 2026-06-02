#pragma once

#include "motor_ros2/leg_model.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace quad {

namespace lm = motor_ros2::leg_model;

constexpr double kPI = 3.14159265358979323846;
constexpr double k2PI = 2.0 * kPI;
constexpr double kHalfPI = kPI / 2.0;

// ---------------------------------------------------------------------------
// Vec3
// ---------------------------------------------------------------------------
struct Vec3 {
  double x{0}, y{0}, z{0};

  Vec3() = default;
  Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

  Vec3 operator+(const Vec3 &o) const { return {x + o.x, y + o.y, z + o.z}; }
  Vec3 operator-(const Vec3 &o) const { return {x - o.x, y - o.y, z - o.z}; }
  Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
  Vec3 &operator+=(const Vec3 &o)
  {
    x += o.x;
    y += o.y;
    z += o.z;
    return *this; 
  }

  double norm() const { return std::sqrt(x * x + y * y + z * z); }
};

// ---------------------------------------------------------------------------
// 单条腿状态 (FK 反馈层)
// ---------------------------------------------------------------------------
struct LegState {
  lm::LegJoints<double> joint_pos;  // 当前关节角 (joint-cmd 空间)
  lm::LegJoints<double> joint_vel;
  Vec3 foot_pos_j1;    // FK 算出, J1 局部系 (X前 Y左 Z上)
  Vec3 foot_pos_body;  // FK 算出, body 系
  bool in_contact{true}; // 是否在支撑中，实际接触状态， 预留未用
};

// ---------------------------------------------------------------------------
// 全机器人状态
// ---------------------------------------------------------------------------
struct RobotState {
  std::array<LegState, lm::kLegCount> legs{};
  Vec3 body_rpy{};          // 来自 IMU, 没有则全 0
  Vec3 body_angular_vel{};  // 机体角速度
  double timestamp{0};      // 时间戳, 单位: s
};

// ---------------------------------------------------------------------------
// 已滤波的运动指令 (内部口径)
// ---------------------------------------------------------------------------
struct MotionCommand {
  double vx{0};
  double vy{0};
  double yaw_rate{0};
  double body_height{0.290};  // 绝对值, m, 当前简化零位下 J3 到地面
};

// ---------------------------------------------------------------------------
// 步态类型 — 与 dog_msgs::LocomotionCommand::GAIT_* 整型值对齐
// ---------------------------------------------------------------------------
enum class GaitType : uint8_t {
  STAND = 0,
  STEPPING = 1,
  WALK = 2,
  TROT = 3,
};

// ---------------------------------------------------------------------------
// 步态参数
// ---------------------------------------------------------------------------
struct GaitParams {
  GaitType type{GaitType::STAND};
  double frequency{1.5};
  double duty_factor{0.65};
  double step_height{0.05};
  double max_step_length{0.12};
  std::array<double, lm::kLegCount> phase_offsets{0, kPI, kPI, 0};
  bool joint1_lock_zero{false};
};

// 默认 phase_offsets: LF=0 RF=1 LB=2 RB=3
// TROT (对角): 0, π, π, 0
// WALK (顺序): 0, π/2, π, 3π/2
// STEPPING (前后): 0, 0, π, π
inline std::array<double, lm::kLegCount> default_phase_offsets(GaitType gait)
{
  switch (gait) {
    case GaitType::WALK:     return {0, kHalfPI, kPI, kHalfPI + kPI};
    case GaitType::STEPPING: return {0, 0, kPI, kPI};
    default:                 return {0, kPI, kPI, 0};  // TROT / STAND
  }
}

// ---------------------------------------------------------------------------
// 接触计划
// ---------------------------------------------------------------------------
struct ContactSchedule {
  std::array<bool, lm::kLegCount> in_stance{};  // 是否在支撑中
  std::array<double, lm::kLegCount> phase_in_cycle{};   // [0,1)
  std::array<double, lm::kLegCount> swing_progress{};   // 摆动 [0,1], 支撑时 -1
  std::array<double, lm::kLegCount> stance_progress{};  // 支撑 [0,1], 摆动时 -1
  double global_phase{0};                                // [0, 2π)
};

// ---------------------------------------------------------------------------
// 足端目标
// ---------------------------------------------------------------------------
struct FootTarget {
  Vec3 position;       // body 系下足端目标 (X前 Y左 Z上)
  bool force_contact;  // 支撑相 true  预留接口
};

// ---------------------------------------------------------------------------
// 关节指令 (最终输出)
// ---------------------------------------------------------------------------
struct JointCommand {
  lm::QuadJoints<double> positions;  // joint-cmd 空间
};
//未来可加 lm::QuadJoints<double> velocities;     // 速度指令（未来）
//        lm::QuadJoints<double> efforts;        // 力矩指令（未来）

// ---------------------------------------------------------------------------
// GaitType ↔ dog_msgs 整型转换
// ---------------------------------------------------------------------------
inline GaitType to_gait_type(uint8_t mode)
{
  switch (mode) {
    case 0: return GaitType::STAND;
    case 1: return GaitType::STEPPING;
    case 2: return GaitType::WALK;
    case 3: return GaitType::TROT;
    default: return GaitType::STAND;
  }
}

inline const char *gait_type_name(GaitType t)
{
  switch (t) {
    case GaitType::STAND: return "STAND";
    case GaitType::STEPPING: return "STEPPING";
    case GaitType::WALK: return "WALK";
    case GaitType::TROT: return "TROT";
    default: return "STAND";
  }
}

}  // namespace quad
