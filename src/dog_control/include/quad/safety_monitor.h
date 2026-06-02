#pragma once

#include "quad/types.h"
#include "quad/leg_kinematics.h"

#include <array>
#include <cstddef>

namespace quad {

// ---------------------------------------------------------------------------
// 安全参数
// ---------------------------------------------------------------------------
struct SafetyParams {
  double joint_margin{0.01};       // 关节限位安全余量, rad
  int ik_fail_threshold{10};       // 连续 IK 失败次数 → 强制 STAND
  double max_joint_vel{5.0};       // 最大关节速度, rad/s (用于平滑限幅)
};

// ---------------------------------------------------------------------------
// 安全监视器: 关节限幅 + IK 失败计数 + 强制 STAND
// ---------------------------------------------------------------------------
class SafetyMonitor {
public:
  SafetyMonitor() = default;
  explicit SafetyMonitor(const SafetyParams &params);

  void set_params(const SafetyParams &params);

  // 检查并限幅关节指令
  // 返回 true = 安全, false = 触发了限幅
  bool check_and_clamp(lm::QuadJoints<double> &joints,
                       const LegKinematics &kin) const;

  // 报告 IK 结果 (每次 compute 后调用)
  void report_ik_result(std::size_t leg_idx, bool success);

  // 是否应该强制回 STAND
  bool should_force_stand() const;

  // 重置计数器
  void reset();

  // 当前 IK 失败计数
  int ik_fail_count(std::size_t leg_idx) const;

private:
  SafetyParams params_;
  std::array<int, lm::kLegCount> consecutive_ik_fails_{};
};

}  // namespace quad
