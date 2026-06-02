#include "quad/safety_monitor.h"

#include <algorithm>
#include <cmath>

namespace quad {

SafetyMonitor::SafetyMonitor(const SafetyParams &params)
: params_(params)
{
}

void SafetyMonitor::set_params(const SafetyParams &params)
{
  params_ = params;
}

// ---------------------------------------------------------------------------
// 关节限幅
// ---------------------------------------------------------------------------
bool SafetyMonitor::check_and_clamp(lm::QuadJoints<double> &joints,
                                     const LegKinematics &kin) const
{
  bool all_ok = true;

  for (std::size_t i = 0; i < lm::kLegCount; ++i) {
    const auto leg = static_cast<lm::LegId>(i);
    const auto &cfg = kin.config(leg);
    auto &leg_j = joints.at(leg);

    const double lo1 = cfg.joint_min.j1 + params_.joint_margin;
    const double hi1 = cfg.joint_max.j1 - params_.joint_margin;
    const double lo2 = cfg.joint_min.j2 + params_.joint_margin;
    const double hi2 = cfg.joint_max.j2 - params_.joint_margin;
    const double lo3 = cfg.joint_min.j3 + params_.joint_margin;
    const double hi3 = cfg.joint_max.j3 - params_.joint_margin;

    if (leg_j.j1 < lo1 || leg_j.j1 > hi1 ||
        leg_j.j2 < lo2 || leg_j.j2 > hi2 ||
        leg_j.j3 < lo3 || leg_j.j3 > hi3) {
      all_ok = false;
    }

    leg_j.j1 = std::min(std::max(leg_j.j1, lo1), hi1);
    leg_j.j2 = std::min(std::max(leg_j.j2, lo2), hi2);
    leg_j.j3 = std::min(std::max(leg_j.j3, lo3), hi3);
  }

  return all_ok;
}

// ---------------------------------------------------------------------------
// IK 失败计数
// ---------------------------------------------------------------------------
void SafetyMonitor::report_ik_result(std::size_t leg_idx, bool success)
{
  if (leg_idx >= lm::kLegCount) return;
  if (success) {
    consecutive_ik_fails_[leg_idx] = 0;
  } else {
    consecutive_ik_fails_[leg_idx]++;
  }
}

bool SafetyMonitor::should_force_stand() const
{
  for (std::size_t i = 0; i < lm::kLegCount; ++i) {
    if (consecutive_ik_fails_[i] >= params_.ik_fail_threshold) {
      return true;
    }
  }
  return false;
}

void SafetyMonitor::reset()
{
  consecutive_ik_fails_.fill(0);
}

int SafetyMonitor::ik_fail_count(std::size_t leg_idx) const
{
  if (leg_idx >= lm::kLegCount) return 0;
  return consecutive_ik_fails_[leg_idx];
}

}  // namespace quad
