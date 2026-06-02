#pragma once

#include "quad/types.h"
#include "quad/leg_kinematics.h"
#include "quad/gait_scheduler.h"
#include "quad/foot_planner.h"
#include "quad/command_filter.h"
#include "quad/safety_monitor.h"

namespace quad {

struct LocomotionDebug {
  MotionCommand command{};
  ContactSchedule contact{};
  std::array<FootPlannerLegDebug, lm::kLegCount> legs{};
};

// ---------------------------------------------------------------------------
// 控制器参数 (聚合各子模块参数)
// ---------------------------------------------------------------------------
struct LocomotionControllerParams {
  LegKinematics *kin{nullptr};  // 外部传入, 不拥有

  CommandFilterParams filter;
  GaitParams gait;
  FootPlannerParams foot;
  SafetyParams safety;

  // 步态切换
  double gait_transition_duration{0.30};  // 步态切换平滑时间, s

  // 每种步态的独立参数 (频率、步高、J1 锁定等)
  GaitParams gait_stand;
  GaitParams gait_stepping;
  GaitParams gait_walk;
  GaitParams gait_trot;
  FootPlannerParams foot_walk;   // WALK 专用 planner 参数
  FootPlannerParams foot_trot;   // TROT 专用 planner 参数
};

// ---------------------------------------------------------------------------
// 主控制器: 串联所有模块
// ---------------------------------------------------------------------------
class LocomotionController {
public:
  LocomotionController() = default;
  explicit LocomotionController(const LocomotionControllerParams &params);

  void set_params(const LocomotionControllerParams &params);

  // 收到新指令 (来自 ROS 回调)
  void set_command(const MotionCommand &cmd, GaitType gait, double timestamp);

  // 每周期调用, 返回关节指令
  JointCommand step(double dt);

  // 状态查询
  bool is_timed_out() const { return filter_.is_timed_out(); }
  bool should_force_stand() const { return safety_.should_force_stand(); }
  GaitType current_gait() const { return current_gait_; }
  const LocomotionDebug &last_debug() const { return last_debug_; }

private:
  LocomotionControllerParams params_;

  CommandFilter filter_;
  GaitScheduler scheduler_;
  FootPlanner planner_;
  SafetyMonitor safety_;

  GaitType current_gait_{GaitType::STAND};
  GaitType target_gait_{GaitType::STAND};
  LocomotionDebug last_debug_{};
};

}  // namespace quad
