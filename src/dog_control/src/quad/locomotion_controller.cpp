#include "quad/locomotion_controller.h"

#include <cmath>

namespace quad {

namespace {

// 根据步态类型选择对应的 GaitParams
const GaitParams &select_gait_params(const LocomotionControllerParams &p, GaitType gait)
{
  switch (gait) {
    case GaitType::STAND:    return p.gait_stand;
    case GaitType::STEPPING: return p.gait_stepping;
    case GaitType::WALK:     return p.gait_walk;
    case GaitType::TROT:     return p.gait_trot;
    default:                 return p.gait_stand;
  }
}

// 根据步态类型选择对应的 FootPlannerParams
const FootPlannerParams &select_foot_params(const LocomotionControllerParams &p, GaitType gait)
{
  switch (gait) {
    case GaitType::WALK: return p.foot_walk;
    case GaitType::TROT: return p.foot_trot;
    default:             return p.foot;  // STAND / STEPPING 用默认
  }
}

double average_stand_foot_z(const LegKinematics &kin)
{
  double sum = 0.0;
  for (std::size_t i = 0; i < lm::kLegCount; ++i) {
    const auto leg = static_cast<lm::LegId>(i);
    const auto &cfg = kin.config(leg);
    sum += (cfg.j1_to_j3_at_zero + cfg.stand_foot_from_j3).z;
  }
  return sum / static_cast<double>(lm::kLegCount);
}

void fill_stand_debug(const MotionCommand &cmd,
                      const ContactSchedule &cs,
                      const LegKinematics &kin,
                      LocomotionDebug &debug)
{
  debug.command = cmd;
  debug.contact = cs;

  for (std::size_t i = 0; i < lm::kLegCount; ++i) {
    const auto leg = static_cast<lm::LegId>(i);
    const auto &cfg = kin.config(leg);
    auto &d = debug.legs[i];

    d.in_stance = true;
    d.stance_progress = cs.stance_progress[i];
    d.swing_progress = cs.swing_progress[i];
    d.foot_pos_j1 = cfg.j1_to_j3_at_zero + cfg.stand_foot_from_j3;
    d.joints = {};
    d.ik_success = true;
  }
}

}  // namespace

// ---------------------------------------------------------------------------
// 构造
// ---------------------------------------------------------------------------
LocomotionController::LocomotionController(const LocomotionControllerParams &params)
: params_(params),
  filter_(params.filter),
  planner_(params.foot),
  safety_(params.safety)
{
  current_gait_ = params.gait.type;
  target_gait_ = params.gait.type;

  // 用对应步态的参数初始化 scheduler 和 planner
  const auto &gp = select_gait_params(params, current_gait_);
  GaitParams gp_init = gp;
  gp_init.type = current_gait_;
  scheduler_.set_gait(gp_init);

  auto fp = select_foot_params(params, current_gait_);
  fp.gait_frequency = gp.frequency;
  planner_.set_params(fp);
}

void LocomotionController::set_params(const LocomotionControllerParams &params)
{
  params_ = params;
  filter_.set_params(params.filter);
  safety_.set_params(params.safety);

  const auto &gp = select_gait_params(params, current_gait_);
  auto fp = select_foot_params(params, current_gait_);
  fp.gait_frequency = gp.frequency;
  planner_.set_params(fp);
}

// ---------------------------------------------------------------------------
// 收到新指令
// ---------------------------------------------------------------------------
void LocomotionController::set_command(
  const MotionCommand &cmd, GaitType gait, double timestamp)
{
  filter_.set_command(cmd, timestamp);

  if (gait != target_gait_) {
    target_gait_ = gait;
    current_gait_ = gait;

    // 用对应步态的参数
    const auto &gp = select_gait_params(params_, gait);
    GaitParams gp_set = gp;
    gp_set.type = gait;
    scheduler_.set_gait(gp_set);

    auto fp = select_foot_params(params_, gait);
    fp.gait_frequency = gp.frequency;
    planner_.set_params(fp);
  }
}

// ---------------------------------------------------------------------------
// 每周期主循环
// ---------------------------------------------------------------------------
JointCommand LocomotionController::step(double dt)
{
  // 1. 指令滤波
  MotionCommand cmd = filter_.step(dt);

  // 超时 → 强制 STAND
  if (filter_.is_timed_out() || safety_.should_force_stand()) {
    cmd = MotionCommand{};
    if (current_gait_ != GaitType::STAND) {
      GaitParams gp;
      gp.type = GaitType::STAND;
      scheduler_.set_gait(gp);
      current_gait_ = GaitType::STAND;
      target_gait_ = GaitType::STAND;

      auto fp = select_foot_params(params_, GaitType::STAND);
      fp.gait_frequency = 0.0;
      planner_.set_params(fp);
    }
  }

  // 2. 步态调度
  ContactSchedule cs = scheduler_.step(dt);

  if (current_gait_ == GaitType::STAND) {
    fill_stand_debug(cmd, cs, *params_.kin, last_debug_);
    return JointCommand{};
  }

  // 3. 足端规划 + IK
  // stand_foot_from_j3 是每条腿站姿主真源; body_height 只转成相对平均站姿的统一 z 偏移。
  const double stand_avg_z = average_stand_foot_z(*params_.kin);
  const double foot_z_offset = -std::abs(cmd.body_height) - stand_avg_z;
  last_debug_.command = cmd;
  last_debug_.contact = cs;
  auto joints = planner_.compute(cs, cmd, foot_z_offset, *params_.kin, &last_debug_.legs);

  // 4. 安全检查
  for (std::size_t i = 0; i < lm::kLegCount; ++i) {
    if (current_gait_ != GaitType::STAND) {
      auto leg = static_cast<lm::LegId>(i);
      const auto &q = joints.at(leg);
      const bool ik_ok = (std::abs(q.j1) > 1e-6 ||
                          std::abs(q.j2) > 1e-6 ||
                          std::abs(q.j3) > 1e-6);
      safety_.report_ik_result(i, ik_ok || cs.in_stance[i]);
    }
  }

  safety_.check_and_clamp(joints, *params_.kin);

  JointCommand result;
  result.positions = joints;
  return result;
}

}  // namespace quad
