#include "quad/foot_planner.h"

#include <algorithm>
#include <cmath>

namespace quad {

namespace {

// 5 控制点 Bezier 曲线
double bezier5(double t, double p0, double p1, double p2, double p3, double p4)
{
  const double u = 1.0 - t;
  const double u2 = u * u;
  const double u3 = u2 * u;
  const double u4 = u3 * u;
  const double t2 = t * t;
  const double t3 = t2 * t;
  const double t4 = t3 * t;
  return u4 * p0 + 4.0 * u3 * t * p1 + 6.0 * u2 * t2 * p2
       + 4.0 * u * t3 * p3 + t4 * p4;
}

double clamp(double v, double lo, double hi)
{
  return std::min(std::max(v, lo), hi);
}

double smooth_step(double t)
{
  const double c = clamp(t, 0.0, 1.0);
  return c * c * (3.0 - 2.0 * c);
}

}  // namespace

// ---------------------------------------------------------------------------
// 构造 / 参数
// ---------------------------------------------------------------------------
FootPlanner::FootPlanner(const FootPlannerParams &params)
: params_(params)
{
  params_.step_height = std::max(params_.step_height, params_.min_step_height);
}

void FootPlanner::set_params(const FootPlannerParams &params)
{
  params_ = params;
  params_.step_height = std::max(params_.step_height, params_.min_step_height);
}

bool FootPlanner::is_front_leg(lm::LegId leg)
{
  return leg == lm::LegId::LF || leg == lm::LegId::RF;
}

// ---------------------------------------------------------------------------
// 计算某条腿的单步足端位移.
// vx/vy/yaw_rate 是速度口径, 这里按步频换成一个 gait cycle 的位移.
// ---------------------------------------------------------------------------
void FootPlanner::compute_step(lm::LegId leg,
                                const MotionCommand &cmd,
                                const Vec3 &nominal_foot_body,
                                double &step_x, double &step_y) const
{
  const double freq = std::max(params_.gait_frequency, 1e-6);
  const double vx = cmd.vx / freq;
  const double vy = cmd.vy / freq;
  const double yaw = cmd.yaw_rate;

  // 判断是否原地转
  const bool in_place = (std::abs(vx) < params_.in_place_turn_vx_threshold)
                     && (std::abs(vy) < params_.in_place_turn_vy_threshold);
  const double yaw_gain = in_place ? params_.yaw_step_gain_in_place : params_.yaw_step_gain;

  // 步幅缩放 (前/后腿)
  const double swing_scale = is_front_leg(leg) ? params_.swing_scale_front : params_.swing_scale_rear;

  const double yaw_angle = (yaw / freq) * yaw_gain;
  const double yaw_step_x = -yaw_angle * nominal_foot_body.y;
  const double yaw_step_y = params_.yaw_lateral_gain * yaw_angle * nominal_foot_body.x;

  // 总步长
  step_x = (vx + yaw_step_x) * swing_scale;
  step_y = (vy + yaw_step_y) * swing_scale;

  // 限幅
  const double len = std::sqrt(step_x * step_x + step_y * step_y);
  if (len > params_.max_step_length) {
    const double scale = params_.max_step_length / len;
    step_x *= scale;
    step_y *= scale;
  }
}

// ---------------------------------------------------------------------------
// Bezier 摆动高度
// ---------------------------------------------------------------------------
double FootPlanner::bezier_swing_height(double t, double lift_ratio)
{
  const double tt = clamp(t, 0.0, 1.0);
  const double peak = 1.0;
  const double lift = peak * clamp(lift_ratio, 0.0, 1.0);
  return bezier5(tt, 0.0, lift, peak, lift, 0.0);
}

// ---------------------------------------------------------------------------
// 支撑相偏移: 线性从 +step/2 到 -step/2 (smooth_step 平滑)
// ---------------------------------------------------------------------------
double FootPlanner::stance_offset(double t, double step_len)
{
  const double tt = clamp(t, 0.0, 1.0);
  const double s = smooth_step(tt);
  return (0.5 - s) * step_len;
}

// ---------------------------------------------------------------------------
// 核心: 计算每条腿关节角
// ---------------------------------------------------------------------------
lm::QuadJoints<double> FootPlanner::compute(
  const ContactSchedule &cs,
  const MotionCommand &cmd,
  double foot_z_offset_j1,
  const LegKinematics &kin,
  std::array<FootPlannerLegDebug, lm::kLegCount> *debug) const
{
  lm::QuadJoints<double> result{};

  for (std::size_t i = 0; i < lm::kLegCount; ++i) {
    const auto leg = static_cast<lm::LegId>(i);
    const auto &cfg = kin.config(leg);
    const Vec3 nom = cfg.j1_to_j3_at_zero + cfg.stand_foot_from_j3;
    const Vec3 nom_body = cfg.j1_origin_in_body + nom;
    const double foot_z_j1 = nom.z + foot_z_offset_j1;

    // 计算步长
    double step_x = 0, step_y = 0;
    compute_step(leg, cmd, nom_body, step_x, step_y);

    Vec3 foot_pos;
    double off_x = 0;  // 跟踪 x 偏移, 用于 j1_lock_forward
    if (cs.in_stance[i]) {
      // 支撑相: 足端在地面, body 前移 → 足端后移
      // stance_offset: t=0 → +step/2, t=1 → -step/2
      off_x = stance_offset(cs.stance_progress[i], step_x);
      const double off_y = stance_offset(cs.stance_progress[i], step_y);
      foot_pos = Vec3(nom.x + off_x, nom.y + off_y, foot_z_j1);
    } else {
      // 摆动相: 从 liftoff 到 touchdown
      const double t = clamp(cs.swing_progress[i], 0.0, 1.0);

      // liftoff = 支撑相末端 (t=1)
      const double liftoff_x = nom.x + stance_offset(1.0, step_x);
      const double liftoff_y = nom.y + stance_offset(1.0, step_y);

      // touchdown = 支撑相起点 (t=0) + 步长偏移
      const double touchdown_x = nom.x + stance_offset(0.0, step_x);
      const double touchdown_y = nom.y + stance_offset(0.0, step_y);

      // x/y: smooth_step 插值 liftoff → touchdown
      const double s = smooth_step(t);
      foot_pos.x = liftoff_x + s * (touchdown_x - liftoff_x);
      foot_pos.y = liftoff_y + s * (touchdown_y - liftoff_y);

      // z: Bezier 抬升
      const double height = bezier_swing_height(t, params_.bezier_lift_ratio);
      foot_pos.z = foot_z_j1 + height * params_.step_height;

      off_x = foot_pos.x - nom.x;
    }

    // j1_lock_forward: J1 计算时减去 x 偏移, 前进不驱动髋关节
    const double j1_ign = params_.j1_lock_forward ? off_x : 0.0;

    lm::LegJoints<double> q;
    const bool ik_success = kin.inverse_kinematics(leg, foot_pos, q, j1_ign);
    if (ik_success) {
      result.at(leg) = q;
    }

    if (debug) {
      auto &d = (*debug)[i];
      d.in_stance = cs.in_stance[i];
      d.stance_progress = cs.stance_progress[i];
      d.swing_progress = cs.swing_progress[i];
      d.foot_pos_j1 = foot_pos;
      d.joints = q;
      d.ik_success = ik_success;
    }
  }

  return result;
}

}  // namespace quad
