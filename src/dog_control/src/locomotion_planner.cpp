#include "motor_ros2/locomotion_planner.h"

#include <algorithm>
#include <cmath>

namespace motor_ros2 {
namespace locomotion {

namespace {

constexpr std::array<lm::LegId, lm::kLegCount> kWalkLegSequence = {
  lm::LegId::LF,
  lm::LegId::RB,
  lm::LegId::RF,
  lm::LegId::LB
};

constexpr double kWalkSlotRatio = 0.25;
constexpr double kWalkPhaseEps = 1e-9;

double wrap_unit(double x)
{
  while (x >= 1.0) x -= 1.0;
  while (x < 0.0) x += 1.0;
  return x;
}

double cosine_ease(double t)
{
  const double c = clamp_value(t, 0.0, 1.0);
  return 0.5 * (1.0 - std::cos(kPI * c));
}

double walk_swing_ratio(double duty_factor)
{
  return clamp_value(1.0 - duty_factor, 1e-4, kWalkSlotRatio);
}

double walk_pre_shift_ratio(double duty_factor, double pre_shift_ratio)
{
  const double swing_ratio = walk_swing_ratio(duty_factor);
  return clamp_value(pre_shift_ratio, 0.0, kWalkSlotRatio - swing_ratio);
}

double walk_swing_start_ratio(double duty_factor, double pre_shift_ratio)
{
  return walk_pre_shift_ratio(duty_factor, pre_shift_ratio);
}

double walk_swing_end_ratio(double duty_factor, double pre_shift_ratio)
{
  return walk_swing_start_ratio(duty_factor, pre_shift_ratio) + walk_swing_ratio(duty_factor);
}

bool walk_in_swing(double local_u, double duty_factor, double pre_shift_ratio)
{
  const double u = wrap_unit(local_u);
  const double swing_start = walk_swing_start_ratio(duty_factor, pre_shift_ratio);
  const double swing_end = walk_swing_end_ratio(duty_factor, pre_shift_ratio);
  return (u > swing_start + kWalkPhaseEps) && (u < swing_end - kWalkPhaseEps);
}

double walk_swing_phase(double local_u, double duty_factor, double pre_shift_ratio)
{
  const double u = wrap_unit(local_u);
  const double swing_start = walk_swing_start_ratio(duty_factor, pre_shift_ratio);
  const double swing_ratio = walk_swing_ratio(duty_factor);
  return clamp_value((u - swing_start) / swing_ratio, 0.0, 1.0);
}

double walk_contact_progress(double local_u, double duty_factor, double pre_shift_ratio)
{
  const double u = wrap_unit(local_u);
  const double swing_end = walk_swing_end_ratio(duty_factor, pre_shift_ratio);
  const double duty = clamp_value(duty_factor, 1e-4, 0.999);

  if (u >= swing_end) {
    return clamp_value((u - swing_end) / duty, 0.0, 1.0);
  }
  return clamp_value((u + 1.0 - swing_end) / duty, 0.0, 1.0);
}

}  // namespace

LocomotionPlanner::LocomotionPlanner(const LocomotionParams &params)
: params_(params),
  logger_(rclcpp::get_logger("locomotion_planner"))
{
}

void LocomotionPlanner::bind_logger(const rclcpp::Logger &logger, const rclcpp::Clock::SharedPtr &clock)
{
  logger_ = logger;
  clock_ = clock;
}

bool LocomotionPlanner::is_left_leg(lm::LegId leg)
{
  return leg == lm::LegId::LF || leg == lm::LegId::LB;
}

bool LocomotionPlanner::is_front_leg(lm::LegId leg)
{
  return leg == lm::LegId::LF || leg == lm::LegId::RF;
}

double LocomotionPlanner::stand_foot_x_for_leg(lm::LegId leg) const
{
  return is_front_leg(leg) ? params_.stand_foot_x_front : params_.stand_foot_x_rear;
}

double LocomotionPlanner::stand_foot_y_for_leg(lm::LegId leg) const
{
  return is_left_leg(leg) ? params_.stand_foot_y : -params_.stand_foot_y;
}

double LocomotionPlanner::foot_x_min_for_leg(lm::LegId leg) const
{
  return is_front_leg(leg) ? params_.foot_x_min_front : params_.foot_x_min_rear;
}

double LocomotionPlanner::swing_scale_for_leg(lm::LegId leg) const
{
  return is_front_leg(leg) ? params_.swing_scale_front : params_.swing_scale_rear;
}

bool LocomotionPlanner::initialize_stand_offsets()
{
  for (std::size_t leg = 0; leg < lm::kLegCount; ++leg) {
    const lm::LegId leg_id = static_cast<lm::LegId>(leg);
    const double x = stand_foot_x_for_leg(leg_id);
    const double y = stand_foot_y_for_leg(leg_id);

    double t1 = 0.0;
    double t2 = 0.0;
    double t3 = 0.0;
    const double stand_z_up = -params_.height_default;
    if (!foot_ik(x, y, stand_z_up, t1, t2, t3)) {
      RCLCPP_ERROR(
        logger_,
        "Stand IK unreachable leg=%zu target_xyz_up=(%.3f,%.3f,%.3f)",
        leg, x, y, stand_z_up);
      return false;
    }

    stand_offsets_[leg] = {t1, t2, t3};
  }

  return true;
}

std::array<LocomotionPlanner::FootTarget, lm::kLegCount> LocomotionPlanner::compute_foot_targets(
  GaitMode mode,
  double phase,
  double vx,
  double vy,
  double yaw,
  double clearance) const
{
  const GaitConfig cfg = gait_config(params_, mode);
  const bool stepping_vertical_active = params_.stepping_strict_vertical && (mode == GaitMode::STEPPING);
  const bool walk_mode = (mode == GaitMode::WALK) && !stepping_vertical_active;

  double step_len_x = 0.0;
  double step_len_y = 0.0;
  if (mode != GaitMode::STAND && cfg.step_freq > 1e-4) {
    const double step_len_limit = walk_mode ? params_.max_step_length_walk : params_.max_step_length;
    step_len_x = clamp_value(vx / cfg.step_freq, -step_len_limit, step_len_limit);
    step_len_y = clamp_value(vy / cfg.step_freq, -step_len_limit, step_len_limit);
  }
  if (stepping_vertical_active) {
    step_len_x = 0.0;
    step_len_y = 0.0;
  }

  std::array<FootTarget, lm::kLegCount> targets{};
  for (std::size_t leg_idx = 0; leg_idx < lm::kLegCount; ++leg_idx) {
    const lm::LegId leg = static_cast<lm::LegId>(leg_idx);
    const bool is_left = is_left_leg(leg);
    const double stand_x = stand_foot_x_for_leg(leg);
    const double foot_y0 = stand_foot_y_for_leg(leg);
    const double foot_x_min = foot_x_min_for_leg(leg);
    const double swing_scale = swing_scale_for_leg(leg);

    double leg_phase = phase;
    if (mode == GaitMode::STEPPING) {
      if (leg_idx == 1) {
        leg_phase = wrap_to_2pi(phase + kPI);
      } else if (leg_idx == 2) {
        leg_phase = wrap_to_2pi(phase + kPI + params_.stepping_diagonal_skew_rad);
      } else if (leg_idx == 3) {
        leg_phase = wrap_to_2pi(phase + params_.stepping_diagonal_skew_rad);
      }
    } else if (mode == GaitMode::WALK) {
      leg_phase = wrap_to_2pi(phase - walk_phase_offset_for_leg(leg));
    } else if (mode == GaitMode::TROT) {
      const bool group_a = (leg_idx == 0 || leg_idx == 3);
      if (!group_a) {
        leg_phase = wrap_to_2pi(leg_phase + kPI);
      }
    }

    const double stride_y = step_len_y * swing_scale;
    FootTarget target;
    target.x = stand_x;
    target.x_trans = stand_x;
    target.y = foot_y0;

    if (!stepping_vertical_active) {
      const bool in_place_turn =
        (std::abs(vx) < params_.in_place_turn_vx_threshold) &&
        (std::abs(vy) < params_.in_place_turn_vy_threshold);
      const double yaw_gain = in_place_turn ? params_.yaw_step_gain_in_place : params_.yaw_step_gain;

      double stride_x_trans = 0.0;
      double yaw_turn = 0.0;
      if (walk_mode) {
        stride_x_trans = swing_scale * walk_stride_offset(
          leg_phase, step_len_x, params_.walk_duty_factor, params_.walk_pre_shift_ratio);
        yaw_turn = swing_scale * walk_stride_offset(
          leg_phase, yaw * yaw_gain, params_.walk_duty_factor, params_.walk_pre_shift_ratio);
        target.y = foot_y0 + walk_stride_offset(
          leg_phase, stride_y, params_.walk_duty_factor, params_.walk_pre_shift_ratio);
      } else {
        stride_x_trans = swing_scale * stride_offset(leg_phase, step_len_x);
        yaw_turn = swing_scale * stride_offset(leg_phase, yaw * yaw_gain);
        target.y = foot_y0 + stride_offset(leg_phase, stride_y);
      }

      target.x_trans = std::max(stand_x + stride_x_trans, foot_x_min);
      const double foot_x_raw = target.x_trans + (is_left ? -yaw_turn : yaw_turn);
      target.x = std::max(foot_x_raw, foot_x_min);
    }

    const double lift = walk_mode ?
      walk_swing_lift(leg_phase, cfg.step_height, params_.walk_duty_factor, params_.walk_pre_shift_ratio) :
      ((mode == GaitMode::WALK) ? walk_swing_lift(leg_phase, cfg.step_height, 0.75, 0.0) :
      swing_lift(leg_phase, cfg.step_height));
    target.z_up = -(clearance - lift);
    target.in_swing = lift > 1e-9;
    targets[leg_idx] = target;
  }

  return targets;
}

lm::QuadJoints<double> LocomotionPlanner::compute(
  GaitMode mode,
  double phase,
  double vx,
  double vy,
  double yaw,
  double clearance) const
{
  const auto targets = compute_foot_targets(mode, phase, vx, vy, yaw, clearance);

  lm::QuadJoints<double> joints{};
  for (std::size_t leg_idx = 0; leg_idx < lm::kLegCount; ++leg_idx) {
    const lm::LegId leg = static_cast<lm::LegId>(leg_idx);
    const double foot_y0 = stand_foot_y_for_leg(leg);
    const FootTarget &target = targets[leg_idx];

    double t1 = 0.0;
    double t2 = 0.0;
    double t3_leg = 0.0;
    if (!foot_ik(target.x, target.y, target.z_up, t1, t2, t3_leg)) {
      if (clock_) {
        RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 2000,
          "IK unreachable leg=%zu xyz_up=(%.3f %.3f %.3f), keep zero-delta for this leg",
          leg_idx, target.x, target.y, target.z_up);
      }
      continue;
    }

    const double t1_x_trans = std::atan2(foot_y0, target.x_trans);
    const double t1_x_total = std::atan2(foot_y0, target.x);
    const double t1_delta_x_trans = t1_x_trans - stand_offsets_[leg_idx].t1;
    const double t1_delta_x_turn = t1_x_total - t1_x_trans;
    const double t1_delta_lateral = t1 - t1_x_total;

    auto &leg_joints = joints.at(leg);
    if (mode == GaitMode::WALK) {
      leg_joints.j1 = 0.0;
    } else {
      leg_joints.j1 =
        params_.joint1_x_coupling_gain * t1_delta_x_trans +
        params_.joint1_yaw_coupling_gain * t1_delta_x_turn +
        t1_delta_lateral;
    }
    leg_joints.j2 = t2 - stand_offsets_[leg_idx].t2;
    leg_joints.j3 = (t3_leg - stand_offsets_[leg_idx].t3) * 2.0;
  }

  const double lim = deg2rad(params_.joint_limit_deg);
  for (std::size_t leg_idx = 0; leg_idx < lm::kLegCount; ++leg_idx) {
    auto &leg_joints = joints.at(static_cast<lm::LegId>(leg_idx));
    leg_joints.j1 = clamp_value(leg_joints.j1, -lim, lim);
    leg_joints.j2 = clamp_value(leg_joints.j2, -lim, lim);
    leg_joints.j3 = clamp_value(leg_joints.j3, params_.joint3_cmd_min, params_.joint3_cmd_max);
  }

  return joints;
}

double LocomotionPlanner::stride_offset(double phase, double step_len)
{
  constexpr double stance_ratio = 0.6;
  const double u = wrap_to_2pi(phase) / (2.0 * kPI);
  if (u < stance_ratio) {
    const double t = u / stance_ratio;
    const double s = smooth_step(t);
    return (0.5 - s) * step_len;
  }

  const double t = (u - stance_ratio) / (1.0 - stance_ratio);
  const double s = smooth_step(t);
  return (-0.5 + s) * step_len;
}

double LocomotionPlanner::walk_phase_offset_for_leg(lm::LegId leg)
{
  switch (leg) {
    case lm::LegId::LF:
      return 0.0;
    case lm::LegId::RB:
      return 0.5 * kPI;
    case lm::LegId::RF:
      return 1.0 * kPI;
    case lm::LegId::LB:
      return 1.5 * kPI;
    default:
      return 0.0;
  }
}

double LocomotionPlanner::walk_stride_offset(
  double phase,
  double step_len,
  double duty_factor,
  double pre_shift_ratio)
{
  const double u = wrap_to_2pi(phase) / (2.0 * kPI);
  if (walk_in_swing(u, duty_factor, pre_shift_ratio)) {
    const double s = walk_swing_phase(u, duty_factor, pre_shift_ratio);
    return (-0.5 + cosine_ease(s)) * step_len;
  }

  const double progress = walk_contact_progress(u, duty_factor, pre_shift_ratio);
  return (0.5 - progress) * step_len;
}

double LocomotionPlanner::walk_swing_lift(
  double phase,
  double step_height,
  double duty_factor,
  double pre_shift_ratio)
{
  const double u = wrap_to_2pi(phase) / (2.0 * kPI);
  if (!walk_in_swing(u, duty_factor, pre_shift_ratio)) {
    return 0.0;
  }

  const double s = walk_swing_phase(u, duty_factor, pre_shift_ratio);
  return std::max(step_height, 0.0) * std::sin(kPI * s);
}

double LocomotionPlanner::swing_lift(double phase, double step_height)
{
  constexpr double stance_ratio = 0.6;
  const double u = wrap_to_2pi(phase) / (2.0 * kPI);
  if (u < stance_ratio) {
    return 0.0;
  }

  const double t = (u - stance_ratio) / (1.0 - stance_ratio);
  const double s = std::sin(kPI * t);
  const double s2 = s * s;
  return std::max(step_height, 0.0) * s2 * s2;
}

bool LocomotionPlanner::foot_ik(
  double x,
  double y,
  double z_up,
  double &t1,
  double &t2,
  double &t3_leg) const
{
  t1 = std::atan2(y, x);

  const double l_horizontal = std::sqrt(x * x + y * y);
  const double l_vertical_down = -z_up;
  const double distance = std::sqrt(
    l_horizontal * l_horizontal + l_vertical_down * l_vertical_down);

  const double max_reach = params_.thigh_length + params_.shank_length;
  const double min_reach = std::abs(params_.thigh_length - params_.shank_length);
  if (distance > max_reach || distance < min_reach) {
    return false;
  }

  double cos_t3 =
    (distance * distance - params_.thigh_length * params_.thigh_length -
    params_.shank_length * params_.shank_length) /
    (2.0 * params_.thigh_length * params_.shank_length);
  cos_t3 = clamp_value(cos_t3, -1.0, 1.0);
  t3_leg = std::acos(cos_t3);

  const double alpha = std::atan2(l_vertical_down, l_horizontal);
  double tmp = params_.shank_length * std::sin(t3_leg) / std::max(distance, 1e-6);
  tmp = clamp_value(tmp, -1.0, 1.0);
  const double beta = std::asin(tmp);
  t2 = alpha - beta;
  return true;
}

}  // namespace locomotion
}  // namespace motor_ros2
