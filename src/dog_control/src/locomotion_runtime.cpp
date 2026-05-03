#include "motor_ros2/locomotion_runtime.h"

namespace motor_ros2 {
namespace locomotion {

LocomotionRuntime::LocomotionRuntime(const LocomotionParams &params)
: params_(params),
  height_filtered_(params.height_default)
{
}

void LocomotionRuntime::reset(const rclcpp::Time &now)
{
  has_cmd_ = false;
  active_cmd_source_ = "none";

  cmd_mode_ = GaitMode::STAND;
  mode_current_ = GaitMode::STAND;
  mode_target_ = GaitMode::STAND;
  transition_from_mode_ = GaitMode::STAND;

  transition_active_ = false;
  transition_t_ = 1.0;
  transition_start_time_ = now;
  cmd_time_ = now;
  last_update_time_ = now;

  target_vx_ = 0.0;
  target_vy_ = 0.0;
  target_yaw_rate_ = 0.0;
  target_height_offset_ = 0.0;

  vx_filtered_ = 0.0;
  vy_filtered_ = 0.0;
  yaw_filtered_ = 0.0;
  height_filtered_ = params_.height_default;
  gait_phase_ = 0.0;
}

void LocomotionRuntime::set_command(
  const std::string &source,
  GaitMode mode,
  double vx,
  double vy,
  double yaw_rate,
  double height_offset,
  const rclcpp::Time &now)
{
  active_cmd_source_ = source;
  cmd_mode_ = mode;
  target_vx_ = vx;
  target_vy_ = vy;
  target_yaw_rate_ = yaw_rate;
  target_height_offset_ = clamp_value(height_offset, params_.height_offset_min, params_.height_offset_max);

  has_cmd_ = true;
  cmd_time_ = now;
}

RuntimeSnapshot LocomotionRuntime::step(const rclcpp::Time &now)
{
  double dt = (now - last_update_time_).seconds();
  if (dt <= 0.0 || dt > 0.2) dt = 0.02;
  last_update_time_ = now;

  const bool timed_out = (!has_cmd_) || ((now - cmd_time_).seconds() > params_.cmd_timeout_s);
  const GaitMode run_mode = timed_out ? GaitMode::STAND : cmd_mode_;

  if (run_mode != mode_target_) {
    if (run_mode == GaitMode::WALK && mode_target_ != GaitMode::WALK) {
      gait_phase_ = 0.0;
    }

    if (run_mode == GaitMode::STEPPING &&
      (mode_target_ == GaitMode::WALK || mode_target_ == GaitMode::TROT))
    {
      vx_filtered_ = 0.0;
      vy_filtered_ = 0.0;
      yaw_filtered_ = 0.0;
    }

    transition_from_mode_ = mode_current_;
    mode_target_ = run_mode;
    transition_start_time_ = now;
    transition_t_ = 0.0;
    const bool walk_mode_change =
      (transition_from_mode_ == GaitMode::WALK) || (mode_target_ == GaitMode::WALK);
    transition_active_ = (transition_from_mode_ != mode_target_) && !walk_mode_change;
    if (!transition_active_) {
      mode_current_ = mode_target_;
    }
  }

  double vx_target = timed_out ? 0.0 : target_vx_;
  double vy_target = timed_out ? 0.0 : target_vy_;
  double yaw_target = timed_out ? 0.0 : target_yaw_rate_;
  const double height_offset_target = timed_out ? 0.0 : target_height_offset_;
  double height_target = params_.height_default + height_offset_target;

  const bool stepping_vertical_active =
    params_.stepping_strict_vertical && (run_mode == GaitMode::STEPPING);
  if (stepping_vertical_active) {
    vx_target = 0.0;
    vy_target = 0.0;
    yaw_target = 0.0;
  }

  height_target = clamp_value(height_target, params_.height_min, params_.height_max);

  const double vx_lp = lowpass_step(vx_filtered_, vx_target);
  const double vy_lp = lowpass_step(vy_filtered_, vy_target);
  const double yaw_lp = lowpass_step(yaw_filtered_, yaw_target);
  const double h_lp = lowpass_step(height_filtered_, height_target);

  vx_filtered_ = slew_step(vx_filtered_, vx_lp, params_.vx_slew_rate, dt);
  vy_filtered_ = slew_step(vy_filtered_, vy_lp, params_.vy_slew_rate, dt);
  yaw_filtered_ = slew_step(yaw_filtered_, yaw_lp, params_.yaw_slew_rate, dt);
  height_filtered_ = slew_step(height_filtered_, h_lp, params_.height_slew_rate, dt);
  height_filtered_ = clamp_value(height_filtered_, params_.height_min, params_.height_max);

  double blend = 1.0;
  if (transition_active_) {
    const double duration = std::max(params_.gait_transition_duration_s, 1e-4);
    transition_t_ = clamp_value((now - transition_start_time_).seconds() / duration, 0.0, 1.0);
    blend = smooth_step(transition_t_);

    if (transition_t_ >= 1.0) {
      transition_active_ = false;
      mode_current_ = mode_target_;
    }
  } else {
    transition_t_ = 1.0;
    mode_current_ = mode_target_;
  }

  const GaitMode phase_mode_from = transition_active_ ? transition_from_mode_ : mode_current_;
  const GaitMode phase_mode_to = mode_target_;
  const GaitConfig cfg_from = gait_config(params_, phase_mode_from);
  const GaitConfig cfg_to = gait_config(params_, phase_mode_to);
  const double step_freq_blend = transition_active_ ? lerp(cfg_from.step_freq, cfg_to.step_freq, blend) : cfg_to.step_freq;

  if (step_freq_blend > 1e-4) {
    gait_phase_ = wrap_to_2pi(gait_phase_ + 2.0 * kPI * step_freq_blend * dt);
  }

  RuntimeSnapshot snapshot;
  snapshot.timed_out = timed_out;
  snapshot.transition_active = transition_active_;
  snapshot.transition_t = transition_t_;
  snapshot.transition_blend = blend;
  snapshot.mode_current = mode_current_;
  snapshot.mode_target = mode_target_;
  snapshot.vx = vx_filtered_;
  snapshot.vy = vy_filtered_;
  snapshot.yaw = yaw_filtered_;
  snapshot.height = height_filtered_;
  snapshot.gait_phase = gait_phase_;
  snapshot.active_cmd_source = active_cmd_source_;
  return snapshot;
}

double LocomotionRuntime::lowpass_step(double current, double target) const
{
  const double alpha = clamp_value(params_.command_filter_alpha, 0.0, 1.0);
  return current + alpha * (target - current);
}

double LocomotionRuntime::slew_step(double current, double target, double rate, double dt)
{
  const double max_delta = std::max(rate, 0.0) * std::max(dt, 1e-4);
  const double delta = clamp_value(target - current, -max_delta, max_delta);
  return current + delta;
}

}  // namespace locomotion
}  // namespace motor_ros2
