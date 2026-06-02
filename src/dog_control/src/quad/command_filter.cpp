#include "quad/command_filter.h"

#include <algorithm>
#include <cmath>

namespace quad {

namespace {

double clamp(double v, double lo, double hi)
{
  return std::min(std::max(v, lo), hi);
}

}  // namespace

// ---------------------------------------------------------------------------
// 构造 / 参数
// ---------------------------------------------------------------------------
CommandFilter::CommandFilter(const CommandFilterParams &params)
: params_(params)
{
}

void CommandFilter::set_params(const CommandFilterParams &params)
{
  params_ = params;
}

// ---------------------------------------------------------------------------
// 收到新指令
// ---------------------------------------------------------------------------
void CommandFilter::set_command(const MotionCommand &cmd, double timestamp)
{
  raw_cmd_ = cmd;
  raw_cmd_.body_height = clamp(raw_cmd_.body_height, params_.height_min, params_.height_max);
  last_cmd_time_ = timestamp;
  timed_out_ = false;
}

// ---------------------------------------------------------------------------
// 每周期滤波
// ---------------------------------------------------------------------------
MotionCommand CommandFilter::step(double dt)
{
  current_time_ += dt;
  time_since_cmd_ = current_time_ - last_cmd_time_;
  timed_out_ = (time_since_cmd_ > params_.cmd_timeout_s);

  MotionCommand target;
  if (timed_out_) {
    target = MotionCommand{};  // 全零, height = 0.290 默认值
  } else {
    target = raw_cmd_;
  }

  // 低通滤波
  const double vx_lp = lowpass(filtered_.vx, target.vx, params_.alpha);
  const double vy_lp = lowpass(filtered_.vy, target.vy, params_.alpha);
  const double yaw_lp = lowpass(filtered_.yaw_rate, target.yaw_rate, params_.alpha);
  const double h_lp = lowpass(filtered_.body_height, target.body_height, params_.alpha);

  // 斜率限幅
  filtered_.vx = slew(filtered_.vx, vx_lp, params_.vx_slew_rate, dt);
  filtered_.vy = slew(filtered_.vy, vy_lp, params_.vy_slew_rate, dt);
  filtered_.yaw_rate = slew(filtered_.yaw_rate, yaw_lp, params_.yaw_slew_rate, dt);
  filtered_.body_height = slew(filtered_.body_height, h_lp, params_.height_slew_rate, dt);
  filtered_.body_height = clamp(filtered_.body_height, params_.height_min, params_.height_max);

  return filtered_;
}

// ---------------------------------------------------------------------------
// 静态工具
// ---------------------------------------------------------------------------
double CommandFilter::lowpass(double current, double target, double alpha)
{
  const double a = clamp(alpha, 0.0, 1.0);
  return current + a * (target - current);
}

double CommandFilter::slew(double current, double target, double rate, double dt)
{
  const double max_delta = std::max(rate, 0.0) * std::max(dt, 1e-4);
  const double delta = clamp(target - current, -max_delta, max_delta);
  return current + delta;
}

}  // namespace quad
