#pragma once

#include "quad/types.h"

namespace quad {

// ---------------------------------------------------------------------------
// 指令滤波参数
// ---------------------------------------------------------------------------
struct CommandFilterParams {
  double alpha{0.40};            // 低通滤波系数 (0~1), 越小越平滑
  double vx_slew_rate{0.80};     // vx 变化率限幅, m/s²
  double vy_slew_rate{0.60};     // vy 变化率限幅, m/s²
  double yaw_slew_rate{2.5};     // yaw_rate 变化率限幅, rad/s²
  double height_slew_rate{0.05}; // height 变化率限幅, m/s
  double cmd_timeout_s{0.25};    // 指令超时时间, s
  double height_min{0.22};       // 最低站立高度, m
  double height_max{0.32};       // 最高站立高度, m
};

// ---------------------------------------------------------------------------
// 指令滤波器: 低通 + 斜率限幅 + 超时检测
// ---------------------------------------------------------------------------
class CommandFilter {
public:
  CommandFilter() = default;
  explicit CommandFilter(const CommandFilterParams &params);

  void set_params(const CommandFilterParams &params);

  // 收到新指令时调用
  void set_command(const MotionCommand &cmd, double timestamp);

  // 每周期调用, 返回滤波后的指令
  // 超时则返回 STAND + 零速度
  MotionCommand step(double dt);

  bool is_timed_out() const { return timed_out_; }
  double time_since_last_cmd() const { return time_since_cmd_; }

  // 静态工具函数
  static double lowpass(double current, double target, double alpha);
  static double slew(double current, double target, double rate, double dt);

private:
  CommandFilterParams params_;
  MotionCommand raw_cmd_{};
  MotionCommand filtered_{};
  double last_cmd_time_{0};
  double current_time_{0};
  bool timed_out_{true};
  double time_since_cmd_{0};
};

}  // namespace quad
