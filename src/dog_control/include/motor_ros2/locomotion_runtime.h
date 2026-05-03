#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>

#include "motor_ros2/locomotion_core_types.h"

namespace motor_ros2 {
namespace locomotion {

struct RuntimeSnapshot
{
  bool timed_out{true};
  bool transition_active{false};
  double transition_t{1.0};
  double transition_blend{1.0};

  GaitMode mode_current{GaitMode::STAND};
  GaitMode mode_target{GaitMode::STAND};

  double vx{0.0};
  double vy{0.0};
  double yaw{0.0};
  double height{0.0};
  double gait_phase{0.0};

  std::string active_cmd_source{"none"};
};

class LocomotionRuntime
{
public:
  explicit LocomotionRuntime(const LocomotionParams &params);

  void reset(const rclcpp::Time &now);

  void set_command(
    const std::string &source,
    GaitMode mode,
    double vx,
    double vy,
    double yaw_rate,
    double height_offset,
    const rclcpp::Time &now);

  RuntimeSnapshot step(const rclcpp::Time &now);

private:
  double lowpass_step(double current, double target) const;
  static double slew_step(double current, double target, double rate, double dt);

private:
  LocomotionParams params_;

  bool has_cmd_{false};
  std::string active_cmd_source_{"none"};
  GaitMode cmd_mode_{GaitMode::STAND};
  GaitMode mode_current_{GaitMode::STAND};
  GaitMode mode_target_{GaitMode::STAND};
  GaitMode transition_from_mode_{GaitMode::STAND};

  bool transition_active_{false};
  double transition_t_{1.0};
  rclcpp::Time transition_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};

  double target_vx_{0.0};
  double target_vy_{0.0};
  double target_yaw_rate_{0.0};
  double target_height_offset_{0.0};

  double vx_filtered_{0.0};
  double vy_filtered_{0.0};
  double yaw_filtered_{0.0};
  double height_filtered_{0.0};
  double gait_phase_{0.0};
};

}  // namespace locomotion
}  // namespace motor_ros2
