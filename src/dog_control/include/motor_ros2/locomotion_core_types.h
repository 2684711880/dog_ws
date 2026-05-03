#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

#include <dog_msgs/msg/locomotion_command.hpp>

namespace motor_ros2 {
namespace locomotion {

enum class GaitMode : uint8_t
{
  STAND = dog_msgs::msg::LocomotionCommand::GAIT_STAND,
  STEPPING = dog_msgs::msg::LocomotionCommand::GAIT_STEPPING,
  WALK = dog_msgs::msg::LocomotionCommand::GAIT_WALK,
  TROT = dog_msgs::msg::LocomotionCommand::GAIT_TROT
};

struct GaitConfig
{
  double step_height{0.0};
  double step_freq{0.0};
  const char *name{"STAND"};
};

struct LocomotionParams
{
  std::string input_source{"xbox"};
  double cmd_timeout_s{0.25};

  double thigh_length{0.21};
  double shank_length{0.21};
  double stand_foot_x_front{0.08};
  double stand_foot_x_rear{0.12};
  double stand_foot_y{0.095};
  double hip_x_front{0.15};
  double hip_x_rear{0.15};
  double hip_y{0.05};

  double height_min{0.22};
  double height_max{0.32};
  double height_default{0.305};
  double height_offset_min{-0.05};
  double height_offset_max{0.03};

  double step_height_stand{0.0};
  double step_height_stepping{0.025};
  double step_height_walk{0.045};
  double step_height_trot{0.030};

  double step_freq_stepping{2.0};
  double step_freq_walk{0.833333};
  double step_freq_trot{2.3};

  double joint_limit_deg{105.0};
  double joint3_cmd_min{-0.8};
  double joint3_cmd_max{1.8};
  double joint1_x_coupling_gain{0.0};
  double joint1_yaw_coupling_gain{1.0};

  double yaw_step_gain{0.030};
  double yaw_step_gain_in_place{0.070};
  double in_place_turn_vx_threshold{0.03};
  double in_place_turn_vy_threshold{0.02};

  double max_step_length{0.12};
  double max_step_length_walk{0.06};
  double foot_x_min_front{0.00};
  double foot_x_min_rear{0.00};
  double swing_scale_front{1.0};
  double swing_scale_rear{1.0};
  double walk_duty_factor{0.80};
  double walk_lateral_shift_m{0.018};
  double walk_pre_shift_ratio{0.05};
  bool stepping_strict_vertical{true};
  double stepping_diagonal_skew_rad{0.314};

  double command_filter_alpha{0.40};
  double vx_slew_rate{0.80};
  double vy_slew_rate{0.60};
  double yaw_slew_rate{2.5};
  double height_slew_rate{0.05};
  double gait_transition_duration_s{0.30};
};

constexpr double kPI = 3.14159265358979323846;

template<typename T>
inline T clamp_value(T v, T lo, T hi)
{
  return std::min(std::max(v, lo), hi);
}

inline double wrap_to_2pi(double x)
{
  const double two_pi = 2.0 * kPI;
  while (x >= two_pi) x -= two_pi;
  while (x < 0.0) x += two_pi;
  return x;
}

inline double smooth_step(double t)
{
  const double c = clamp_value(t, 0.0, 1.0);
  return c * c * (3.0 - 2.0 * c);
}

inline double deg2rad(double deg)
{
  return deg * kPI / 180.0;
}

template<typename T>
inline T lerp(T a, T b, double t)
{
  return static_cast<T>(a + (b - a) * t);
}

inline GaitMode to_gait_mode(uint8_t mode)
{
  switch (mode) {
    case dog_msgs::msg::LocomotionCommand::GAIT_STAND:
      return GaitMode::STAND;
    case dog_msgs::msg::LocomotionCommand::GAIT_STEPPING:
      return GaitMode::STEPPING;
    case dog_msgs::msg::LocomotionCommand::GAIT_WALK:
      return GaitMode::WALK;
    case dog_msgs::msg::LocomotionCommand::GAIT_TROT:
      return GaitMode::TROT;
    default:
      return GaitMode::STAND;
  }
}

inline GaitConfig gait_config(const LocomotionParams &params, GaitMode mode)
{
  switch (mode) {
    case GaitMode::STAND:
      return {params.step_height_stand, 0.0, "STAND"};
    case GaitMode::STEPPING:
      return {params.step_height_stepping, params.step_freq_stepping, "STEPPING"};
    case GaitMode::WALK:
      return {params.step_height_walk, params.step_freq_walk, "WALK"};
    case GaitMode::TROT:
      return {params.step_height_trot, params.step_freq_trot, "TROT"};
    default:
      return {params.step_height_stand, 0.0, "STAND"};
  }
}

}  // namespace locomotion
}  // namespace motor_ros2
