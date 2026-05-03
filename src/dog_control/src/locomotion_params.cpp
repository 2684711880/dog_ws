#include "motor_ros2/locomotion_params.h"

#include <algorithm>

namespace motor_ros2 {
namespace locomotion {

void declare_locomotion_parameters(rclcpp::Node &node)
{
  node.declare_parameter<std::string>("input_source", "xbox");
  node.declare_parameter<double>("cmd_timeout_s", 0.25);

  node.declare_parameter<double>("thigh_length", 0.21);
  node.declare_parameter<double>("shank_length", 0.21);
  node.declare_parameter<double>("stand_foot_x_front", 0.08);
  node.declare_parameter<double>("stand_foot_x_rear", 0.12);
  node.declare_parameter<double>("stand_foot_y", 0.095);
  node.declare_parameter<double>("hip_x_front", 0.15);
  node.declare_parameter<double>("hip_x_rear", 0.15);
  node.declare_parameter<double>("hip_y", 0.05);

  node.declare_parameter<double>("height_min", 0.22);
  node.declare_parameter<double>("height_max", 0.32);
  node.declare_parameter<double>("height_default", 0.305);

  node.declare_parameter<double>("step_height_stand", 0.0);
  node.declare_parameter<double>("step_height_stepping", 0.025);
  node.declare_parameter<double>("step_height_walk", 0.045);
  node.declare_parameter<double>("step_height_trot", 0.030);

  node.declare_parameter<double>("step_freq_stepping", 2.0);
  node.declare_parameter<double>("step_freq_walk", 0.833333);
  node.declare_parameter<double>("step_freq_trot", 2.3);

  node.declare_parameter<double>("joint_limit_deg", 105.0);
  node.declare_parameter<double>("joint3_cmd_min", -0.8);
  node.declare_parameter<double>("joint3_cmd_max", 1.8);
  node.declare_parameter<double>("joint1_x_coupling_gain", 0.0);
  node.declare_parameter<double>("joint1_yaw_coupling_gain", 1.0);
  node.declare_parameter<double>("yaw_step_gain", 0.030);
  node.declare_parameter<double>("yaw_step_gain_in_place", 0.070);
  node.declare_parameter<double>("in_place_turn_vx_threshold", 0.03);
  node.declare_parameter<double>("in_place_turn_vy_threshold", 0.02);
  node.declare_parameter<double>("max_step_length", 0.12);
  node.declare_parameter<double>("max_step_length_walk", 0.06);
  node.declare_parameter<double>("foot_x_min_front", 0.00);
  node.declare_parameter<double>("foot_x_min_rear", 0.00);
  node.declare_parameter<double>("swing_scale_front", 1.0);
  node.declare_parameter<double>("swing_scale_rear", 1.0);
  node.declare_parameter<double>("walk_duty_factor", 0.80);
  node.declare_parameter<double>("walk_lateral_shift_m", 0.018);
  node.declare_parameter<double>("walk_pre_shift_ratio", 0.05);
  node.declare_parameter<bool>("stepping_strict_vertical", true);
  node.declare_parameter<double>("stepping_diagonal_skew_rad", 0.314);

  node.declare_parameter<double>("command_filter_alpha", 0.40);
  node.declare_parameter<double>("vx_slew_rate", 0.80);
  node.declare_parameter<double>("vy_slew_rate", 0.60);
  node.declare_parameter<double>("yaw_slew_rate", 2.5);
  node.declare_parameter<double>("height_slew_rate", 0.05);
  node.declare_parameter<double>("gait_transition_duration_s", 0.30);
}

LocomotionParams load_locomotion_parameters(rclcpp::Node &node)
{
  LocomotionParams params{};

  params.input_source = node.get_parameter("input_source").as_string();
  params.cmd_timeout_s = node.get_parameter("cmd_timeout_s").as_double();

  params.thigh_length = node.get_parameter("thigh_length").as_double();
  params.shank_length = node.get_parameter("shank_length").as_double();
  params.stand_foot_x_front = node.get_parameter("stand_foot_x_front").as_double();
  params.stand_foot_x_rear = node.get_parameter("stand_foot_x_rear").as_double();
  params.stand_foot_y = node.get_parameter("stand_foot_y").as_double();
  params.hip_x_front = std::max(node.get_parameter("hip_x_front").as_double(), 0.0);
  params.hip_x_rear = std::max(node.get_parameter("hip_x_rear").as_double(), 0.0);
  params.hip_y = std::max(node.get_parameter("hip_y").as_double(), 0.0);

  params.height_min = node.get_parameter("height_min").as_double();
  params.height_max = node.get_parameter("height_max").as_double();
  params.height_default = node.get_parameter("height_default").as_double();
  params.height_offset_min = params.height_min - params.height_default;
  params.height_offset_max = params.height_max - params.height_default;

  params.step_height_stand = node.get_parameter("step_height_stand").as_double();
  params.step_height_stepping = node.get_parameter("step_height_stepping").as_double();
  params.step_height_walk = node.get_parameter("step_height_walk").as_double();
  params.step_height_trot = node.get_parameter("step_height_trot").as_double();

  params.step_freq_stepping = node.get_parameter("step_freq_stepping").as_double();
  params.step_freq_walk = node.get_parameter("step_freq_walk").as_double();
  params.step_freq_trot = node.get_parameter("step_freq_trot").as_double();

  params.joint_limit_deg = node.get_parameter("joint_limit_deg").as_double();
  params.joint3_cmd_min = node.get_parameter("joint3_cmd_min").as_double();
  params.joint3_cmd_max = node.get_parameter("joint3_cmd_max").as_double();
  if (params.joint3_cmd_min > params.joint3_cmd_max) {
    RCLCPP_WARN(
      node.get_logger(),
      "joint3_cmd_min(%.3f) > joint3_cmd_max(%.3f), swap them.",
      params.joint3_cmd_min, params.joint3_cmd_max);
    std::swap(params.joint3_cmd_min, params.joint3_cmd_max);
  }

  params.joint1_x_coupling_gain = clamp_value(
    node.get_parameter("joint1_x_coupling_gain").as_double(), 0.0, 1.0);
  params.joint1_yaw_coupling_gain = clamp_value(
    node.get_parameter("joint1_yaw_coupling_gain").as_double(), 0.0, 2.0);

  params.yaw_step_gain = node.get_parameter("yaw_step_gain").as_double();
  params.yaw_step_gain_in_place = node.get_parameter("yaw_step_gain_in_place").as_double();
  params.in_place_turn_vx_threshold = std::max(
    node.get_parameter("in_place_turn_vx_threshold").as_double(), 0.0);
  params.in_place_turn_vy_threshold = std::max(
    node.get_parameter("in_place_turn_vy_threshold").as_double(), 0.0);

  params.max_step_length = node.get_parameter("max_step_length").as_double();
  params.max_step_length_walk = node.get_parameter("max_step_length_walk").as_double();
  params.foot_x_min_front = node.get_parameter("foot_x_min_front").as_double();
  params.foot_x_min_rear = node.get_parameter("foot_x_min_rear").as_double();
  params.swing_scale_front = std::max(node.get_parameter("swing_scale_front").as_double(), 0.0);
  params.swing_scale_rear = std::max(node.get_parameter("swing_scale_rear").as_double(), 0.0);
  params.walk_duty_factor = clamp_value(
    node.get_parameter("walk_duty_factor").as_double(), 0.75, 0.90);
  params.walk_lateral_shift_m = clamp_value(
    node.get_parameter("walk_lateral_shift_m").as_double(), 0.0, 0.025);
  params.walk_pre_shift_ratio = clamp_value(
    node.get_parameter("walk_pre_shift_ratio").as_double(), 0.0, 0.10);
  params.stepping_strict_vertical = node.get_parameter("stepping_strict_vertical").as_bool();
  params.stepping_diagonal_skew_rad = clamp_value(
    node.get_parameter("stepping_diagonal_skew_rad").as_double(), 0.0, kPI / 2.0);

  if (params.foot_x_min_front > params.stand_foot_x_front) {
    RCLCPP_WARN(
      node.get_logger(),
      "foot_x_min_front(%.3f) > stand_foot_x_front(%.3f), clamp to stand_foot_x_front.",
      params.foot_x_min_front, params.stand_foot_x_front);
    params.foot_x_min_front = params.stand_foot_x_front;
  }
  if (params.foot_x_min_rear > params.stand_foot_x_rear) {
    RCLCPP_WARN(
      node.get_logger(),
      "foot_x_min_rear(%.3f) > stand_foot_x_rear(%.3f), clamp to stand_foot_x_rear.",
      params.foot_x_min_rear, params.stand_foot_x_rear);
    params.foot_x_min_rear = params.stand_foot_x_rear;
  }

  params.command_filter_alpha = node.get_parameter("command_filter_alpha").as_double();
  params.vx_slew_rate = node.get_parameter("vx_slew_rate").as_double();
  params.vy_slew_rate = node.get_parameter("vy_slew_rate").as_double();
  params.yaw_slew_rate = node.get_parameter("yaw_slew_rate").as_double();
  params.height_slew_rate = node.get_parameter("height_slew_rate").as_double();
  params.gait_transition_duration_s = std::max(
    node.get_parameter("gait_transition_duration_s").as_double(), 0.0);

  return params;
}

LocomotionParams declare_and_load_locomotion_parameters(rclcpp::Node &node)
{
  declare_locomotion_parameters(node);
  return load_locomotion_parameters(node);
}

}  // namespace locomotion
}  // namespace motor_ros2
