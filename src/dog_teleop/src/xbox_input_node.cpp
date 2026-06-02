#include <rclcpp/rclcpp.hpp>

#include <dog_msgs/msg/locomotion_command.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

class XboxInputNode : public rclcpp::Node
{
public:
  XboxInputNode()
  : Node("xbox_input_node")
  {
    declare_and_load_params();

    cmd_pub_ = create_publisher<dog_msgs::msg::LocomotionCommand>("/locomotion_cmd", 20);
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 20, std::bind(&XboxInputNode::joy_callback, this, std::placeholders::_1));

    height_offset_cmd_ = 0.0;
    mode_cmd_ = dog_msgs::msg::LocomotionCommand::GAIT_STAND;

    RCLCPP_INFO(
      get_logger(),
      "xbox_input_node started: axis1=-vx axis0=-vy axis7=height_offset(+1 raises body, via lower clearance) axis6=yaw(right:+) axis5<thr enables A/B/X/Y");
  }

private:
  template<typename T>
  static T clamp_value(T v, T lo, T hi)
  {
    return std::min(std::max(v, lo), hi);
  }

  double axis_with_deadband(double x) const
  {
    if (std::abs(x) < joy_deadband_) return 0.0;
    return x;
  }

  static double axis_safe(const sensor_msgs::msg::Joy::SharedPtr & msg, int idx)
  {
    if (!msg || idx < 0 || static_cast<size_t>(idx) >= msg->axes.size()) return 0.0;
    return msg->axes[static_cast<size_t>(idx)];
  }

  static int button_safe(const sensor_msgs::msg::Joy::SharedPtr & msg, int idx)
  {
    if (!msg || idx < 0 || static_cast<size_t>(idx) >= msg->buttons.size()) return 0;
    return msg->buttons[static_cast<size_t>(idx)];
  }

  static double scaled_signed_axis(double axis, double positive_max, double negative_max)
  {
    return axis * ((axis >= 0.0) ? positive_max : negative_max);
  }

  bool rising_edge(const sensor_msgs::msg::Joy::SharedPtr & msg, int idx) const
  {
    if (idx < 0) return false;
    const bool cur = button_safe(msg, idx) != 0;
    const size_t uidx = static_cast<size_t>(idx);
    const bool prev = (uidx < prev_buttons_.size()) ? (prev_buttons_[uidx] != 0) : false;
    return cur && !prev;
  }

  void declare_and_load_params()
  {
    declare_parameter<double>("joy_deadband", 0.08);
    declare_parameter<double>("mode_gate_threshold", -0.5);
    declare_parameter<bool>("mode_gate_less_than", true);

    declare_parameter<double>("vx_max_walk_forward", 0.18);
    declare_parameter<double>("vx_max_walk_backward", 0.18);
    declare_parameter<double>("vx_max_trot_forward", 0.42);
    declare_parameter<double>("vx_max_trot_backward", 0.42);
    declare_parameter<double>("vy_max_walk", 0.03);
    declare_parameter<double>("vy_max_trot", 0.10);
    declare_parameter<double>("yaw_rate_max", 0.35);

    declare_parameter<double>("height_min", 0.24);
    declare_parameter<double>("height_max", 0.32);
    declare_parameter<double>("height_default", 0.31);
    declare_parameter<double>("height_step_per_input", 0.005);

    joy_deadband_ = get_parameter("joy_deadband").as_double();
    mode_gate_threshold_ = get_parameter("mode_gate_threshold").as_double();
    mode_gate_less_than_ = get_parameter("mode_gate_less_than").as_bool();

    vx_max_walk_forward_ = get_parameter("vx_max_walk_forward").as_double();
    vx_max_walk_backward_ = get_parameter("vx_max_walk_backward").as_double();
    vx_max_trot_forward_ = get_parameter("vx_max_trot_forward").as_double();
    vx_max_trot_backward_ = get_parameter("vx_max_trot_backward").as_double();
    vy_max_walk_ = get_parameter("vy_max_walk").as_double();
    vy_max_trot_ = get_parameter("vy_max_trot").as_double();
    yaw_rate_max_ = get_parameter("yaw_rate_max").as_double();

    height_min_ = get_parameter("height_min").as_double();
    height_max_ = get_parameter("height_max").as_double();
    height_default_ = get_parameter("height_default").as_double();
    height_step_per_input_ = get_parameter("height_step_per_input").as_double();
    height_offset_min_ = height_min_ - height_default_;
    height_offset_max_ = height_max_ - height_default_;
  }

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (!msg) return;

    const double axis_vx = axis_with_deadband(axis_safe(msg, 1));
    const double axis_vy = -axis_with_deadband(axis_safe(msg, 0));
    const double axis_height = axis_with_deadband(axis_safe(msg, 7));
    const double axis_yaw = axis_with_deadband(axis_safe(msg, 6));
    const double gate_axis = axis_safe(msg, 5);

    if (std::abs(axis_height) > 1e-6) {
      height_offset_cmd_ -= axis_height * height_step_per_input_;
      height_offset_cmd_ = clamp_value(height_offset_cmd_, height_offset_min_, height_offset_max_);
    }

    const bool gate_ok = mode_gate_less_than_
      ? (gate_axis < mode_gate_threshold_)
      : (gate_axis > mode_gate_threshold_);

    if (gate_ok) {
      if (rising_edge(msg, 0)) mode_cmd_ = dog_msgs::msg::LocomotionCommand::GAIT_STAND;
      if (rising_edge(msg, 1)) mode_cmd_ = dog_msgs::msg::LocomotionCommand::GAIT_STEPPING;
      if (rising_edge(msg, 2)) mode_cmd_ = dog_msgs::msg::LocomotionCommand::GAIT_WALK;
      if (rising_edge(msg, 3)) mode_cmd_ = dog_msgs::msg::LocomotionCommand::GAIT_TROT;
    }

    double vx = 0.0;
    double vy = 0.0;
    const double axis_vy_shaped = axis_vy * std::abs(axis_vy);
    if (mode_cmd_ == dog_msgs::msg::LocomotionCommand::GAIT_WALK) {
      vx = scaled_signed_axis(axis_vx, vx_max_walk_forward_, vx_max_walk_backward_);
      vy = axis_vy_shaped * vy_max_walk_;
    } else if (mode_cmd_ == dog_msgs::msg::LocomotionCommand::GAIT_TROT) {
      vx = scaled_signed_axis(axis_vx, vx_max_trot_forward_, vx_max_trot_backward_);
      vy = axis_vy_shaped * vy_max_trot_;
    }

    const double yaw_rate =
      (mode_cmd_ == dog_msgs::msg::LocomotionCommand::GAIT_STAND) ? 0.0 : axis_yaw * yaw_rate_max_;

    dog_msgs::msg::LocomotionCommand cmd;
    cmd.header.stamp = now();
    cmd.source = "xbox";
    cmd.gait_mode = mode_cmd_;
    cmd.vx = static_cast<float>(vx);
    cmd.vy = static_cast<float>(vy);
    cmd.yaw_rate = static_cast<float>(yaw_rate);
    cmd.height = static_cast<float>(height_offset_cmd_);
    cmd_pub_->publish(cmd);

    prev_buttons_ = msg->buttons;
  }

private:
  rclcpp::Publisher<dog_msgs::msg::LocomotionCommand>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  std::vector<int32_t> prev_buttons_;

  double joy_deadband_{0.08};
  double mode_gate_threshold_{-0.5};
  bool mode_gate_less_than_{true};

  double vx_max_walk_forward_{0.18};
  double vx_max_walk_backward_{0.18};
  double vx_max_trot_forward_{0.42};
  double vx_max_trot_backward_{0.42};
  double vy_max_walk_{0.03};
  double vy_max_trot_{0.10};
  double yaw_rate_max_{0.35};

  double height_min_{0.24};
  double height_max_{0.32};
  double height_default_{0.31};
  double height_offset_min_{-0.04};
  double height_offset_max_{0.04};
  double height_step_per_input_{0.005};

  double height_offset_cmd_{0.0};
  uint8_t mode_cmd_{dog_msgs::msg::LocomotionCommand::GAIT_STAND};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<XboxInputNode>());
  rclcpp::shutdown();
  return 0;
}
