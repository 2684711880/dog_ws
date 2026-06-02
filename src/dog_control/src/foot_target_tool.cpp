#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cctype>
#include <string>
#include <vector>

#include "motor_ros2/leg_model.h"
#include "quad/leg_kinematics.h"
#include "quad/types.h"

namespace lm = motor_ros2::leg_model;

namespace {

constexpr double kPi = 3.14159265358979323846;

std::string upper(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
    return static_cast<char>(std::toupper(c));
  });
  return s;
}

bool parse_leg(const std::string &input, lm::LegId &leg)
{
  const std::string tag = upper(input);
  if (tag == "LF") {
    leg = lm::LegId::LF;
    return true;
  }
  if (tag == "RF") {
    leg = lm::LegId::RF;
    return true;
  }
  if (tag == "LB") {
    leg = lm::LegId::LB;
    return true;
  }
  if (tag == "RB") {
    leg = lm::LegId::RB;
    return true;
  }
  return false;
}

std::string leg_name(lm::LegId leg)
{
  switch (leg) {
    case lm::LegId::LF: return "LF";
    case lm::LegId::RF: return "RF";
    case lm::LegId::LB: return "LB";
    case lm::LegId::RB: return "RB";
    default: return "??";
  }
}

double rad2deg(double rad)
{
  return rad * 180.0 / kPi;
}

double smooth_step(double t)
{
  const double c = std::min(std::max(t, 0.0), 1.0);
  return c * c * (3.0 - 2.0 * c);
}

std::array<double, 3> to_array(const lm::LegJoints<double> &q)
{
  return {q.j1, q.j2, q.j3};
}

lm::LegJoints<double> interpolate(const lm::LegJoints<double> &a,
                                  const lm::LegJoints<double> &b,
                                  double ratio)
{
  const double s = smooth_step(ratio);
  return {
    a.j1 + (b.j1 - a.j1) * s,
    a.j2 + (b.j2 - a.j2) * s,
    a.j3 + (b.j3 - a.j3) * s
  };
}

}  // namespace

class FootTargetTool : public rclcpp::Node
{
public:
  FootTargetTool()
  : Node("foot_target_tool"),
    kin_(quad::make_default_leg_configs())
  {
    declare_parameter<std::string>("leg", "LF");
    declare_parameter<double>("x", 0.06);
    declare_parameter<double>("y", 0.095);
    declare_parameter<double>("z", -0.290);
    declare_parameter<double>("duration_s", 1.0);
    declare_parameter<double>("publish_hz", 100.0);
    declare_parameter<bool>("hold_after_reach", true);
    declare_parameter<bool>("print_only", false);

    const std::string leg_param = get_parameter("leg").as_string();
    if (!parse_leg(leg_param, leg_)) {
      RCLCPP_ERROR(
        get_logger(),
        "Invalid leg='%s'. Expected one of LF/RF/LB/RB.",
        leg_param.c_str());
      should_spin_ = false;
      startup_error_ = true;
      return;
    }

    target_foot_.x = get_parameter("x").as_double();
    target_foot_.y = get_parameter("y").as_double();
    target_foot_.z = get_parameter("z").as_double();
    duration_s_ = std::max(get_parameter("duration_s").as_double(), 1e-3);
    publish_hz_ = std::max(get_parameter("publish_hz").as_double(), 1.0);
    hold_after_reach_ = get_parameter("hold_after_reach").as_bool();
    print_only_ = get_parameter("print_only").as_bool();

    if (!kin_.inverse_kinematics(leg_, target_foot_, target_q_)) {
      RCLCPP_ERROR(
        get_logger(),
        "%s target foot xyz in J1 frame is unreachable: x=%.6f y=%.6f z=%.6f",
        leg_name(leg_).c_str(), target_foot_.x, target_foot_.y, target_foot_.z);
      should_spin_ = false;
      startup_error_ = true;
      return;
    }

    const bool limits_ok = validate_against_limits(target_q_);
    print_target(target_q_, limits_ok);
    if (!limits_ok) {
      should_spin_ = false;
      startup_error_ = true;
      return;
    }

    if (print_only_) {
      RCLCPP_INFO(get_logger(), "print_only=true, exit without publishing.");
      should_spin_ = false;
      return;
    }

    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    fill_joint_names();
    boot_time_ = now();

    const auto period = std::chrono::duration<double>(1.0 / publish_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&FootTargetTool::on_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "foot_target_tool publishing %s only: duration=%.3fs hold_after_reach=%d publish_hz=%.1f",
      leg_name(leg_).c_str(), duration_s_, hold_after_reach_ ? 1 : 0, publish_hz_);
  }

  bool should_spin() const
  {
    return should_spin_;
  }

  int exit_code() const
  {
    return startup_error_ ? 1 : 0;
  }

private:
  bool validate_against_limits(const lm::LegJoints<double> &q) const
  {
    bool ok = true;
    const auto values = to_array(q);
    for (std::size_t ji = 0; ji < lm::kJointsPerLeg; ++ji) {
      const auto joint = static_cast<lm::JointId>(ji);
      const std::size_t idx = lm::flat_index(leg_, joint);
      const double lo = static_cast<double>(lm::kJointLimits[idx].min_rad);
      const double hi = static_cast<double>(lm::kJointLimits[idx].max_rad);
      if (values[ji] < lo || values[ji] > hi) {
        RCLCPP_ERROR(
          get_logger(),
          "Limit violation %s: cmd=%.6f rad, limit=[%.6f, %.6f]",
          lm::kJointNames[idx], values[ji], lo, hi);
        ok = false;
      }
    }
    return ok;
  }

  void print_target(const lm::LegJoints<double> &q, bool limits_ok) const
  {
    const auto fk = kin_.forward_kinematics(leg_, q);
    RCLCPP_INFO(
      get_logger(),
      "%s foot target in J1 frame: x=%.6f y=%.6f z=%.6f",
      leg_name(leg_).c_str(), target_foot_.x, target_foot_.y, target_foot_.z);
    RCLCPP_INFO(
      get_logger(),
      "IK joint-command result: j1=% .6f rad (% .2f deg), j2=% .6f rad (% .2f deg), j3=% .6f rad (% .2f deg)",
      q.j1, rad2deg(q.j1), q.j2, rad2deg(q.j2), q.j3, rad2deg(q.j3));
    RCLCPP_INFO(
      get_logger(),
      "FK check in J1 frame: x=%.6f y=%.6f z=%.6f err=%.6f m",
      fk.x, fk.y, fk.z, (fk - target_foot_).norm());

    const auto values = to_array(q);
    for (std::size_t ji = 0; ji < lm::kJointsPerLeg; ++ji) {
      const auto joint = static_cast<lm::JointId>(ji);
      const std::size_t idx = lm::flat_index(leg_, joint);
      RCLCPP_INFO(
        get_logger(),
        "  %-10s cmd=% .6f rad (% .2f deg), limit=[%.3f, %.3f]",
        lm::kJointNames[idx], values[ji], rad2deg(values[ji]),
        static_cast<double>(lm::kJointLimits[idx].min_rad),
        static_cast<double>(lm::kJointLimits[idx].max_rad));
    }

    if (!limits_ok) {
      RCLCPP_ERROR(get_logger(), "Target rejected because one or more joints exceed limits.");
    }
  }

  void fill_joint_names()
  {
    joint_names_.clear();
    for (std::size_t ji = 0; ji < lm::kJointsPerLeg; ++ji) {
      const auto joint = static_cast<lm::JointId>(ji);
      joint_names_.push_back(lm::kJointNames[lm::flat_index(leg_, joint)]);
    }
  }

  void on_timer()
  {
    const double elapsed = (now() - boot_time_).seconds();
    if (elapsed > duration_s_ && !hold_after_reach_) {
      RCLCPP_INFO(get_logger(), "Target reached and hold_after_reach=false, shutdown.");
      rclcpp::shutdown();
      return;
    }

    const double ratio = elapsed / duration_s_;
    const lm::LegJoints<double> q = interpolate(start_q_, target_q_, ratio);

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.name = joint_names_;
    msg.position = {q.j1, q.j2, q.j3};
    joint_pub_->publish(msg);
  }

  quad::LegKinematics kin_;
  lm::LegId leg_{lm::LegId::LF};
  quad::Vec3 target_foot_{};
  lm::LegJoints<double> start_q_{};
  lm::LegJoints<double> target_q_{};
  std::vector<std::string> joint_names_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time boot_time_{0, 0, RCL_ROS_TIME};
  double duration_s_{1.0};
  double publish_hz_{100.0};
  bool hold_after_reach_{true};
  bool print_only_{false};
  bool should_spin_{true};
  bool startup_error_{false};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FootTargetTool>();
  if (node->should_spin()) {
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return node->exit_code();
}
