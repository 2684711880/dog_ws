#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include "motor_ros2/leg_model.h"

namespace lm = motor_ros2::leg_model;

class PoseHoldTool : public rclcpp::Node
{
public:
  PoseHoldTool()
  : Node("pose_hold_tool")
  {
    declare_parameters();
    load_scalar_params();

    current_cmd_.fill(0.0);
    start_cmd_.fill(0.0);
    goal_cmd_.fill(0.0);

    joint_names_ = lm::joint_names_vector();
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    param_cb_handle_ = add_on_set_parameters_callback(
      std::bind(&PoseHoldTool::on_set_parameters, this, std::placeholders::_1));

    if (!refresh_goal_from_parameters(true)) {
      RCLCPP_ERROR(get_logger(), "Initial goal check failed. Exit.");
      startup_abort_ = true;
      return;
    }

    if (print_only_) {
      RCLCPP_INFO(get_logger(), "print_only=true, exit without publishing.");
      startup_abort_ = true;
      return;
    }

    boot_time_ = now();
    last_pub_time_ = boot_time_;
    trajectory_started_ = false;

    timer_ = create_wall_timer(
      std::chrono::milliseconds(5),
      std::bind(&PoseHoldTool::on_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "pose_hold_tool started. wait_graph=%.2fs duration=%.2fs hold_after_reach=%d publish_hz=%.1f",
      graph_wait_s_, duration_s_, hold_after_reach_ ? 1 : 0, publish_hz_);
  }

private:
  static constexpr double kPi = 3.14159265358979323846;

  void declare_parameters()
  {
    declare_parameter<double>("j1", 0.0);
    declare_parameter<double>("j2", 1.066);
    declare_parameter<double>("j3", 0.087);

    const double nan = std::numeric_limits<double>::quiet_NaN();
    declare_parameter<double>("lf_j1", nan);
    declare_parameter<double>("lf_j2", nan);
    declare_parameter<double>("lf_j3", nan);
    declare_parameter<double>("rf_j1", nan);
    declare_parameter<double>("rf_j2", nan);
    declare_parameter<double>("rf_j3", nan);
    declare_parameter<double>("lb_j1", nan);
    declare_parameter<double>("lb_j2", nan);
    declare_parameter<double>("lb_j3", nan);
    declare_parameter<double>("rb_j1", nan);
    declare_parameter<double>("rb_j2", nan);
    declare_parameter<double>("rb_j3", nan);

    declare_parameter<double>("duration_s", 2.0);
    declare_parameter<double>("publish_hz", 100.0);
    declare_parameter<bool>("hold_after_reach", true);
    declare_parameter<double>("graph_wait_s", 1.0);
    declare_parameter<bool>("print_only", false);
  }

  void load_scalar_params()
  {
    j1_ = get_parameter("j1").as_double();
    j2_ = get_parameter("j2").as_double();
    j3_ = get_parameter("j3").as_double();

    duration_s_ = get_parameter("duration_s").as_double();
    publish_hz_ = get_parameter("publish_hz").as_double();
    hold_after_reach_ = get_parameter("hold_after_reach").as_bool();
    graph_wait_s_ = get_parameter("graph_wait_s").as_double();
    print_only_ = get_parameter("print_only").as_bool();

    duration_s_ = std::max(duration_s_, 1e-3);
    publish_hz_ = std::max(publish_hz_, 1.0);
    graph_wait_s_ = std::max(graph_wait_s_, 0.0);
  }

  static double rad2deg(double rad)
  {
    return rad * 180.0 / kPi;
  }

  bool is_finite(double v) const
  {
    return std::isfinite(v);
  }

  std::string leg_tag(std::size_t leg_idx) const
  {
    switch (leg_idx) {
      case 0: return "lf";
      case 1: return "rf";
      case 2: return "lb";
      case 3: return "rb";
      default: return "??";
    }
  }

  double leg_value(std::size_t leg_idx, std::size_t joint_idx, double global_v) const
  {
    const std::string tag = leg_tag(leg_idx);
    const std::string name = tag + "_j" + std::to_string(joint_idx + 1);
    const double v = get_parameter(name).as_double();
    return is_finite(v) ? v : global_v;
  }

  bool validate_goal_against_limits(const std::array<double, lm::kJointCount> &goal) const
  {
    bool ok = true;
    for (std::size_t i = 0; i < lm::kJointCount; ++i) {
      const double cmd = goal[i];
      const double lo = static_cast<double>(lm::kJointLimits[i].min_rad);
      const double hi = static_cast<double>(lm::kJointLimits[i].max_rad);
      if (cmd < lo || cmd > hi) {
        RCLCPP_ERROR(
          get_logger(),
          "Limit violation %s: cmd=%.6f rad, limit=[%.6f, %.6f]",
          lm::kJointNames[i], cmd, lo, hi);
        ok = false;
      }
    }
    return ok;
  }

  void print_goal(const std::array<double, lm::kJointCount> &goal) const
  {
    RCLCPP_INFO(get_logger(), "Target joint commands (command space), limits in same space:");
    for (std::size_t i = 0; i < lm::kJointCount; ++i) {
      const double cmd = goal[i];
      RCLCPP_INFO(
        get_logger(),
        "  %-10s cmd=% .6f rad (% .2f deg), limit=[%.2f, %.2f]",
        lm::kJointNames[i], cmd, rad2deg(cmd),
        static_cast<double>(lm::kJointLimits[i].min_rad),
        static_cast<double>(lm::kJointLimits[i].max_rad));
    }
  }

  bool has_other_joint_state_publishers() const
  {
    const auto infos = get_publishers_info_by_topic("/joint_states");
    int other = 0;
    for (const auto &info : infos) {
      const bool self =
        (info.node_name() == get_name()) &&
        (info.node_namespace() == get_namespace());
      if (!self) {
        ++other;
        RCLCPP_WARN(
          get_logger(),
          "Found /joint_states publisher: node=%s ns=%s topic_type=%s",
          info.node_name().c_str(), info.node_namespace().c_str(), info.topic_type().c_str());
      }
    }
    return other > 0;
  }

  void start_transition_to_goal(const rclcpp::Time &t)
  {
    start_cmd_ = current_cmd_;
    start_time_ = t;
    trajectory_started_ = true;
    reached_ = false;

    RCLCPP_INFO(
      get_logger(),
      "Start transition to goal: duration=%.3fs",
      duration_s_);
  }

  std::array<double, lm::kJointCount> build_goal(
    double j1,
    double j2,
    double j3,
    const std::array<double, lm::kJointCount> &overrides) const
  {
    std::array<double, lm::kJointCount> out{};
    for (std::size_t leg_idx = 0; leg_idx < lm::kLegCount; ++leg_idx) {
      const std::size_t i1 = lm::flat_index(static_cast<lm::LegId>(leg_idx), lm::JointId::J1);
      const std::size_t i2 = lm::flat_index(static_cast<lm::LegId>(leg_idx), lm::JointId::J2);
      const std::size_t i3 = lm::flat_index(static_cast<lm::LegId>(leg_idx), lm::JointId::J3);
      out[i1] = is_finite(overrides[i1]) ? overrides[i1] : j1;
      out[i2] = is_finite(overrides[i2]) ? overrides[i2] : j2;
      out[i3] = is_finite(overrides[i3]) ? overrides[i3] : j3;
    }
    return out;
  }

  std::array<double, lm::kJointCount> load_overrides_from_params() const
  {
    std::array<double, lm::kJointCount> overrides{};
    overrides.fill(std::numeric_limits<double>::quiet_NaN());
    for (std::size_t leg_idx = 0; leg_idx < lm::kLegCount; ++leg_idx) {
      const std::size_t i1 = lm::flat_index(static_cast<lm::LegId>(leg_idx), lm::JointId::J1);
      const std::size_t i2 = lm::flat_index(static_cast<lm::LegId>(leg_idx), lm::JointId::J2);
      const std::size_t i3 = lm::flat_index(static_cast<lm::LegId>(leg_idx), lm::JointId::J3);
      overrides[i1] = leg_value(leg_idx, 0, std::numeric_limits<double>::quiet_NaN());
      overrides[i2] = leg_value(leg_idx, 1, std::numeric_limits<double>::quiet_NaN());
      overrides[i3] = leg_value(leg_idx, 2, std::numeric_limits<double>::quiet_NaN());
    }
    return overrides;
  }

  bool refresh_goal_from_parameters(bool force_log)
  {
    load_scalar_params();
    const std::array<double, lm::kJointCount> overrides = load_overrides_from_params();
    const std::array<double, lm::kJointCount> new_goal = build_goal(j1_, j2_, j3_, overrides);

    bool changed = false;
    for (std::size_t i = 0; i < lm::kJointCount; ++i) {
      if (std::abs(new_goal[i] - goal_cmd_[i]) > 1e-12) { changed = true; break; }
    }
    goal_cmd_ = new_goal;
    if (force_log || changed) { print_goal(goal_cmd_); }

    if (!validate_goal_against_limits(goal_cmd_)) return false;

    if (changed && trajectory_started_) start_transition_to_goal(now());

    return true;
  }

  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> &params)
  {
    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;

    auto is_joint_double_param = [&](const std::string &name) {
      if (name == "j1" || name == "j2" || name == "j3") return true;
      for (std::size_t leg_idx = 0; leg_idx < lm::kLegCount; ++leg_idx) {
        const std::string tag = leg_tag(leg_idx);
        if (name == tag + "_j1" || name == tag + "_j2" || name == tag + "_j3") return true;
      }
      return false;
    };

    for (const auto &p : params) {
      if (is_joint_double_param(p.get_name())) {
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
          res.successful = false;
          res.reason = p.get_name() + " must be double";
          return res;
        }
      }
      if (p.get_name() == "duration_s") {
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE || p.as_double() <= 1e-3) {
          res.successful = false;
          res.reason = "duration_s must be > 1e-3";
          return res;
        }
      }
      if (p.get_name() == "publish_hz") {
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE || p.as_double() < 1.0) {
          res.successful = false;
          res.reason = "publish_hz must be >= 1.0";
          return res;
        }
      }
      if (p.get_name() == "graph_wait_s") {
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE || p.as_double() < 0.0) {
          res.successful = false;
          res.reason = "graph_wait_s must be >= 0.0";
          return res;
        }
      }
      if (p.get_name() == "hold_after_reach" || p.get_name() == "print_only") {
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
          res.successful = false;
          res.reason = p.get_name() + " must be bool";
          return res;
        }
      }
    }

    std::unordered_map<std::string, rclcpp::Parameter> updates;
    for (const auto &p : params) updates.emplace(p.get_name(), p);

    auto cand_double = [&](const std::string &name) -> double {
      auto it = updates.find(name);
      if (it != updates.end()) return it->second.as_double();
      return get_parameter(name).as_double();
    };

    const double cj1 = cand_double("j1");
    const double cj2 = cand_double("j2");
    const double cj3 = cand_double("j3");

    std::array<double, lm::kJointCount> overrides = load_overrides_from_params();
    for (std::size_t leg_idx = 0; leg_idx < lm::kLegCount; ++leg_idx) {
      const std::string tag = leg_tag(leg_idx);
      const std::size_t i1 = lm::flat_index(static_cast<lm::LegId>(leg_idx), lm::JointId::J1);
      const std::size_t i2 = lm::flat_index(static_cast<lm::LegId>(leg_idx), lm::JointId::J2);
      const std::size_t i3 = lm::flat_index(static_cast<lm::LegId>(leg_idx), lm::JointId::J3);

      const auto it1 = updates.find(tag + "_j1");
      const auto it2 = updates.find(tag + "_j2");
      const auto it3 = updates.find(tag + "_j3");
      if (it1 != updates.end()) overrides[i1] = it1->second.as_double();
      if (it2 != updates.end()) overrides[i2] = it2->second.as_double();
      if (it3 != updates.end()) overrides[i3] = it3->second.as_double();
    }

    const std::array<double, lm::kJointCount> candidate_goal = build_goal(cj1, cj2, cj3, overrides);
    if (!validate_goal_against_limits(candidate_goal)) {
      res.successful = false;
      res.reason = "target violates limits";
      return res;
    }

    need_refresh_from_params_ = true;
    return res;
  }

  void publish_current(const rclcpp::Time &t)
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = t;
    msg.header.frame_id = "base_link";
    msg.name = joint_names_;
    msg.position.assign(current_cmd_.begin(), current_cmd_.end());
    joint_pub_->publish(msg);
    last_pub_time_ = t;
  }

  void on_timer()
  {
    const rclcpp::Time t = now();

    if (need_refresh_from_params_) {
      need_refresh_from_params_ = false;
      if (!refresh_goal_from_parameters(false)) {
        RCLCPP_ERROR(get_logger(), "Target refresh failed after parameter update.");
        rclcpp::shutdown();
        return;
      }
    }

    if (!trajectory_started_) {
      const double waited = (t - boot_time_).seconds();
      if (waited < graph_wait_s_) {
        return;
      }

      if (has_other_joint_state_publishers()) {
        RCLCPP_ERROR(get_logger(), "Detected other /joint_states publishers. Abort to avoid control conflict.");
        rclcpp::shutdown();
        return;
      }

      start_transition_to_goal(t);
    }

    const double pub_dt = (t - last_pub_time_).seconds();
    if (pub_dt < (1.0 / publish_hz_)) {
      return;
    }

    const double alpha = std::min(std::max((t - start_time_).seconds() / duration_s_, 0.0), 1.0);
    for (std::size_t i = 0; i < lm::kJointCount; ++i) {
      current_cmd_[i] = start_cmd_[i] + (goal_cmd_[i] - start_cmd_[i]) * alpha;
    }

    publish_current(t);

    if (!reached_ && alpha >= 1.0) {
      reached_ = true;
      RCLCPP_INFO(get_logger(), "Reached target pose.");
      if (!hold_after_reach_) {
        RCLCPP_INFO(get_logger(), "hold_after_reach=false, exit.");
        rclcpp::shutdown();
      }
    }
  }

public:
  bool startup_abort() const { return startup_abort_; }

private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  std::vector<std::string> joint_names_;

  std::array<double, lm::kJointCount> current_cmd_{};
  std::array<double, lm::kJointCount> start_cmd_{};
  std::array<double, lm::kJointCount> goal_cmd_{};

  double j1_{0.0};
  double j2_{1.066};
  double j3_{0.087};

  double duration_s_{2.0};
  double publish_hz_{100.0};
  bool hold_after_reach_{true};
  double graph_wait_s_{1.0};
  bool print_only_{false};

  bool trajectory_started_{false};
  bool reached_{false};
  bool need_refresh_from_params_{false};
  bool startup_abort_{false};

  rclcpp::Time boot_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_pub_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PoseHoldTool>();
  if (!node->startup_abort()) {
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return 0;
}
