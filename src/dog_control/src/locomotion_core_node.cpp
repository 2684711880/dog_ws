#include <rclcpp/rclcpp.hpp>

#include <dog_msgs/msg/locomotion_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

#include <string>
#include <vector>

#include "motor_ros2/leg_model.h"
#include "motor_ros2/locomotion_core_types.h"
#include "motor_ros2/locomotion_params.h"
#include "motor_ros2/locomotion_planner.h"
#include "motor_ros2/locomotion_runtime.h"

using namespace std::chrono_literals;
namespace lm = motor_ros2::leg_model;
namespace lc = motor_ros2::locomotion;

class LocomotionCoreNode : public rclcpp::Node
{
public:
  LocomotionCoreNode()
  : Node("locomotion_core_node"),
    params_(lc::declare_and_load_locomotion_parameters(*this)),
    runtime_(params_),
    planner_(params_)
  {
    joint_names_ = lm::joint_names_vector();

    planner_.bind_logger(get_logger(), get_clock());
    if (!planner_.initialize_stand_offsets()) {
      RCLCPP_ERROR(get_logger(), "Failed to compute stand offsets from IK.");
      rclcpp::shutdown();
      return;
    }

    runtime_.reset(now());

    cmd_sub_ = create_subscription<dog_msgs::msg::LocomotionCommand>(
      "/locomotion_cmd",
      50,
      std::bind(&LocomotionCoreNode::cmd_callback, this, std::placeholders::_1));
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 20);
    gait_pub_ = create_publisher<std_msgs::msg::String>("/gait_params", 20);

    timer_ = create_wall_timer(5ms, std::bind(&LocomotionCoreNode::timer_callback, this));

    RCLCPP_INFO(
      get_logger(),
      "locomotion_core_node started. input_source=%s cmd_timeout=%.2f",
      params_.input_source.c_str(), params_.cmd_timeout_s);
  }

private:
  void cmd_callback(const dog_msgs::msg::LocomotionCommand::SharedPtr msg)
  {
    if (!msg) return;

    if (msg->source != params_.input_source) {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignore cmd from source='%s' (active source='%s')",
        msg->source.c_str(), params_.input_source.c_str());
      return;
    }

    runtime_.set_command(
      msg->source,
      lc::to_gait_mode(msg->gait_mode),
      static_cast<double>(msg->vx),
      static_cast<double>(msg->vy),
      static_cast<double>(msg->yaw_rate),
      static_cast<double>(msg->height),
      now());
  }

  void timer_callback()
  {
    const rclcpp::Time now_t = now();
    const lc::RuntimeSnapshot snapshot = runtime_.step(now_t);

    lm::QuadJoints<double> joints = planner_.compute(
      snapshot.mode_current,
      snapshot.gait_phase,
      snapshot.vx,
      snapshot.vy,
      snapshot.yaw,
      snapshot.height);

    if (snapshot.transition_active) {
      const lm::QuadJoints<double> joints_to = planner_.compute(
        snapshot.mode_target,
        snapshot.gait_phase,
        snapshot.vx,
        snapshot.vy,
        snapshot.yaw,
        snapshot.height);

      for (std::size_t li = 0; li < lm::kLegCount; ++li) {
        const lm::LegId leg = static_cast<lm::LegId>(li);
        for (std::size_t ji = 0; ji < lm::kJointsPerLeg; ++ji) {
          const lm::JointId joint = static_cast<lm::JointId>(ji);
          joints.at(leg).at(joint) = lc::lerp(
            joints.at(leg).at(joint),
            joints_to.at(leg).at(joint),
            snapshot.transition_blend);
        }
      }
    }

    sensor_msgs::msg::JointState js;
    js.header.stamp = now_t;
    js.header.frame_id = "base_link";
    js.name = joint_names_;
    js.position = lm::flatten_to_vector(joints);
    joint_pub_->publish(js);

    const lc::GaitConfig cfg = lc::gait_config(params_, snapshot.mode_target);
    std_msgs::msg::String gait_msg;
    gait_msg.data = cfg.name;
    gait_pub_->publish(gait_msg);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "src=%s mode=%s trans=%d a=%.2f vx=%.3f vy=%.3f yaw=%.3f clearance=%.3f j3=[%.2f,%.2f] timeout=%d",
      snapshot.active_cmd_source.c_str(),
      cfg.name,
      snapshot.transition_active ? 1 : 0,
      snapshot.transition_t,
      snapshot.vx,
      snapshot.vy,
      snapshot.yaw,
      snapshot.height,
      params_.joint3_cmd_min,
      params_.joint3_cmd_max,
      snapshot.timed_out ? 1 : 0);
  }

private:
  lc::LocomotionParams params_;
  lc::LocomotionRuntime runtime_;
  lc::LocomotionPlanner planner_;

  rclcpp::Subscription<dog_msgs::msg::LocomotionCommand>::SharedPtr cmd_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gait_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<std::string> joint_names_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocomotionCoreNode>());
  rclcpp::shutdown();
  return 0;
}
