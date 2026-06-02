// sim_bridge_node: 将 quad_locomotion_node 的 /joint_states 转发给
// forward_position_controller，同时应用 kJointSigns 符号校正。
// 仅用于 Gazebo 仿真，不影响实物控制。

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <array>
#include <string>
#include <vector>

#include "motor_ros2/leg_model.h"

namespace lm = motor_ros2::leg_model;

class SimBridgeNode : public rclcpp::Node
{
public:
  SimBridgeNode()
  : Node("sim_bridge_node")
  {
    sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 20,
      std::bind(&SimBridgeNode::js_callback, this, std::placeholders::_1));

    pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/forward_position_controller/commands", 20);

    RCLCPP_INFO(get_logger(), "sim_bridge_node started");
  }

private:
  void js_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (!msg || msg->position.size() != lm::kJointCount) {
      return;
    }

    // 验证关节名称顺序
    if (!lm::has_canonical_joint_order(msg->name)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Joint name order mismatch, skipping");
      return;
    }

    // 应用符号校正：command space → physical space
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.resize(lm::kJointCount);
    for (std::size_t i = 0; i < lm::kJointCount; ++i) {
      cmd.data[i] = msg->position[i] * static_cast<double>(lm::kJointSigns[i]);
    }

    pub_->publish(cmd);
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
