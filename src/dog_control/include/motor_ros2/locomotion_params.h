#pragma once

#include <rclcpp/rclcpp.hpp>

#include "motor_ros2/locomotion_core_types.h"

namespace motor_ros2 {
namespace locomotion {

void declare_locomotion_parameters(rclcpp::Node &node);
LocomotionParams load_locomotion_parameters(rclcpp::Node &node);
LocomotionParams declare_and_load_locomotion_parameters(rclcpp::Node &node);

}  // namespace locomotion
}  // namespace motor_ros2
