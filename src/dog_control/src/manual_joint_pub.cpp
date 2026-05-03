#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("manual_joint_pub");

    // 必须先声明参数
    node->declare_parameter<double>("pos", 0.0);

    auto pub = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    rclcpp::Rate rate(10); // 10Hz

    RCLCPP_INFO(node->get_logger(), "Use: ros2 param set /manual_joint_pub pos <rad>");

    while (rclcpp::ok()) {
        // 关键：处理参数服务请求（不加这行，ros2 param set/list 经常会失败）
        rclcpp::spin_some(node);

        double pos = node->get_parameter("pos").as_double();

        sensor_msgs::msg::JointState msg;
        msg.header.stamp = node->now();
        msg.name = {"LF_Joint_3"};
        msg.position = {pos};

        pub->publish(msg);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
