from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    input_source = LaunchConfiguration("input_source")

    input_source_arg = DeclareLaunchArgument(
        "input_source",
        default_value="xbox",
        description="Input source: xbox or keyboard",
    )

    core_node = Node(
        package="dog_control",
        executable="locomotion_core_node",
        name="locomotion_core_node",
        output="screen",
        parameters=[{"input_source": input_source}],
    )

    # joy_node: 读取手柄硬件 → 发布 sensor_msgs/Joy
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        condition=IfCondition(PythonExpression(["'", input_source, "' == 'xbox'"])),
    )

    # xbox_input_node: Joy → LocomotionCommand (已移至 dog_teleop)
    xbox_node = Node(
        package="dog_teleop",
        executable="xbox_input_node",
        name="xbox_input_node",
        output="screen",
        condition=IfCondition(PythonExpression(["'", input_source, "' == 'xbox'"])),
    )

    return LaunchDescription([
        input_source_arg,
        core_node,
        joy_node,
        xbox_node,
    ])
