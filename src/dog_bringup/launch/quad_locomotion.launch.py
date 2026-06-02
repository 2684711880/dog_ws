import os

from ament_index_python.packages import get_package_share_directory
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

    config_file = os.path.join(
        get_package_share_directory("dog_control"),
        "config",
        "quad_locomotion.yaml",
    )

    core_node = Node(
        package="dog_control",
        executable="quad_locomotion_node",
        name="quad_locomotion_node",
        output="screen",
        parameters=[config_file, {"input_source": input_source}],
    )

    # joy_node: 读取手柄硬件 → 发布 sensor_msgs/Joy
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        condition=IfCondition(PythonExpression(["'", input_source, "' == 'xbox'"])),
    )

    # xbox_input_node: Joy → LocomotionCommand
    xbox_node = Node(
        package="dog_teleop",
        executable="xbox_input_node",
        name="xbox_input_node",
        output="screen",
        condition=IfCondition(PythonExpression(["'", input_source, "' == 'xbox'"])),
    )

    # keyboard_input_node: 键盘 → LocomotionCommand
    keyboard_node = Node(
        package="dog_teleop",
        executable="keyboard_input_node",
        name="keyboard_input_node",
        output="screen",
        prefix="xterm -e",
        condition=IfCondition(PythonExpression(["'", input_source, "' == 'keyboard'"])),
    )

    return LaunchDescription([
        input_source_arg,
        core_node,
        joy_node,
        xbox_node,
        keyboard_node,
    ])
