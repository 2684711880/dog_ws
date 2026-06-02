"""Deprecated: use quad_locomotion.launch.py instead."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_source = LaunchConfiguration("input_source")

    input_source_arg = DeclareLaunchArgument(
        "input_source",
        default_value="xbox",
        description="Input source: xbox or keyboard",
    )

    return LaunchDescription([
        input_source_arg,
        LogInfo(msg=["locomotion_control.launch.py is deprecated. "
                     "Use quad_locomotion.launch.py instead."]),
        Node(
            package="dog_control",
            executable="quad_locomotion_node",
            name="quad_locomotion_node",
            output="screen",
            parameters=[{"input_source": input_source}],
        ),
    ])
