# gazebo_sim.launch.py — 四足机器人 Gazebo 仿真启动文件
# 启动 Gazebo + ros2_control + quad_locomotion_node + 键盘控制

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_control = get_package_share_directory("dog_control")
    urdf_path = os.path.join(pkg_control, "urdf", "dog.urdf")
    config_path = os.path.join(pkg_control, "config", "quad_locomotion.yaml")

    # 读取 URDF
    robot_description = ParameterValue(Command(["cat ", urdf_path]), value_type=str)

    # 1. Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gazebo.launch.py",
            )
        ),
        launch_arguments={"verbose": "false"}.items(),
    )

    # 2. robot_state_publisher
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    # 3. spawn 模型到 Gazebo
    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "dog", "-z", "0.35"],
        output="screen",
    )

    # 4. 加载控制器（等 spawn 完成后再加载）
    load_jsb = ExecuteProcess(
        cmd=[
            "ros2", "control", "load_controller", "--set-state", "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    load_fpc = ExecuteProcess(
        cmd=[
            "ros2", "control", "load_controller", "--set-state", "active",
            "forward_position_controller",
        ],
        output="screen",
    )

    # 5. quad_locomotion_node（复用现有控制逻辑）
    quad_node = Node(
        package="dog_control",
        executable="quad_locomotion_node",
        output="screen",
        parameters=[
            config_path,
            {"input_source": "keyboard", "use_sim_time": True},
        ],
    )

    # 6. sim_bridge_node（符号校正 + 转发）
    bridge_node = Node(
        package="dog_control",
        executable="sim_bridge_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # 事件链：spawn 完成后加载控制器，控制器加载完成后再启动控制节点
    # keyboard_input_node 需要终端输入，请在另一个终端手动运行：
    #   ros2 run dog_control keyboard_input_node --ros-args -p input_source:=keyboard
    spawn_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn,
            on_exit=[
                TimerAction(period=2.0, actions=[load_jsb]),
                TimerAction(period=4.0, actions=[load_fpc]),
                TimerAction(period=6.0, actions=[quad_node, bridge_node]),
            ],
        )
    )

    return LaunchDescription(
        [
            gazebo,
            rsp,
            spawn_handler,
            spawn,
        ]
    )
