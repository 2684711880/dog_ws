import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, LaunchConfigurationNotEquals
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # URDF / RViz 配置来自 dog_control 包
    dog_control_share = get_package_share_directory("dog_control")
    urdf_file = os.path.join(dog_control_share, "urdf", "dog.urdf")
    rviz_config_file = os.path.join(dog_control_share, "config", "dog.rviz")

    # ==================== 调试信息 ====================
    print("=" * 60)
    print("                    调试信息")
    print("=" * 60)
    print(f"📁 包路径: {dog_control_share}")
    print(f"📄 URDF 文件: {urdf_file}")
    print(f"   - 存在:  {os.path.exists(urdf_file)}")
    print(f"📄 RViz 配置:  {rviz_config_file}")
    print(f"   - 存在: {os.path.exists(rviz_config_file)}")

    if os.path.exists(rviz_config_file):
        file_size = os.path.getsize(rviz_config_file)
        print(f"   - 大小: {file_size} bytes")
    else:
        print("   ⚠️  RViz 配置文件不存在！")

    config_dir = os.path.join(dog_control_share, "config")
    if os.path.exists(config_dir):
        print(f"📂 config 目录内容:  {os.listdir(config_dir)}")
    else:
        print("   ⚠️  config 目录不存在！")

    print("=" * 60)

    # 声明 launch 参数
    use_gui_arg = DeclareLaunchArgument(
        "use_gui",
        default_value="true",
        description="Whether to start joint_state_publisher_gui",
    )

    use_gui = LaunchConfiguration("use_gui")

    # 读取 URDF 文件内容
    if os.path.exists(urdf_file):
        with open(urdf_file, "r") as f:
            robot_description = f.read()
        print(f"✅ URDF 文件读取成功，长度: {len(robot_description)} 字符")
    else:
        robot_description = ""
        print("❌ URDF 文件读取失败！")

    # 机器人状态发布器
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # 关节状态发布器（带 GUI）
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=IfCondition(use_gui),
    )

    # 关节状态发布器（不带 GUI）
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        condition=LaunchConfigurationNotEquals("use_gui", "true"),
    )

    # RViz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
    )

    # Launch 日志信息
    log_info = LogInfo(msg=f"🚀 启动 show_dog，RViz 配置: {rviz_config_file}")

    return LaunchDescription([
        log_info,
        use_gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        joint_state_publisher_node,
        rviz_node,
    ])
