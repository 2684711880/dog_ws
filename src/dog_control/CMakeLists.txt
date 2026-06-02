cmake_minimum_required(VERSION 3.8)
project(dog_control)

# 1. C++标准设置 
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 2. 编译警告（合并两个工程的设置）
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 3. 导出编译命令
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 4. 依赖包查找（合并）
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED) 
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dog_msgs REQUIRED)
# 新增：实物控制需要的CAN支持
find_package(can_msgs REQUIRED)
# 5. 包含目录设置
include_directories(
  include  # 这是include/motor_ros2的父目录
)

# 6. 测试配置
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 7. 创建硬件库
add_library(motor_hardware_lib
  src/hardware/motor_cfg.cpp
  src/hardware/dji_3508_controller.cpp
)
ament_target_dependencies(motor_hardware_lib
  rclcpp
  std_msgs
  can_msgs
  sensor_msgs
)

# 8. 新架构 quad 库
add_library(quad_lib
  src/quad/leg_kinematics.cpp
  src/quad/gait_scheduler.cpp
  src/quad/foot_planner.cpp
  src/quad/command_filter.cpp
  src/quad/safety_monitor.cpp
  src/quad/locomotion_controller.cpp
)
target_include_directories(quad_lib PUBLIC include)
ament_target_dependencies(quad_lib
  dog_msgs
)

# 新架构节点
add_executable(quad_locomotion_node
  src/quad_locomotion_node.cpp
)
ament_target_dependencies(quad_locomotion_node
  rclcpp
  sensor_msgs
  std_msgs
  dog_msgs
)
target_link_libraries(quad_locomotion_node
  quad_lib
)

# 8.1 仿真桥接节点
add_executable(sim_bridge_node
  src/sim_bridge_node.cpp
)
ament_target_dependencies(sim_bridge_node
  rclcpp
  sensor_msgs
  std_msgs
)
target_link_libraries(sim_bridge_node
  quad_lib
)
install(TARGETS sim_bridge_node
  DESTINATION lib/${PROJECT_NAME})

# 8.2 实物控制节点
add_executable(real_motor_controller
  src/hardware/main.cpp
)
ament_target_dependencies(real_motor_controller
  rclcpp
  std_msgs
  sensor_msgs
  can_msgs
  dog_msgs
)
target_link_libraries(real_motor_controller
  motor_hardware_lib
)

# 8.3 单腿机械零位标定工具
add_executable(zero_calibrator
  src/hardware/zero_calibrator.cpp
)
ament_target_dependencies(zero_calibrator
  rclcpp
  std_msgs
  sensor_msgs
  can_msgs
)
target_link_libraries(zero_calibrator
  motor_hardware_lib
)

# 8.4 DJI 3508(C620) 单电机点转测试工具（标准11-bit CAN帧）
add_executable(dji_3508_spin_test
  src/hardware/dji_3508_spin_test.cpp
)
target_link_libraries(dji_3508_spin_test
  motor_hardware_lib
)

# 10. 安装规则（合并）
install(TARGETS
  quad_locomotion_node
  quad_lib
  real_motor_controller
  zero_calibrator
  dji_3508_spin_test
  motor_hardware_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(
  DIRECTORY include/
  DESTINATION include
)

# 安装其他目录（launch 已移至 dog_bringup）
install(DIRECTORY config urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

add_executable(manual_joint_pub src/manual_joint_pub.cpp)
ament_target_dependencies(manual_joint_pub rclcpp sensor_msgs)
install(TARGETS manual_joint_pub
  DESTINATION lib/${PROJECT_NAME})

add_executable(keyboard_input_node src/keyboard_input_node.cpp)
ament_target_dependencies(keyboard_input_node rclcpp dog_msgs)
install(TARGETS keyboard_input_node
  DESTINATION lib/${PROJECT_NAME})

add_executable(pose_hold_tool
  src/pose_hold_tool.cpp
)
ament_target_dependencies(pose_hold_tool
  rclcpp
  rcl_interfaces
  sensor_msgs
)
install(TARGETS pose_hold_tool
  DESTINATION lib/${PROJECT_NAME})

add_executable(foot_target_tool
  src/foot_target_tool.cpp
)
ament_target_dependencies(foot_target_tool
  rclcpp
  sensor_msgs
)
target_link_libraries(foot_target_tool
  quad_lib
)
install(TARGETS foot_target_tool
  DESTINATION lib/${PROJECT_NAME})

add_executable(trot_foot_sim
  src/trot_foot_sim.cpp
)
target_link_libraries(trot_foot_sim
  quad_lib
)
install(TARGETS trot_foot_sim
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_types
    test/test_types.cpp
  )
  if(TARGET test_types)
    ament_target_dependencies(test_types
      dog_msgs
    )
  endif()

  ament_add_gtest(test_leg_kinematics
    test/test_leg_kinematics.cpp
  )
  if(TARGET test_leg_kinematics)
    ament_target_dependencies(test_leg_kinematics
      dog_msgs
    )
    target_link_libraries(test_leg_kinematics
      quad_lib
    )
  endif()

  ament_add_gtest(test_gait_scheduler
    test/test_gait_scheduler.cpp
  )
  if(TARGET test_gait_scheduler)
    ament_target_dependencies(test_gait_scheduler
      dog_msgs
    )
    target_link_libraries(test_gait_scheduler
      quad_lib
    )
  endif()

  ament_add_gtest(test_foot_planner
    test/test_foot_planner.cpp
  )
  if(TARGET test_foot_planner)
    ament_target_dependencies(test_foot_planner
      dog_msgs
    )
    target_link_libraries(test_foot_planner
      quad_lib
    )
  endif()

  ament_add_gtest(test_command_filter
    test/test_command_filter.cpp
  )
  if(TARGET test_command_filter)
    ament_target_dependencies(test_command_filter
      dog_msgs
    )
    target_link_libraries(test_command_filter
      quad_lib
    )
  endif()

  ament_add_gtest(test_safety_monitor
    test/test_safety_monitor.cpp
  )
  if(TARGET test_safety_monitor)
    ament_target_dependencies(test_safety_monitor
      dog_msgs
    )
    target_link_libraries(test_safety_monitor
      quad_lib
    )
  endif()

  ament_add_gtest(test_locomotion_controller
    test/test_locomotion_controller.cpp
  )
  if(TARGET test_locomotion_controller)
    ament_target_dependencies(test_locomotion_controller
      dog_msgs
    )
    target_link_libraries(test_locomotion_controller
      quad_lib
    )
  endif()
endif()

ament_package()
