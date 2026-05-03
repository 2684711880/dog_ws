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

# 8. 统一步态控制核心（订阅 locomotion_cmd）
add_library(locomotion_core_lib
  src/locomotion_params.cpp
  src/locomotion_runtime.cpp
  src/locomotion_planner.cpp
)
ament_target_dependencies(locomotion_core_lib
  rclcpp
  sensor_msgs
  std_msgs
  dog_msgs
)

add_executable(locomotion_core_node
  src/locomotion_core_node.cpp
)
ament_target_dependencies(locomotion_core_node
  rclcpp
  sensor_msgs
  std_msgs
  dog_msgs
)
target_link_libraries(locomotion_core_node
  locomotion_core_lib
)

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
  locomotion_core_node
  locomotion_core_lib
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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_locomotion_core
    test/test_locomotion_core.cpp
  )
  if(TARGET test_locomotion_core)
    ament_target_dependencies(test_locomotion_core
      rclcpp
      sensor_msgs
      std_msgs
      dog_msgs
    )
    target_link_libraries(test_locomotion_core
      locomotion_core_lib
    )
  endif()
endif()

ament_package()
