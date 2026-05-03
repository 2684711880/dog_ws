cmake_minimum_required(VERSION 3.8)
project(dog_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dog_msgs REQUIRED)

add_executable(xbox_input_node
  src/xbox_input_node.cpp
)
ament_target_dependencies(xbox_input_node
  rclcpp
  sensor_msgs
  dog_msgs
)

install(TARGETS xbox_input_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
