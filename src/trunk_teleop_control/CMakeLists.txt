cmake_minimum_required(VERSION 3.8)
project(trunk_teleop_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

add_executable(trunk_joystick_teleop
  src/trunk_joystick_teleop.cpp
)

ament_target_dependencies(trunk_joystick_teleop
  rclcpp
  rclcpp_action
  sensor_msgs
  trajectory_msgs
  control_msgs
  builtin_interfaces
)
target_compile_features(trunk_joystick_teleop PUBLIC cxx_std_17)

install(TARGETS
  trunk_joystick_teleop
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
