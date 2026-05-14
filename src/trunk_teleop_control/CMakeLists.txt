cmake_minimum_required(VERSION 3.8)
project(trunk_teleop_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetControlMode.srv"
)

add_executable(trunk_joystick_teleop
  src/trunk_joystick_teleop.cpp
)

add_executable(control_mode_manager
  src/control_mode_manager.cpp
)

ament_target_dependencies(trunk_joystick_teleop
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  trajectory_msgs
  control_msgs
  builtin_interfaces
)
target_compile_features(trunk_joystick_teleop PUBLIC cxx_std_17)

ament_target_dependencies(control_mode_manager
  rclcpp
  rclcpp_action
  std_msgs
  control_msgs
)
target_compile_features(control_mode_manager PUBLIC cxx_std_17)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(control_mode_manager "${cpp_typesupport_target}")

install(TARGETS
  trunk_joystick_teleop
  control_mode_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
