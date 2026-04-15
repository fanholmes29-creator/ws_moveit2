cmake_minimum_required(VERSION 3.8)
project(trunk_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)


add_executable(joint_planning_multi_target src/joint_planning_multi_target.cpp)


ament_target_dependencies(joint_planning_multi_target
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

install(TARGETS
  joint_planning_multi_target
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()