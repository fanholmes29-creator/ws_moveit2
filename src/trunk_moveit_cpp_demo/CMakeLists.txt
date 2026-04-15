cmake_minimum_required(VERSION 3.8)
project(trunk_moveit_cpp_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_executable(trunk_moveit_cpp_demo
  src/trunk_moveit_cpp_demo.cpp
  src/trunk_planner.cpp
)

ament_target_dependencies(trunk_moveit_cpp_demo
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)
target_compile_features(trunk_moveit_cpp_demo PUBLIC cxx_std_17)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  trunk_moveit_cpp_demo
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()