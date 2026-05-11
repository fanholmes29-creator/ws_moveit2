cmake_minimum_required(VERSION 3.8)
project(trunk_two_stage_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PlanToPose.srv"
  DEPENDENCIES geometry_msgs
)

add_library(${PROJECT_NAME}_lib
  src/robot_kinematics_helper.cpp
  src/two_stage_planner.cpp
  src/trajectory_utils.cpp
  src/two_stage_planner_manager.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rclcpp_action
  sensor_msgs
  geometry_msgs
  control_msgs
  trajectory_msgs
  visualization_msgs
  moveit_msgs
  ament_index_cpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
)

target_link_libraries(${PROJECT_NAME}_lib Eigen3::Eigen)
target_compile_features(${PROJECT_NAME}_lib PUBLIC cxx_std_17)

add_executable(two_stage_planner_analysis_tool
  src/analysis_tool_main.cpp
)

add_executable(two_stage_planner_system
  src/system_main.cpp
)

add_executable(fk_pose_from_joint
  src/fk_pose_from_joint_main.cpp
)

add_executable(ik_joint_from_pose
  src/ik_joint_from_pose_main.cpp
)

add_executable(two_stage_planner_service
  src/two_stage_planner_service_main.cpp
)

ament_target_dependencies(two_stage_planner_analysis_tool
  rclcpp
  rclcpp_action
  sensor_msgs
  geometry_msgs
  control_msgs
  trajectory_msgs
  visualization_msgs
  moveit_msgs
  ament_index_cpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
)

target_link_libraries(two_stage_planner_analysis_tool
  ${PROJECT_NAME}_lib
  Eigen3::Eigen
)
target_compile_features(two_stage_planner_analysis_tool PUBLIC cxx_std_17)

ament_target_dependencies(two_stage_planner_system
  rclcpp
  rclcpp_action
  sensor_msgs
  geometry_msgs
  control_msgs
  trajectory_msgs
  visualization_msgs
  moveit_msgs
  ament_index_cpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
)

target_link_libraries(two_stage_planner_system
  ${PROJECT_NAME}_lib
  Eigen3::Eigen
)
target_compile_features(two_stage_planner_system PUBLIC cxx_std_17)

ament_target_dependencies(fk_pose_from_joint
  rclcpp
  rclcpp_action
  sensor_msgs
  geometry_msgs
  control_msgs
  trajectory_msgs
  visualization_msgs
  moveit_msgs
  ament_index_cpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
)

target_link_libraries(fk_pose_from_joint
  ${PROJECT_NAME}_lib
  Eigen3::Eigen
)
target_compile_features(fk_pose_from_joint PUBLIC cxx_std_17)

ament_target_dependencies(ik_joint_from_pose
  rclcpp
  rclcpp_action
  sensor_msgs
  geometry_msgs
  control_msgs
  trajectory_msgs
  visualization_msgs
  moveit_msgs
  ament_index_cpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
)

target_link_libraries(ik_joint_from_pose
  ${PROJECT_NAME}_lib
  Eigen3::Eigen
)
target_compile_features(ik_joint_from_pose PUBLIC cxx_std_17)

ament_target_dependencies(two_stage_planner_service
  rclcpp
  rclcpp_action
  sensor_msgs
  geometry_msgs
  control_msgs
  trajectory_msgs
  visualization_msgs
  moveit_msgs
  ament_index_cpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
)

target_link_libraries(two_stage_planner_service
  ${PROJECT_NAME}_lib
  Eigen3::Eigen
)
target_compile_features(two_stage_planner_service PUBLIC cxx_std_17)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(two_stage_planner_service "${cpp_typesupport_target}")

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  ${PROJECT_NAME}_lib
  two_stage_planner_analysis_tool
  two_stage_planner_system
  fk_pose_from_joint
  ik_joint_from_pose
  two_stage_planner_service
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/example_plan_to_pose_client
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
