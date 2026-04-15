#include <memory>
#include <thread>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include "trunk_moveit_cpp_demo/trunk_planner.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared(
    "trunk_moveit_cpp_demo",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  // ----------------------------
  // 声明并读取 ROS 参数
  // ----------------------------
  node->declare_parameter<std::vector<double>>("home_state", {0.0, 0.0, 0.0, 0.0});
  node->declare_parameter<std::vector<double>>("goal_state", {-1.5, 0.75, 0.4, 0.3});

  node->declare_parameter<bool>("move_to_home_first", true);

  node->declare_parameter<double>("planning_time", 5.0);
  node->declare_parameter<int>("num_planning_attempts", 5);
  node->declare_parameter<double>("velocity_scaling", 0.2);
  node->declare_parameter<double>("acceleration_scaling", 0.2);

  node->declare_parameter<bool>("export_csv", true);
  node->declare_parameter<std::string>("home_csv_path", "/home/wxl/ws_moveit2/csv/home_plan.csv");
  node->declare_parameter<std::string>("goal_csv_path", "/home/wxl/ws_moveit2/csv/goal_plan.csv");

  std::vector<double> home_state =
    node->get_parameter("home_state").as_double_array();
  std::vector<double> goal_state =
    node->get_parameter("goal_state").as_double_array();

  bool move_to_home_first =
    node->get_parameter("move_to_home_first").as_bool();

  double planning_time =
    node->get_parameter("planning_time").as_double();
  int num_planning_attempts =
    node->get_parameter("num_planning_attempts").as_int();
  double velocity_scaling =
    node->get_parameter("velocity_scaling").as_double();
  double acceleration_scaling =
    node->get_parameter("acceleration_scaling").as_double();

  bool export_csv =
    node->get_parameter("export_csv").as_bool();
  std::string home_csv_path =
    node->get_parameter("home_csv_path").as_string();
  std::string goal_csv_path =
    node->get_parameter("goal_csv_path").as_string();

  // ----------------------------
  // 初始化规划器
  // ----------------------------
  TrunkPlanner planner(node, "trunk_group");
  if (!planner.initialize()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize TrunkPlanner.");
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  planner.setPlanningTime(planning_time);
  planner.setNumPlanningAttempts(num_planning_attempts);
  planner.setVelocityScaling(velocity_scaling);
  planner.setAccelerationScaling(acceleration_scaling);

  // 参数打印
  RCLCPP_INFO(node->get_logger(), "===== Parameters =====");
  RCLCPP_INFO(node->get_logger(), "move_to_home_first: %s", move_to_home_first ? "true" : "false");
  RCLCPP_INFO(node->get_logger(), "planning_time: %.3f", planning_time);
  RCLCPP_INFO(node->get_logger(), "num_planning_attempts: %d", num_planning_attempts);
  RCLCPP_INFO(node->get_logger(), "velocity_scaling: %.3f", velocity_scaling);
  RCLCPP_INFO(node->get_logger(), "acceleration_scaling: %.3f", acceleration_scaling);
  RCLCPP_INFO(node->get_logger(), "export_csv: %s", export_csv ? "true" : "false");

  for (size_t i = 0; i < home_state.size(); ++i) {
    RCLCPP_INFO(node->get_logger(), "home[%zu] = %.6f", i, home_state[i]);
  }
  for (size_t i = 0; i < goal_state.size(); ++i) {
    RCLCPP_INFO(node->get_logger(), "goal[%zu] = %.6f", i, goal_state[i]);
  }

  bool success = true;

  // ----------------------------
  // Step 1: current -> home
  // ----------------------------
  if (move_to_home_first) {
    RCLCPP_INFO(node->get_logger(), "Step 1: Move to home state.");

    moveit::planning_interface::MoveGroupInterface::Plan home_plan;
    success = planner.planToJointTarget(home_state, home_plan);
    if (!success) {
      RCLCPP_ERROR(node->get_logger(), "Failed to plan to home state.");
      executor.cancel();
      spinner.join();
      rclcpp::shutdown();
      return 1;
    }

    if (export_csv) {
      if (!planner.exportPlanTrajectoryToCSV(home_plan, home_csv_path)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to export home trajectory CSV.");
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
        return 1;
      }
    }

    success = planner.executePlan(home_plan);
    if (!success) {
      RCLCPP_ERROR(node->get_logger(), "Failed to move to home state.");
      executor.cancel();
      spinner.join();
      rclcpp::shutdown();
      return 1;
    }
  }

  // ----------------------------
  // Step 2: home/current -> goal
  // ----------------------------
  RCLCPP_INFO(node->get_logger(), "Step 2: Move to goal state.");

  moveit::planning_interface::MoveGroupInterface::Plan goal_plan;
  success = planner.planToJointTarget(goal_state, goal_plan);
  if (!success) {
    RCLCPP_ERROR(node->get_logger(), "Failed to plan to goal state.");
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  if (export_csv) {
    if (!planner.exportPlanTrajectoryToCSV(goal_plan, goal_csv_path)) {
      RCLCPP_ERROR(node->get_logger(), "Failed to export goal trajectory CSV.");
      executor.cancel();
      spinner.join();
      rclcpp::shutdown();
      return 1;
    }
  }

  success = planner.executePlan(goal_plan);
  if (!success) {
    RCLCPP_ERROR(node->get_logger(), "Failed to move to goal state.");
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Successfully finished trajectory execution.");

  if (export_csv) {
    RCLCPP_INFO(node->get_logger(), "CSV saved:");
    if (move_to_home_first) {
      RCLCPP_INFO(node->get_logger(), "  home plan: %s", home_csv_path.c_str());
    }
    RCLCPP_INFO(node->get_logger(), "  goal plan: %s", goal_csv_path.c_str());
  }

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}