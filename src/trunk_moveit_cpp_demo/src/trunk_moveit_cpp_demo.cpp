#include <cstdio>
#include <memory>
#include <thread>
#include <vector>
#include <string>
#include <exception>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include "trunk_moveit_cpp_demo/trunk_planner.hpp"

// 节点入口：对 trunk_group 做关节空间规划与执行（可选先回 home 再到 goal），并可导出两段轨迹 CSV。
// 依赖外部已加载 MoveIt 参数（通常由 launch + TrunkPlanner 使用）。

namespace {

// 仅为未在 launch/YAML 中出现的参数提供默认值。

void declare_if_missing_bool(
  const rclcpp::Node::SharedPtr & node, const std::string & name, bool default_value)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter<bool>(name, default_value);
  }
}

void declare_if_missing_int(
  const rclcpp::Node::SharedPtr & node, const std::string & name, int default_value)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter<int>(name, default_value);
  }
}

void declare_if_missing_double(
  const rclcpp::Node::SharedPtr & node, const std::string & name, double default_value)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter<double>(name, default_value);
  }
}

void declare_if_missing_string(
  const rclcpp::Node::SharedPtr & node, const std::string & name,
  const std::string & default_value)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter<std::string>(name, default_value);
  }
}

void declare_if_missing_double_array(
  const rclcpp::Node::SharedPtr & node, const std::string & name,
  const std::vector<double> & default_value)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter<std::vector<double>>(name, default_value);
  }
}

}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  try {
  // 创建节点并在后台 spin，供 MoveIt 接口使用。
  auto node = rclcpp::Node::make_shared(
    "trunk_moveit_cpp_demo",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  // 读取 home/goal 关节目标、规划缩放、是否导出 CSV 等。
  declare_if_missing_double_array(node, "home_state", {0.0, 0.0, 0.0, 0.0});
  declare_if_missing_double_array(node, "goal_state", {-1.5, 0.75, 0.4, 0.3});

  declare_if_missing_bool(node, "move_to_home_first", true);

  declare_if_missing_double(node, "planning_time", 5.0);
  declare_if_missing_int(node, "num_planning_attempts", 5);
  declare_if_missing_double(node, "velocity_scaling", 0.2);
  declare_if_missing_double(node, "acceleration_scaling", 0.2);

  declare_if_missing_bool(node, "export_csv", true);
  declare_if_missing_string(
    node, "home_csv_path", "/home/wxl/ws_moveit2/csv/home_plan.csv");
  declare_if_missing_string(
    node, "goal_csv_path", "/home/wxl/ws_moveit2/csv/goal_plan.csv");

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

  // 绑定规划组 trunk_group，应用规划时间与速度相关设置。
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

  // 可选：当前状态 -> home，规划、导出、执行。
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

  // 自当前状态 -> goal，规划、导出、执行。
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

  } catch (const std::exception & e) {
    // 参数或 MoveIt 调用异常时避免无信息退出。
    std::fprintf(stderr, "trunk_moveit_cpp_demo: uncaught exception: %s\n", e.what());
    rclcpp::shutdown();
    return 2;
  }
}