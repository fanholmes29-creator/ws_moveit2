#include "trunk_moveit_cpp_demo/trunk_planner.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>

TrunkPlanner::TrunkPlanner(
  const rclcpp::Node::SharedPtr& node,
  const std::string& planning_group)
: node_(node), planning_group_(planning_group)
{
}

bool TrunkPlanner::initialize()
{
  try {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      node_, planning_group_);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Failed to create MoveGroupInterface for group [%s]: %s",
      planning_group_.c_str(),
      e.what());
    return false;
  }

  // 默认参数：先保守，保证调试稳定
  move_group_->setPlanningTime(5.0);
  move_group_->setNumPlanningAttempts(5);
  move_group_->setMaxVelocityScalingFactor(0.2);
  move_group_->setMaxAccelerationScalingFactor(0.2);

  logBasicInfo();
  return true;
}

bool TrunkPlanner::planAndExecuteJointTarget(const std::vector<double>& target_joint_values)
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (!planToJointTarget(target_joint_values, plan)) {
    return false;
  }

  return executePlan(plan);
}

bool TrunkPlanner::planToJointTarget(
  const std::vector<double>& target_joint_values,
  moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
  if (!validateTarget(target_joint_values)) {
    return false;
  }

  move_group_->setStartStateToCurrentState();
  move_group_->setJointValueTarget(target_joint_values);

  RCLCPP_INFO(node_->get_logger(), "Planning to joint target...");

  const auto result = move_group_->plan(plan);
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Planning failed.");
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "Planning succeeded.");
  printPlanTrajectory(plan);
  return true;
}

bool TrunkPlanner::executePlan(
  const moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
  if (!move_group_) {
    RCLCPP_ERROR(node_->get_logger(), "MoveGroupInterface is not initialized.");
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "Executing trajectory...");

  const auto result = move_group_->execute(plan);
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Execution failed.");
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "Execution succeeded.");
  return true;
}

void TrunkPlanner::printPlanTrajectory(
  const moveit::planning_interface::MoveGroupInterface::Plan& plan) const
{
  const auto& traj = plan.trajectory_.joint_trajectory;

  RCLCPP_INFO(node_->get_logger(), "===== Planned Joint Trajectory Summary =====");

  for (size_t i = 0; i < traj.joint_names.size(); ++i) {
    RCLCPP_INFO(node_->get_logger(), "joint_names[%zu] = %s", i, traj.joint_names[i].c_str());
  }

  if (traj.points.empty()) {
    RCLCPP_WARN(node_->get_logger(), "Trajectory has no waypoints.");
    return;
  }

  const auto& start_pt = traj.points.front();
  const auto& goal_pt = traj.points.back();

  RCLCPP_INFO(node_->get_logger(), "Waypoint count: %zu", traj.points.size());

  RCLCPP_INFO(node_->get_logger(), "Start point:");
  for (size_t j = 0; j < start_pt.positions.size(); ++j) {
    RCLCPP_INFO(node_->get_logger(), "  joint[%zu] start = %.6f", j, start_pt.positions[j]);
  }

  RCLCPP_INFO(node_->get_logger(), "Goal point:");
  for (size_t j = 0; j < goal_pt.positions.size(); ++j) {
    RCLCPP_INFO(node_->get_logger(), "  joint[%zu] goal = %.6f", j, goal_pt.positions[j]);
  }

  const double total_time =
    static_cast<double>(goal_pt.time_from_start.sec) +
    static_cast<double>(goal_pt.time_from_start.nanosec) * 1e-9;

  RCLCPP_INFO(node_->get_logger(), "Total trajectory time: %.6f s", total_time);
  RCLCPP_INFO(node_->get_logger(), "===== End of Trajectory Summary =====");
}

bool TrunkPlanner::exportPlanTrajectoryToCSV(
  const moveit::planning_interface::MoveGroupInterface::Plan& plan,
  const std::string& file_path) const
{
  const auto& traj = plan.trajectory_.joint_trajectory;

  if (traj.joint_names.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Trajectory joint_names is empty. Cannot export CSV.");
    return false;
  }

  if (traj.points.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Trajectory points is empty. Cannot export CSV.");
    return false;
  }

  const std::filesystem::path path(file_path);
  const std::filesystem::path parent = path.parent_path();
  if (!parent.empty()) {
    std::error_code ec;
    std::filesystem::create_directories(parent, ec);
    if (ec) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Failed to create directory for CSV [%s]: %s",
        parent.string().c_str(),
        ec.message().c_str());
      return false;
    }
  }

  std::ofstream ofs(file_path);
  if (!ofs.is_open()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Failed to open CSV file for writing: %s",
      file_path.c_str());
    return false;
  }

  ofs << std::fixed << std::setprecision(6);

  // 表头
  ofs << "time_from_start";
  for (const auto& joint_name : traj.joint_names) {
    ofs << "," << joint_name << "_pos";
  }
  for (const auto& joint_name : traj.joint_names) {
    ofs << "," << joint_name << "_vel";
  }
  for (const auto& joint_name : traj.joint_names) {
    ofs << "," << joint_name << "_acc";
  }
  ofs << "\n";

  // 逐点写入
  for (const auto& pt : traj.points) {
    const double t =
      static_cast<double>(pt.time_from_start.sec) +
      static_cast<double>(pt.time_from_start.nanosec) * 1e-9;

    ofs << t;

    // positions
    for (size_t i = 0; i < traj.joint_names.size(); ++i) {
      const double value = (i < pt.positions.size()) ? pt.positions[i] : 0.0;
      ofs << "," << value;
    }

    // velocities
    for (size_t i = 0; i < traj.joint_names.size(); ++i) {
      const double value = (i < pt.velocities.size()) ? pt.velocities[i] : 0.0;
      ofs << "," << value;
    }

    // accelerations
    for (size_t i = 0; i < traj.joint_names.size(); ++i) {
      const double value = (i < pt.accelerations.size()) ? pt.accelerations[i] : 0.0;
      ofs << "," << value;
    }

    ofs << "\n";
  }

  ofs.close();

  RCLCPP_INFO(
    node_->get_logger(),
    "Trajectory exported to CSV: %s",
    file_path.c_str());

  return true;
}

std::vector<double> TrunkPlanner::getCurrentJointValues() const
{
  if (!move_group_) {
    RCLCPP_ERROR(node_->get_logger(), "MoveGroupInterface is not initialized.");
    return {};
  }

  return move_group_->getCurrentJointValues();
}

std::vector<std::string> TrunkPlanner::getJointNames() const
{
  if (!move_group_) {
    RCLCPP_ERROR(node_->get_logger(), "MoveGroupInterface is not initialized.");
    return {};
  }

  return move_group_->getJointNames();
}

std::string TrunkPlanner::getPlanningGroup() const
{
  return planning_group_;
}

void TrunkPlanner::setPlanningTime(double planning_time)
{
  if (!move_group_) {
    RCLCPP_ERROR(node_->get_logger(), "MoveGroupInterface is not initialized.");
    return;
  }

  if (planning_time <= 0.0) {
    RCLCPP_WARN(node_->get_logger(), "planning_time must be > 0. Ignore this update.");
    return;
  }

  move_group_->setPlanningTime(planning_time);
}

void TrunkPlanner::setNumPlanningAttempts(int num_attempts)
{
  if (!move_group_) {
    RCLCPP_ERROR(node_->get_logger(), "MoveGroupInterface is not initialized.");
    return;
  }

  if (num_attempts <= 0) {
    RCLCPP_WARN(node_->get_logger(), "num_attempts must be > 0. Ignore this update.");
    return;
  }

  move_group_->setNumPlanningAttempts(num_attempts);
}

void TrunkPlanner::setVelocityScaling(double scaling)
{
  if (!move_group_) {
    RCLCPP_ERROR(node_->get_logger(), "MoveGroupInterface is not initialized.");
    return;
  }

  scaling = std::clamp(scaling, 0.0, 1.0);
  move_group_->setMaxVelocityScalingFactor(scaling);
}

void TrunkPlanner::setAccelerationScaling(double scaling)
{
  if (!move_group_) {
    RCLCPP_ERROR(node_->get_logger(), "MoveGroupInterface is not initialized.");
    return;
  }

  scaling = std::clamp(scaling, 0.0, 1.0);
  move_group_->setMaxAccelerationScalingFactor(scaling);
}

void TrunkPlanner::logBasicInfo() const
{
  if (!move_group_) {
    RCLCPP_ERROR(node_->get_logger(), "MoveGroupInterface is not initialized.");
    return;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "Planning group: %s",
    planning_group_.c_str());

  RCLCPP_INFO(
    node_->get_logger(),
    "Planning frame: %s",
    move_group_->getPlanningFrame().c_str());

  RCLCPP_INFO(
    node_->get_logger(),
    "End effector link: %s",
    move_group_->getEndEffectorLink().c_str());

  const auto joint_names = move_group_->getJointNames();
  RCLCPP_INFO(node_->get_logger(), "Joint count: %zu", joint_names.size());

  for (size_t i = 0; i < joint_names.size(); ++i) {
    RCLCPP_INFO(
      node_->get_logger(),
      "Joint[%zu]: %s",
      i,
      joint_names[i].c_str());
  }
}

bool TrunkPlanner::validateTarget(const std::vector<double>& target_joint_values) const
{
  if (!move_group_) {
    RCLCPP_ERROR(node_->get_logger(), "MoveGroupInterface is not initialized.");
    return false;
  }

  const auto current_joint_values = move_group_->getCurrentJointValues();
  const auto joint_names = move_group_->getJointNames();

  if (current_joint_values.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to read current joint values.");
    return false;
  }

  if (joint_names.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to read joint names.");
    return false;
  }

  if (target_joint_values.size() != current_joint_values.size()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Target size mismatch. Expected %zu joints, but got %zu.",
      current_joint_values.size(),
      target_joint_values.size());
    return false;
  }

  if (joint_names.size() != target_joint_values.size()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Joint name count mismatch. joint_names=%zu, target=%zu",
      joint_names.size(),
      target_joint_values.size());
    return false;
  }

  if (!isFiniteVector(target_joint_values)) {
    RCLCPP_ERROR(node_->get_logger(), "Target contains NaN or Inf.");
    return false;
  }

  return true;
}

bool TrunkPlanner::isFiniteVector(const std::vector<double>& values) const
{
  for (const auto& v : values) {
    if (!std::isfinite(v)) {
      return false;
    }
  }
  return true;
}