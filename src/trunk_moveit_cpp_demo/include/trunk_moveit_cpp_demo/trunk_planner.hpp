#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

// 对指定 MoveIt 规划组封装 MoveGroupInterface：关节/位姿目标规划、轨迹执行、日志与 CSV 导出。
class TrunkPlanner
{
public:
  explicit TrunkPlanner(
    const rclcpp::Node::SharedPtr& node,
    const std::string& planning_group = "trunk_group");

  ~TrunkPlanner() = default;

  // 创建 MoveGroupInterface 并完成基础配置。
  bool initialize();

  // 关节目标：规划并立即执行。
  bool planAndExecuteJointTarget(const std::vector<double>& target_joint_values);

  // 关节目标：仅规划，结果写入 plan。
  bool planToJointTarget(
    const std::vector<double>& target_joint_values,
    moveit::planning_interface::MoveGroupInterface::Plan& plan);

  // 末端位姿目标：仅规划（需运动学可用）。
  bool planToPoseTarget(
    const geometry_msgs::msg::Pose& target_pose,
    moveit::planning_interface::MoveGroupInterface::Plan& plan);

  // 执行已生成的轨迹。
  bool executePlan(
    const moveit::planning_interface::MoveGroupInterface::Plan& plan);

  // 将规划轨迹要点打印到日志。
  void printPlanTrajectory(
    const moveit::planning_interface::MoveGroupInterface::Plan& plan) const;

  // 将关节轨迹导出为 CSV（含时间、位置、速度、加速度列）。
  bool exportPlanTrajectoryToCSV(
    const moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const std::string& file_path) const;

  std::vector<double> getCurrentJointValues() const;
  std::vector<std::string> getJointNames() const;
  std::string getPlanningGroup() const;

  // 规划时间、尝试次数与速度/加速度缩放（作用于后续 plan）。
  void setPlanningTime(double planning_time);
  void setNumPlanningAttempts(int num_attempts);
  void setVelocityScaling(double scaling);
  void setAccelerationScaling(double scaling);

  // 打印规划组、规划坐标系、连杆与关节列表等。
  void logBasicInfo() const;

private:
  // 关节目标向量维度与数值合法性检查。
  bool validateTarget(const std::vector<double>& target_joint_values) const;
  bool isFiniteVector(const std::vector<double>& values) const;

  rclcpp::Node::SharedPtr node_;
  std::string planning_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};