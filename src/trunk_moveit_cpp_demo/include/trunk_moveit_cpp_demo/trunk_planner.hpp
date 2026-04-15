#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
class TrunkPlanner
{
public:
  explicit TrunkPlanner(
    const rclcpp::Node::SharedPtr& node,
    const std::string& planning_group = "trunk_group");

  ~TrunkPlanner() = default;

  // 初始化 MoveGroupInterface，并设置默认参数
  bool initialize();

  // 规划并执行到目标关节值
  bool planAndExecuteJointTarget(const std::vector<double>& target_joint_values);

  // 只规划，不执行
  bool planToJointTarget(
    const std::vector<double>& target_joint_values,
    moveit::planning_interface::MoveGroupInterface::Plan& plan);
    
  bool planToPoseTarget(
    const geometry_msgs::msg::Pose& target_pose,
    moveit::planning_interface::MoveGroupInterface::Plan& plan);

  // 执行已经规划好的轨迹
  bool executePlan(
    const moveit::planning_interface::MoveGroupInterface::Plan& plan);

  // 打印轨迹摘要
  void printPlanTrajectory(
    const moveit::planning_interface::MoveGroupInterface::Plan& plan) const;

  // 导出轨迹到 CSV 文件
  bool exportPlanTrajectoryToCSV(
    const moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const std::string& file_path) const;



  // 读取当前关节值
  std::vector<double> getCurrentJointValues() const;

  // 读取关节名
  std::vector<std::string> getJointNames() const;

  // 读取规划组名称
  std::string getPlanningGroup() const;

  // 参数设置
  void setPlanningTime(double planning_time);
  void setNumPlanningAttempts(int num_attempts);
  void setVelocityScaling(double scaling);
  void setAccelerationScaling(double scaling);

  // 打印当前基本信息
  void logBasicInfo() const;

private:
  // 基础校验
  bool validateTarget(const std::vector<double>& target_joint_values) const;

  // 工具函数：判断数值是否合法
  bool isFiniteVector(const std::vector<double>& values) const;

private:
  rclcpp::Node::SharedPtr node_;
  std::string planning_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};