#pragma once

#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>

#include "trunk_two_stage_planner/types.hpp"

namespace trunk_two_stage_planner
{

class RobotKinematicsHelper
{
public:
  bool initialize(const rclcpp::Node::SharedPtr& node, const PlannerConfig& config);

  Eigen::Isometry3d getLinkTransform(const std::vector<double>& q, const std::string& link_name) const;
  Eigen::Vector3d getLinkPosition(const std::vector<double>& q, const std::string& link_name) const;

  bool solveIK(const geometry_msgs::msg::Pose& target_pose, std::vector<double>& q_solution,
    const std::vector<double>& seed = {}) const;

  Axis3D computeJointAxisInWorld(const std::vector<double>& q, const std::string& joint_name) const;

  bool isStateWithinBounds(const std::vector<double>& q) const;
  std::pair<double, double> getJointPositionBounds(const std::string& joint_name) const;
  std::vector<double> sampleRandomState(std::mt19937& rng) const;

  bool getJointIndex(const std::string& joint_name, std::size_t& index) const;
  std::pair<std::string, std::string> getJointParentChildLinks(const std::string& joint_name) const;

  moveit::core::RobotModelConstPtr getRobotModel() const;
  const moveit::core::JointModelGroup* getJointModelGroup() const;
  const std::vector<std::string>& getJointNames() const;
  const std::string& getTipLinkName() const;
  const std::string& getUrdfPath() const;
  const std::string& getSrdfPath() const;
  bool hasIKSolver() const;

private:
  moveit::core::RobotState makeRobotState(const std::vector<double>& q) const;
  std::string readTextFile(const std::string& file_path) const;
  void declareOrKeepStringParameter(const std::string& name, const std::string& value) const;
  void declareOrKeepDoubleParameter(const std::string& name, double value) const;

  rclcpp::Node::SharedPtr node_;
  PlannerConfig config_;
  std::shared_ptr<robot_model_loader::RobotModelLoader> model_loader_;
  moveit::core::RobotModelPtr robot_model_;
  const moveit::core::JointModelGroup* joint_model_group_ = nullptr;
  std::vector<std::string> joint_names_;
  std::string tip_link_name_;
  std::string urdf_path_;
  std::string srdf_path_;
};

}  // namespace trunk_two_stage_planner
