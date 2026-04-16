#pragma once

#include <vector>

#include <geometry_msgs/msg/pose.hpp>

#include "trunk_two_stage_planner/robot_kinematics_helper.hpp"
#include "trunk_two_stage_planner/types.hpp"

namespace trunk_two_stage_planner
{

class TwoStagePlanner
{
public:
  TwoStagePlanner(const RobotKinematicsHelper& kinematics, const PlannerConfig& config);

  bool plan(
    const std::vector<double>& q_start,
    const geometry_msgs::msg::Pose& target_pose,
    PlanningSummary& summary) const;

  bool solveFinalIK(
    const std::vector<double>& q_start,
    const geometry_msgs::msg::Pose& target_pose,
    std::vector<double>& q_goal,
    bool& used_fallback_goal) const;

  Axis3D computeJointAxisInWorld(
    const std::vector<double>& q,
    const std::string& joint_name) const;

  Eigen::Vector3d projectPointToAxis(const Eigen::Vector3d& p, const Axis3D& axis) const;

  std::vector<double> composeStage1State(double q1, double q2, double q4_fix) const;

  Eigen::Vector3d getStage1ReferencePoint(const std::vector<double>& q) const;

  double jointLimitPenalty(const std::vector<double>& q) const;

  double stage1Cost(
    double q1,
    double q2,
    const std::vector<double>& q_start,
    const Eigen::Vector3d& p_goal_proj,
    const Eigen::Vector3d& p_d,
    double* projection_error_out = nullptr,
    double* stage2_pos_error_out = nullptr,
    Eigen::Vector3d* o4_out = nullptr,
    Eigen::Vector3d* o4_proj_out = nullptr) const;

  bool searchStage1PreparatoryState(
    const std::vector<double>& q_start,
    const Eigen::Vector3d& p_goal_proj,
    const Eigen::Vector3d& p_d,
    const Eigen::Quaterniond& q_d,
    Stage1SearchResult& result) const;

private:
  double minStage2PositionErrorForFixedQ1Q2(
    const std::vector<double>& q_with_fixed_q1q2,
    const Eigen::Vector3d& p_d) const;
  double minStage2PoseErrorForFixedQ1Q2(
    const std::vector<double>& q_with_fixed_q1q2,
    const Eigen::Vector3d& p_d,
    const Eigen::Quaterniond& q_d,
    double* best_pos_error = nullptr) const;

  bool optimizeStage2LockedQ1Q2(
    const std::vector<double>& q_pre,
    const geometry_msgs::msg::Pose& target_pose,
    const std::vector<double>& q_goal_ik,
    std::vector<double>& q_goal_stage2,
    double& position_error,
    double& orientation_error_deg) const;

  double stage2LockedCost(
    double q3,
    double q4,
    const std::vector<double>& q_pre,
    const geometry_msgs::msg::Pose& target_pose,
    const std::vector<double>& q_goal_ik,
    double* position_error = nullptr,
    double* orientation_error_deg = nullptr) const;

  static std::vector<double> subtractVectors(
    const std::vector<double>& lhs,
    const std::vector<double>& rhs);
  static Eigen::Vector3d orientationErrorVector(
    const Eigen::Matrix3d& current,
    const Eigen::Quaterniond& target);
  static double orientationErrorDeg(
    const Eigen::Matrix3d& current,
    const Eigen::Quaterniond& target);
  double ikCandidateCost(const std::vector<double>& q, const std::vector<double>& q_start) const;
  static double squaredDistance2D(double a1, double a2, double b1, double b2);

  const RobotKinematicsHelper& kinematics_;
  PlannerConfig config_;
  std::size_t idx_q1_ = 0;
  std::size_t idx_q2_ = 1;
  std::size_t idx_q3_ = 2;
  std::size_t idx_q4_ = 3;
};

}  // namespace trunk_two_stage_planner
