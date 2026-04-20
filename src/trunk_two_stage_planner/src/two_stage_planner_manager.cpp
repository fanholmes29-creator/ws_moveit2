#include "trunk_two_stage_planner/two_stage_planner_manager.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <thread>

#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace trunk_two_stage_planner
{

namespace
{

Eigen::Quaterniond poseToEigenQuaternion(const geometry_msgs::msg::Pose& pose)
{
  return Eigen::Quaterniond(
    pose.orientation.w,
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z);
}

geometry_msgs::msg::Pose eigenToPose(const Eigen::Isometry3d& tf)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = tf.translation().x();
  pose.position.y = tf.translation().y();
  pose.position.z = tf.translation().z();

  const Eigen::Quaterniond q(tf.rotation());
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

std::vector<geometry_msgs::msg::Point> toPoints(
  const RobotKinematicsHelper& helper,
  const moveit_msgs::msg::RobotTrajectory& trajectory,
  const std::string& tip_link)
{
  std::vector<geometry_msgs::msg::Point> points;
  const auto& joint_traj = trajectory.joint_trajectory;
  points.reserve(joint_traj.points.size());
  for (const auto& point : joint_traj.points) {
    geometry_msgs::msg::Point p;
    const Eigen::Vector3d pos = helper.getLinkPosition(point.positions, tip_link);
    p.x = pos.x();
    p.y = pos.y();
    p.z = pos.z();
    points.push_back(p);
  }
  return points;
}

visualization_msgs::msg::Marker makeSphereMarker(
  const std::string& frame,
  int id,
  double x,
  double y,
  double z,
  float r,
  float g,
  float b,
  const std::string& ns)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.scale.x = 0.03;
  marker.scale.y = 0.03;
  marker.scale.z = 0.03;
  marker.color.a = 1.0;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  return marker;
}

visualization_msgs::msg::Marker makeLineStripMarker(
  const std::string& frame,
  int id,
  const std::vector<geometry_msgs::msg::Point>& points,
  float r,
  float g,
  float b,
  const std::string& ns)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.01;
  marker.color.a = 1.0;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.points = points;
  return marker;
}

}  // namespace

TwoStagePlannerManager::TwoStagePlannerManager(const rclcpp::Node::SharedPtr& node)
: node_(node)
{
}

bool TwoStagePlannerManager::initialize(
  const PlannerConfig& algorithm_config,
  const TwoStageSystemConfig& system_config)
{
  // 文件职责：
  // 连接算法层输出与 MoveIt 执行层。
  // 该方法完成依赖装配与运行时发布器初始化。
  algorithm_config_ = algorithm_config;
  system_config_ = system_config;

  if (!kinematics_.initialize(node_, algorithm_config_)) {
    return false;
  }

  algorithm_ = std::make_unique<TwoStagePlanner>(kinematics_, algorithm_config_);

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  display_pub_ = node_->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
    system_config_.display_trajectory_topic, qos);
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    system_config_.marker_topic, qos);
  return true;
}

bool TwoStagePlannerManager::planTwoStageToTarget(const geometry_msgs::msg::Pose& target_pose)
{
  // 运行主链路：
  // 1) 算法层求解 q_pre 与 stage2 目标
  // 2) MoveIt 先规划 stage1，再规划 stage2
  // 3) 发布/导出 RViz 与调试输出
  const std::vector<double> q_start = algorithm_config_.q_start;

  PlanningSummary summary;
  if (!algorithm_->plan(q_start, target_pose, summary)) {
    RCLCPP_ERROR(node_->get_logger(), "Algorithm layer failed to compute q_pre/q_goal.");
    return false;
  }

  moveit_msgs::msg::RobotTrajectory stage1_traj;
  moveit::planning_interface::MoveGroupInterface::Plan stage1_plan;
  if (!planStage1(q_start, summary.q_pre, stage1_plan)) {
    RCLCPP_ERROR(node_->get_logger(), "Stage1 planning failed.");
    return false;
  }
  stage1_traj = stage1_plan.trajectory_;

  const std::vector<double> q_stage1_end = extractJointPositionsFromTrajectory(stage1_traj);
  moveit::planning_interface::MoveGroupInterface::Plan stage2_plan;
  if (!planStage2(q_stage1_end, summary.q_pre, summary.q_goal_stage2, stage2_plan)) {
    RCLCPP_ERROR(node_->get_logger(), "Stage2 planning failed.");
    return false;
  }

  publishDisplayTrajectories(stage1_traj, stage2_plan.trajectory_, q_start);
  publishDebugMarkers(summary, stage1_traj, stage2_plan.trajectory_);

  RCLCPP_INFO(node_->get_logger(), "===== Two-Stage Planning Summary =====");
  RCLCPP_INFO(node_->get_logger(), "q_start: [%f, %f, %f, %f]",
    q_start[0], q_start[1], q_start[2], q_start[3]);
  RCLCPP_INFO(node_->get_logger(), "q_goal_ik: [%f, %f, %f, %f]",
    summary.q_goal_ik[0], summary.q_goal_ik[1], summary.q_goal_ik[2], summary.q_goal_ik[3]);
  RCLCPP_INFO(node_->get_logger(), "q_pre: [%f, %f, %f, %f]",
    summary.q_pre[0], summary.q_pre[1], summary.q_pre[2], summary.q_pre[3]);
  RCLCPP_INFO(node_->get_logger(), "stage1 planning group: %s", system_config_.stage1_group_name.c_str());
  RCLCPP_INFO(node_->get_logger(), "stage2 planning group: %s", system_config_.stage2_group_name.c_str());

  const std::vector<double> q_stage2_end = extractJointPositionsFromTrajectory(stage2_plan.trajectory_);
  const Eigen::Isometry3d final_tf = kinematics_.getLinkTransform(
    q_stage2_end, kinematics_.getTipLinkName());
  const Eigen::Vector3d final_pos_err(
    final_tf.translation().x() - target_pose.position.x,
    final_tf.translation().y() - target_pose.position.y,
    final_tf.translation().z() - target_pose.position.z);
  const Eigen::Quaterniond target_q = poseToEigenQuaternion(target_pose);
  const Eigen::Quaterniond final_q(final_tf.rotation());
  const double quat_dot = std::clamp(std::abs(final_q.dot(target_q.normalized())), 0.0, 1.0);
  const double final_ori_deg = 2.0 * std::acos(quat_dot) * 180.0 / M_PI;

  RCLCPP_INFO(
    node_->get_logger(),
    "stage2 final position error norm [m]: %.6f",
    final_pos_err.norm());
  RCLCPP_INFO(
    node_->get_logger(),
    "stage2 final orientation error [deg]: %.6f",
    final_ori_deg);
  RCLCPP_INFO(
    node_->get_logger(),
    "stage2 pose recoverability threshold satisfied: %s",
    summary.selected_stage1_stage2_pose_error < algorithm_config_.stage2_pose_epsilon ? "true" : "false");

  if (system_config_.export_csv) {
    // 注意：CSV 导出属于诊断增强，运动结果在此前已确定。
    const auto stage1_points = convertRobotTrajectoryToTrajPoints(stage1_traj, 1, 0.0);
    const double time_offset = stage1_points.empty() ? 0.0 : stage1_points.back().t;
    const auto stage2_points = convertRobotTrajectoryToTrajPoints(stage2_plan.trajectory_, 2, time_offset);
    const auto merged = concatenateTrajectories(stage1_points, stage2_points);

    std::filesystem::create_directories(system_config_.output_dir);
    exportTrajectoryCsv(system_config_.output_dir + "/trajectory.csv", kinematics_.getJointNames(), merged);
    exportSummaryTxt(system_config_.output_dir + "/summary.txt", kinematics_.getJointNames(), summary);
    exportGeometryCsv(system_config_.output_dir + "/geometry_points.csv", summary);
    exportStage1HeatmapCsv(system_config_.output_dir + "/stage1_heatmap.csv", summary.heatmap_samples);
  }

  return true;
}

moveit::core::RobotState TwoStagePlannerManager::buildRobotState(const std::vector<double>& q) const
{
  moveit::core::RobotState state(kinematics_.getRobotModel());
  state.setToDefaultValues();
  state.setJointGroupPositions(kinematics_.getJointModelGroup(), q);
  state.update();
  return state;
}

geometry_msgs::msg::Pose TwoStagePlannerManager::buildTargetPose() const
{
  if (algorithm_config_.use_goal_state_as_target_pose) {
    return eigenToPose(
      kinematics_.getLinkTransform(algorithm_config_.goal_joint_target, kinematics_.getTipLinkName()));
  }

  geometry_msgs::msg::Pose pose;
  pose.position.x = algorithm_config_.target_position[0];
  pose.position.y = algorithm_config_.target_position[1];
  pose.position.z = algorithm_config_.target_position[2];
  pose.orientation.x = algorithm_config_.target_orientation[0];
  pose.orientation.y = algorithm_config_.target_orientation[1];
  pose.orientation.z = algorithm_config_.target_orientation[2];
  pose.orientation.w = algorithm_config_.target_orientation[3];
  return pose;
}

bool TwoStagePlannerManager::planStage1(
  const std::vector<double>& q_start,
  const std::vector<double>& q_pre,
  moveit::planning_interface::MoveGroupInterface::Plan& stage1_plan) const
{
  // 警告：
  // 每次阶段调用均创建本地 MoveGroup 客户端。
  // 共享长生命周期客户端曾导致 action 目标响应冲突。
  auto stage1_node = rclcpp::Node::make_shared(
    "stage1_move_group_client",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(stage1_node);
  std::thread spinner([&executor]() { executor.spin(); });

  moveit::planning_interface::MoveGroupInterface::Options options(
    system_config_.stage1_group_name, "robot_description");
  options.robot_model_ = kinematics_.getRobotModel();
  moveit::planning_interface::MoveGroupInterface stage1_group(stage1_node, options);
  stage1_group.setPlanningTime(system_config_.planning_time);
  stage1_group.setNumPlanningAttempts(system_config_.planning_attempts);
  stage1_group.setMaxVelocityScalingFactor(system_config_.velocity_scaling);
  stage1_group.setMaxAccelerationScalingFactor(system_config_.acceleration_scaling);
  stage1_group.setPoseReferenceFrame(system_config_.planning_frame);

  moveit::core::RobotState start_state = buildRobotState(q_start);
  stage1_group.setStartState(start_state);

  const Eigen::Isometry3d pre_tf = kinematics_.getLinkTransform(q_pre, kinematics_.getTipLinkName());
  const Eigen::Quaterniond q_pre_rot(pre_tf.rotation());

  moveit_msgs::msg::Constraints constraints;
  constraints.name = "stage1_upright";
  moveit_msgs::msg::OrientationConstraint oc;
  oc.link_name = kinematics_.getTipLinkName();
  oc.header.frame_id = system_config_.planning_frame;
  oc.orientation.x = q_pre_rot.x();
  oc.orientation.y = q_pre_rot.y();
  oc.orientation.z = q_pre_rot.z();
  oc.orientation.w = q_pre_rot.w();
  oc.absolute_x_axis_tolerance = 0.35;
  oc.absolute_y_axis_tolerance = 0.35;
  oc.absolute_z_axis_tolerance = M_PI;
  oc.weight = 1.0;
  constraints.orientation_constraints.push_back(oc);

  stage1_group.setJointValueTarget(q_pre);
  stage1_group.setPathConstraints(constraints);
  // 优先尝试带约束规划，以保持 stage1 姿态设计意图。
  bool ok =
    (stage1_group.plan(stage1_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  stage1_group.clearPathConstraints();
  if (!ok) {
    // 回退策略：
    // 若 upright 约束导致规划过度受限，则允许无约束搜索，
    // 在保持终点 q_pre 不变的前提下提升可用性。
    RCLCPP_WARN(
      node_->get_logger(),
      "Stage1 constrained planning failed. Falling back to unconstrained stage1_group planning.");
    moveit::planning_interface::MoveGroupInterface retry_group(stage1_node, options);
    retry_group.setPlanningTime(system_config_.planning_time);
    retry_group.setNumPlanningAttempts(system_config_.planning_attempts);
    retry_group.setMaxVelocityScalingFactor(system_config_.velocity_scaling);
    retry_group.setMaxAccelerationScalingFactor(system_config_.acceleration_scaling);
    retry_group.setPoseReferenceFrame(system_config_.planning_frame);
    retry_group.setStartState(start_state);
    retry_group.setJointValueTarget(q_pre);
    ok = (retry_group.plan(stage1_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  executor.cancel();
  spinner.join();
  return ok;
}

bool TwoStagePlannerManager::planStage2(
  const std::vector<double>& q_stage1_end,
  const std::vector<double>& q_pre,
  const std::vector<double>& q_goal_stage2,
  moveit::planning_interface::MoveGroupInterface::Plan& stage2_plan) const
{
  // stage2 强制阶段分离：q1/q2 保持接近 q_pre，由 q3/q4 完成目标位姿恢复。
  auto stage2_node = rclcpp::Node::make_shared(
    "stage2_move_group_client",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(stage2_node);
  std::thread spinner([&executor]() { executor.spin(); });

  moveit::planning_interface::MoveGroupInterface::Options options(
    system_config_.stage2_group_name, "robot_description");
  options.robot_model_ = kinematics_.getRobotModel();
  moveit::planning_interface::MoveGroupInterface stage2_group(stage2_node, options);
  stage2_group.setPlanningTime(system_config_.planning_time);
  stage2_group.setNumPlanningAttempts(system_config_.planning_attempts);
  stage2_group.setMaxVelocityScalingFactor(system_config_.velocity_scaling);
  stage2_group.setMaxAccelerationScalingFactor(system_config_.acceleration_scaling);
  stage2_group.setPoseReferenceFrame(system_config_.planning_frame);

  moveit::core::RobotState start_state = buildRobotState(q_stage1_end);
  stage2_group.setStartState(start_state);

  moveit_msgs::msg::Constraints constraints;
  constraints.name = "stage2_lock_q12";

  moveit_msgs::msg::JointConstraint jc1;
  jc1.joint_name = algorithm_config_.joint1_name;
  jc1.position = q_pre[0];
  jc1.tolerance_above = system_config_.stage2_q12_tolerance;
  jc1.tolerance_below = system_config_.stage2_q12_tolerance;
  jc1.weight = 1.0;
  constraints.joint_constraints.push_back(jc1);

  moveit_msgs::msg::JointConstraint jc2;
  jc2.joint_name = algorithm_config_.joint2_name;
  jc2.position = q_pre[1];
  jc2.tolerance_above = system_config_.stage2_q12_tolerance;
  jc2.tolerance_below = system_config_.stage2_q12_tolerance;
  jc2.weight = 1.0;
  constraints.joint_constraints.push_back(jc2);

  stage2_group.setPathConstraints(constraints);
  stage2_group.setJointValueTarget(q_goal_stage2);
  const bool ok =
    (stage2_group.plan(stage2_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  stage2_group.clearPathConstraints();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  executor.cancel();
  spinner.join();
  return ok;
}

std::vector<double> TwoStagePlannerManager::extractJointPositionsFromTrajectory(
  const moveit_msgs::msg::RobotTrajectory& trajectory) const
{
  const auto& joint_traj = trajectory.joint_trajectory;
  if (joint_traj.points.empty()) {
    throw std::runtime_error("Trajectory has no points.");
  }
  return joint_traj.points.back().positions;
}

std::vector<TrajPoint> TwoStagePlannerManager::convertRobotTrajectoryToTrajPoints(
  const moveit_msgs::msg::RobotTrajectory& trajectory,
  int stage_id,
  double time_offset) const
{
  std::vector<TrajPoint> points;
  const auto& joint_traj = trajectory.joint_trajectory;
  points.reserve(joint_traj.points.size());
  for (const auto& point : joint_traj.points) {
    TrajPoint tp;
    tp.stage_id = stage_id;
    tp.t =
      time_offset +
      static_cast<double>(point.time_from_start.sec) +
      static_cast<double>(point.time_from_start.nanosec) * 1e-9;
    tp.q = point.positions;
    tp.dq = point.velocities;
    tp.ddq = point.accelerations;
    points.push_back(tp);
  }
  return points;
}

void TwoStagePlannerManager::publishDisplayTrajectories(
  const moveit_msgs::msg::RobotTrajectory& stage1_traj,
  const moveit_msgs::msg::RobotTrajectory& stage2_traj,
  const std::vector<double>& q_start) const
{
  // 两阶段轨迹一并发布，便于在 RViz 中直观看到阶段切换序列。
  moveit_msgs::msg::DisplayTrajectory msg;
  moveit::core::RobotState start_state = buildRobotState(q_start);
  moveit::core::robotStateToRobotStateMsg(start_state, msg.trajectory_start);
  msg.model_id = kinematics_.getRobotModel()->getName();
  msg.trajectory.push_back(stage1_traj);
  msg.trajectory.push_back(stage2_traj);
  display_pub_->publish(msg);
}

void TwoStagePlannerManager::publishDebugMarkers(
  const PlanningSummary& summary,
  const moveit_msgs::msg::RobotTrajectory& stage1_traj,
  const moveit_msgs::msg::RobotTrajectory& stage2_traj) const
{
  // 标记约定：
  // - 球体：stage1 几何逻辑关键点
  // - 折线：各阶段末端轨迹
  visualization_msgs::msg::MarkerArray array;
  const std::string frame = system_config_.planning_frame;

  array.markers.push_back(makeSphereMarker(
    frame, 0, summary.p_d.x(), summary.p_d.y(), summary.p_d.z(), 1.0f, 0.0f, 0.0f, "goal"));
  array.markers.push_back(makeSphereMarker(
    frame, 1, summary.p_d_proj_goal_axis.x(), summary.p_d_proj_goal_axis.y(), summary.p_d_proj_goal_axis.z(),
    1.0f, 0.5f, 0.0f, "goal_projection"));
  array.markers.push_back(makeSphereMarker(
    frame, 2, summary.o4_pre.x(), summary.o4_pre.y(), summary.o4_pre.z(), 0.0f, 1.0f, 0.0f, "stage1_pre"));
  array.markers.push_back(makeSphereMarker(
    frame, 3, summary.o4_proj_pre.x(), summary.o4_proj_pre.y(), summary.o4_proj_pre.z(),
    0.0f, 1.0f, 1.0f, "stage1_projection"));

  const auto stage1_points = toPoints(kinematics_, stage1_traj, kinematics_.getTipLinkName());
  const auto stage2_points = toPoints(kinematics_, stage2_traj, kinematics_.getTipLinkName());
  array.markers.push_back(makeLineStripMarker(frame, 10, stage1_points, 0.1f, 0.9f, 0.1f, "stage_paths"));
  array.markers.push_back(makeLineStripMarker(frame, 11, stage2_points, 0.1f, 0.1f, 0.9f, "stage_paths"));

  marker_pub_->publish(array);
}

}  // namespace trunk_two_stage_planner
