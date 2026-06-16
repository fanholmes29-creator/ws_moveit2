#include "trunk_two_stage_planner/two_stage_planner_manager.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <cctype>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <unordered_set>

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

std::string joinStrings(const std::vector<std::string>& values)
{
  std::ostringstream oss;
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i > 0) {
      oss << ", ";
    }
    oss << values[i];
  }
  return oss.str();
}

std::string joinDoubles(const std::vector<double>& values)
{
  std::ostringstream oss;
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i > 0) {
      oss << ", ";
    }
    oss << values[i];
  }
  return oss.str();
}

std::string normalizeMode(std::string mode)
{
  std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return mode;
}

struct JointStateValidationResult
{
  bool accepted = false;
  std::vector<double> positions;
  std::vector<std::string> unknown_joints;
  std::string reject_reason;
};

JointStateValidationResult validateJointStateMessage(
  const sensor_msgs::msg::JointState& msg,
  const std::vector<std::string>& expected_joint_names,
  bool strict_joint_states)
{
  JointStateValidationResult result;

  if (msg.name.size() != msg.position.size()) {
    result.reject_reason =
      "name/position size mismatch: " + std::to_string(msg.name.size()) + " vs " +
      std::to_string(msg.position.size());
    return result;
  }

  if (expected_joint_names.empty()) {
    result.reject_reason = "expected_joint_names is empty";
    return result;
  }

  std::unordered_set<std::string> expected_set(
    expected_joint_names.begin(), expected_joint_names.end());
  for (const auto& name : msg.name) {
    if (expected_set.find(name) == expected_set.end()) {
      result.unknown_joints.push_back(name);
    }
  }

  if (strict_joint_states && !result.unknown_joints.empty()) {
    result.reject_reason = "unknown joints in strict mode: " + joinStrings(result.unknown_joints);
    return result;
  }

  result.positions.assign(expected_joint_names.size(), 0.0);
  for (std::size_t i = 0; i < expected_joint_names.size(); ++i) {
    const auto it = std::find(msg.name.begin(), msg.name.end(), expected_joint_names[i]);
    if (it == msg.name.end()) {
      result.reject_reason = "missing expected joint: " + expected_joint_names[i];
      return result;
    }

    const std::size_t index = static_cast<std::size_t>(std::distance(msg.name.begin(), it));
    const double value = msg.position[index];
    if (!std::isfinite(value)) {
      result.reject_reason = "non-finite position for joint: " + expected_joint_names[i];
      return result;
    }
    result.positions[i] = value;
  }

  result.accepted = true;
  return result;
}

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

geometry_msgs::msg::Pose interpolatePose(
  const geometry_msgs::msg::Pose& a,
  const geometry_msgs::msg::Pose& b,
  double t)
{
  const double clamped_t = std::clamp(t, 0.0, 1.0);
  geometry_msgs::msg::Pose pose;
  pose.position.x = a.position.x + (b.position.x - a.position.x) * clamped_t;
  pose.position.y = a.position.y + (b.position.y - a.position.y) * clamped_t;
  pose.position.z = a.position.z + (b.position.z - a.position.z) * clamped_t;

  const Eigen::Quaterniond qa = poseToEigenQuaternion(a).normalized();
  const Eigen::Quaterniond qb = poseToEigenQuaternion(b).normalized();
  const Eigen::Quaterniond q_interp = qa.slerp(clamped_t, qb).normalized();
  pose.orientation.x = q_interp.x();
  pose.orientation.y = q_interp.y();
  pose.orientation.z = q_interp.z();
  pose.orientation.w = q_interp.w();
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
  system_config_.auto_control_mode = normalizeMode(system_config_.auto_control_mode);

  if (!kinematics_.initialize(node_, algorithm_config_)) {
    return false;
  }

  if (system_config_.expected_joint_names.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "expected_joint_names must not be empty.");
    return false;
  }
  for (const auto& joint_name : system_config_.expected_joint_names) {
    std::size_t joint_index = 0;
    if (!kinematics_.getJointIndex(joint_name, joint_index)) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Expected joint '%s' is not part of planning group '%s'.",
        joint_name.c_str(),
        algorithm_config_.group_name.c_str());
      return false;
    }
  }
  if (system_config_.expected_joint_names != kinematics_.getJointNames()) {
    RCLCPP_WARN(
      node_->get_logger(),
      "expected_joint_names order differs from planning group joint order. "
      "Cached live start states will follow expected_joint_names order: [%s]",
      joinStrings(system_config_.expected_joint_names).c_str());
  }

  algorithm_ = std::make_unique<TwoStagePlanner>(kinematics_, algorithm_config_);

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  display_pub_ = node_->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
    system_config_.display_trajectory_topic, qos);
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    system_config_.marker_topic, qos);
  joint_traj_pub_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    system_config_.joint_trajectory_topic, qos);
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    system_config_.joint_states_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&TwoStagePlannerManager::jointStateCallback, this, std::placeholders::_1));
  control_mode_sub_ = node_->create_subscription<std_msgs::msg::String>(
    system_config_.control_mode_state_topic,
    rclcpp::QoS(1).transient_local().reliable(),
    std::bind(&TwoStagePlannerManager::controlModeStateCallback, this, std::placeholders::_1));
  if (system_config_.republish_display_trajectory) {
    display_republish_timer_ = node_->create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&TwoStagePlannerManager::republishLatestDisplayTrajectory, this));
  }
  return true;
}

bool TwoStagePlannerManager::planTwoStageToTarget(const geometry_msgs::msg::Pose& target_pose)
{
  return planTwoStageToTargetDetailed(target_pose).success;
}

PlannerResult TwoStagePlannerManager::planTwoStageToTargetDetailed(
  const geometry_msgs::msg::Pose& target_pose)
{
  return planTwoStageToTargetDetailed(
    target_pose, system_config_.execute_joint_trajectory);
}

PlannerResult TwoStagePlannerManager::planTwoStageToTargetDetailed(
  const geometry_msgs::msg::Pose& target_pose,
  bool execute_trajectory)
{
  // 运行主链路：
  // 1) 算法层求解 q_pre 与 stage2 目标
  // 2) MoveIt 先规划 stage1，再规划 stage2
  // 3) 发布/导出 RViz 与调试输出
  if (!algorithm_) {
    return PlannerResult::fail(
      PlannerError::NotInitialized,
      "TwoStagePlannerManager is not initialized.");
  }

  std::vector<double> q_start;
  bool using_live_start = false;
  const bool execution_requested = execute_trajectory;
  const bool should_wait_for_live_start =
    system_config_.use_live_joint_state_as_start || execution_requested;

  if (execution_requested && !system_config_.use_live_joint_state_as_start) {
    RCLCPP_WARN(
      node_->get_logger(),
      "execute_joint_trajectory is true, so live joint state is required even though "
      "use_live_joint_state_as_start is false.");
  }

  if (should_wait_for_live_start) {
    using_live_start = waitForCurrentJointState(q_start);
    if (!using_live_start) {
      if (execution_requested) {
        RCLCPP_ERROR(
          node_->get_logger(),
          "Refusing to execute: no valid live joint state was received from %s within %.2f s. "
          "Set execute_joint_trajectory:=false to allow planning-only fallback to q_start.",
          system_config_.joint_states_topic.c_str(),
          system_config_.joint_state_wait_timeout_sec);
        return PlannerResult::fail(
          PlannerError::StartStateUnavailable,
          "Execution requires a valid live joint state.");
      }

      if (!system_config_.allow_start_state_fallback_to_config) {
        RCLCPP_ERROR(
          node_->get_logger(),
          "Failed to acquire live joint state from %s within %.2f s.",
          system_config_.joint_states_topic.c_str(),
          system_config_.joint_state_wait_timeout_sec);
        return PlannerResult::fail(
          PlannerError::StartStateUnavailable,
          "Failed to acquire live joint state and start-state fallback is disabled.");
      }

      q_start = algorithm_config_.q_start;
      RCLCPP_WARN(
        node_->get_logger(),
        "Live joint state unavailable on %s; falling back to configured q_start because "
        "execute_joint_trajectory is false.",
        system_config_.joint_states_topic.c_str());
    }
  } else {
    q_start = algorithm_config_.q_start;
    RCLCPP_INFO(
      node_->get_logger(),
      "Using configured q_start because use_live_joint_state_as_start and "
      "execute_joint_trajectory are both false. q_start=[%s]",
      joinDoubles(q_start).c_str());
  }

  if (using_live_start) {
    RCLCPP_INFO(
      node_->get_logger(),
      "Using live joint state from %s as planning start. strict_joint_states=%s, "
      "expected_joint_names=[%s], q_start=[%s]",
      system_config_.joint_states_topic.c_str(),
      system_config_.strict_joint_states ? "true" : "false",
      joinStrings(system_config_.expected_joint_names).c_str(),
      joinDoubles(q_start).c_str());
  }

  RCLCPP_INFO(node_->get_logger(), "Solving two-stage algorithm target...");
  const PlannerResult algorithm_result = algorithm_->planDetailed(q_start, target_pose);
  if (!algorithm_result.success) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Algorithm layer failed: [%s] %s",
      plannerErrorToString(algorithm_result.error),
      algorithm_result.message.c_str());
    return algorithm_result;
  }
  PlanningSummary summary = algorithm_result.summary;
  RCLCPP_INFO(
    node_->get_logger(),
    "Algorithm solved. q_pre=[%s], q_goal_stage2=[%s]",
    joinDoubles(summary.q_pre).c_str(),
    joinDoubles(summary.q_goal_stage2).c_str());

  moveit_msgs::msg::RobotTrajectory stage1_traj;
  moveit::planning_interface::MoveGroupInterface::Plan stage1_plan;
  RCLCPP_INFO(node_->get_logger(), "Planning stage1 with MoveIt group '%s'...", system_config_.stage1_group_name.c_str());
  if (!planStage1(q_start, summary.q_pre, stage1_plan)) {
    RCLCPP_ERROR(node_->get_logger(), "Stage1 planning failed.");
    return PlannerResult::fail(
      PlannerError::Stage1MoveItPlanningFailed,
      "Stage1 MoveIt planning failed.",
      summary);
  }
  stage1_traj = stage1_plan.trajectory_;
  RCLCPP_INFO(node_->get_logger(), "Stage1 planning succeeded.");

  const std::vector<double> q_stage1_end = extractJointPositionsFromTrajectory(stage1_traj);
  moveit::planning_interface::MoveGroupInterface::Plan stage2_plan;
  RCLCPP_INFO(node_->get_logger(), "Planning stage2 with MoveIt group '%s'...", system_config_.stage2_group_name.c_str());
  if (!planStage2(q_stage1_end, summary.q_pre, summary.q_goal_stage2, stage2_plan)) {
    RCLCPP_ERROR(node_->get_logger(), "Stage2 planning failed.");
    return PlannerResult::fail(
      PlannerError::Stage2MoveItPlanningFailed,
      "Stage2 MoveIt planning failed.",
      summary);
  }
  RCLCPP_INFO(node_->get_logger(), "Stage2 planning succeeded.");

  publishDisplayTrajectories(stage1_traj, stage2_plan.trajectory_, q_start);
  publishDebugMarkers(summary, stage1_traj, stage2_plan.trajectory_);
  const auto merged_joint_trajectory =
    concatenateJointTrajectories(stage1_traj, stage2_plan.trajectory_);
  joint_traj_pub_->publish(merged_joint_trajectory);
  {
    std::lock_guard<std::mutex> lock(cached_trajectory_mutex_);
    cached_joint_trajectory_ = merged_joint_trajectory;
    cached_summary_ = summary;
    has_cached_joint_trajectory_ = !merged_joint_trajectory.points.empty();
  }
  bool execution_skipped_by_control_mode = false;
  const bool execution_allowed_by_mode =
    !execution_requested || isAutoExecutionAllowed();
  if (execution_requested && !execution_allowed_by_mode) {
    execution_skipped_by_control_mode = true;
    RCLCPP_WARN(
      node_->get_logger(),
      "Planning succeeded and trajectory was published, but FollowJointTrajectory execution was "
      "skipped because control mode is not '%s'.",
      system_config_.auto_control_mode.c_str());
  }
  if (
    execution_requested && execution_allowed_by_mode &&
    !executeJointTrajectory(merged_joint_trajectory))
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to execute merged joint trajectory.");
    return PlannerResult::fail(
      PlannerError::TrajectoryExecutionFailed,
      "Failed to execute merged joint trajectory.",
      summary);
  }
  if (execution_requested && execution_allowed_by_mode) {
    clearDisplayTrajectory();
  }

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

  if (execution_skipped_by_control_mode) {
    return PlannerResult::ok(
      summary,
      "Planning succeeded; execution skipped because control mode is not " +
      system_config_.auto_control_mode + ".");
  }
  return PlannerResult::ok(summary);
}

PlannerResult TwoStagePlannerManager::executeCachedTrajectory()
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  PlanningSummary summary;
  {
    std::lock_guard<std::mutex> lock(cached_trajectory_mutex_);
    if (!has_cached_joint_trajectory_ || cached_joint_trajectory_.points.empty()) {
      return PlannerResult::fail(
        PlannerError::InvalidInput,
        "No previewed trajectory is cached. Run preview_plan_to_pose first.");
    }
    trajectory = cached_joint_trajectory_;
    summary = cached_summary_;
  }
  if (!isAutoExecutionAllowed()) {
    return PlannerResult::fail(
      PlannerError::InvalidInput,
      "Execution requires control mode " + system_config_.auto_control_mode + ".",
      summary);
  }
  if (!executeJointTrajectory(trajectory)) {
    return PlannerResult::fail(
      PlannerError::TrajectoryExecutionFailed,
      "Failed to execute cached joint trajectory.",
      summary);
  }
  clearDisplayTrajectory();
  {
    std::lock_guard<std::mutex> lock(cached_trajectory_mutex_);
    has_cached_joint_trajectory_ = true;
  }
  return PlannerResult::ok(summary, "Cached trajectory executed.");
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
    node_->get_namespace(),
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(stage1_node);
  std::thread spinner([&executor]() { executor.spin(); });

  moveit::planning_interface::MoveGroupInterface::Options options(
    system_config_.stage1_group_name, "robot_description", node_->get_namespace());
  options.robot_model_ = kinematics_.getRobotModel();
  RCLCPP_INFO(
    node_->get_logger(),
    "Connecting stage1 MoveGroupInterface to namespace '%s'...",
    node_->get_namespace());
  moveit::planning_interface::MoveGroupInterface stage1_group(stage1_node, options);
  RCLCPP_INFO(node_->get_logger(), "Stage1 MoveGroupInterface connected.");
  stage1_group.setPlanningTime(system_config_.planning_time);
  stage1_group.setNumPlanningAttempts(system_config_.planning_attempts);
  stage1_group.setMaxVelocityScalingFactor(system_config_.velocity_scaling);
  stage1_group.setMaxAccelerationScalingFactor(system_config_.acceleration_scaling);
  stage1_group.setPoseReferenceFrame(system_config_.planning_frame);
  stage1_group.setEndEffectorLink(system_config_.stage1_reference_link);
  stage1_group.setPlannerId("RRTConnectkConfigDefault");   // 默认
// 或根据参数改成 "RRTstarkConfigDefault" / "PRMkConfigDefault"

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

  if (system_config_.stage1_lock_q4) {
    std::size_t q4_index = 0;
    if (!kinematics_.getJointIndex(algorithm_config_.joint4_name, q4_index) ||
        q4_index >= q_start.size()) {
      throw std::runtime_error("Failed to resolve joint4 index for stage1 q4 lock.");
    }
    moveit_msgs::msg::JointConstraint jc4;
    jc4.joint_name = algorithm_config_.joint4_name;
    jc4.position = q_start[q4_index];
    jc4.tolerance_above = system_config_.stage1_q4_tolerance;
    jc4.tolerance_below = system_config_.stage1_q4_tolerance;
    jc4.weight = 1.0;
    constraints.joint_constraints.push_back(jc4);
  }

  bool ok = false;
  if (system_config_.stage1_use_cartesian) {
    const auto waypoints = buildStage1CartesianWaypoints(q_start, q_pre);
    if (waypoints.size() >= 2) {
      RCLCPP_INFO(
        node_->get_logger(),
        "Stage1 computing Cartesian path with %zu waypoints...",
        waypoints.size());
      stage1_group.setPathConstraints(constraints);
      moveit_msgs::msg::RobotTrajectory cart_traj;
      const double fraction = stage1_group.computeCartesianPath(
        waypoints,
        system_config_.stage1_eef_step,
        system_config_.stage1_jump_threshold,
        cart_traj,
        true);
      stage1_group.clearPathConstraints();
      RCLCPP_INFO(
        node_->get_logger(),
        "Stage1 Cartesian fraction=%.3f mode=%s reference_link=%s",
        fraction,
        system_config_.stage1_cartesian_mode.c_str(),
        system_config_.stage1_reference_link.c_str());
      if (fraction >= system_config_.stage1_min_fraction && !cart_traj.joint_trajectory.points.empty()) {
        ok = timeParameterizeTrajectory(q_start, system_config_.stage1_group_name, cart_traj);
        if (ok) {
          stage1_plan.trajectory_ = cart_traj;
        }
      }
    }
  }

  if (!ok) {
    RCLCPP_INFO(
      node_->get_logger(),
      "Stage1 planning constrained joint target with planning_time=%.2f attempts=%d...",
      system_config_.planning_time,
      system_config_.planning_attempts);
    stage1_group.setJointValueTarget(q_pre);
    stage1_group.setPathConstraints(constraints);
    // 优先尝试带约束规划，以保持 stage1 姿态设计意图。
    ok = (stage1_group.plan(stage1_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    stage1_group.clearPathConstraints();
  }

  if (!ok) {
    // 回退策略：
    // 若 upright 约束导致规划过度受限，则允许无约束搜索，
    // 在保持终点 q_pre 不变的前提下提升可用性。
    RCLCPP_WARN(
      node_->get_logger(),
      "Stage1 constrained planning failed. Falling back to unconstrained stage1_group planning.");
    moveit::planning_interface::MoveGroupInterface retry_group(stage1_node, options);
    RCLCPP_INFO(node_->get_logger(), "Stage1 retry MoveGroupInterface connected.");
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
    node_->get_namespace(),
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(stage2_node);
  std::thread spinner([&executor]() { executor.spin(); });

  moveit::planning_interface::MoveGroupInterface::Options options(
    system_config_.stage2_group_name, "robot_description", node_->get_namespace());
  options.robot_model_ = kinematics_.getRobotModel();
  RCLCPP_INFO(
    node_->get_logger(),
    "Connecting stage2 MoveGroupInterface to namespace '%s'...",
    node_->get_namespace());
  moveit::planning_interface::MoveGroupInterface stage2_group(stage2_node, options);
  RCLCPP_INFO(node_->get_logger(), "Stage2 MoveGroupInterface connected.");
  stage2_group.setPlanningTime(system_config_.planning_time);
  stage2_group.setNumPlanningAttempts(system_config_.planning_attempts);
  stage2_group.setMaxVelocityScalingFactor(system_config_.velocity_scaling);
  stage2_group.setMaxAccelerationScalingFactor(system_config_.acceleration_scaling);
  stage2_group.setPoseReferenceFrame(system_config_.planning_frame);
  stage2_group.setPlannerId("RRTConnectkConfigDefault");   // 默认
// 或根据参数改成 "RRTstarkConfigDefault" / "PRMkConfigDefault"

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
  RCLCPP_INFO(
    node_->get_logger(),
    "Stage2 planning constrained joint target with planning_time=%.2f attempts=%d...",
    system_config_.planning_time,
    system_config_.planning_attempts);
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

  {
    std::lock_guard<std::mutex> lock(display_trajectory_mutex_);
    latest_display_trajectory_ = msg;
    has_latest_display_trajectory_ = true;
    cached_display_trajectory_ = msg;
    has_cached_display_trajectory_ = true;
  }
  RCLCPP_INFO(
    node_->get_logger(),
    "Published DisplayTrajectory with %zu stage trajectories on %s. It will be republished "
    "periodically while the planner node is alive so RViz can reconnect and animate it.",
    msg.trajectory.size(),
    system_config_.display_trajectory_topic.c_str());
}

void TwoStagePlannerManager::republishLatestDisplayTrajectory() const
{
  moveit_msgs::msg::DisplayTrajectory msg;
  {
    std::lock_guard<std::mutex> lock(display_trajectory_mutex_);
    if (!has_latest_display_trajectory_) {
      return;
    }
    msg = latest_display_trajectory_;
  }
  display_pub_->publish(msg);
}

void TwoStagePlannerManager::clearDisplayTrajectory() const
{
  // RViz's DisplayTrajectory panel can keep and loop the last trajectory in its
  // own cache. Publishing empty or single-point DisplayTrajectory messages is
  // not a reliable "clear" command across RViz configurations. Treat this as a
  // display pause: stop marking any trajectory as actively displayed, and clear
  // the debug marker overlay that we control. The cached trajectory remains
  // available and can be displayed again via setCachedTrajectoryDisplay(true).
  {
    std::lock_guard<std::mutex> lock(display_trajectory_mutex_);
    has_latest_display_trajectory_ = false;
  }

  visualization_msgs::msg::MarkerArray clear_markers;
  visualization_msgs::msg::Marker marker;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  clear_markers.markers.push_back(marker);
  marker_pub_->publish(clear_markers);
  RCLCPP_INFO(
    node_->get_logger(),
    "Paused cached trajectory display and cleared debug markers. "
    "The latest planned trajectory remains cached and can be displayed again.");
}

PlannerResult TwoStagePlannerManager::setCachedTrajectoryDisplay(bool show) const
{
  if (!show) {
    clearDisplayTrajectory();
    return PlannerResult::ok(PlanningSummary(), "Trajectory display paused.");
  }

  moveit_msgs::msg::DisplayTrajectory display_msg;
  visualization_msgs::msg::MarkerArray marker_msg;
  bool has_markers = false;
  {
    std::lock_guard<std::mutex> lock(display_trajectory_mutex_);
    if (!has_cached_display_trajectory_) {
      return PlannerResult::fail(
        PlannerError::InvalidInput,
        "No cached planned trajectory is available. Run preview_plan_to_pose first.");
    }
    display_msg = cached_display_trajectory_;
    marker_msg = cached_debug_markers_;
    has_markers = has_cached_debug_markers_;
    latest_display_trajectory_ = cached_display_trajectory_;
    has_latest_display_trajectory_ = true;
  }
  display_pub_->publish(display_msg);
  if (has_markers) {
    marker_pub_->publish(marker_msg);
  }
  RCLCPP_INFO(
    node_->get_logger(),
    "Displayed cached planned trajectory with %zu stage trajectories on %s.",
    display_msg.trajectory.size(),
    system_config_.display_trajectory_topic.c_str());
  return PlannerResult::ok(PlanningSummary(), "Cached planned trajectory displayed.");
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
  {
    std::lock_guard<std::mutex> lock(display_trajectory_mutex_);
    cached_debug_markers_ = array;
    has_cached_debug_markers_ = true;
  }
}

std::vector<geometry_msgs::msg::Pose> TwoStagePlannerManager::buildStage1CartesianWaypoints(
  const std::vector<double>& q_start,
  const std::vector<double>& q_pre) const
{
  std::vector<geometry_msgs::msg::Pose> waypoints;
  const geometry_msgs::msg::Pose start_pose = eigenToPose(
    kinematics_.getLinkTransform(q_start, system_config_.stage1_reference_link));
  const geometry_msgs::msg::Pose goal_pose = eigenToPose(
    kinematics_.getLinkTransform(q_pre, system_config_.stage1_reference_link));

  const int samples = std::max(system_config_.stage1_waypoint_count, 2);
  std::string mode = system_config_.stage1_cartesian_mode;
  std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });

  waypoints.reserve(static_cast<std::size_t>(samples));
  if (mode == "arc") {
    const Eigen::Vector3d p0(start_pose.position.x, start_pose.position.y, start_pose.position.z);
    const Eigen::Vector3d p1(goal_pose.position.x, goal_pose.position.y, goal_pose.position.z);
    const Eigen::Vector3d chord = p1 - p0;
    Eigen::Vector3d lateral = chord.cross(Eigen::Vector3d::UnitZ());
    if (lateral.norm() < 1e-9) {
      lateral = chord.cross(Eigen::Vector3d::UnitY());
    }
    if (lateral.norm() < 1e-9) {
      lateral = Eigen::Vector3d::UnitX();
    }
    lateral.normalize();
    const Eigen::Vector3d control = 0.5 * (p0 + p1) + system_config_.stage1_arc_height * lateral;

    for (int i = 0; i < samples; ++i) {
      const double t = (samples <= 1) ? 0.0 : static_cast<double>(i) / (samples - 1);
      const Eigen::Vector3d point =
        (1.0 - t) * (1.0 - t) * p0 + 2.0 * (1.0 - t) * t * control + t * t * p1;
      geometry_msgs::msg::Pose pose = interpolatePose(start_pose, goal_pose, t);
      pose.position.x = point.x();
      pose.position.y = point.y();
      pose.position.z = point.z();
      waypoints.push_back(pose);
    }
    return waypoints;
  }

  for (int i = 0; i < samples; ++i) {
    const double t = (samples <= 1) ? 0.0 : static_cast<double>(i) / (samples - 1);
    waypoints.push_back(interpolatePose(start_pose, goal_pose, t));
  }
  return waypoints;
}

bool TwoStagePlannerManager::timeParameterizeTrajectory(
  const std::vector<double>& q_start,
  const std::string& group_name,
  moveit_msgs::msg::RobotTrajectory& trajectory) const
{
  if (trajectory.joint_trajectory.points.empty()) {
    return false;
  }

  moveit::core::RobotState start_state = buildRobotState(q_start);
  robot_trajectory::RobotTrajectory robot_traj(kinematics_.getRobotModel(), group_name);
  robot_traj.setRobotTrajectoryMsg(start_state, trajectory);

  trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  const bool ok = totg.computeTimeStamps(
    robot_traj, system_config_.velocity_scaling, system_config_.acceleration_scaling);
  if (!ok) {
    return false;
  }

  robot_traj.getRobotTrajectoryMsg(trajectory);
  return true;
}

trajectory_msgs::msg::JointTrajectory TwoStagePlannerManager::concatenateJointTrajectories(
  const moveit_msgs::msg::RobotTrajectory& stage1_traj,
  const moveit_msgs::msg::RobotTrajectory& stage2_traj) const
{
  const auto& stage1_joint_traj = stage1_traj.joint_trajectory;
  const auto& stage2_joint_traj = stage2_traj.joint_trajectory;

  if (stage1_joint_traj.points.empty()) {
    return stage2_joint_traj;
  }
  if (stage2_joint_traj.points.empty()) {
    return stage1_joint_traj;
  }

  trajectory_msgs::msg::JointTrajectory merged = stage1_joint_traj;
  const auto& last_stage1_time = stage1_joint_traj.points.back().time_from_start;
  const int64_t stage1_offset_ns =
    static_cast<int64_t>(last_stage1_time.sec) * 1000000000LL +
    static_cast<int64_t>(last_stage1_time.nanosec);

  const std::size_t start_index = 1;  // Skip duplicated stage boundary point.
  for (std::size_t i = start_index; i < stage2_joint_traj.points.size(); ++i) {
    auto point = stage2_joint_traj.points[i];
    const int64_t point_ns =
      static_cast<int64_t>(point.time_from_start.sec) * 1000000000LL +
      static_cast<int64_t>(point.time_from_start.nanosec);
    const int64_t merged_ns = stage1_offset_ns + point_ns;
    point.time_from_start.sec = static_cast<int32_t>(merged_ns / 1000000000LL);
    point.time_from_start.nanosec = static_cast<uint32_t>(merged_ns % 1000000000LL);
    merged.points.push_back(point);
  }
  return merged;
}

bool TwoStagePlannerManager::executeJointTrajectory(
  const trajectory_msgs::msg::JointTrajectory& trajectory) const
{
  if (trajectory.points.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Cannot execute empty joint trajectory.");
    return false;
  }

  const auto unique_suffix = std::to_string(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
  auto exec_node = rclcpp::Node::make_shared(
    "joint_trajectory_exec_client_" + unique_suffix,
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(exec_node);
  std::thread spinner([&executor]() { executor.spin(); });

  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  const auto action_client =
    rclcpp_action::create_client<FollowJointTrajectory>(
      exec_node, system_config_.follow_joint_trajectory_action);

  const auto server_wait =
    std::chrono::duration<double>(std::max(0.0, system_config_.execute_action_server_wait_sec));
  if (!action_client->wait_for_action_server(server_wait)) {
    executor.cancel();
    spinner.join();
    RCLCPP_ERROR(
      node_->get_logger(),
      "FollowJointTrajectory action server %s was not available within %.2f s.",
      system_config_.follow_joint_trajectory_action.c_str(),
      system_config_.execute_action_server_wait_sec);
    return false;
  }

  FollowJointTrajectory::Goal goal;
  goal.trajectory = trajectory;
  auto goal_future = action_client->async_send_goal(goal);
  const auto result_wait =
    std::chrono::duration<double>(std::max(0.0, system_config_.execute_result_wait_sec));
  if (goal_future.wait_for(result_wait) != std::future_status::ready) {
    executor.cancel();
    spinner.join();
    RCLCPP_ERROR(
      node_->get_logger(),
      "Timed out while sending FollowJointTrajectory goal after %.2f s.",
      system_config_.execute_result_wait_sec);
    return false;
  }

  const auto goal_handle = goal_future.get();
  if (!goal_handle) {
    executor.cancel();
    spinner.join();
    RCLCPP_ERROR(node_->get_logger(), "FollowJointTrajectory goal was rejected by controller.");
    return false;
  }

  auto result_future = action_client->async_get_result(goal_handle);
  if (result_future.wait_for(result_wait) != std::future_status::ready) {
    executor.cancel();
    spinner.join();
    RCLCPP_ERROR(
      node_->get_logger(),
      "Timed out while waiting for FollowJointTrajectory result after %.2f s.",
      system_config_.execute_result_wait_sec);
    return false;
  }

  const auto wrapped_result = result_future.get();
  executor.cancel();
  spinner.join();

  if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "FollowJointTrajectory execution failed with code %d and error_code %d.",
      static_cast<int>(wrapped_result.code),
      wrapped_result.result ? wrapped_result.result->error_code : -1);
    return false;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "FollowJointTrajectory execution succeeded on %s.",
    system_config_.follow_joint_trajectory_action.c_str());
  return true;
}

void TwoStagePlannerManager::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  const JointStateValidationResult validation = validateJointStateMessage(
    *msg,
    system_config_.expected_joint_names,
    system_config_.strict_joint_states);

  if (!validation.accepted) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Rejected joint state message from %s: %s. expected_joint_names=[%s], received_names=[%s]",
      system_config_.joint_states_topic.c_str(),
      validation.reject_reason.c_str(),
      joinStrings(system_config_.expected_joint_names).c_str(),
      joinStrings(msg->name).c_str());
    return;
  }

  if (!system_config_.strict_joint_states &&
      system_config_.warn_unknown_joints &&
      !validation.unknown_joints.empty()) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Accepted joint state from %s with unknown joints ignored: [%s]",
      system_config_.joint_states_topic.c_str(),
      joinStrings(validation.unknown_joints).c_str());
  }

  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  latest_joint_state_ = validation.positions;
  has_latest_joint_state_ = true;
  if (!logged_first_joint_state_) {
    RCLCPP_INFO(
      node_->get_logger(),
      "Received live joint state from %s. expected_joint_names=[%s], q_current=[%s]",
      system_config_.joint_states_topic.c_str(),
      joinStrings(system_config_.expected_joint_names).c_str(),
      joinDoubles(latest_joint_state_).c_str());
    logged_first_joint_state_ = true;
  }
}

void TwoStagePlannerManager::controlModeStateCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  const std::string mode = normalizeMode(msg->data);
  std::lock_guard<std::mutex> lock(control_mode_mutex_);
  if (!has_control_mode_state_ || latest_control_mode_ != mode) {
    RCLCPP_INFO(
      node_->get_logger(), "Received control mode state: '%s' (auto required: '%s').",
      mode.c_str(), system_config_.auto_control_mode.c_str());
  }
  latest_control_mode_ = mode;
  has_control_mode_state_ = true;
}

bool TwoStagePlannerManager::getCurrentJointState(std::vector<double>& q_current) const
{
  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  if (!has_latest_joint_state_ ||
      latest_joint_state_.size() != system_config_.expected_joint_names.size()) {
    return false;
  }
  q_current = latest_joint_state_;
  return true;
}

bool TwoStagePlannerManager::waitForCurrentJointState(std::vector<double>& q_current) const
{
  const double timeout_sec = std::max(0.0, system_config_.joint_state_wait_timeout_sec);
  RCLCPP_INFO(
    node_->get_logger(),
    "Waiting up to %.2f s for live joint state from %s. expected_joint_names=[%s]",
    timeout_sec,
    system_config_.joint_states_topic.c_str(),
    joinStrings(system_config_.expected_joint_names).c_str());

  const auto wait_deadline =
    std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_sec);
  while (rclcpp::ok()) {
    if (getCurrentJointState(q_current)) {
      RCLCPP_INFO(
        node_->get_logger(),
        "Using latest live joint state from %s as planning start. q_start=[%s]",
        system_config_.joint_states_topic.c_str(),
        joinDoubles(q_current).c_str());
      return true;
    }

    if (std::chrono::steady_clock::now() >= wait_deadline) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return false;
}

bool TwoStagePlannerManager::waitForControlModeState() const
{
  if (!system_config_.require_control_mode) {
    return true;
  }

  const double timeout_sec = std::max(0.0, system_config_.control_mode_wait_timeout_sec);
  const auto wait_deadline =
    std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_sec);
  while (rclcpp::ok()) {
    {
      std::lock_guard<std::mutex> lock(control_mode_mutex_);
      if (has_control_mode_state_) {
        return true;
      }
    }

    if (std::chrono::steady_clock::now() >= wait_deadline) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return false;
}

bool TwoStagePlannerManager::isAutoExecutionAllowed() const
{
  if (!system_config_.require_control_mode) {
    return true;
  }
  if (!waitForControlModeState()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Refusing FollowJointTrajectory execution: no control mode state was received from '%s' "
      "within %.2f s.",
      system_config_.control_mode_state_topic.c_str(),
      system_config_.control_mode_wait_timeout_sec);
    return false;
  }

  std::lock_guard<std::mutex> lock(control_mode_mutex_);
  if (latest_control_mode_ != system_config_.auto_control_mode) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Refusing FollowJointTrajectory execution: current control mode is '%s', required '%s'.",
      latest_control_mode_.c_str(),
      system_config_.auto_control_mode.c_str());
    return false;
  }
  return true;
}

}  // namespace trunk_two_stage_planner
