#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "trunk_two_stage_planner/robot_kinematics_helper.hpp"
#include "trunk_two_stage_planner/trajectory_utils.hpp"
#include "trunk_two_stage_planner/two_stage_planner.hpp"

namespace trunk_two_stage_planner
{

/**
 * @brief MoveIt 执行与可视化的运行时/工程化配置。
 *
 * 这些字段不直接改变算法数学形式，但会显著影响运行行为：
 *  - 规划鲁棒性与耗时
 *  - 阶段切换约束严格度（`stage2_q12_tolerance`）
 *  - 调试导出内容与 RViz 话题
 */
struct TwoStageSystemConfig
{
  std::string stage1_group_name = "stage1_group";
  std::string stage2_group_name = "stage2_group";
  std::string planning_frame = "chassis_base_link";
  std::string marker_topic = "two_stage_debug_markers";
  std::string display_trajectory_topic = "display_planned_path";
  std::string joint_trajectory_topic = "two_stage_joint_trajectory";

  double planning_time = 5.0;
  int planning_attempts = 5;
  double velocity_scaling = 0.2;
  double acceleration_scaling = 0.2;

  double stage1_eef_step = 0.01;
  double stage1_jump_threshold = 0.0;
  double stage1_min_fraction = 0.95;
  int stage1_waypoint_count = 20;
  bool stage1_use_cartesian = true;
  std::string stage1_cartesian_mode = "line";
  std::string stage1_reference_link = "trunk_link3";
  bool stage1_lock_q4 = true;
  double stage1_q4_tolerance = 0.05;
  double stage1_arc_height = 0.05;

  double stage2_q12_tolerance = 0.03;
  bool use_live_joint_state_as_start = true;
  bool allow_start_state_fallback_to_config = true;
  double live_start_state_wait_sec = 2.0;
  double joint_state_wait_timeout_sec = 2.0;
  std::string joint_states_topic = "joint_states";
  std::vector<std::string> expected_joint_names{
    "trunk_joint1", "trunk_joint2", "trunk_joint3", "trunk_joint4" };
  bool strict_joint_states = true;
  bool warn_unknown_joints = true;
  bool execute_joint_trajectory = true;
  std::string follow_joint_trajectory_action = "trunk_group_controller/follow_joint_trajectory";
  double execute_action_server_wait_sec = 5.0;
  double execute_result_wait_sec = 30.0;
  bool require_control_mode = true;
  std::string control_mode_state_topic = "control_mode_state";
  std::string auto_control_mode = "auto_plan_execute";
  double control_mode_wait_timeout_sec = 1.0;
  bool republish_display_trajectory = false;

  bool export_csv = false;
  std::string output_dir = "csv/two_stage_system";
};

/**
 * @brief 两阶段规划的工程调度层。
 *
 * 职责：
 *  - 初始化运动学与算法组件。
 *  - 调用算法层求解 `q_pre` 与 stage2 目标。
 *  - 以阶段约束执行 stage1/stage2 MoveIt 规划。
 *  - 发布轨迹/标记并导出诊断数据。
 *
 * 生命周期：
 *  - 由 ROS 节点构造。
 *  - 调用一次 `initialize(...)` 完成初始化。
 *  - 每个目标调用一次 `planTwoStageToTarget(...)`。
 *
 * 警告：
 *  - 本类将算法策略映射为 MoveIt 约束。
 *    若算法假设与约束配置不一致，容易引入回归问题。
 */
class TwoStagePlannerManager
{
public:
  explicit TwoStagePlannerManager(const rclcpp::Node::SharedPtr& node);

  /// 初始化 helper/planner 实例与发布器。
  bool initialize(const PlannerConfig& algorithm_config, const TwoStageSystemConfig& system_config);
  /// 端到端执行：算法求解 + MoveIt 两阶段规划 + 可视化/导出。
  bool planTwoStageToTarget(const geometry_msgs::msg::Pose& target_pose);
  /// 端到端执行，并返回明确失败原因与算法诊断摘要。
  PlannerResult planTwoStageToTargetDetailed(const geometry_msgs::msg::Pose& target_pose);
  /// 端到端规划，可选择是否执行；规划成功后缓存合并轨迹供 Web 二次确认执行。
  PlannerResult planTwoStageToTargetDetailed(
    const geometry_msgs::msg::Pose& target_pose,
    bool execute_trajectory);
  /// 执行最近一次规划缓存，并在完成后清除 RViz 规划显示。
  PlannerResult executeCachedTrajectory();
  /// 清除 RViz 中的规划轨迹/调试标记显示。
  void clearDisplayTrajectory() const;
  /// 展示或隐藏最近一次规划轨迹；执行后仍可重新展示最近规划结果。
  PlannerResult setCachedTrajectoryDisplay(bool show) const;

private:
  moveit::core::RobotState buildRobotState(const std::vector<double>& q) const;
  geometry_msgs::msg::Pose buildTargetPose() const;
  bool planStage1(
    const std::vector<double>& q_start,
    const std::vector<double>& q_pre,
    moveit::planning_interface::MoveGroupInterface::Plan& stage1_plan) const;
  // stage2 对 q1/q2 施加围绕 q_pre 的锁定策略，保证阶段划分语义不被破坏。
  bool planStage2(
    const std::vector<double>& q_stage1_end,
    const std::vector<double>& q_pre,
    const std::vector<double>& q_goal_stage2,
    moveit::planning_interface::MoveGroupInterface::Plan& stage2_plan) const;

  std::vector<double> extractJointPositionsFromTrajectory(
    const moveit_msgs::msg::RobotTrajectory& trajectory) const;
  std::vector<TrajPoint> convertRobotTrajectoryToTrajPoints(
    const moveit_msgs::msg::RobotTrajectory& trajectory,
    int stage_id,
    double time_offset = 0.0) const;

  void publishDisplayTrajectories(
    const moveit_msgs::msg::RobotTrajectory& stage1_traj,
    const moveit_msgs::msg::RobotTrajectory& stage2_traj,
    const std::vector<double>& q_start) const;
  void publishDebugMarkers(
    const PlanningSummary& summary,
    const moveit_msgs::msg::RobotTrajectory& stage1_traj,
    const moveit_msgs::msg::RobotTrajectory& stage2_traj) const;
  std::vector<geometry_msgs::msg::Pose> buildStage1CartesianWaypoints(
    const std::vector<double>& q_start,
    const std::vector<double>& q_pre) const;
  bool timeParameterizeTrajectory(
    const std::vector<double>& q_start,
    const std::string& group_name,
    moveit_msgs::msg::RobotTrajectory& trajectory) const;
  trajectory_msgs::msg::JointTrajectory concatenateJointTrajectories(
    const moveit_msgs::msg::RobotTrajectory& stage1_traj,
    const moveit_msgs::msg::RobotTrajectory& stage2_traj) const;
  bool executeJointTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory) const;
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void controlModeStateCallback(const std_msgs::msg::String::SharedPtr msg);
  bool getCurrentJointState(std::vector<double>& q_current) const;
  bool waitForCurrentJointState(std::vector<double>& q_current) const;
  bool waitForControlModeState() const;
  bool isAutoExecutionAllowed() const;
  void republishLatestDisplayTrajectory() const;

  rclcpp::Node::SharedPtr node_;
  PlannerConfig algorithm_config_;
  TwoStageSystemConfig system_config_;

  // 运动学 helper 持有模型/关节元信息，供算法与可视化共用。
  RobotKinematicsHelper kinematics_;
  // 算法对象仅在运动学初始化成功后创建。
  std::unique_ptr<TwoStagePlanner> algorithm_;

  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_traj_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_mode_sub_;
  rclcpp::TimerBase::SharedPtr display_republish_timer_;
  mutable std::mutex joint_state_mutex_;
  std::vector<double> latest_joint_state_;
  bool has_latest_joint_state_ = false;
  bool logged_first_joint_state_ = false;
  mutable std::mutex control_mode_mutex_;
  std::string latest_control_mode_ = "idle";
  bool has_control_mode_state_ = false;
  mutable std::mutex display_trajectory_mutex_;
  mutable moveit_msgs::msg::DisplayTrajectory latest_display_trajectory_;
  mutable bool has_latest_display_trajectory_ = false;
  mutable moveit_msgs::msg::DisplayTrajectory cached_display_trajectory_;
  mutable bool has_cached_display_trajectory_ = false;
  mutable visualization_msgs::msg::MarkerArray cached_debug_markers_;
  mutable bool has_cached_debug_markers_ = false;
  mutable std::mutex cached_trajectory_mutex_;
  trajectory_msgs::msg::JointTrajectory cached_joint_trajectory_;
  bool has_cached_joint_trajectory_ = false;
  PlanningSummary cached_summary_;
};

}  // namespace trunk_two_stage_planner
