#pragma once

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
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
  std::string marker_topic = "/two_stage_debug_markers";
  std::string display_trajectory_topic = "/display_planned_path";

  double planning_time = 5.0;
  int planning_attempts = 5;
  double velocity_scaling = 0.2;
  double acceleration_scaling = 0.2;

  double stage1_eef_step = 0.01;
  double stage1_jump_threshold = 0.0;
  double stage1_min_fraction = 0.95;
  int stage1_waypoint_count = 20;

  double stage2_q12_tolerance = 0.03;

  bool export_csv = true;
  std::string output_dir = "/home/wxl/ws_moveit2/csv/two_stage_system";
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

  rclcpp::Node::SharedPtr node_;
  PlannerConfig algorithm_config_;
  TwoStageSystemConfig system_config_;

  // 运动学 helper 持有模型/关节元信息，供算法与可视化共用。
  RobotKinematicsHelper kinematics_;
  // 算法对象仅在运动学初始化成功后创建。
  std::unique_ptr<TwoStagePlanner> algorithm_;

  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

}  // namespace trunk_two_stage_planner
