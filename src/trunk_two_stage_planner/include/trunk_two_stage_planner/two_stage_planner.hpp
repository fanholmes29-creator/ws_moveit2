#pragma once

#include <vector>

#include <geometry_msgs/msg/pose.hpp>

#include "trunk_two_stage_planner/robot_kinematics_helper.hpp"
#include "trunk_two_stage_planner/types.hpp"

namespace trunk_two_stage_planner
{

/**
 * @brief 两阶段预备状态规划的核心算法层。
 *
 * 流程位置：
 *  - 在 MoveIt 分阶段规划之前由 `TwoStagePlannerManager` 调用。
 *  - 输出 stage1 预备状态（`q_pre`）与 stage2 目标（`q_goal_stage2`）。
 *
 * 输入：
 *  - 起始关节状态与笛卡尔目标位姿。
 *
 * 输出：
 *  - `PlanningSummary`，包含选中状态与诊断指标。
 *
 * 说明：
 *  - 本类有意与 MoveGroup 规划调用解耦。
 *  - 需要调策略阈值/权重时，应优先在本层调整，避免改 ROS 接口。
 */
class TwoStagePlanner
{
public:
  TwoStagePlanner(const RobotKinematicsHelper& kinematics, const PlannerConfig& config);

  /// 执行完整算法链：终点 IK -> stage1 搜索 -> stage2 锁定优化。
  bool plan(
    const std::vector<double>& q_start,
    const geometry_msgs::msg::Pose& target_pose,
    PlanningSummary& summary) const;
  /// 执行完整算法链，并返回明确失败原因与部分诊断摘要。
  PlannerResult planDetailed(
    const std::vector<double>& q_start,
    const geometry_msgs::msg::Pose& target_pose) const;

  /// 求解目标位姿 IK，并按正则化代价选出最优候选。
  bool solveFinalIK(
    const std::vector<double>& q_start,
    const geometry_msgs::msg::Pose& target_pose,
    std::vector<double>& q_goal,
    bool& used_fallback_goal) const;

  /// 返回指定关节在世界坐标系下的轴线，用于几何投影计算。
  Axis3D computeJointAxisInWorld(
    const std::vector<double>& q,
    const std::string& joint_name) const;

  /// 将三维点投影到无限长三维轴线上。
  Eigen::Vector3d projectPointToAxis(const Eigen::Vector3d& p, const Axis3D& axis) const;

  /// 依据 trunk 特定规则构造 stage1 候选：q3 = -(q1 + q2)，q4 固定。
  std::vector<double> composeStage1State(double q1, double q2, double q4_fix) const;

  /// 获取 stage1 几何参考点 O4(q)，通常为配置中的参考连杆位置。
  Eigen::Vector3d getStage1ReferencePoint(const std::vector<double>& q) const;

  /// 对接近关节限位的状态施加软惩罚；返回 inf 表示越界/无效。
  double jointLimitPenalty(const std::vector<double>& q) const;

  /**
   * @brief 回退排序时使用的 stage1 目标函数。
   *
   * 代价项：
   *  - 投影一致性（`w1`）
   *  - stage2 位置可恢复性代理项（`w2`）
   *  - 偏离起始姿态（`w3`）
   *  - 限位裕度惩罚（`w4`）
   *
   * 警告：
   *  - 调整 `w1..w4` 会改变安全/舒适/效率之间的平衡。
   *  - 建议小步调整，并结合 heatmap 对比结果。
   */
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

  /**
   * @brief 对 stage1 的 (q1,q2) 做网格搜索，并应用“阈值优先”筛选策略。
   *
   * 主策略：
   *  - 保留 stage2 位姿误差小于 `stage2_pose_epsilon` 的候选
   *  - 在可行子集中用过滤代价排序（投影 + 起点偏置 + 限位裕度）
   *  - 仅当不存在阈值可行候选时，回退到完整加权代价
   */
  bool searchStage1PreparatoryState(
    const std::vector<double>& q_start,
    const Eigen::Vector3d& p_goal_proj,
    const Eigen::Vector3d& p_d,
    const Eigen::Quaterniond& q_d,
    Stage1SearchResult& result) const;

private:
  /// 粗粒度可恢复性估计：固定 (q1,q2) 下的最小可达位置误差。
  double minStage2PositionErrorForFixedQ1Q2(
    const std::vector<double>& q_with_fixed_q1q2,
    const Eigen::Vector3d& p_d) const;
  /// 位姿感知可恢复性估计：固定 (q1,q2) 下的最小加权位姿残差。
  double minStage2PoseErrorForFixedQ1Q2(
    const std::vector<double>& q_with_fixed_q1q2,
    const Eigen::Vector3d& p_d,
    const Eigen::Quaterniond& q_d,
    double* best_pos_error = nullptr) const;

  /// 最终 stage2 搜索：锁定 q1/q2 接近 q_pre，并优化 q3/q4 目标。
  bool optimizeStage2LockedQ1Q2(
    const std::vector<double>& q_pre,
    const geometry_msgs::msg::Pose& target_pose,
    const std::vector<double>& q_goal_ik,
    std::vector<double>& q_goal_stage2,
    double& position_error,
    double& orientation_error_deg) const;

  /// stage2 标量评分辅助函数；可按需输出位置/姿态残差。
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
  // 复制配置快照，保证多次调用间算法行为可复现。
  PlannerConfig config_;
  // 关节索引一次缓存，避免热点循环中重复名称查找。
  std::size_t idx_q1_ = 0;
  std::size_t idx_q2_ = 1;
  std::size_t idx_q3_ = 2;
  std::size_t idx_q4_ = 3;
};

}  // namespace trunk_two_stage_planner
