#include "trunk_two_stage_planner/two_stage_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>

namespace trunk_two_stage_planner
{

namespace
{

double squaredNormDiff(const std::vector<double>& a, const std::vector<double>& b)
{
  if (a.size() != b.size()) {
    throw std::runtime_error("Vector size mismatch while computing distance.");
  }

  double sum = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i) {
    const double d = a[i] - b[i];
    sum += d * d;
  }
  return sum;
}

bool nearlySameState(const std::vector<double>& a, const std::vector<double>& b, double tol = 1e-6)
{
  return squaredNormDiff(a, b) <= tol * tol * static_cast<double>(a.size());
}

double lerp(double a, double b, double t)
{
  return a + (b - a) * t;
}

}  // namespace

TwoStagePlanner::TwoStagePlanner(
  const RobotKinematicsHelper& kinematics,
  const PlannerConfig& config)
: kinematics_(kinematics), config_(config)
{
  // 关节索引仅在构造时缓存一次。
  // 警告：若 SRDF 中 group 排序变化，此映射检查是第一道安全防线。
  if (!kinematics_.getJointIndex(config_.joint1_name, idx_q1_) ||
      !kinematics_.getJointIndex(config_.joint2_name, idx_q2_) ||
      !kinematics_.getJointIndex(config_.joint3_name, idx_q3_) ||
      !kinematics_.getJointIndex(config_.joint4_name, idx_q4_)) {
    throw std::runtime_error("Failed to map stage-1 joint names to trunk_group order.");
  }
}

bool TwoStagePlanner::plan(
  const std::vector<double>& q_start,
  const geometry_msgs::msg::Pose& target_pose,
  PlanningSummary& summary) const
{
  // 文件职责：
  // 纯算法流水线（不发起 MoveIt 规划请求）。
  // 产出阶段边界状态与诊断数据，供 manager/导出模块消费。
  summary = PlanningSummary();
  summary.q_start = q_start;

  bool used_fallback_goal = false;
  std::vector<double> q_goal_ik;
  if (!solveFinalIK(q_start, target_pose, q_goal_ik, used_fallback_goal)) {
    return false;
  }
  summary.q_goal_ik = q_goal_ik;
  summary.used_fallback_goal = used_fallback_goal;

  summary.joint3_axis_world = computeJointAxisInWorld(q_goal_ik, config_.joint3_name);
  summary.p_d = Eigen::Vector3d(
    target_pose.position.x,
    target_pose.position.y,
    target_pose.position.z);
  summary.p_d_proj_goal_axis = projectPointToAxis(summary.p_d, summary.joint3_axis_world);
  const Eigen::Quaterniond q_d(
    target_pose.orientation.w,
    target_pose.orientation.x,
    target_pose.orientation.y,
    target_pose.orientation.z);

  Stage1SearchResult search_result;
  if (!searchStage1PreparatoryState(
        q_start, summary.p_d_proj_goal_axis, summary.p_d, q_d, search_result)) {
    return false;
  }

  summary.q_pre = search_result.q_pre;
  summary.o4_pre = search_result.o4_pre;
  summary.o4_proj_pre = search_result.o4_proj_pre;
  summary.best_stage1_cost = search_result.best_cost;
  summary.o4_proj_to_goal_proj_distance = search_result.distance_proj_to_goal_proj;
  summary.selected_stage1_stage2_pos_error = search_result.selected_stage2_pos_error;
  summary.best_projection_error = search_result.best_projection_error;
  summary.best_stage2_pos_error = search_result.best_stage2_pos_error;
  summary.best_stage2_pose_error = search_result.best_stage2_pose_error;
  summary.q_pre_best_projection = search_result.q_pre_best_projection;
  summary.q_pre_best_stage2_pos = search_result.q_pre_best_stage2_pos;
  summary.q_pre_best_stage2_pose = search_result.q_pre_best_stage2_pose;
  summary.o4_proj_best_projection = search_result.o4_proj_best_projection;
  summary.o4_proj_best_stage2_pos = search_result.o4_proj_best_stage2_pos;
  summary.o4_proj_best_stage2_pose = search_result.o4_proj_best_stage2_pose;
  summary.threshold_feasible_count = search_result.threshold_feasible_count;
  summary.used_threshold_filter = search_result.used_threshold_filter;
  summary.selected_stage1_stage2_pose_error = search_result.selected_stage2_pose_error;
  summary.evaluated_samples = search_result.evaluated_samples;
  summary.valid_samples = search_result.valid_samples;
  summary.heatmap_samples = search_result.heatmap_samples;
  summary.delta_to_ik_goal = subtractVectors(q_goal_ik, summary.q_pre);
  summary.projection_best_matches_stage2_best =
    nearlySameState(summary.q_pre_best_projection, summary.q_pre_best_stage2_pos);
  summary.stage2_pos_best_matches_stage2_pose_best =
    nearlySameState(summary.q_pre_best_stage2_pos, summary.q_pre_best_stage2_pose);

  std::vector<double> q_goal_stage2;
  if (!optimizeStage2LockedQ1Q2(
        summary.q_pre,
        target_pose,
        q_goal_ik,
        q_goal_stage2,
        summary.stage2_position_error,
        summary.stage2_orientation_error_deg)) {
    return false;
  }
  summary.q_goal_stage2 = q_goal_stage2;
  summary.delta_stage2_used = subtractVectors(q_goal_stage2, summary.q_pre);
  return true;
}

bool TwoStagePlanner::solveFinalIK(
  const std::vector<double>& q_start,
  const geometry_msgs::msg::Pose& target_pose,
  std::vector<double>& q_goal,
  bool& used_fallback_goal) const
{
  // 策略：
  // 1) 用确定性 seed + 随机 seed 收集 IK 候选
  // 2) 按“接近起点平滑性 + 限位裕度代价”排序
  // 3) 仅在允许且 IK 全失败时回退到配置目标
  used_fallback_goal = false;
  std::vector<std::vector<double>> candidates;

  if (kinematics_.hasIKSolver()) {
    const auto try_seed = [&](const std::vector<double>& seed) {
      std::vector<double> candidate;
      if (kinematics_.solveIK(target_pose, candidate, seed) &&
          kinematics_.isStateWithinBounds(candidate)) {
        for (const auto& existing : candidates) {
          if (nearlySameState(existing, candidate)) {
            return;
          }
        }
        candidates.push_back(candidate);
      }
    };

    try_seed(q_start);
    if (config_.goal_joint_target.size() == q_start.size()) {
      try_seed(config_.goal_joint_target);
    }
    if (config_.q_goal_fallback.size() == q_start.size()) {
      try_seed(config_.q_goal_fallback);
    }

    std::mt19937 rng(42);
    // 固定随机种子，保证工程调试可复现。
    for (int i = 0; i < config_.ik_attempts; ++i) {
      try_seed(kinematics_.sampleRandomState(rng));
    }
  }

  if (!candidates.empty()) {
    double best_score = std::numeric_limits<double>::infinity();
    std::vector<double> best_candidate;
    for (const auto& candidate : candidates) {
      const double score = ikCandidateCost(candidate, q_start);
      if (score < best_score) {
        best_score = score;
        best_candidate = candidate;
      }
    }
    q_goal = best_candidate;
    return true;
  }

  if (config_.allow_goal_fallback && config_.q_goal_fallback.size() == q_start.size()) {
    // 警告：回退路径会绕过笛卡尔 IK 的精确性保证。
    // 仅在更看重运行鲁棒性而非位姿严格精确时启用。
    q_goal = config_.q_goal_fallback;
    used_fallback_goal = true;
    return true;
  }

  return false;
}

Axis3D TwoStagePlanner::computeJointAxisInWorld(
  const std::vector<double>& q,
  const std::string& joint_name) const
{
  return kinematics_.computeJointAxisInWorld(q, joint_name);
}

Eigen::Vector3d TwoStagePlanner::projectPointToAxis(
  const Eigen::Vector3d& p,
  const Axis3D& axis) const
{
  const Eigen::Vector3d delta = p - axis.origin;
  return axis.origin + axis.direction * axis.direction.dot(delta);
}

std::vector<double> TwoStagePlanner::composeStage1State(
  double q1,
  double q2,
  double q4_fix) const
{
  // trunk 场景特定 stage1 规则：
  // q3 跟随 q1+q2 耦合；q4 固定为起始阶段值。
  std::vector<double> q(kinematics_.getJointNames().size(), 0.0);
  q[idx_q1_] = q1;
  q[idx_q2_] = q2;
  q[idx_q3_] = q1 + q2;
  q[idx_q4_] = q4_fix;
  return q;
}

Eigen::Vector3d TwoStagePlanner::getStage1ReferencePoint(const std::vector<double>& q) const
{
  return kinematics_.getLinkPosition(q, config_.reference_link_name);
}

double TwoStagePlanner::jointLimitPenalty(const std::vector<double>& q) const
{
  // 这是一个“软限位代价”，不是简单的越界判定：
  // 1) 若直接越界，返回 inf，表示该状态彻底不可用
  // 2) 若未越界但离上下限太近，则按距离限位的相对比例施加连续惩罚
  //
  // 设计目的：
  // - 给优化过程一个“提前远离限位”的趋势，而不是等到碰到限位才突然失败
  // - 让最终选中的解在工程上更保守、更可执行
  //
  // `joint_limit_margin_ratio` 可以理解为“安全缓冲区”宽度占关节总行程的比例。
  // 例如 0.1 表示：若某关节进入距边界 10% 行程的区域，就开始被惩罚。
  constexpr double inf = std::numeric_limits<double>::infinity();

  double penalty = 0.0;
  const auto& joint_names = kinematics_.getJointNames();
  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    const auto [lower, upper] = kinematics_.getJointPositionBounds(joint_names[i]);
    const double range = upper - lower;
    if (range <= 0.0) {
      continue;
    }

    if (q[i] < lower || q[i] > upper) {
      return inf;
    }

    const double margin = std::min(q[i] - lower, upper - q[i]);
    const double margin_ratio = margin / range;
    if (margin_ratio < config_.joint_limit_margin_ratio) {
      // 这里把“离安全缓冲区边界还差多少”归一化到 [0, 1]：
      // - 贴近限位时 normalized 接近 1，惩罚最大
      // - 刚好到达安全缓冲区边界时 normalized = 0，惩罚消失
      //
      // 使用平方而不是线性项，是为了：
      // - 在轻微接近限位时惩罚更温和
      // - 在非常接近限位时惩罚迅速增大
      // 这种形状更符合“工程上可接受一定接近，但不希望贴边运行”的需求。
      //
      // 增大 `joint_limit_margin_ratio` 会扩大保守安全缓冲区。
      const double normalized =
        (config_.joint_limit_margin_ratio - margin_ratio) / config_.joint_limit_margin_ratio;
      penalty += normalized * normalized;
    }
  }

  return penalty;
}

double TwoStagePlanner::stage1Cost(
  double q1,
  double q2,
  const std::vector<double>& q_start,
  const Eigen::Vector3d& p_goal_proj,
  const Eigen::Vector3d& p_d,
  double* projection_error_out,
  double* stage2_pos_error_out,
  Eigen::Vector3d* o4_out,
  Eigen::Vector3d* o4_proj_out) const
{
  // 这是 stage1 的“完整加权代价”，但只在“没有阈值可行候选”时才作为回退排序使用。
  //
  // 它不是单纯几何最优，而是把四类工程诉求揉成一个标量：
  // 1) projection_error   : 第一阶段几何引导是否贴近期望投影
  // 2) stage2_pos_error   : 固定 q1/q2 后，第二阶段至少在位置上是否还恢复得回来
  // 3) term_start_bias    : 是否离起始姿态太远（平滑性/舒适性）
  // 4) limit_penalty      : 是否过分逼近关节限位（安全性）
  //
  // 这里采用“加权和 + 误差平方”的形式，是典型的工程折中做法：
  // - 把不同来源的误差压缩到一个便于比较的标量
  // - 平方项会放大较大的误差，促使优化优先消除明显不好的候选
  const std::vector<double> q_stage1 = composeStage1State(q1, q2, q_start[idx_q4_]);
  const double limit_penalty = jointLimitPenalty(q_stage1);
  if (!std::isfinite(limit_penalty) || !kinematics_.isStateWithinBounds(q_stage1)) {
    return std::numeric_limits<double>::infinity();
  }

  const Eigen::Vector3d o4 = getStage1ReferencePoint(q_stage1);
  const Axis3D axis_current = computeJointAxisInWorld(q_stage1, config_.joint3_name);
  const Eigen::Vector3d o4_proj = projectPointToAxis(o4, axis_current);
  if (o4_out) {
    *o4_out = o4;
  }
  if (o4_proj_out) {
    *o4_proj_out = o4_proj;
  }

  const double projection_error = (o4_proj - p_goal_proj).norm();
  const double stage2_pos_error = minStage2PositionErrorForFixedQ1Q2(q_stage1, p_d);
  if (projection_error_out) {
    *projection_error_out = projection_error;
  }
  if (stage2_pos_error_out) {
    *stage2_pos_error_out = stage2_pos_error;
  }

  const double term_start_bias = squaredDistance2D(q1, q2, q_start[idx_q1_], q_start[idx_q2_]);
  // 各项权重的含义：
  // - `w1`：投影对齐压力。越大，stage1 越强调几何引导效果。
  // - `w2`：第二阶段粗可恢复性压力。越大，越不愿选“后续很难恢复”的 q_pre。
  // - `w3`：起始姿态偏移惩罚。越大，越偏好离 q_start 更近、更平滑的预备态。
  // - `w4`：限位安全代价。越大，越偏好远离关节边界的姿态。
  //
  // 为什么 projection_error 和 stage2_pos_error 要平方：
  // - 这样大误差会被显著放大，避免出现“某项特别差但仍被其它小项抵消”的情况。
  //
  // 为什么 term_start_bias 本身不再额外平方：
  // - 因为 squaredDistance2D 已经是平方距离了。
  return config_.w1 * projection_error * projection_error +
         config_.w2 * stage2_pos_error * stage2_pos_error +
         config_.w3 * term_start_bias +
         config_.w4 * limit_penalty;
}

bool TwoStagePlanner::searchStage1PreparatoryState(
  const std::vector<double>& q_start,
  const Eigen::Vector3d& p_goal_proj,
  const Eigen::Vector3d& p_d,
  const Eigen::Quaterniond& q_d,
  Stage1SearchResult& result) const
{
  // 在 (q1, q2) 上执行 stage1 网格搜索，q3/q4 由 trunk 规则确定。
  // 选择策略为“阈值优先，其次加权回退”。
  //
  // 这里的核心思想不是“全局只看一个加权和”，而是分两层：
  // - 第一层：先看 stage2 是否足够可恢复（pose_error 是否小于阈值）
  // - 第二层：只有在满足这个硬门槛后，才去比较几何/舒适/限位等次级指标
  //
  // 这样做的原因：
  // - 如果不先设门槛，优化很容易选出“stage1 看起来很漂亮，但 stage2 实际接不回目标”的假优解
  // - 这正是两阶段规划与普通单阶段几何启发式规划最大的区别
  const auto [q1_lower, q1_upper] = kinematics_.getJointPositionBounds(config_.joint1_name);
  const auto [q2_lower, q2_upper] = kinematics_.getJointPositionBounds(config_.joint2_name);
  const double q4_fix = q_start[idx_q4_];
  double best_filtered_cost = std::numeric_limits<double>::infinity();
  double best_fallback_cost = std::numeric_limits<double>::infinity();
  bool found_threshold_feasible = false;

  for (int i = 0; i < config_.stage1_q1_samples; ++i) {
    const double t1 =
      (config_.stage1_q1_samples <= 1) ? 0.0 : static_cast<double>(i) / (config_.stage1_q1_samples - 1);
    const double q1 = lerp(q1_lower, q1_upper, t1);

    for (int j = 0; j < config_.stage1_q2_samples; ++j) {
      const double t2 =
        (config_.stage1_q2_samples <= 1) ? 0.0 : static_cast<double>(j) / (config_.stage1_q2_samples - 1);
      const double q2 = lerp(q2_lower, q2_upper, t2);
      ++result.evaluated_samples;

      Stage1HeatmapSample heatmap_sample;
      heatmap_sample.q1 = q1;
      heatmap_sample.q2 = q2;

      const std::vector<double> q_candidate = composeStage1State(q1, q2, q4_fix);
      if (!kinematics_.isStateWithinBounds(q_candidate)) {
        result.heatmap_samples.push_back(heatmap_sample);
        continue;
      }

      ++result.valid_samples;
      Eigen::Vector3d o4;
      Eigen::Vector3d o4_proj;
      double projection_error = std::numeric_limits<double>::infinity();
      double stage2_pos_error = std::numeric_limits<double>::infinity();
      double stage2_pose_error = std::numeric_limits<double>::infinity();
      const double cost = stage1Cost(
        q1, q2, q_start, p_goal_proj, p_d,
        &projection_error, &stage2_pos_error, &o4, &o4_proj);
      stage2_pose_error = minStage2PoseErrorForFixedQ1Q2(
        q_candidate, p_d, q_d, nullptr);
      heatmap_sample.valid = std::isfinite(cost);
      heatmap_sample.cost = cost;
      heatmap_sample.projection_error = projection_error;
      heatmap_sample.dist_proj_to_goal_proj = (o4_proj - p_goal_proj).norm();
      heatmap_sample.stage2_pos_error = stage2_pos_error;
      heatmap_sample.stage2_pose_error = stage2_pose_error;
      heatmap_sample.o4_projection = o4_proj;
      result.heatmap_samples.push_back(heatmap_sample);
      if (!std::isfinite(cost) || !std::isfinite(stage2_pose_error)) {
        continue;
      }

      result.success = true;
      if (projection_error < result.best_projection_error) {
        result.best_projection_error = projection_error;
        result.q_pre_best_projection = q_candidate;
        result.o4_proj_best_projection = o4_proj;
      }
      if (stage2_pos_error < result.best_stage2_pos_error) {
        result.best_stage2_pos_error = stage2_pos_error;
        result.q_pre_best_stage2_pos = q_candidate;
        result.o4_proj_best_stage2_pos = o4_proj;
      }
      if (stage2_pose_error < result.best_stage2_pose_error) {
        result.best_stage2_pose_error = stage2_pose_error;
        result.q_pre_best_stage2_pose = q_candidate;
        result.o4_proj_best_stage2_pose = o4_proj;
      }

      const double term_start_bias = squaredDistance2D(q1, q2, q_start[idx_q1_], q_start[idx_q2_]);
      const double limit_penalty = jointLimitPenalty(q_candidate);
      const double filtered_cost =
        config_.w1 * projection_error * projection_error +
        config_.w3 * term_start_bias +
        config_.w4 * limit_penalty;
      // 注意：filtered_cost 有意移除了 `w2 * stage2_pos_error^2`。
      // 因为一旦候选已经满足 stage2_pose_epsilon，说明它在“可恢复性”这一层已经过关，
      // 此时继续让 `w2` 参与排序，会把“已经合格”的候选再次按可恢复性粗指标拉开，
      // 反而可能压过 stage1 几何和姿态平滑性这些更该在第二层比较的内容。

      if (stage2_pose_error < config_.stage2_pose_epsilon) {
        // 主分支：
        // 候选已满足 stage2 可恢复性阈值，用过滤后的 stage1 目标排序。
        // 一旦满足位姿可恢复性，这里有意不再使用 `w2` 参与排序。
        ++result.threshold_feasible_count;
        found_threshold_feasible = true;
        if (filtered_cost < best_filtered_cost) {
          best_filtered_cost = filtered_cost;
          result.best_cost = filtered_cost;
          result.q_pre = q_candidate;
          result.o4_pre = o4;
          result.o4_proj_pre = o4_proj;
          result.distance_proj_to_goal_proj = projection_error;
          result.selected_stage2_pos_error = stage2_pos_error;
          result.selected_stage2_pose_error = stage2_pose_error;
          result.used_threshold_filter = true;
        }
      } else if (!found_threshold_feasible && cost < best_fallback_cost) {
        // 回退分支：
        // 仅在尚未出现阈值可行点时启用。
        // 可提升困难目标下的可用性，但可能削弱严格阶段分离质量。
        best_fallback_cost = cost;
        result.best_cost = cost;
        result.q_pre = q_candidate;
        result.o4_pre = o4;
        result.o4_proj_pre = o4_proj;
        result.distance_proj_to_goal_proj = projection_error;
        result.selected_stage2_pos_error = stage2_pos_error;
        result.selected_stage2_pose_error = stage2_pose_error;
        result.used_threshold_filter = false;
      }
    }
  }

  return result.success;
}

double TwoStagePlanner::minStage2PositionErrorForFixedQ1Q2(
  const std::vector<double>& q_with_fixed_q1q2,
  const Eigen::Vector3d& p_d) const
{
  // 用于 stage1 排序的粗粒度评估器：
  // 扫描 (q3,q4)，记录到目标点的最小位置误差。
  //
  // 这是一个“便宜但不完整”的可恢复性指标：
  // - 只看位置，不看姿态
  // - 用于 stage1Cost 的粗排序项
  //
  // 它的作用不是替代真正的位姿判断，而是快速告诉我们：
  // “如果 q1/q2 固定住，后两关节大概还能不能把末端拉回目标附近”
  const auto [q3_lower, q3_upper] = kinematics_.getJointPositionBounds(config_.joint3_name);
  const auto [q4_lower, q4_upper] = kinematics_.getJointPositionBounds(config_.joint4_name);

  moveit::core::RobotState state(kinematics_.getRobotModel());
  state.setToDefaultValues();

  std::vector<double> q = q_with_fixed_q1q2;
  double best_error = std::numeric_limits<double>::infinity();

  for (int i = 0; i < config_.stage2_eval_q3_samples; ++i) {
    const double t3 =
      (config_.stage2_eval_q3_samples <= 1) ? 0.0 : static_cast<double>(i) / (config_.stage2_eval_q3_samples - 1);
    q[idx_q3_] = lerp(q3_lower, q3_upper, t3);

    for (int j = 0; j < config_.stage2_eval_q4_samples; ++j) {
      const double t4 =
        (config_.stage2_eval_q4_samples <= 1) ? 0.0 : static_cast<double>(j) / (config_.stage2_eval_q4_samples - 1);
      q[idx_q4_] = lerp(q4_lower, q4_upper, t4);

      state.setJointGroupPositions(kinematics_.getJointModelGroup(), q);
      state.update();
      const Eigen::Vector3d p = state.getGlobalLinkTransform(kinematics_.getTipLinkName()).translation();
      best_error = std::min(best_error, (p - p_d).norm());
    }
  }

  return best_error;
}

double TwoStagePlanner::minStage2PoseErrorForFixedQ1Q2(
  const std::vector<double>& q_with_fixed_q1q2,
  const Eigen::Vector3d& p_d,
  const Eigen::Quaterniond& q_d,
  double* best_pos_error) const
{
  // 用于阈值筛选（`stage2_pose_epsilon`）的位姿感知评估器。
  //
  // 和上面的 minStage2PositionErrorForFixedQ1Q2 相比，这里更“贵”也更完整：
  // - 同时考虑位置误差和姿态误差
  // - 其输出直接参与“是否进入可行候选子集”的判断
  //
  // 这相当于在问：
  // “如果 stage1 选了这个 q1/q2，后续只调 q3/q4，理论上最好的 stage2 位姿还能接近目标到什么程度？”
  const auto [q3_lower, q3_upper] = kinematics_.getJointPositionBounds(config_.joint3_name);
  const auto [q4_lower, q4_upper] = kinematics_.getJointPositionBounds(config_.joint4_name);

  moveit::core::RobotState state(kinematics_.getRobotModel());
  state.setToDefaultValues();

  std::vector<double> q = q_with_fixed_q1q2;
  double best_pose_error = std::numeric_limits<double>::infinity();
  double best_pos = std::numeric_limits<double>::infinity();

  for (int i = 0; i < config_.stage2_eval_q3_samples; ++i) {
    const double t3 =
      (config_.stage2_eval_q3_samples <= 1) ? 0.0 :
      static_cast<double>(i) / (config_.stage2_eval_q3_samples - 1);
    q[idx_q3_] = lerp(q3_lower, q3_upper, t3);

    for (int j = 0; j < config_.stage2_eval_q4_samples; ++j) {
      const double t4 =
        (config_.stage2_eval_q4_samples <= 1) ? 0.0 :
        static_cast<double>(j) / (config_.stage2_eval_q4_samples - 1);
      q[idx_q4_] = lerp(q4_lower, q4_upper, t4);

      state.setJointGroupPositions(kinematics_.getJointModelGroup(), q);
      state.update();

      const Eigen::Isometry3d tf = state.getGlobalLinkTransform(kinematics_.getTipLinkName());
      const double pos_error = (tf.translation() - p_d).norm();
      const Eigen::Vector3d e_R = orientationErrorVector(tf.rotation(), q_d);
      const double pose_error =
        config_.stage2_pose_wp * pos_error * pos_error +
        config_.stage2_pose_wR * e_R.squaredNorm();
      // 调参建议：
      // - 增大 `stage2_pose_wp`：更强调笛卡尔位置闭合
      // - 增大 `stage2_pose_wR`：更强调姿态可恢复性收敛
      //
      // 这里本质上也是一个加权和：
      // pose_error = 位置项 + 姿态项
      // 其中位置项使用米的平方，姿态项使用旋转误差向量范数平方。
      //
      // `stage2_pose_epsilon` 不是“位置阈值”也不是“角度阈值”，
      // 而是这个综合位姿误差的阈值，所以调它时必须结合 wp/wR 一起理解。

      if (pose_error < best_pose_error) {
        best_pose_error = pose_error;
        best_pos = pos_error;
      }
    }
  }

  if (best_pos_error) {
    *best_pos_error = best_pos;
  }
  return best_pose_error;
}

bool TwoStagePlanner::optimizeStage2LockedQ1Q2(
  const std::vector<double>& q_pre,
  const geometry_msgs::msg::Pose& target_pose,
  const std::vector<double>& q_goal_ik,
  std::vector<double>& q_goal_stage2,
  double& position_error,
  double& orientation_error_deg) const
{
  // 在 q1/q2 锁定语义下执行最终 stage2 目标搜索。
  // 使用词典序优先级可避免指标间不稳定权衡。
  //
  // 和 stage1 不同，这里最终没有直接用一个加权和来“拍板”选最优解，
  // 而是采用词典序：
  //   1) 先比较综合位姿误差 pose_err
  //   2) pose_err 相同或极接近时，再比位置误差
  //   3) 再比姿态误差
  //   4) 最后才比 q3/q4 是否接近 q_goal_ik
  //
  // 原因：
  // - stage2 的核心使命是“恢复最终位姿”
  // - 如果继续单纯做加权和，很容易出现某一项略优却掩盖主目标退化的问题
  // - 词典序能明确表达“先把主任务做好，再谈次任务”
  const auto [q3_lower, q3_upper] = kinematics_.getJointPositionBounds(config_.joint3_name);
  const auto [q4_lower, q4_upper] = kinematics_.getJointPositionBounds(config_.joint4_name);

  std::vector<double> best_q;
  double best_pos_err = std::numeric_limits<double>::infinity();
  double best_ori_err = std::numeric_limits<double>::infinity();
  double best_pose_err = std::numeric_limits<double>::infinity();
  double best_q34_bias = std::numeric_limits<double>::infinity();

  for (int i = 0; i < config_.stage2_q3_samples; ++i) {
    const double t3 =
      (config_.stage2_q3_samples <= 1) ? 0.0 : static_cast<double>(i) / (config_.stage2_q3_samples - 1);
    const double q3 = lerp(q3_lower, q3_upper, t3);

    for (int j = 0; j < config_.stage2_q4_samples; ++j) {
      const double t4 =
        (config_.stage2_q4_samples <= 1) ? 0.0 : static_cast<double>(j) / (config_.stage2_q4_samples - 1);
      const double q4 = lerp(q4_lower, q4_upper, t4);

      double pos_err = 0.0;
      double ori_err_deg = 0.0;
      stage2LockedCost(
        q3, q4, q_pre, target_pose, q_goal_ik, &pos_err, &ori_err_deg);
      if (!std::isfinite(pos_err) || !std::isfinite(ori_err_deg)) {
        continue;
      }

      const Eigen::Vector3d target_p(
        target_pose.position.x, target_pose.position.y, target_pose.position.z);
      const Eigen::Quaterniond target_q(
        target_pose.orientation.w,
        target_pose.orientation.x,
        target_pose.orientation.y,
        target_pose.orientation.z);
      std::vector<double> q_eval = q_pre;
      q_eval[idx_q3_] = q3;
      q_eval[idx_q4_] = q4;
      const Eigen::Isometry3d tf_eval =
        kinematics_.getLinkTransform(q_eval, kinematics_.getTipLinkName());
      const Eigen::Vector3d e_R = orientationErrorVector(tf_eval.rotation(), target_q);
      const double pose_err =
        config_.stage2_pose_wp * (tf_eval.translation() - target_p).squaredNorm() +
        config_.stage2_pose_wR * e_R.squaredNorm();
      const double q34_bias =
        squaredDistance2D(q3, q4, q_goal_ik[idx_q3_], q_goal_ik[idx_q4_]);

      const bool better_pose = pose_err + 1e-12 < best_pose_err;
      const bool same_pose = std::abs(pose_err - best_pose_err) <= 1e-12;
      const bool better_position = pos_err + 1e-9 < best_pos_err;
      const bool same_position = std::abs(pos_err - best_pos_err) <= 1e-9;
      const bool better_orientation = ori_err_deg + 1e-9 < best_ori_err;
      const bool same_orientation = std::abs(ori_err_deg - best_ori_err) <= 1e-9;
      const bool better_bias = q34_bias + 1e-12 < best_q34_bias;

      if (!(better_pose ||
            (same_pose && better_position) ||
            (same_pose && same_position && better_orientation) ||
            (same_pose && same_position && same_orientation && better_bias))) {
        continue;
      }
      // 词典序优先级：
      // 1) 位姿残差  2) 位置误差  3) 姿态误差  4) 接近 IK q3/q4
      // 警告：调整该顺序会全局改变 stage2 行为。

      best_q = q_pre;
      best_q[idx_q3_] = q3;
      best_q[idx_q4_] = q4;
      best_pose_err = pose_err;
      best_pos_err = pos_err;
      best_ori_err = ori_err_deg;
      best_q34_bias = q34_bias;
    }
  }

  if (best_q.empty()) {
    return false;
  }

  q_goal_stage2 = best_q;
  position_error = best_pos_err;
  orientation_error_deg = best_ori_err;
  return true;
}

double TwoStagePlanner::stage2LockedCost(
  double q3,
  double q4,
  const std::vector<double>& q_pre,
  const geometry_msgs::msg::Pose& target_pose,
  const std::vector<double>& q_goal_ik,
  double* position_error,
  double* orientation_error_deg) const
{
  // 辅助标量代价（主要用于诊断）；最终选择以上方词典序为准。
  //
  // 这个函数仍然保留，是因为它对“观察趋势”和“打印调试信息”很有价值：
  // - 能快速看出位置/姿态/偏置/限位四类因素叠加后的总趋势
  // - 但它不再承担最终最优解决策职责
  //
  // 这也是为什么 optimizeStage2LockedQ1Q2 会额外单独计算 pose_err / pos_err / ori_err_deg / q34_bias。
  std::vector<double> q = q_pre;
  q[idx_q3_] = q3;
  q[idx_q4_] = q4;

  const double limit_penalty = jointLimitPenalty(q);
  if (!std::isfinite(limit_penalty) || !kinematics_.isStateWithinBounds(q)) {
    return std::numeric_limits<double>::infinity();
  }

  const Eigen::Isometry3d tip_tf =
    kinematics_.getLinkTransform(q, kinematics_.getTipLinkName());
  const Eigen::Vector3d target_p(
    target_pose.position.x, target_pose.position.y, target_pose.position.z);
  const Eigen::Quaterniond target_q(
    target_pose.orientation.w,
    target_pose.orientation.x,
    target_pose.orientation.y,
    target_pose.orientation.z);

  const double pos_err = (tip_tf.translation() - target_p).norm();
  const double ori_err_deg = orientationErrorDeg(tip_tf.rotation(), target_q);
  if (position_error) {
    *position_error = pos_err;
  }
  if (orientation_error_deg) {
    *orientation_error_deg = ori_err_deg;
  }

  const double q34_bias =
    squaredDistance2D(q3, q4, q_goal_ik[idx_q3_], q_goal_ik[idx_q4_]);

  // 各项解释：
  // - stage2_pos_weight * pos_err^2
  //     末端位置误差，越小越好
  // - stage2_ori_weight * ori_err_deg^2
  //     姿态误差（单位是度），越小越好
  // - stage2_q34_bias_weight * q34_bias
  //     希望最终 q3/q4 不要偏离 q_goal_ik 太远，作为次级正则项
  // - w3 * limit_penalty
  //     继承限位安全惩罚，避免最终目标贴边
  //
  // 注意姿态项这里用的是“角度（度）平方”，而不是旋转向量范数平方，
  // 所以它和 minStage2PoseErrorForFixedQ1Q2 里的 pose_error 不是同一个量纲体系。
  // 这也是为什么这里更适合作为诊断/趋势指标，而最终选择改由词典序控制。
  return config_.stage2_pos_weight * pos_err * pos_err +
         config_.stage2_ori_weight * ori_err_deg * ori_err_deg +
         config_.stage2_q34_bias_weight * q34_bias +
         config_.w3 * limit_penalty;
}

std::vector<double> TwoStagePlanner::subtractVectors(
  const std::vector<double>& lhs,
  const std::vector<double>& rhs)
{
  if (lhs.size() != rhs.size()) {
    throw std::runtime_error("Vector size mismatch while computing delta.");
  }

  std::vector<double> delta(lhs.size(), 0.0);
  for (std::size_t i = 0; i < lhs.size(); ++i) {
    delta[i] = lhs[i] - rhs[i];
  }
  return delta;
}

Eigen::Vector3d TwoStagePlanner::orientationErrorVector(
  const Eigen::Matrix3d& current,
  const Eigen::Quaterniond& target)
{
  const Eigen::Matrix3d R_err = target.normalized().toRotationMatrix().transpose() * current;
  const Eigen::AngleAxisd aa(R_err);
  if (std::abs(aa.angle()) < 1e-12 || !std::isfinite(aa.angle())) {
    return Eigen::Vector3d::Zero();
  }
  return aa.axis() * aa.angle();
}

double TwoStagePlanner::orientationErrorDeg(
  const Eigen::Matrix3d& current,
  const Eigen::Quaterniond& target)
{
  return orientationErrorVector(current, target).norm() * 180.0 / M_PI;
}

double TwoStagePlanner::ikCandidateCost(
  const std::vector<double>& q,
  const std::vector<double>& q_start) const
{
  // IK 候选排序代价：
  // - 第一项 squaredNormDiff(q, q_start)
  //     偏好离当前起点更近的 IK 分支，减少大幅跳变
  // - 第二项 ik_limit_penalty_weight * jointLimitPenalty(q)
  //     偏好更远离限位的 IK 解
  //
  // 这不是“数学唯一最优”定义，而是工程偏好：
  // 在多个 IK 解都成立时，优先选更平滑、更保守、更不贴边的那个。
  return squaredNormDiff(q, q_start) + config_.ik_limit_penalty_weight * jointLimitPenalty(q);
}

double TwoStagePlanner::squaredDistance2D(double a1, double a2, double b1, double b2)
{
  const double d1 = a1 - b1;
  const double d2 = a2 - b2;
  return d1 * d1 + d2 * d2;
}

}  // namespace trunk_two_stage_planner
