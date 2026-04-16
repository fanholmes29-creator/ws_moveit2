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

      if (stage2_pose_error < config_.stage2_pose_epsilon) {
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
  return squaredNormDiff(q, q_start) + config_.ik_limit_penalty_weight * jointLimitPenalty(q);
}

double TwoStagePlanner::squaredDistance2D(double a1, double a2, double b1, double b2)
{
  const double d1 = a1 - b1;
  const double d2 = a2 - b2;
  return d1 * d1 + d2 * d2;
}

}  // namespace trunk_two_stage_planner
