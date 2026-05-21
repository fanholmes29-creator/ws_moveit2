#include <cmath>
#include <cstdio>
#include <exception>
#include <iomanip>
#include <sstream>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include "trunk_two_stage_planner/robot_kinematics_helper.hpp"
#include "trunk_two_stage_planner/two_stage_planner.hpp"

namespace trunk_two_stage_planner
{

namespace
{

double squaredNormDiff(const std::vector<double>& a, const std::vector<double>& b)
{
  if (a.size() != b.size()) {
    throw std::runtime_error("Vector size mismatch while computing IK candidate distance.");
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

// 参数声明辅助函数：
// 复用当前工程的参数加载风格，确保既能吃默认值，也能吃外部 params-file 覆盖。
void declareIfMissingBool(const rclcpp::Node::SharedPtr& node, const std::string& name, bool value)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter<bool>(name, value);
  }
}

void declareIfMissingInt(const rclcpp::Node::SharedPtr& node, const std::string& name, int value)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter<int>(name, value);
  }
}

void declareIfMissingDouble(const rclcpp::Node::SharedPtr& node, const std::string& name, double value)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter<double>(name, value);
  }
}

void declareIfMissingString(
  const rclcpp::Node::SharedPtr& node, const std::string& name, const std::string& value)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter<std::string>(name, value);
  }
}

void declareIfMissingDoubleArray(
  const rclcpp::Node::SharedPtr& node, const std::string& name, const std::vector<double>& value)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter<std::vector<double>>(name, value);
  }
}

PlannerConfig loadAlgorithmConfig(const rclcpp::Node::SharedPtr& node)
{
  // 该工具只需要算法层配置，不需要 MoveIt 运行时调度配置。
  // 这里保持与系统主入口一致，避免工具与主程序出现参数语义漂移。
  PlannerConfig config;

  declareIfMissingString(node, "robot_description_package", config.robot_description_package);
  declareIfMissingString(node, "urdf_relative_path", config.urdf_relative_path);
  declareIfMissingString(node, "moveit_config_package", config.moveit_config_package);
  declareIfMissingString(node, "srdf_relative_path", config.srdf_relative_path);
  declareIfMissingString(node, "group_name", config.group_name);
  declareIfMissingString(node, "joint1_name", config.joint1_name);
  declareIfMissingString(node, "joint2_name", config.joint2_name);
  declareIfMissingString(node, "joint3_name", config.joint3_name);
  declareIfMissingString(node, "joint4_name", config.joint4_name);
  declareIfMissingString(node, "reference_link_name", config.reference_link_name);

  declareIfMissingDoubleArray(node, "q_start", config.q_start);
  declareIfMissingDoubleArray(node, "goal_joint_target", config.goal_joint_target);
  declareIfMissingDoubleArray(node, "q_goal_fallback", config.q_goal_fallback);
  declareIfMissingBool(node, "use_goal_state_as_target_pose", config.use_goal_state_as_target_pose);
  declareIfMissingDoubleArray(node, "target_position", config.target_position);
  declareIfMissingDoubleArray(node, "target_orientation", config.target_orientation);
  declareIfMissingBool(node, "allow_goal_fallback", config.allow_goal_fallback);
  declareIfMissingInt(node, "ik_attempts", config.ik_attempts);
  declareIfMissingDouble(node, "ik_timeout", config.ik_timeout);
  declareIfMissingDouble(node, "ik_limit_penalty_weight", config.ik_limit_penalty_weight);
  declareIfMissingString(node, "stage1_pre_mode", config.stage1_pre_mode);
  declareIfMissingInt(node, "stage1_q1_samples", config.stage1_q1_samples);
  declareIfMissingInt(node, "stage1_q2_samples", config.stage1_q2_samples);
  declareIfMissingDouble(node, "w1", config.w1);
  declareIfMissingDouble(node, "w2", config.w2);
  declareIfMissingDouble(node, "w3", config.w3);
  declareIfMissingDouble(node, "w4", config.w4);
  declareIfMissingDouble(node, "stage2_pose_wp", config.stage2_pose_wp);
  declareIfMissingDouble(node, "stage2_pose_wR", config.stage2_pose_wR);
  declareIfMissingDouble(node, "stage2_pose_epsilon", config.stage2_pose_epsilon);
  declareIfMissingDouble(node, "joint_limit_margin_ratio", config.joint_limit_margin_ratio);
  declareIfMissingInt(node, "stage2_eval_q3_samples", config.stage2_eval_q3_samples);
  declareIfMissingInt(node, "stage2_eval_q4_samples", config.stage2_eval_q4_samples);
  declareIfMissingInt(node, "stage2_q3_samples", config.stage2_q3_samples);
  declareIfMissingInt(node, "stage2_q4_samples", config.stage2_q4_samples);
  declareIfMissingDouble(node, "stage2_pos_weight", config.stage2_pos_weight);
  declareIfMissingDouble(node, "stage2_ori_weight", config.stage2_ori_weight);
  declareIfMissingDouble(node, "stage2_q34_bias_weight", config.stage2_q34_bias_weight);
  declareIfMissingDouble(node, "stage1_duration", config.stage1_duration);
  declareIfMissingDouble(node, "stage2_duration", config.stage2_duration);
  declareIfMissingDouble(node, "dt", config.dt);
  declareIfMissingString(node, "output_dir", config.output_dir);

  config.robot_description_package = node->get_parameter("robot_description_package").as_string();
  config.urdf_relative_path = node->get_parameter("urdf_relative_path").as_string();
  config.moveit_config_package = node->get_parameter("moveit_config_package").as_string();
  config.srdf_relative_path = node->get_parameter("srdf_relative_path").as_string();
  config.group_name = node->get_parameter("group_name").as_string();
  config.joint1_name = node->get_parameter("joint1_name").as_string();
  config.joint2_name = node->get_parameter("joint2_name").as_string();
  config.joint3_name = node->get_parameter("joint3_name").as_string();
  config.joint4_name = node->get_parameter("joint4_name").as_string();
  config.reference_link_name = node->get_parameter("reference_link_name").as_string();
  config.q_start = node->get_parameter("q_start").as_double_array();
  config.goal_joint_target = node->get_parameter("goal_joint_target").as_double_array();
  config.q_goal_fallback = node->get_parameter("q_goal_fallback").as_double_array();
  config.use_goal_state_as_target_pose = node->get_parameter("use_goal_state_as_target_pose").as_bool();
  config.target_position = node->get_parameter("target_position").as_double_array();
  config.target_orientation = node->get_parameter("target_orientation").as_double_array();
  config.allow_goal_fallback = node->get_parameter("allow_goal_fallback").as_bool();
  config.ik_attempts = node->get_parameter("ik_attempts").as_int();
  config.ik_timeout = node->get_parameter("ik_timeout").as_double();
  config.ik_limit_penalty_weight = node->get_parameter("ik_limit_penalty_weight").as_double();
  config.stage1_pre_mode = node->get_parameter("stage1_pre_mode").as_string();
  config.stage1_q1_samples = node->get_parameter("stage1_q1_samples").as_int();
  config.stage1_q2_samples = node->get_parameter("stage1_q2_samples").as_int();
  config.w1 = node->get_parameter("w1").as_double();
  config.w2 = node->get_parameter("w2").as_double();
  config.w3 = node->get_parameter("w3").as_double();
  config.w4 = node->get_parameter("w4").as_double();
  config.stage2_pose_wp = node->get_parameter("stage2_pose_wp").as_double();
  config.stage2_pose_wR = node->get_parameter("stage2_pose_wR").as_double();
  config.stage2_pose_epsilon = node->get_parameter("stage2_pose_epsilon").as_double();
  config.joint_limit_margin_ratio = node->get_parameter("joint_limit_margin_ratio").as_double();
  config.stage2_eval_q3_samples = node->get_parameter("stage2_eval_q3_samples").as_int();
  config.stage2_eval_q4_samples = node->get_parameter("stage2_eval_q4_samples").as_int();
  config.stage2_q3_samples = node->get_parameter("stage2_q3_samples").as_int();
  config.stage2_q4_samples = node->get_parameter("stage2_q4_samples").as_int();
  config.stage2_pos_weight = node->get_parameter("stage2_pos_weight").as_double();
  config.stage2_ori_weight = node->get_parameter("stage2_ori_weight").as_double();
  config.stage2_q34_bias_weight = node->get_parameter("stage2_q34_bias_weight").as_double();
  config.stage1_duration = node->get_parameter("stage1_duration").as_double();
  config.stage2_duration = node->get_parameter("stage2_duration").as_double();
  config.dt = node->get_parameter("dt").as_double();
  config.output_dir = node->get_parameter("output_dir").as_string();
  return config;
}

std::string toYamlArray(const std::vector<double>& values)
{
  // 输出格式故意做成可直接复制回 YAML 的数组形式。
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(6);
  oss << "[";
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i > 0) {
      oss << ", ";
    }
    oss << values[i];
  }
  oss << "]";
  return oss.str();
}

double jointLimitPenalty(
  const RobotKinematicsHelper& helper,
  const PlannerConfig& config,
  const std::vector<double>& q)
{
  constexpr double inf = std::numeric_limits<double>::infinity();
  const auto& joint_names = helper.getJointNames();
  if (q.size() != joint_names.size()) {
    throw std::runtime_error("Joint vector size does not match planning group dimension.");
  }

  double penalty = 0.0;
  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    const auto [lower, upper] = helper.getJointPositionBounds(joint_names[i]);
    const double range = upper - lower;
    if (range <= 0.0) {
      continue;
    }
    if (q[i] < lower || q[i] > upper) {
      return inf;
    }

    const double margin = std::min(q[i] - lower, upper - q[i]);
    const double margin_ratio = margin / range;
    if (margin_ratio < config.joint_limit_margin_ratio) {
      const double normalized =
        (config.joint_limit_margin_ratio - margin_ratio) / config.joint_limit_margin_ratio;
      penalty += normalized * normalized;
    }
  }
  return penalty;
}

double ikCandidateCost(
  const RobotKinematicsHelper& helper,
  const PlannerConfig& config,
  const std::vector<double>& candidate,
  const std::vector<double>& q_start)
{
  return squaredNormDiff(candidate, q_start) +
         config.ik_limit_penalty_weight * jointLimitPenalty(helper, config, candidate);
}

std::vector<std::vector<double>> collectIkCandidates(
  const RobotKinematicsHelper& helper,
  const PlannerConfig& config,
  const geometry_msgs::msg::Pose& target_pose)
{
  std::vector<std::vector<double>> candidates;
  if (!helper.hasIKSolver()) {
    return candidates;
  }

  const auto try_seed = [&](const std::vector<double>& seed) {
    std::vector<double> candidate;
    if (helper.solveIK(target_pose, candidate, seed) && helper.isStateWithinBounds(candidate)) {
      for (const auto& existing : candidates) {
        if (nearlySameState(existing, candidate)) {
          return;
        }
      }
      candidates.push_back(candidate);
    }
  };

  try_seed(config.q_start);
  if (config.goal_joint_target.size() == config.q_start.size()) {
    try_seed(config.goal_joint_target);
  }
  if (config.q_goal_fallback.size() == config.q_start.size()) {
    try_seed(config.q_goal_fallback);
  }

  std::mt19937 rng(42);
  for (int i = 0; i < config.ik_attempts; ++i) {
    try_seed(helper.sampleRandomState(rng));
  }

  return candidates;
}

}  // namespace

}  // namespace trunk_two_stage_planner

int main(int argc, char* argv[])
{
  // 文件职责：
  // 这是一个独立的 IK 反解小工具。
  // 输入 target_position / target_orientation，输出可粘贴的 goal_joint_target。
  rclcpp::init(argc, argv);

  try {
    auto node = rclcpp::Node::make_shared(
      "two_stage_planner_system",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    auto config = trunk_two_stage_planner::loadAlgorithmConfig(node);

    // 严格检查位姿输入维度，避免静默读取错误参数。
    if (config.target_position.size() != 3) {
      throw std::runtime_error("target_position must contain exactly 3 values.");
    }
    if (config.target_orientation.size() != 4) {
      throw std::runtime_error("target_orientation must contain exactly 4 values.");
    }

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = config.target_position[0];
    target_pose.position.y = config.target_position[1];
    target_pose.position.z = config.target_position[2];
    target_pose.orientation.x = config.target_orientation[0];
    target_pose.orientation.y = config.target_orientation[1];
    target_pose.orientation.z = config.target_orientation[2];
    target_pose.orientation.w = config.target_orientation[3];

    // 姿态输入要求是四元数；这里自动归一化，减少上层输入误差带来的问题。
    Eigen::Quaterniond q(
      target_pose.orientation.w,
      target_pose.orientation.x,
      target_pose.orientation.y,
      target_pose.orientation.z);
    if (!std::isfinite(q.norm()) || q.norm() < 1e-8) {
      throw std::runtime_error("target_orientation is invalid: quaternion norm is too small.");
    }
    q.normalize();
    target_pose.orientation.x = q.x();
    target_pose.orientation.y = q.y();
    target_pose.orientation.z = q.z();
    target_pose.orientation.w = q.w();

    trunk_two_stage_planner::RobotKinematicsHelper helper;
    if (!helper.initialize(node, config)) {
      RCLCPP_ERROR(node->get_logger(), "Failed to initialize RobotKinematicsHelper.");
      rclcpp::shutdown();
      return 1;
    }

    // 先收集所有找到的 IK 候选，用于完整输出。
    const auto candidates = trunk_two_stage_planner::collectIkCandidates(helper, config, target_pose);

    // 关闭 fallback，确保工具输出的是实际 IK 结果，而不是兜底目标。
    config.allow_goal_fallback = false;
    trunk_two_stage_planner::TwoStagePlanner planner(helper, config);

    std::vector<double> q_goal;
    bool used_fallback_goal = false;
    if (!planner.solveFinalIK(config.q_start, target_pose, q_goal, used_fallback_goal)) {
      std::fprintf(stderr, "ik_joint_from_pose: no valid IK solution found.\n");
      rclcpp::shutdown();
      return 2;
    }

    // 结果输出成可直接复制回参数文件的形式。
    std::printf("target_position: %s\n",
      trunk_two_stage_planner::toYamlArray(config.target_position).c_str());
    std::printf(
      "target_orientation: %s\n",
      trunk_two_stage_planner::toYamlArray(
        {target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w})
        .c_str());
    std::printf("ik_candidate_count: %zu\n", candidates.size());
    for (std::size_t i = 0; i < candidates.size(); ++i) {
      std::printf(
        "ik_candidate_%zu: %s\n",
        i,
        trunk_two_stage_planner::toYamlArray(candidates[i]).c_str());
      std::printf(
        "ik_candidate_%zu_cost: %.6f\n",
        i,
        trunk_two_stage_planner::ikCandidateCost(helper, config, candidates[i], config.q_start));
    }
    std::printf("goal_joint_target: %s\n",
      trunk_two_stage_planner::toYamlArray(q_goal).c_str());
    std::printf("used_fallback_goal: %s\n", used_fallback_goal ? "true" : "false");

    rclcpp::shutdown();
    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "ik_joint_from_pose: %s\n", e.what());
    rclcpp::shutdown();
    return 3;
  }
}
