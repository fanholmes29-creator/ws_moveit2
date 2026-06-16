#include <algorithm>
#include <cmath>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <sstream>

#include <rclcpp/rclcpp.hpp>

#include "trunk_two_stage_planner/robot_kinematics_helper.hpp"
#include "trunk_two_stage_planner/trajectory_utils.hpp"
#include "trunk_two_stage_planner/two_stage_planner.hpp"

namespace trunk_two_stage_planner
{

namespace
{

// 与运行时入口保持一致的参数声明辅助函数。
// 在 launch/CLI 注入 YAML 覆盖参数时，保证参数加载鲁棒。
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

std::string formatVector(const std::vector<double>& values)
{
  std::ostringstream oss;
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

std::string formatVec3(const Eigen::Vector3d& p)
{
  std::ostringstream oss;
  oss << "[" << p.x() << ", " << p.y() << ", " << p.z() << "]";
  return oss.str();
}

geometry_msgs::msg::Pose toPoseMsg(const Eigen::Isometry3d& tf)
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

PlannerConfig loadConfig(const rclcpp::Node::SharedPtr& node)
{
  // 分析模式与运行模式共享同一套算法配置结构。
  // 这样可保证离线调参结果可直接迁移到在线运行。
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

}  // namespace

}  // namespace trunk_two_stage_planner

int main(int argc, char* argv[])
{
  // 文件职责：
  // 离线分析入口：评估算法行为并导出诊断数据。
  // 这里不会发起 MoveGroup 规划请求。
  rclcpp::init(argc, argv);

  try {
    auto node = rclcpp::Node::make_shared(
      "two_stage_planner_analysis_tool",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    auto config = trunk_two_stage_planner::loadConfig(node);

    trunk_two_stage_planner::RobotKinematicsHelper helper;
    if (!helper.initialize(node, config)) {
      RCLCPP_ERROR(node->get_logger(), "Failed to initialize RobotKinematicsHelper.");
      rclcpp::shutdown();
      return 1;
    }

    const auto [joint3_parent, joint3_child] = helper.getJointParentChildLinks(config.joint3_name);

    RCLCPP_INFO(node->get_logger(), "===== Workspace Analysis =====");
    RCLCPP_INFO(node->get_logger(), "robot package: %s", config.robot_description_package.c_str());
    RCLCPP_INFO(node->get_logger(), "URDF: %s", helper.getUrdfPath().c_str());
    RCLCPP_INFO(node->get_logger(), "MoveIt config package: %s", config.moveit_config_package.c_str());
    RCLCPP_INFO(node->get_logger(), "SRDF: %s", helper.getSrdfPath().c_str());
    RCLCPP_INFO(node->get_logger(), "planning group: %s", config.group_name.c_str());
    {
      std::ostringstream oss;
      oss << "[";
      const auto& names = helper.getJointNames();
      for (std::size_t i = 0; i < names.size(); ++i) {
        if (i > 0) {
          oss << ", ";
        }
        oss << names[i];
      }
      oss << "]";
      RCLCPP_INFO(node->get_logger(), "joint names: %s", oss.str().c_str());
    }
    RCLCPP_INFO(node->get_logger(), "tip link: %s", helper.getTipLinkName().c_str());
    RCLCPP_INFO(
      node->get_logger(), "%s parent/child: %s -> %s",
      config.joint3_name.c_str(), joint3_parent.c_str(), joint3_child.c_str());
    RCLCPP_INFO(node->get_logger(), "IK available: %s", helper.hasIKSolver() ? "true" : "false");

    geometry_msgs::msg::Pose target_pose;
    if (config.use_goal_state_as_target_pose) {
      target_pose = trunk_two_stage_planner::toPoseMsg(
        helper.getLinkTransform(config.goal_joint_target, helper.getTipLinkName()));
    } else {
      if (config.target_position.size() != 3 || config.target_orientation.size() != 4) {
        throw std::runtime_error("target_position/target_orientation dimension is invalid.");
      }
      target_pose.position.x = config.target_position[0];
      target_pose.position.y = config.target_position[1];
      target_pose.position.z = config.target_position[2];
      target_pose.orientation.x = config.target_orientation[0];
      target_pose.orientation.y = config.target_orientation[1];
      target_pose.orientation.z = config.target_orientation[2];
      target_pose.orientation.w = config.target_orientation[3];
    }

    trunk_two_stage_planner::TwoStagePlanner planner(helper, config);
    trunk_two_stage_planner::PlanningSummary summary;
    if (!planner.plan(config.q_start, target_pose, summary)) {
      RCLCPP_ERROR(node->get_logger(), "Two-stage offline kinematics planning failed.");
      rclcpp::shutdown();
      return 1;
    }

    const auto stage1 = trunk_two_stage_planner::generateQuinticTrajectory(
      summary.q_start, summary.q_pre, config.stage1_duration, config.dt, 1);
    const auto stage2 = trunk_two_stage_planner::generateQuinticTrajectory(
      summary.q_pre, summary.q_goal_stage2, config.stage2_duration, config.dt, 2);
    const auto full_trajectory = trunk_two_stage_planner::concatenateTrajectories(stage1, stage2);

    summary.stage1_point_count = stage1.size();
    summary.stage2_point_count = stage2.size();

    {
      // stage1 直立性指标，用于调参时跟踪舒适性/安全性趋势。
      double tilt_sum_deg = 0.0;
      double tilt_max_deg = 0.0;
      for (const auto& point : stage1) {
        const Eigen::Isometry3d tf = helper.getLinkTransform(point.q, config.reference_link_name);
        const Eigen::Vector3d link_z_world = tf.linear() * Eigen::Vector3d::UnitZ();
        const double cos_angle = std::clamp(
          link_z_world.normalized().dot(Eigen::Vector3d::UnitZ()), -1.0, 1.0);
        const double tilt_deg = std::acos(cos_angle) * 180.0 / M_PI;
        tilt_sum_deg += tilt_deg;
        tilt_max_deg = std::max(tilt_max_deg, tilt_deg);
      }
      summary.stage1_max_tilt_deg = tilt_max_deg;
      summary.stage1_mean_tilt_deg =
        stage1.empty() ? 0.0 : tilt_sum_deg / static_cast<double>(stage1.size());
    }

    std::filesystem::create_directories(config.output_dir);
    const std::string traj_csv = config.output_dir + "/trajectory.csv";
    const std::string summary_txt = config.output_dir + "/summary.txt";
    const std::string geometry_csv = config.output_dir + "/geometry_points.csv";
    const std::string heatmap_csv = config.output_dir + "/stage1_heatmap.csv";

    if (!trunk_two_stage_planner::exportTrajectoryCsv(
          traj_csv, helper.getJointNames(), full_trajectory) ||
        !trunk_two_stage_planner::exportSummaryTxt(
          summary_txt, helper.getJointNames(), summary) ||
        !trunk_two_stage_planner::exportStage1HeatmapCsv(
          heatmap_csv, summary.heatmap_samples) ||
        !trunk_two_stage_planner::exportGeometryCsv(
          geometry_csv, summary)) {
      throw std::runtime_error("Failed to export offline verification outputs.");
    }

    RCLCPP_INFO(node->get_logger(), "===== Result Summary =====");
    RCLCPP_INFO(
      node->get_logger(), "q_start: %s",
      trunk_two_stage_planner::formatVector(summary.q_start).c_str());
    RCLCPP_INFO(
      node->get_logger(), "q_goal_ik: %s",
      trunk_two_stage_planner::formatVector(summary.q_goal_ik).c_str());
    RCLCPP_INFO(
      node->get_logger(), "q_goal_stage2: %s",
      trunk_two_stage_planner::formatVector(summary.q_goal_stage2).c_str());
    RCLCPP_INFO(
      node->get_logger(), "q_pre: %s",
      trunk_two_stage_planner::formatVector(summary.q_pre).c_str());
    RCLCPP_INFO(
      node->get_logger(), "delta(q_goal_ik - q_pre): %s",
      trunk_two_stage_planner::formatVector(summary.delta_to_ik_goal).c_str());
    RCLCPP_INFO(
      node->get_logger(), "delta(stage2_used - q_pre): %s",
      trunk_two_stage_planner::formatVector(summary.delta_stage2_used).c_str());
    RCLCPP_INFO(
      node->get_logger(), "q_pre_best_projection: %s",
      trunk_two_stage_planner::formatVector(summary.q_pre_best_projection).c_str());
    RCLCPP_INFO(
      node->get_logger(), "q_pre_best_stage2_pos: %s",
      trunk_two_stage_planner::formatVector(summary.q_pre_best_stage2_pos).c_str());
    RCLCPP_INFO(
      node->get_logger(), "q_pre_best_stage2_pose: %s",
      trunk_two_stage_planner::formatVector(summary.q_pre_best_stage2_pose).c_str());
    RCLCPP_INFO(
      node->get_logger(), "joint3 axis origin: %s",
      trunk_two_stage_planner::formatVec3(summary.joint3_axis_world.origin).c_str());
    RCLCPP_INFO(
      node->get_logger(), "joint3 axis direction: %s",
      trunk_two_stage_planner::formatVec3(summary.joint3_axis_world.direction).c_str());
    RCLCPP_INFO(
      node->get_logger(), "p_d: %s",
      trunk_two_stage_planner::formatVec3(summary.p_d).c_str());
    RCLCPP_INFO(
      node->get_logger(), "proj_{L3(q_goal)}(p_d): %s",
      trunk_two_stage_planner::formatVec3(summary.p_d_proj_goal_axis).c_str());
    RCLCPP_INFO(
      node->get_logger(), "O4(q_pre): %s",
      trunk_two_stage_planner::formatVec3(summary.o4_pre).c_str());
    RCLCPP_INFO(
      node->get_logger(), "Proj_{L3(q_pre)}(O4(q_pre)): %s",
      trunk_two_stage_planner::formatVec3(summary.o4_proj_pre).c_str());
    RCLCPP_INFO(
      node->get_logger(), "||Proj_{L3(q_pre)}(O4(q_pre)) - Proj_{L3(q_goal)}(p_d)||: %.6f",
      summary.o4_proj_to_goal_proj_distance);
    RCLCPP_INFO(
      node->get_logger(), "best projection / best stage2 pos / best stage2 pose: %.6f / %.6f / %.6f",
      summary.best_projection_error, summary.best_stage2_pos_error, summary.best_stage2_pose_error);
    RCLCPP_INFO(
      node->get_logger(), "threshold feasible count / used threshold filter: %zu / %s",
      summary.threshold_feasible_count,
      summary.used_threshold_filter ? "true" : "false");
    RCLCPP_INFO(
      node->get_logger(), "selected q_pre stage2 pos / pose error: %.6f / %.6f",
      summary.selected_stage1_stage2_pos_error, summary.selected_stage1_stage2_pose_error);
    RCLCPP_INFO(
      node->get_logger(), "projection-best matches stage2-best: %s",
      summary.projection_best_matches_stage2_best ? "true" : "false");
    RCLCPP_INFO(
      node->get_logger(), "stage2-pos-best matches stage2-pose-best: %s",
      summary.stage2_pos_best_matches_stage2_pose_best ? "true" : "false");
    RCLCPP_INFO(
      node->get_logger(), "stage1 points: %zu, stage2 points: %zu",
      summary.stage1_point_count, summary.stage2_point_count);
    RCLCPP_INFO(
      node->get_logger(), "stage1 tilt max/mean [deg]: %.6f / %.6f",
      summary.stage1_max_tilt_deg, summary.stage1_mean_tilt_deg);
    RCLCPP_INFO(
      node->get_logger(), "stage2 target pose error [pos m / ori deg]: %.6f / %.6f",
      summary.stage2_position_error, summary.stage2_orientation_error_deg);
    RCLCPP_INFO(
      node->get_logger(), "samples evaluated: %zu, valid: %zu",
      summary.evaluated_samples, summary.valid_samples);
    RCLCPP_INFO(node->get_logger(), "trajectory csv: %s", traj_csv.c_str());
    RCLCPP_INFO(node->get_logger(), "summary txt: %s", summary_txt.c_str());
    RCLCPP_INFO(node->get_logger(), "geometry csv: %s", geometry_csv.c_str());
    RCLCPP_INFO(node->get_logger(), "stage1 heatmap csv: %s", heatmap_csv.c_str());

    rclcpp::shutdown();
    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "two_stage_planner_analysis_tool: %s\n", e.what());
    rclcpp::shutdown();
    return 2;
  }
}
