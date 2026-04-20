#include <cstdio>
#include <exception>
#include <iomanip>
#include <sstream>

#include <rclcpp/rclcpp.hpp>

#include "trunk_two_stage_planner/robot_kinematics_helper.hpp"

namespace trunk_two_stage_planner
{

namespace
{

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

}  // namespace

}  // namespace trunk_two_stage_planner

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  try {
    // Reuse "two_stage_planner_system" node key so existing params file can be passed directly.
    auto node = rclcpp::Node::make_shared(
      "two_stage_planner_system",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    auto config = trunk_two_stage_planner::loadAlgorithmConfig(node);

    trunk_two_stage_planner::RobotKinematicsHelper helper;
    if (!helper.initialize(node, config)) {
      RCLCPP_ERROR(node->get_logger(), "Failed to initialize RobotKinematicsHelper.");
      rclcpp::shutdown();
      return 1;
    }

    if (config.goal_joint_target.size() != helper.getJointNames().size()) {
      throw std::runtime_error("goal_joint_target dimension does not match planning group DOF.");
    }

    const Eigen::Isometry3d tf = helper.getLinkTransform(config.goal_joint_target, helper.getTipLinkName());
    const Eigen::Quaterniond q(tf.rotation());
    const std::vector<double> position{
      tf.translation().x(), tf.translation().y(), tf.translation().z()
    };
    const std::vector<double> orientation{
      q.x(), q.y(), q.z(), q.w()
    };

    std::printf("target_position: %s\n", trunk_two_stage_planner::toYamlArray(position).c_str());
    std::printf("target_orientation: %s\n", trunk_two_stage_planner::toYamlArray(orientation).c_str());
    std::printf("source_goal_joint_target: %s\n",
      trunk_two_stage_planner::toYamlArray(config.goal_joint_target).c_str());

    rclcpp::shutdown();
    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "fk_pose_from_joint: %s\n", e.what());
    rclcpp::shutdown();
    return 2;
  }
}
