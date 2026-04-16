#include <chrono>
#include <cstdio>
#include <exception>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "trunk_two_stage_planner/two_stage_planner_manager.hpp"

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

TwoStageSystemConfig loadSystemConfig(const rclcpp::Node::SharedPtr& node)
{
  TwoStageSystemConfig config;
  declareIfMissingString(node, "stage1_group_name", config.stage1_group_name);
  declareIfMissingString(node, "stage2_group_name", config.stage2_group_name);
  declareIfMissingString(node, "planning_frame", config.planning_frame);
  declareIfMissingString(node, "marker_topic", config.marker_topic);
  declareIfMissingString(node, "display_trajectory_topic", config.display_trajectory_topic);
  declareIfMissingDouble(node, "planning_time", config.planning_time);
  declareIfMissingInt(node, "planning_attempts", config.planning_attempts);
  declareIfMissingDouble(node, "velocity_scaling", config.velocity_scaling);
  declareIfMissingDouble(node, "acceleration_scaling", config.acceleration_scaling);
  declareIfMissingDouble(node, "stage1_eef_step", config.stage1_eef_step);
  declareIfMissingDouble(node, "stage1_jump_threshold", config.stage1_jump_threshold);
  declareIfMissingDouble(node, "stage1_min_fraction", config.stage1_min_fraction);
  declareIfMissingInt(node, "stage1_waypoint_count", config.stage1_waypoint_count);
  declareIfMissingDouble(node, "stage2_q12_tolerance", config.stage2_q12_tolerance);
  declareIfMissingBool(node, "export_csv", config.export_csv);

  config.stage1_group_name = node->get_parameter("stage1_group_name").as_string();
  config.stage2_group_name = node->get_parameter("stage2_group_name").as_string();
  config.planning_frame = node->get_parameter("planning_frame").as_string();
  config.marker_topic = node->get_parameter("marker_topic").as_string();
  config.display_trajectory_topic = node->get_parameter("display_trajectory_topic").as_string();
  config.planning_time = node->get_parameter("planning_time").as_double();
  config.planning_attempts = node->get_parameter("planning_attempts").as_int();
  config.velocity_scaling = node->get_parameter("velocity_scaling").as_double();
  config.acceleration_scaling = node->get_parameter("acceleration_scaling").as_double();
  config.stage1_eef_step = node->get_parameter("stage1_eef_step").as_double();
  config.stage1_jump_threshold = node->get_parameter("stage1_jump_threshold").as_double();
  config.stage1_min_fraction = node->get_parameter("stage1_min_fraction").as_double();
  config.stage1_waypoint_count = node->get_parameter("stage1_waypoint_count").as_int();
  config.stage2_q12_tolerance = node->get_parameter("stage2_q12_tolerance").as_double();
  config.export_csv = node->get_parameter("export_csv").as_bool();
  return config;
}

geometry_msgs::msg::Pose buildTargetPose(
  const PlannerConfig& config,
  const RobotKinematicsHelper& helper)
{
  if (config.use_goal_state_as_target_pose) {
    const Eigen::Isometry3d tf =
      helper.getLinkTransform(config.goal_joint_target, helper.getTipLinkName());
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

  geometry_msgs::msg::Pose pose;
  pose.position.x = config.target_position[0];
  pose.position.y = config.target_position[1];
  pose.position.z = config.target_position[2];
  pose.orientation.x = config.target_orientation[0];
  pose.orientation.y = config.target_orientation[1];
  pose.orientation.z = config.target_orientation[2];
  pose.orientation.w = config.target_orientation[3];
  return pose;
}

}  // namespace

}  // namespace trunk_two_stage_planner

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  try {
    auto node = rclcpp::Node::make_shared(
      "two_stage_planner_system",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });

    auto algorithm_config = trunk_two_stage_planner::loadAlgorithmConfig(node);
    auto system_config = trunk_two_stage_planner::loadSystemConfig(node);

    trunk_two_stage_planner::TwoStagePlannerManager manager(node);
    if (!manager.initialize(algorithm_config, system_config)) {
      RCLCPP_ERROR(node->get_logger(), "Failed to initialize TwoStagePlannerManager.");
      executor.cancel();
      spinner.join();
      rclcpp::shutdown();
      return 1;
    }

    trunk_two_stage_planner::RobotKinematicsHelper pose_helper;
    if (!pose_helper.initialize(node, algorithm_config)) {
      RCLCPP_ERROR(node->get_logger(), "Failed to initialize target-pose helper.");
      executor.cancel();
      spinner.join();
      rclcpp::shutdown();
      return 1;
    }

    const auto target_pose = trunk_two_stage_planner::buildTargetPose(
      algorithm_config, pose_helper);

    if (!manager.planTwoStageToTarget(target_pose)) {
      RCLCPP_ERROR(node->get_logger(), "Two-stage planning system failed.");
      executor.cancel();
      spinner.join();
      rclcpp::shutdown();
      return 1;
    }

    RCLCPP_INFO(
      node->get_logger(),
      "Two-stage planner system is running. Keep RViz open to inspect trajectories and markers.");

    while (rclcpp::ok()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "two_stage_planner_system: %s\n", e.what());
    rclcpp::shutdown();
    return 2;
  }
}
