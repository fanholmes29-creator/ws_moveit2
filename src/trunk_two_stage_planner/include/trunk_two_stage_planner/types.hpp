#pragma once

#include <Eigen/Geometry>
#include <limits>
#include <string>
#include <vector>

namespace trunk_two_stage_planner
{

struct Axis3D
{
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  Eigen::Vector3d direction = Eigen::Vector3d::UnitZ();
};

struct TrajPoint
{
  double t = 0.0;
  std::vector<double> q;
  std::vector<double> dq;
  std::vector<double> ddq;
  int stage_id = 0;
};

struct Stage1HeatmapSample
{
  double q1 = 0.0;
  double q2 = 0.0;
  double cost = std::numeric_limits<double>::infinity();
  bool valid = false;
  double projection_error = std::numeric_limits<double>::infinity();
  double dist_proj_to_goal_proj = std::numeric_limits<double>::infinity();
  double stage2_pos_error = std::numeric_limits<double>::infinity();
  double stage2_pose_error = std::numeric_limits<double>::infinity();
  Eigen::Vector3d o4_projection = Eigen::Vector3d::Zero();
};

struct Stage1SearchResult
{
  bool success = false;
  std::vector<double> q_pre;
  Eigen::Vector3d o4_pre = Eigen::Vector3d::Zero();
  Eigen::Vector3d o4_proj_pre = Eigen::Vector3d::Zero();
  double best_cost = std::numeric_limits<double>::infinity();
  double distance_proj_to_goal_proj = std::numeric_limits<double>::infinity();
  double selected_stage2_pos_error = std::numeric_limits<double>::infinity();
  double selected_stage2_pose_error = std::numeric_limits<double>::infinity();
  double best_projection_error = std::numeric_limits<double>::infinity();
  double best_stage2_pos_error = std::numeric_limits<double>::infinity();
  double best_stage2_pose_error = std::numeric_limits<double>::infinity();
  std::vector<double> q_pre_best_projection;
  std::vector<double> q_pre_best_stage2_pos;
  std::vector<double> q_pre_best_stage2_pose;
  Eigen::Vector3d o4_proj_best_projection = Eigen::Vector3d::Zero();
  Eigen::Vector3d o4_proj_best_stage2_pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d o4_proj_best_stage2_pose = Eigen::Vector3d::Zero();
  std::size_t threshold_feasible_count = 0;
  bool used_threshold_filter = false;
  std::size_t evaluated_samples = 0;
  std::size_t valid_samples = 0;
  std::vector<Stage1HeatmapSample> heatmap_samples;
};

struct PlanningSummary
{
  std::vector<double> q_start;
  std::vector<double> q_goal_ik;
  std::vector<double> q_goal_stage2;
  std::vector<double> q_pre;
  std::vector<double> delta_to_ik_goal;
  std::vector<double> delta_stage2_used;

  Axis3D joint3_axis_world;
  Eigen::Vector3d p_d = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_d_proj_goal_axis = Eigen::Vector3d::Zero();
  Eigen::Vector3d o4_pre = Eigen::Vector3d::Zero();
  Eigen::Vector3d o4_proj_pre = Eigen::Vector3d::Zero();
  std::vector<double> q_pre_best_projection;
  std::vector<double> q_pre_best_stage2_pos;
  std::vector<double> q_pre_best_stage2_pose;
  Eigen::Vector3d o4_proj_best_projection = Eigen::Vector3d::Zero();
  Eigen::Vector3d o4_proj_best_stage2_pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d o4_proj_best_stage2_pose = Eigen::Vector3d::Zero();

  double best_stage1_cost = std::numeric_limits<double>::infinity();
  double o4_proj_to_goal_proj_distance = std::numeric_limits<double>::infinity();
  double selected_stage1_stage2_pos_error = std::numeric_limits<double>::infinity();
  double selected_stage1_stage2_pose_error = std::numeric_limits<double>::infinity();
  double best_projection_error = std::numeric_limits<double>::infinity();
  double best_stage2_pos_error = std::numeric_limits<double>::infinity();
  double best_stage2_pose_error = std::numeric_limits<double>::infinity();
  double stage2_position_error = std::numeric_limits<double>::infinity();
  double stage2_orientation_error_deg = std::numeric_limits<double>::infinity();
  double stage1_max_tilt_deg = 0.0;
  double stage1_mean_tilt_deg = 0.0;

  std::size_t evaluated_samples = 0;
  std::size_t valid_samples = 0;
  std::size_t stage1_point_count = 0;
  std::size_t stage2_point_count = 0;

  bool used_fallback_goal = false;
  std::string stage2_strategy = "lock_q1q2_search_q3_q4";
  bool projection_best_matches_stage2_best = false;
  bool stage2_pos_best_matches_stage2_pose_best = false;
  bool used_threshold_filter = false;
  std::size_t threshold_feasible_count = 0;
  std::vector<Stage1HeatmapSample> heatmap_samples;
};

struct PlannerConfig
{
  std::string robot_description_package = "robot_model";
  std::string urdf_relative_path = "urdf/trunk_robot.urdf";
  std::string moveit_config_package = "trunk_configure";
  std::string srdf_relative_path = "config/trunk_robot.srdf";

  std::string group_name = "trunk_group";
  std::string joint1_name = "trunk_joint1";
  std::string joint2_name = "trunk_joint2";
  std::string joint3_name = "trunk_joint3";
  std::string joint4_name = "trunk_joint4";
  std::string reference_link_name = "trunk_link4";

  std::vector<double> q_start{ 0.0, 0.0, 0.0, 0.0 };
  std::vector<double> goal_joint_target{ -1.5, 0.75, 0.4, 0.3 };
  std::vector<double> q_goal_fallback{ -1.5, 0.75, 0.4, 0.3 };

  bool use_goal_state_as_target_pose = true;
  std::vector<double> target_position{ 0.25, 0.0, 0.35 };
  std::vector<double> target_orientation{ 0.0, 0.0, 0.0, 1.0 };

  bool allow_goal_fallback = true;
  int ik_attempts = 12;
  double ik_timeout = 0.02;
  double ik_limit_penalty_weight = 0.2;

  int stage1_q1_samples = 181;
  int stage1_q2_samples = 181;
  double w1 = 1.0;
  double w2 = 10.0;
  double w3 = 0.05;
  double w4 = 0.5;
  double stage2_pose_wp = 1.0;
  double stage2_pose_wR = 0.2;
  double stage2_pose_epsilon = 0.002;
  double joint_limit_margin_ratio = 0.1;

  int stage2_eval_q3_samples = 41;
  int stage2_eval_q4_samples = 41;
  int stage2_q3_samples = 181;
  int stage2_q4_samples = 181;
  double stage2_pos_weight = 1.0;
  double stage2_ori_weight = 0.2;
  double stage2_q34_bias_weight = 0.02;

  double stage1_duration = 2.0;
  double stage2_duration = 3.0;
  double dt = 0.02;

  std::string output_dir = "/home/wxl/ws_moveit2/csv/two_stage_kinematics";
};

}  // namespace trunk_two_stage_planner
