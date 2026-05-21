#pragma once

#include <Eigen/Geometry>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace trunk_two_stage_planner
{

/// 世界坐标系下的三维轴线表示（原点 + 单位方向）。
struct Axis3D
{
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  Eigen::Vector3d direction = Eigen::Vector3d::UnitZ();
};

/// 统一轨迹点格式，用于离线导出与阶段轨迹拼接。
struct TrajPoint
{
  double t = 0.0;
  std::vector<double> q;
  std::vector<double> dq;
  std::vector<double> ddq;
  int stage_id = 0;
};

/// stage1 搜索热力图的单网格采样诊断信息。
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

/// stage1 搜索完整结果，包含入选候选与多维最优指标。
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

/**
 * @brief 传递给运行时管理器与导出模块的完整算法摘要。
 *
 * 包含：
 *  - 选中状态（`q_pre`、`q_goal_stage2`）
 *  - 几何诊断量与误差指标
 *  - stage1 网格搜索统计与热力图数据
 */
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

/// 规划链路中的可诊断失败类型。
enum class PlannerError
{
  Ok = 0,
  NotInitialized,
  InvalidInput,
  StartStateUnavailable,
  KinematicsInitializationFailed,
  IkFailed,
  Stage1SearchFailed,
  Stage2OptimizationFailed,
  Stage1MoveItPlanningFailed,
  Stage2MoveItPlanningFailed,
  TrajectoryExecutionFailed,
  ExportFailed,
  Unknown
};

inline const char* plannerErrorToString(PlannerError error)
{
  switch (error) {
    case PlannerError::Ok:
      return "Ok";
    case PlannerError::NotInitialized:
      return "NotInitialized";
    case PlannerError::InvalidInput:
      return "InvalidInput";
    case PlannerError::StartStateUnavailable:
      return "StartStateUnavailable";
    case PlannerError::KinematicsInitializationFailed:
      return "KinematicsInitializationFailed";
    case PlannerError::IkFailed:
      return "IkFailed";
    case PlannerError::Stage1SearchFailed:
      return "Stage1SearchFailed";
    case PlannerError::Stage2OptimizationFailed:
      return "Stage2OptimizationFailed";
    case PlannerError::Stage1MoveItPlanningFailed:
      return "Stage1MoveItPlanningFailed";
    case PlannerError::Stage2MoveItPlanningFailed:
      return "Stage2MoveItPlanningFailed";
    case PlannerError::TrajectoryExecutionFailed:
      return "TrajectoryExecutionFailed";
    case PlannerError::ExportFailed:
      return "ExportFailed";
    case PlannerError::Unknown:
      return "Unknown";
  }
  return "Unknown";
}

/// 带错误原因与诊断摘要的规划结果，供 SDK/API 层直接返回给调用方。
struct PlannerResult
{
  bool success = false;
  PlannerError error = PlannerError::Unknown;
  std::string message;
  PlanningSummary summary;

  static PlannerResult ok(
    const PlanningSummary& summary,
    std::string message = "Planning succeeded.")
  {
    PlannerResult result;
    result.success = true;
    result.error = PlannerError::Ok;
    result.message = std::move(message);
    result.summary = summary;
    return result;
  }

  static PlannerResult fail(
    PlannerError error,
    std::string message,
    const PlanningSummary& summary = PlanningSummary())
  {
    PlannerResult result;
    result.success = false;
    result.error = error;
    result.message = std::move(message);
    result.summary = summary;
    return result;
  }
};

/**
 * @brief 两阶段规划的算法层可调参数集合。
 *
 * 高影响调参分组：
 *  - stage1 加权目标：`w1..w4`
 *  - stage2 可恢复性阈值：`stage2_pose_epsilon`
 *  - 位姿残差构成：`stage2_pose_wp`、`stage2_pose_wR`
 *  - 网格搜索密度：`stage1_*samples`、`stage2_*samples`
 *
 * 警告：
 *  - 增大采样数可提升精度，但计算代价近似二次上升。
 *  - 修改 `stage2_pose_epsilon` 会直接改变策略切换行为。
 */
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

  bool allow_goal_fallback = false;
  int ik_attempts = 12;
  double ik_timeout = 0.02;
  double ik_limit_penalty_weight = 0.2;

  std::string stage1_pre_mode = "ik_q12_straight";
  int stage1_q1_samples = 181;
  int stage1_q2_samples = 181;
  // stage1 加权目标项。
  double w1 = 1.0;
  double w2 = 10.0;
  double w3 = 0.05;
  double w4 = 0.5;
  // stage2 可恢复性评估时的位姿残差构成权重。
  double stage2_pose_wp = 1.0;
  double stage2_pose_wR = 0.2;
  // 切换到“可恢复候选子集”策略的阈值。
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

  std::string output_dir = "csv/two_stage_kinematics";
};

}  // namespace trunk_two_stage_planner
