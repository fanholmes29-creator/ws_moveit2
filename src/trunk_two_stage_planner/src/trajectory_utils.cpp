#include "trunk_two_stage_planner/trajectory_utils.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace trunk_two_stage_planner
{

namespace
{

void ensureParentDirectory(const std::string& file_path)
{
  const std::filesystem::path path(file_path);
  const auto parent = path.parent_path();
  if (!parent.empty()) {
    std::filesystem::create_directories(parent);
  }
}

std::string vectorToCsv(const std::vector<double>& values)
{
  std::ostringstream oss;
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i > 0) {
      oss << ",";
    }
    oss << values[i];
  }
  return oss.str();
}

std::string vec3ToCsv(const Eigen::Vector3d& p)
{
  std::ostringstream oss;
  oss << p.x() << "," << p.y() << "," << p.z();
  return oss.str();
}

}  // namespace

std::vector<TrajPoint> generateQuinticTrajectory(
  const std::vector<double>& q_start,
  const std::vector<double>& q_goal,
  double duration,
  double dt,
  int stage_id)
{
  // 文件职责：
  // 仅提供确定性的轨迹生成与导出工具。
  // 为保证离线/在线结果可对比，此处行为应保持稳定。
  if (q_start.size() != q_goal.size()) {
    throw std::runtime_error("Quintic trajectory endpoints have different dimensions.");
  }
  if (duration <= 0.0 || dt <= 0.0) {
    throw std::runtime_error("Quintic trajectory requires positive duration and dt.");
  }

  const std::size_t dof = q_start.size();
  const std::size_t point_count =
    static_cast<std::size_t>(std::floor(duration / dt)) + 1;

  std::vector<TrajPoint> trajectory;
  trajectory.reserve(point_count + 1);

  for (std::size_t i = 0; i <= point_count; ++i) {
    const double t = std::min(duration, static_cast<double>(i) * dt);
    TrajPoint point;
    point.t = t;
    point.stage_id = stage_id;
    point.q.resize(dof, 0.0);
    point.dq.resize(dof, 0.0);
    point.ddq.resize(dof, 0.0);

    for (std::size_t j = 0; j < dof; ++j) {
      const double delta = q_goal[j] - q_start[j];
      const double T = duration;
      const double a0 = q_start[j];
      const double a1 = 0.0;
      const double a2 = 0.0;
      const double a3 = 10.0 * delta / std::pow(T, 3);
      const double a4 = -15.0 * delta / std::pow(T, 4);
      const double a5 = 6.0 * delta / std::pow(T, 5);

      point.q[j] = a0 + a1 * t + a2 * t * t + a3 * std::pow(t, 3) + a4 * std::pow(t, 4) + a5 * std::pow(t, 5);
      point.dq[j] = a1 + 2.0 * a2 * t + 3.0 * a3 * t * t + 4.0 * a4 * std::pow(t, 3) + 5.0 * a5 * std::pow(t, 4);
      point.ddq[j] = 2.0 * a2 + 6.0 * a3 * t + 12.0 * a4 * t * t + 20.0 * a5 * std::pow(t, 3);
    }

    trajectory.push_back(point);
    if (t >= duration) {
      break;
    }
  }

  return trajectory;
}

std::vector<TrajPoint> concatenateTrajectories(
  const std::vector<TrajPoint>& stage1,
  const std::vector<TrajPoint>& stage2)
{
  // 当 stage1 非空时，跳过重复边界采样点。
  std::vector<TrajPoint> merged = stage1;
  if (stage2.empty()) {
    return merged;
  }

  const double time_offset = stage1.empty() ? 0.0 : stage1.back().t;
  const std::size_t start_index = stage1.empty() ? 0 : 1;
  for (std::size_t i = start_index; i < stage2.size(); ++i) {
    TrajPoint shifted = stage2[i];
    shifted.t += time_offset;
    merged.push_back(shifted);
  }
  return merged;
}

bool exportTrajectoryCsv(
  const std::string& file_path,
  const std::vector<std::string>& joint_names,
  const std::vector<TrajPoint>& trajectory)
{
  // 有意使用纯 CSV 导出，便于后处理工具无依赖接入。
  ensureParentDirectory(file_path);
  std::ofstream ofs(file_path);
  if (!ofs.is_open()) {
    return false;
  }

  ofs << std::fixed << std::setprecision(6);
  ofs << "t,stage_id";
  for (const auto& name : joint_names) {
    ofs << "," << name;
  }
  for (const auto& name : joint_names) {
    ofs << ",d" << name;
  }
  for (const auto& name : joint_names) {
    ofs << ",dd" << name;
  }
  ofs << "\n";

  for (const auto& point : trajectory) {
    ofs << point.t << "," << point.stage_id;
    for (double value : point.q) {
      ofs << "," << value;
    }
    for (double value : point.dq) {
      ofs << "," << value;
    }
    for (double value : point.ddq) {
      ofs << "," << value;
    }
    ofs << "\n";
  }

  return true;
}

bool exportSummaryTxt(
  const std::string& file_path,
  const std::vector<std::string>& joint_names,
  const PlanningSummary& summary)
{
  // 导出可读摘要，记录选中路径与阶段切换诊断信息。
  ensureParentDirectory(file_path);
  std::ofstream ofs(file_path);
  if (!ofs.is_open()) {
    return false;
  }

  ofs << std::fixed << std::setprecision(6);
  ofs << "joint_name_labels: ";
  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    if (i > 0) {
      ofs << ",";
    }
    ofs << joint_names[i];
  }
  ofs << "\n";
  ofs << "q_start: " << vectorToCsv(summary.q_start) << "\n";
  ofs << "q_goal_ik: " << vectorToCsv(summary.q_goal_ik) << "\n";
  ofs << "q_goal_stage2: " << vectorToCsv(summary.q_goal_stage2) << "\n";
  ofs << "q_pre: " << vectorToCsv(summary.q_pre) << "\n";
  ofs << "q_pre_best_projection: " << vectorToCsv(summary.q_pre_best_projection) << "\n";
  ofs << "q_pre_best_stage2_pos: " << vectorToCsv(summary.q_pre_best_stage2_pos) << "\n";
  ofs << "q_pre_best_stage2_pose: " << vectorToCsv(summary.q_pre_best_stage2_pose) << "\n";
  ofs << "delta_to_ik_goal: " << vectorToCsv(summary.delta_to_ik_goal) << "\n";
  ofs << "delta_stage2_used: " << vectorToCsv(summary.delta_stage2_used) << "\n";
  ofs << "joint3_axis_origin: " << vec3ToCsv(summary.joint3_axis_world.origin) << "\n";
  ofs << "joint3_axis_direction: " << vec3ToCsv(summary.joint3_axis_world.direction) << "\n";
  ofs << "p_d: " << vec3ToCsv(summary.p_d) << "\n";
  ofs << "p_d_proj_goal_axis: " << vec3ToCsv(summary.p_d_proj_goal_axis) << "\n";
  ofs << "o4_pre: " << vec3ToCsv(summary.o4_pre) << "\n";
  ofs << "o4_proj_pre: " << vec3ToCsv(summary.o4_proj_pre) << "\n";
  ofs << "stage1_geometry_definition: compare ||Proj_{L3(q)}(O4(q)) - Proj_{L3(q_goal)}(p_d)||^2\n";
  ofs << "distance_o4_proj_pre_to_goal_proj: " << summary.o4_proj_to_goal_proj_distance << "\n";
  ofs << "selected_stage1_stage2_pos_error: " << summary.selected_stage1_stage2_pos_error << "\n";
  ofs << "best_stage1_cost: " << summary.best_stage1_cost << "\n";
  ofs << "best_projection_error: " << summary.best_projection_error << "\n";
  ofs << "best_stage2_pos_error: " << summary.best_stage2_pos_error << "\n";
  ofs << "best_stage2_pose_error: " << summary.best_stage2_pose_error << "\n";
  ofs << "threshold_feasible_count: " << summary.threshold_feasible_count << "\n";
  ofs << "used_threshold_filter: " << (summary.used_threshold_filter ? "true" : "false") << "\n";
  ofs << "selected_stage1_stage2_pos_error: " << summary.selected_stage1_stage2_pos_error << "\n";
  ofs << "selected_stage1_stage2_pose_error: " << summary.selected_stage1_stage2_pose_error << "\n";
  ofs << "projection_best_matches_stage2_best: "
      << (summary.projection_best_matches_stage2_best ? "true" : "false") << "\n";
  ofs << "stage2_pos_best_matches_stage2_pose_best: "
      << (summary.stage2_pos_best_matches_stage2_pose_best ? "true" : "false") << "\n";
  ofs << "stage2_strategy: " << summary.stage2_strategy << "\n";
  ofs << "stage2_position_error: " << summary.stage2_position_error << "\n";
  ofs << "stage2_orientation_error_deg: " << summary.stage2_orientation_error_deg << "\n";
  ofs << "stage1_max_tilt_deg: " << summary.stage1_max_tilt_deg << "\n";
  ofs << "stage1_mean_tilt_deg: " << summary.stage1_mean_tilt_deg << "\n";
  ofs << "evaluated_samples: " << summary.evaluated_samples << "\n";
  ofs << "valid_samples: " << summary.valid_samples << "\n";
  ofs << "stage1_point_count: " << summary.stage1_point_count << "\n";
  ofs << "stage2_point_count: " << summary.stage2_point_count << "\n";
  ofs << "used_fallback_goal: " << (summary.used_fallback_goal ? "true" : "false") << "\n";
  return true;
}

bool exportGeometryCsv(
  const std::string& file_path,
  const PlanningSummary& summary)
{
  ensureParentDirectory(file_path);
  std::ofstream ofs(file_path);
  if (!ofs.is_open()) {
    return false;
  }

  ofs << std::fixed << std::setprecision(6);
  ofs << "name,x,y,z\n";
  ofs << "p_d," << vec3ToCsv(summary.p_d) << "\n";
  ofs << "p_d_proj_goal_axis," << vec3ToCsv(summary.p_d_proj_goal_axis) << "\n";
  ofs << "o4_pre," << vec3ToCsv(summary.o4_pre) << "\n";
  ofs << "o4_proj_pre," << vec3ToCsv(summary.o4_proj_pre) << "\n";
  ofs << "joint3_axis_origin," << vec3ToCsv(summary.joint3_axis_world.origin) << "\n";
  return true;
}

bool exportStage1HeatmapCsv(
  const std::string& file_path,
  const std::vector<Stage1HeatmapSample>& heatmap_samples)
{
  // 注意：
  // 该文件是调节阈值/权重与候选筛选逻辑时的核心分析产物。
  ensureParentDirectory(file_path);
  std::ofstream ofs(file_path);
  if (!ofs.is_open()) {
    return false;
  }

  ofs << std::fixed << std::setprecision(6);
  ofs << "q1,q2,cost,valid_flag,projection_error,dist_proj_to_goal_proj,stage2_pos_error,stage2_pose_error,"
         "o4_proj_x,o4_proj_y,o4_proj_z\n";
  for (const auto& sample : heatmap_samples) {
    ofs << sample.q1 << "," << sample.q2 << ",";
    if (std::isfinite(sample.cost)) {
      ofs << sample.cost;
    } else {
      ofs << "inf";
    }
    ofs << "," << (sample.valid ? 1 : 0) << ",";
    if (std::isfinite(sample.projection_error)) {
      ofs << sample.projection_error;
    } else {
      ofs << "inf";
    }
    ofs << ",";
    if (std::isfinite(sample.dist_proj_to_goal_proj)) {
      ofs << sample.dist_proj_to_goal_proj;
    } else {
      ofs << "inf";
    }
    ofs << ",";
    if (std::isfinite(sample.stage2_pos_error)) {
      ofs << sample.stage2_pos_error;
    } else {
      ofs << "inf";
    }
    ofs << ",";
    if (std::isfinite(sample.stage2_pose_error)) {
      ofs << sample.stage2_pose_error;
    } else {
      ofs << "inf";
    }
    ofs << "," << sample.o4_projection.x()
        << "," << sample.o4_projection.y()
        << "," << sample.o4_projection.z() << "\n";
  }

  return true;
}

}  // namespace trunk_two_stage_planner
