#pragma once

#include <string>
#include <vector>

#include "trunk_two_stage_planner/types.hpp"

namespace trunk_two_stage_planner
{

/// 为每个关节生成端点速度/加速度为零的五次多项式轨迹。
std::vector<TrajPoint> generateQuinticTrajectory(
  const std::vector<double>& q_start,
  const std::vector<double>& q_goal,
  double duration,
  double dt,
  int stage_id);

/// 拼接两阶段轨迹，保持时间连续并保留阶段标识。
std::vector<TrajPoint> concatenateTrajectories(
  const std::vector<TrajPoint>& stage1,
  const std::vector<TrajPoint>& stage2);

/// 导出完整时序轨迹数据，用于回放与分析。
bool exportTrajectoryCsv(
  const std::string& file_path,
  const std::vector<std::string>& joint_names,
  const std::vector<TrajPoint>& trajectory);

/// 导出精简规划诊断信息，便于回归对比。
bool exportSummaryTxt(
  const std::string& file_path,
  const std::vector<std::string>& joint_names,
  const PlanningSummary& summary);

/// 导出 stage1 投影几何推理所需的关键点。
bool exportGeometryCsv(
  const std::string& file_path,
  const PlanningSummary& summary);

/// 导出 stage1 网格搜索热力图诊断数据，用于调参。
bool exportStage1HeatmapCsv(
  const std::string& file_path,
  const std::vector<Stage1HeatmapSample>& heatmap_samples);

}  // namespace trunk_two_stage_planner
