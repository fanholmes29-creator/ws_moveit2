#pragma once

#include <string>
#include <vector>

#include "trunk_two_stage_planner/types.hpp"

namespace trunk_two_stage_planner
{

std::vector<TrajPoint> generateQuinticTrajectory(
  const std::vector<double>& q_start,
  const std::vector<double>& q_goal,
  double duration,
  double dt,
  int stage_id);

std::vector<TrajPoint> concatenateTrajectories(
  const std::vector<TrajPoint>& stage1,
  const std::vector<TrajPoint>& stage2);

bool exportTrajectoryCsv(
  const std::string& file_path,
  const std::vector<std::string>& joint_names,
  const std::vector<TrajPoint>& trajectory);

bool exportSummaryTxt(
  const std::string& file_path,
  const std::vector<std::string>& joint_names,
  const PlanningSummary& summary);

bool exportGeometryCsv(
  const std::string& file_path,
  const PlanningSummary& summary);

bool exportStage1HeatmapCsv(
  const std::string& file_path,
  const std::vector<Stage1HeatmapSample>& heatmap_samples);

}  // namespace trunk_two_stage_planner
