#include "trunk_two_stage_planner/robot_kinematics_helper.hpp"

#include <fstream>
#include <random>
#include <sstream>
#include <stdexcept>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <moveit/robot_model/joint_model.h>
#include <moveit/robot_model/link_model.h>
#include <moveit/robot_model/prismatic_joint_model.h>
#include <moveit/robot_model/revolute_joint_model.h>
#include <moveit/kinematics_plugin_loader/kinematics_plugin_loader.h>

namespace trunk_two_stage_planner
{

namespace
{

std::string joinPath(const std::string& lhs, const std::string& rhs)
{
  if (lhs.empty()) {
    return rhs;
  }
  if (lhs.back() == '/') {
    return lhs + rhs;
  }
  return lhs + "/" + rhs;
}

}  // namespace

bool RobotKinematicsHelper::initialize(
  const rclcpp::Node::SharedPtr& node,
  const PlannerConfig& config)
{
  // 文件职责：
  // 提供稳定且显式的运动学初始化路径。
  // 避免对外部 launch 参数加载顺序产生隐式依赖。
  node_ = node;
  config_ = config;

  urdf_path_ = joinPath(
    ament_index_cpp::get_package_share_directory(config_.robot_description_package),
    config_.urdf_relative_path);
  srdf_path_ = joinPath(
    ament_index_cpp::get_package_share_directory(config_.moveit_config_package),
    config_.srdf_relative_path);

  const std::string urdf_xml = readTextFile(urdf_path_);
  const std::string srdf_xml = readTextFile(srdf_path_);

  declareOrKeepStringParameter("robot_description", urdf_xml);
  declareOrKeepStringParameter("robot_description_semantic", srdf_xml);
  declareOrKeepStringParameter(
    "robot_description_kinematics." + config_.group_name + ".kinematics_solver",
    "kdl_kinematics_plugin/KDLKinematicsPlugin");
  declareOrKeepDoubleParameter(
    "robot_description_kinematics." + config_.group_name + ".kinematics_solver_search_resolution",
    0.005);
  declareOrKeepDoubleParameter(
    "robot_description_kinematics." + config_.group_name + ".kinematics_solver_timeout",
    config_.ik_timeout);

  robot_model_loader::RobotModelLoader::Options options(urdf_xml, srdf_xml);
  options.robot_description_ = "robot_description";
  // 显式关闭自动加载，避免不同环境下插件选择歧义。
  options.load_kinematics_solvers_ = false;

  model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(node_, options);
  const auto explicit_kinematics_loader =
    std::make_shared<kinematics_plugin_loader::KinematicsPluginLoader>(
    node_,
    "kdl_kinematics_plugin/KDLKinematicsPlugin",
    config_.ik_timeout,
    "robot_description",
    0.005);
  model_loader_->loadKinematicsSolvers(explicit_kinematics_loader);
  robot_model_ = model_loader_->getModel();
  if (!robot_model_) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to create RobotModel.");
    return false;
  }

  joint_model_group_ = robot_model_->getJointModelGroup(config_.group_name);
  if (!joint_model_group_) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "JointModelGroup [%s] was not found.",
      config_.group_name.c_str());
    return false;
  }

  joint_names_ = joint_model_group_->getActiveJointModelNames();
  const auto& link_names = joint_model_group_->getLinkModelNames();
  if (link_names.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "JointModelGroup has no links.");
    return false;
  }
  tip_link_name_ = link_names.back();
  return true;
}

Eigen::Isometry3d RobotKinematicsHelper::getLinkTransform(
  const std::vector<double>& q,
  const std::string& link_name) const
{
  moveit::core::RobotState state = makeRobotState(q);
  return state.getGlobalLinkTransform(link_name);
}

Eigen::Vector3d RobotKinematicsHelper::getLinkPosition(
  const std::vector<double>& q,
  const std::string& link_name) const
{
  return getLinkTransform(q, link_name).translation();
}

bool RobotKinematicsHelper::solveIK(
  const geometry_msgs::msg::Pose& target_pose,
  std::vector<double>& q_solution,
  const std::vector<double>& seed) const
{
  // 注意：
  // seed 质量会直接影响 IK 收敛性，以及多解空间中的分支选择。
  if (!hasIKSolver()) {
    return false;
  }

  moveit::core::RobotState state(robot_model_);
  state.setToDefaultValues();
  if (!seed.empty()) {
    if (seed.size() != joint_names_.size()) {
      throw std::runtime_error("IK seed size does not match trunk_group dimension.");
    }
    state.setJointGroupPositions(joint_model_group_, seed);
  }
  state.update();

  const bool ok = state.setFromIK(
    joint_model_group_,
    target_pose,
    tip_link_name_,
    config_.ik_timeout);
  if (!ok) {
    return false;
  }

  state.copyJointGroupPositions(joint_model_group_, q_solution);
  return true;
}

Axis3D RobotKinematicsHelper::computeJointAxisInWorld(
  const std::vector<double>& q,
  const std::string& joint_name) const
{
  // 将关节坐标系中的轴转换到世界坐标系后计算。
  // 警告：修改此实现会影响所有基于投影的 stage1 几何逻辑。
  moveit::core::RobotState state = makeRobotState(q);
  const moveit::core::JointModel* joint_model = robot_model_->getJointModel(joint_name);
  if (!joint_model) {
    throw std::runtime_error("Joint not found: " + joint_name);
  }

  const moveit::core::LinkModel* parent_link = joint_model->getParentLinkModel();
  const moveit::core::LinkModel* child_link = joint_model->getChildLinkModel();
  if (!child_link) {
    throw std::runtime_error("Child link not found for joint: " + joint_name);
  }

  const Eigen::Isometry3d parent_tf =
    parent_link ? state.getGlobalLinkTransform(parent_link) : Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d joint_frame_tf = parent_tf * child_link->getJointOriginTransform();

  Eigen::Vector3d local_axis = Eigen::Vector3d::UnitZ();
  if (joint_model->getType() == moveit::core::JointModel::REVOLUTE) {
    const auto* revolute =
      dynamic_cast<const moveit::core::RevoluteJointModel*>(joint_model);
    local_axis = revolute->getAxis();
  } else if (joint_model->getType() == moveit::core::JointModel::PRISMATIC) {
    const auto* prismatic =
      dynamic_cast<const moveit::core::PrismaticJointModel*>(joint_model);
    local_axis = prismatic->getAxis();
  } else {
    throw std::runtime_error("Joint axis query only supports revolute/prismatic joints.");
  }

  Axis3D axis;
  axis.origin = joint_frame_tf.translation();
  axis.direction = (joint_frame_tf.linear() * local_axis).normalized();
  return axis;
}

bool RobotKinematicsHelper::isStateWithinBounds(const std::vector<double>& q) const
{
  // 集中式边界检查，供算法层与辅助工具层共用。
  moveit::core::RobotState state = makeRobotState(q);
  return state.satisfiesBounds(joint_model_group_);
}

std::pair<double, double> RobotKinematicsHelper::getJointPositionBounds(
  const std::string& joint_name) const
{
  const auto& bounds = robot_model_->getVariableBounds(joint_name);
  return {bounds.min_position_, bounds.max_position_};
}

std::vector<double> RobotKinematicsHelper::sampleRandomState(std::mt19937& rng) const
{
  // 在关节边界内均匀采样，用于 IK 多样化求解。
  std::vector<double> q;
  q.reserve(joint_names_.size());
  for (const auto& joint_name : joint_names_) {
    const auto [lower, upper] = getJointPositionBounds(joint_name);
    std::uniform_real_distribution<double> dist(lower, upper);
    q.push_back(dist(rng));
  }
  return q;
}

bool RobotKinematicsHelper::getJointIndex(
  const std::string& joint_name,
  std::size_t& index) const
{
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    if (joint_names_[i] == joint_name) {
      index = i;
      return true;
    }
  }
  return false;
}

std::pair<std::string, std::string> RobotKinematicsHelper::getJointParentChildLinks(
  const std::string& joint_name) const
{
  const moveit::core::JointModel* joint_model = robot_model_->getJointModel(joint_name);
  if (!joint_model) {
    throw std::runtime_error("Joint not found: " + joint_name);
  }
  const auto* parent = joint_model->getParentLinkModel();
  const auto* child = joint_model->getChildLinkModel();
  return {
    parent ? parent->getName() : std::string(),
    child ? child->getName() : std::string()
  };
}

moveit::core::RobotModelConstPtr RobotKinematicsHelper::getRobotModel() const
{
  return robot_model_;
}

const moveit::core::JointModelGroup* RobotKinematicsHelper::getJointModelGroup() const
{
  return joint_model_group_;
}

const std::vector<std::string>& RobotKinematicsHelper::getJointNames() const
{
  return joint_names_;
}

const std::string& RobotKinematicsHelper::getTipLinkName() const
{
  return tip_link_name_;
}

const std::string& RobotKinematicsHelper::getUrdfPath() const
{
  return urdf_path_;
}

const std::string& RobotKinematicsHelper::getSrdfPath() const
{
  return srdf_path_;
}

bool RobotKinematicsHelper::hasIKSolver() const
{
  return joint_model_group_ && joint_model_group_->canSetStateFromIK(tip_link_name_);
}

moveit::core::RobotState RobotKinematicsHelper::makeRobotState(const std::vector<double>& q) const
{
  if (q.size() != joint_names_.size()) {
    throw std::runtime_error("Joint vector size does not match trunk_group dimension.");
  }

  moveit::core::RobotState state(robot_model_);
  state.setToDefaultValues();
  state.setJointGroupPositions(joint_model_group_, q);
  state.update();
  return state;
}

std::string RobotKinematicsHelper::readTextFile(const std::string& file_path) const
{
  std::ifstream ifs(file_path);
  if (!ifs.is_open()) {
    throw std::runtime_error("Failed to open file: " + file_path);
  }
  std::ostringstream oss;
  oss << ifs.rdbuf();
  return oss.str();
}

void RobotKinematicsHelper::declareOrKeepStringParameter(
  const std::string& name,
  const std::string& value) const
{
  if (!node_->has_parameter(name)) {
    node_->declare_parameter<std::string>(name, value);
  }
}

void RobotKinematicsHelper::declareOrKeepDoubleParameter(
  const std::string& name,
  double value) const
{
  if (!node_->has_parameter(name)) {
    node_->declare_parameter<double>(name, value);
  }
}

}  // namespace trunk_two_stage_planner
