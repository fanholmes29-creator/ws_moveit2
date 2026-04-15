#include <memory>
#include <thread>
#include <vector>
#include <string>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

using moveit::planning_interface::MoveGroupInterface;

// 打印一组关节值，方便观察当前状态或目标状态
void printJoints(const rclcpp::Logger & logger,
                 const std::string & label,
                 const std::vector<double> & joints)
{
  std::ostringstream oss;
  oss << label << " [";
  for (size_t i = 0; i < joints.size(); ++i)
  {
    oss << joints[i];
    if (i + 1 < joints.size())
    {
      oss << ", ";
    }
  }
  oss << "]";
  RCLCPP_INFO(logger, "%s", oss.str().c_str());
}

// 函数：给定一组目标关节角，执行一次 joint-space planning
bool planToJointTarget(MoveGroupInterface & move_group,
                       const rclcpp::Logger & logger,
                       const std::vector<double> & target_joints,
                       const std::string & target_name)
{
  // 读取当前关节值，主要用于检查维度是否一致
  std::vector<double> current_joints = move_group.getCurrentJointValues();

  if (current_joints.size() != target_joints.size())
  {
    RCLCPP_ERROR(
      logger,
      "Target [%s] size mismatch: current joint size = %zu, target joint size = %zu",
      target_name.c_str(),
      current_joints.size(),
      target_joints.size());
    return false;
  }

  // 打印当前状态和目标状态
  printJoints(logger, "Current joints", current_joints);
  printJoints(logger, "Target  joints", target_joints);

  // 设置 joint-space 目标
  move_group.setJointValueTarget(target_joints);

  // 创建规划结果对象
  MoveGroupInterface::Plan plan;

  RCLCPP_INFO(logger, "Planning to target [%s] ...", target_name.c_str());

  // 发起规划
  bool success =
    (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success)
  {
    RCLCPP_INFO(logger, "Plan to target [%s] succeeded.", target_name.c_str());
  }
  else
  {
    RCLCPP_ERROR(logger, "Plan to target [%s] failed.", target_name.c_str());
  }

  return success;
}

int main(int argc, char * argv[])
{
  // 1. 初始化 ROS 2
  rclcpp::init(argc, argv);

  // 2. 创建节点
  auto node = std::make_shared<rclcpp::Node>(
    "joint_planning_multi_target",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // 3. 启动执行器，保证节点能正常处理 MoveIt 通信
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // 4. 指定 planning group，必须与 SRDF 完全一致
  static const std::string PLANNING_GROUP = "trunk_group";

  // 5. 创建 MoveGroupInterface
  MoveGroupInterface move_group(node, PLANNING_GROUP);

  // 6. 打印基本信息
  RCLCPP_INFO(node->get_logger(), "Planning frame: %s",
              move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(node->get_logger(), "End effector link: %s",
              move_group.getEndEffectorLink().c_str());

  // 7. 设置规划参数
  move_group.setMaxVelocityScalingFactor(0.1);
  move_group.setMaxAccelerationScalingFactor(0.1);
  move_group.setPlanningTime(5.0);

  // 8. 检查当前 group 的关节数是否符合预期
  std::vector<double> current_joints = move_group.getCurrentJointValues();
  if (current_joints.size() != 4)
  {
    RCLCPP_ERROR(node->get_logger(),
                 "Expected 4 joints in trunk_group, but got %zu.",
                 current_joints.size());
    rclcpp::shutdown();
    return 1;
  }

  // 9. 定义多组测试目标
  std::vector<double> target_1 = {0.2, -0.2, 0.2, -0.2};
  std::vector<double> target_2 = {0.5, -0.7, 0.7, -0.4};
  std::vector<double> target_3 = {1.0, -1.0, 0.8, -0.5};

  // 10. 依次测试多个 joint-space target
  planToJointTarget(move_group, node->get_logger(), target_1, "target_1");
  planToJointTarget(move_group, node->get_logger(), target_2, "target_2");
  planToJointTarget(move_group, node->get_logger(), target_3, "target_3");

  // 11. 关闭 ROS 2
  rclcpp::shutdown();
  return 0;
}