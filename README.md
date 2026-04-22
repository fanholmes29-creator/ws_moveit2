# ws_moveit2

## Mainline Navigation

- 当前主线包：`trunk_two_stage_planner`
- 当前主节点：`two_stage_planner_system`
- 当前主 launch：`ros2 launch trunk_two_stage_planner two_stage_planner_system.launch.py`
- Service 模式 launch：`ros2 launch trunk_two_stage_planner two_stage_planner_service.launch.py`
- Service 接口：`/two_stage_planner/plan_to_pose`
- Service 类型：`trunk_two_stage_planner/srv/PlanToPose`
- 上层模拟客户端：`ros2 run trunk_two_stage_planner example_plan_to_pose_client --use-default`
- 上层模拟客户端（外部位姿）：`ros2 run trunk_two_stage_planner example_plan_to_pose_client --position 0.196101 0.0 0.602433 --orientation -0.014919 -0.098712 0.148692 0.983831`
- 分析工具入口：`ros2 run trunk_two_stage_planner two_stage_planner_analysis_tool --ros-args --params-file ~/ws_moveit2/src/trunk_two_stage_planner/config/tools/two_stage_planner_analysis_tool.yaml`
- legacy / reference 包：
  - `trunk_moveit_cpp_demo`
  - `trunk_moveit`

## Workspace Overview

该工作空间当前的工程主线是：

- `src/trunk_two_stage_planner`

它实现了一个基于 MoveIt2 / RViz 的两阶段 trunk 规划系统，入口为：

- 节点：`two_stage_planner_system`
- launch：`two_stage_planner_system.launch.py`
- service 节点：`two_stage_planner_service`
- service launch：`two_stage_planner_service.launch.py`
- 示例上层客户端：`example_plan_to_pose_client`

如果你要从上层系统直接下发目标位姿，建议走 service 模式：

- 启动：`ros2 launch trunk_two_stage_planner two_stage_planner_service.launch.py`
- 调用：`/two_stage_planner/plan_to_pose`
- 默认回退：若上层不提供外部位姿，则回退到 `src/trunk_two_stage_planner/config/two_stage_system_params.yaml` 中的默认目标逻辑

当前工作空间中的其他 trunk 相关包已按用途分层：

- `trunk_two_stage_planner`：工程主线
- `trunk_configure`：MoveIt2 配置与 launch
- `robot_model`：URDF / 模型资源
- `trunk_moveit_cpp_demo`：legacy 单包 demo，仅保留作参考
- `trunk_moveit`：legacy 早期实验包，仅保留作参考

完整工程说明、编译方式、启动方式和 RViz 使用说明，请查看：

- `src/trunk_two_stage_planner/README.md`
- `src/trunk_two_stage_planner/config/README.md`
