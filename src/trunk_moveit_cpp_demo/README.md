# Legacy Package: trunk_moveit_cpp_demo

This package is legacy/reference only and is not part of the current engineering workflow.

该包是前期单包式 MoveIt2 示例，保留它仅用于：

- 回归对比
- 最小样例参考
- 查阅早期 `MoveGroupInterface` 封装方式

它**不再是当前工程主入口**。当前主线系统请使用：

- `trunk_two_stage_planner`
- 入口节点：`two_stage_planner_system`
- 入口 launch：`two_stage_planner_system.launch.py`

如果你只是想查看旧代码：

- `src/trunk_moveit_cpp_demo.cpp`
- `src/trunk_planner.cpp`
- `launch/trunk_moveit_cpp_demo.launch.py`

如果你要运行当前工程版两阶段系统，请不要从这个包启动。