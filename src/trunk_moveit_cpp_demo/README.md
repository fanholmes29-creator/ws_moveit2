# trunk_moveit_cpp_demo

一个基于 ROS2 Humble + MoveIt2 的腰部关节空间规划示例包，用于控制 `trunk_group` 完成 `current -> home -> goal` 的规划与执行，并导出 CSV 轨迹。

## 只保留一套推荐启动方式

本包统一推荐只使用下面这一条 launch：

- `ros2 launch trunk_moveit_cpp_demo trunk_moveit_cpp_demo.launch.py`

该 launch 会自动：

1. include `trunk_configure/demo.launch.py`（启动 move_group / rviz / ros2_control 等）
2. 加载 `trunk_moveit_cpp_demo/config/trunk_demo_params.yaml`
3. 启动 `trunk_moveit_cpp_demo` 节点执行规划与轨迹导出

## 统一运行命令

```bash
cd ~/ws_moveit2
source /opt/ros/humble/setup.bash
colcon build --packages-select trunk_moveit_cpp_demo --symlink-install
source install/setup.bash
ros2 launch trunk_moveit_cpp_demo trunk_moveit_cpp_demo.launch.py
```

## 统一 CSV 默认路径

默认输出到同一目录：

- `home_csv_path`: `/home/wxl/ws_moveit2/csv/home_plan.csv`
- `goal_csv_path`: `/home/wxl/ws_moveit2/csv/goal_plan.csv`

CSV 目录会在导出时自动创建，无需手动 `mkdir`。

## 关键文件职责

- `src/trunk_moveit_cpp_demo.cpp`: 主流程入口，读取参数并执行 home/goal 规划流程
- `src/trunk_planner.cpp`: MoveGroupInterface 封装，含规划/执行/导出 CSV
- `config/trunk_demo_params.yaml`: 推荐启动方式下的统一参数源
- `launch/trunk_moveit_cpp_demo.launch.py`: 唯一推荐入口，串联 MoveIt 环境与 demo 节点