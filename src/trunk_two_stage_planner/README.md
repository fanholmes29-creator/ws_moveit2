# trunk_two_stage_planner

基于 MoveIt2 / RViz 的 trunk 两阶段规划工程系统。

## 项目概述

本工程面向一个 4 自由度 trunk 机构的两阶段规划问题。目标不是单次把 `trunk_group` 从起点直接规划到终点，而是把整体过程拆成两个阶段：

- 第一阶段：笛卡尔预备态规划
- 第二阶段：最终位姿恢复

这样拆分的原因是：

- 第一阶段主要负责中间过程的空间运动形态控制，用于降低本体碰撞风险并保持胸部稳定
- 第二阶段主要负责从预备态恢复到最终目标位姿
- 第一阶段的终点不能只看几何投影关系，还必须满足第二阶段的恢复性要求

当前工程使用两个 MoveIt planning groups：

- `stage1_group`
- `stage2_group`

它们在 SRDF 层面共享同一条 trunk 运动链，但在工程层面承载不同的阶段逻辑与约束。

## 当前系统架构

当前工程按功能分为四层：

- MoveIt 配置层：`trunk_configure`
- 算法层：`trunk_two_stage_planner` 中的几何、IK、`q_pre` 搜索与恢复性验证
- 调度层：`TwoStagePlannerManager`
- 可视化与调试层：`DisplayTrajectory`、RViz marker、CSV/summary 导出

核心文件职责：

- `include/trunk_two_stage_planner/two_stage_planner_manager.hpp`
  - 工程主入口类声明
- `src/two_stage_planner_manager.cpp`
  - 两阶段调度、MoveIt 规划调用、轨迹显示、marker 发布、阶段衔接
- `include/trunk_two_stage_planner/robot_kinematics_helper.hpp`
  - RobotModel / RobotState / FK / IK / joint axis 查询
- `src/robot_kinematics_helper.cpp`
  - 模型加载、KDL IK 接入、世界坐标系关节轴计算
- `include/trunk_two_stage_planner/two_stage_planner.hpp`
  - 两阶段算法层接口
- `src/two_stage_planner.cpp`
  - `q_goal` 求解、`q_pre` 搜索、第二阶段恢复性评估
- `include/trunk_two_stage_planner/trajectory_utils.hpp`
  - 轨迹导出、summary 和 heatmap 导出接口
- `src/trajectory_utils.cpp`
  - CSV / summary / geometry_points 导出实现
- `src/fk_pose_from_joint_main.cpp`
  - 关节空间目标转目标位姿（FK）的小工具入口，直接输出可粘贴的 `target_position/target_orientation`

## 两阶段算法流程

### 方法层定义

当前主线方法定义为：

**带第二阶段恢复性约束的两阶段笛卡尔预备态规划**

流程如下：

1. 输入最终目标位姿 `T_d`
2. 先求最终 IK 解 `q_goal`
3. 构造第一阶段受限候选族  
   `q_pre(q1, q2) = [q1, q2, q1 + q2, q4_fix]`
4. 对每个候选 `q_pre`，先检查：
   - 在固定 `q1, q2` 下，仅允许 `q3, q4` 变化时，第二阶段是否能恢复最终目标位姿
5. 仅在满足恢复性阈值的候选子集内，再综合比较：
   - 投影几何引导量
   - 离起点偏移
   - 关节限位
   - 安全裕度（当前尚未完全工程化）
6. 选出最终 `q_pre`
7. 第一阶段规划到 `q_pre`
8. 第二阶段从第一阶段终点继续，恢复到最终目标位姿

### 投影点的角色

投影点现在只是第一阶段预备态终点的**几何引导量之一**，不再是唯一目标。当前算法已经验证出：

- 投影最优的候选不一定等于第二阶段恢复最优的候选
- 如果只看投影，不足以保证最终阶段可恢复

### 第一阶段为何不是直接最终位姿规划

第一阶段的核心不是“直接朝最终位姿逼近”，而是保证中间过程的空间形态合理，例如：

- 胸部主轴稳定
- 本体中间过程不过分摆动
- 后续阶段仍然可恢复

因此第一阶段必须被理解为“笛卡尔预备态规划”，而不是单纯的终点关节目标规划。

## 当前工程实现现状

这里必须严格区分“方法定义”和“当前实现层级”。

### 已实现

- 目标位姿 `T_d` 到最终 IK 解 `q_goal` 的求解
- 第一阶段候选预备态 `q_pre` 搜索
- 第二阶段完整位姿恢复性验证
- `stage1_group / stage2_group` 两阶段 MoveIt 规划调度
- RViz 轨迹显示
- RViz marker 显示关键几何点与阶段路径
- 两阶段轨迹导出
- summary / heatmap / geometry_points 导出

### 当前仍是工程近似的地方

- `stage1_group` / `stage2_group` 不是物理上拆开的链，只是工程入口分组
- `q3 = q1 + q2` 没有在 MoveIt 中实现为原生硬约束，而是由算法层用于候选构造
- 第一阶段还不是完整的一阶段连续笛卡尔过程约束优化器
- 第一阶段当前采用的是工程近似：
  - 先由算法层求 `q_pre`
  - MoveIt 规划到 `q_pre`
  - 优先尝试 upright path constraint
  - 若失败则回退为无 path constraint 的 `stage1_group` 规划
- 第二阶段当前采用的是工程近似：
  - 算法层先给出 `q_goal_stage2`
  - MoveIt 再规划到该 joint target
  - 对 `q1 / q2` 加 joint constraints 近似固定前两轴

### 当前不能夸大的地方

当前版本**不能**被描述为：

- 已经完成完整的一阶段连续笛卡尔过程约束规划
- 已经原生支持 `q3 = q1 + q2` 硬约束
- 已经原生实现“仅由 q3、q4 完成恢复”的 MoveIt 模型层约束

它目前是一个：

**可运行的工程版第一版系统**

## 目录结构说明

当前主线包：

- `trunk_two_stage_planner/`
  - `include/trunk_two_stage_planner/`
    - `robot_kinematics_helper.hpp`
    - `two_stage_planner.hpp`
    - `two_stage_planner_manager.hpp`
    - `trajectory_utils.hpp`
    - `types.hpp`
  - `src/`
    - `robot_kinematics_helper.cpp`
    - `two_stage_planner.cpp`
    - `two_stage_planner_manager.cpp`
    - `trajectory_utils.cpp`
    - `system_main.cpp`
    - `analysis_tool_main.cpp`  
      说明：离线分析工具入口，对应 `two_stage_planner_analysis_tool`
    - `fk_pose_from_joint_main.cpp`
      说明：FK 打印工具入口，对应 `fk_pose_from_joint`
  - `config/`
    - `two_stage_system_params.yaml`
    - `two_stage_system.rviz`
    - `tools/`
      - `two_stage_planner_analysis_tool.yaml`
  - `launch/`
    - `two_stage_planner_system.launch.py`
  - `README.md`

支撑包：

- `trunk_configure/`
  - MoveIt2 配置、SRDF、kinematics、OMPL、launch
- `robot_model/`
  - URDF、mesh、模型资源

保留但降级为 legacy 的包：

- `trunk_moveit_cpp_demo/`
- `trunk_moveit/`

## 如何编译

建议直接编译主线包及其配置包：

```bash
cd ~/ws_moveit2
source /opt/ros/humble/setup.bash
colcon build --packages-select trunk_configure trunk_two_stage_planner --symlink-install --allow-overriding trunk_configure
source install/setup.bash
```

如果你只修改了主线包源码，也可以只编译：

```bash
colcon build --packages-select trunk_two_stage_planner --symlink-install
source install/setup.bash
```

## 如何启动

### 正常带 RViz 的工程启动

```bash
cd ~/ws_moveit2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch trunk_two_stage_planner two_stage_planner_system.launch.py
```

### 无界面调试

```bash
mkdir -p ~/ws_moveit2/log/ros
export ROS_LOG_DIR=~/ws_moveit2/log/ros
ros2 launch trunk_two_stage_planner two_stage_planner_system.launch.py system_use_rviz:=false manager_delay_sec:=2.0
```

### 离线分析工具（保留但降级）

```bash
ros2 run trunk_two_stage_planner two_stage_planner_analysis_tool \
  --ros-args \
  --params-file ~/ws_moveit2/src/trunk_two_stage_planner/config/tools/two_stage_planner_analysis_tool.yaml
```

### FK 打印工具（输入关节目标，输出可粘贴位姿）

用途：

- 输入关节空间目标（`goal_joint_target`）
- 直接打印两行可粘贴到 `two_stage_system_params.yaml` 的：
  - `target_position: [...]`
  - `target_orientation: [...]`

示例命令：

```
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run trunk_two_stage_planner fk_pose_from_joint --ros-args -p goal_joint_target:="[-1.5, 1.5, 0.7, 0.6]"
```

示例输出（格式示意）：

```text
target_position: [0.172874, 0.000000, 0.610959]
target_orientation: [0.029503, 0.095375, 0.294044, 0.950564]
source_goal_joint_target: [-1.000000, 1.500000, 0.700000, 0.600000]
```

注意：

- 参数数组必须同类型，建议统一写成浮点（如 `-1.0` 而不是 `-1`）。
- 若你只想用 FK 结果进行规划，可以把输出复制到：
  - `target_position`
  - `target_orientation`
  并设置 `use_goal_state_as_target_pose: false`。

## RViz 中看什么

启动工程系统后，重点查看两类显示：

### 1. MotionPlanning

这里会显示 manager 发布的 `display_planned_path`。

你应该观察：

- stage1 是否先规划到预备态
- stage2 是否以前一阶段终点为起点继续规划
- 两段轨迹是否连续，而不是互相割裂

### 2. TwoStageDebug

这里会显示关键 marker：

- 目标点 `p_d`
- 目标投影点
- `O4(q_pre)`
- `Proj_{L3(q_pre)}(O4(q_pre))`
- stage1 路径线
- stage2 路径线

通过这些可以快速判断：

- 第一阶段终点几何是否合理
- 两阶段切换是否清楚
- 第二阶段是否是在 stage1 终点的基础上恢复

## 关键输出与调试文件

工程系统和分析工具仍会导出调试文件：

- `trajectory.csv`
  - 两阶段轨迹点、关节位置/速度/加速度
- `summary.txt`
  - `q_start / q_goal / q_pre / 误差 / 恢复性统计`
- `stage1_heatmap.csv`
  - 第一阶段候选搜索热图数据
- `geometry_points.csv`
  - 关键几何点导出

这些文件默认输出到：

- 系统版：`/home/wxl/ws_moveit2/csv/two_stage_system`
- 分析工具版：由 analysis tool 参数决定

## 当前限制与下一步增强方向

当前第一版工程系统已做到：

- MoveIt2 / RViz 接入
- 双 planning groups
- 两阶段 manager 调度
- 轨迹显示
- marker 调试

但仍有明显增强空间：

1. 将 stage1 从“规划到 q_pre”进一步升级为真正的笛卡尔参考路径驱动
2. 增强 stage1 中间过程的显式空间约束与避障表达
3. 在 MoveIt 层面更严格表达 stage2 对 `q1 / q2` 的锁定或弱松弛
4. 将 `q3 = q1 + q2` 从算法层近似推进到更强的工程约束表达
5. 对 stage1 / stage2 误差收敛做更系统的参数整定

## Legacy 包说明

以下包仍保留，但不再是工程主入口：

- `trunk_moveit_cpp_demo`
  - 早期单包 MoveIt2 demo
- `trunk_moveit`
  - 早期实验性 MoveIt 入口

它们保留的原因：

- 供回归对比
- 查阅早期实现
- 作为最小参考样例

如果你要运行当前工程系统，请始终从：

- `trunk_two_stage_planner`
- `two_stage_planner_system.launch.py`

进入。
