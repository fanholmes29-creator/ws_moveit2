# config 目录说明

本目录存放 `trunk_two_stage_planner` 的运行参数、离线分析参数和 RViz 显示配置。

如果你要调整目标位姿、两阶段策略、规划稳定性、可视化效果，通常都从这里开始。

---

## 文件总览

- `two_stage_system_params.yaml`  
  工程运行主配置（`two_stage_planner_system` 节点使用）。
- `tools/two_stage_planner_analysis_tool.yaml`  
  离线分析工具配置（`two_stage_planner_analysis_tool` 节点使用）。
- `two_stage_system.rviz`  
  RViz 布局与显示项配置（只影响显示，不改算法行为）。

---

## 1) `two_stage_system_params.yaml`（工程运行主配置）

### 作用

- 控制在线两阶段系统的核心行为：
  - 目标位姿来源
  - 两阶段算法参数
  - MoveIt 规划行为
  - 导出与话题

### 常改参数（按用途）

- **目标输入**
  - `use_goal_state_as_target_pose`
    - `true`：从 `goal_joint_target` FK 生成目标位姿
    - `false`：直接使用 `target_position` + `target_orientation`
  - `goal_joint_target`
  - `target_position`, `target_orientation`

- **阶段策略与可恢复性**
  - `stage2_pose_epsilon`：阶段切换可恢复性阈值（越小越严格）
  - `w1..w4`：stage1 代价权重（几何、恢复性、起点偏移、限位）
  - `stage2_pose_wp`, `stage2_pose_wR`：stage2 位姿误差组成权重

- **搜索密度与计算量**
  - `stage1_q1_samples`, `stage1_q2_samples`
  - `stage2_eval_q3_samples`, `stage2_eval_q4_samples`
  - `stage2_q3_samples`, `stage2_q4_samples`

- **MoveIt 执行行为**
  - `planning_time`, `planning_attempts`
  - `velocity_scaling`, `acceleration_scaling`
  - `stage2_q12_tolerance`：stage2 锁定 q1/q2 的容差

- **输出**
  - `output_dir`
  - `export_csv`
  - `marker_topic`, `display_trajectory_topic`

### 修改建议

- 先改一个维度，再看结果；不要同时大改多个参数。
- 先保证“可规划成功”，再追求“更严格阶段分离”。
- 采样数提高会显著增加耗时，先做小步调整。

---

## 2) `tools/two_stage_planner_analysis_tool.yaml`（离线分析配置）

### 作用

- 用于离线算法评估，不依赖在线 MoveIt 执行链路。
- 常用于：
  - 权重/阈值敏感性分析
  - 热力图对比
  - 诊断“为何某目标难以满足阶段切换约束”

### 常改参数

- 与主配置类似，但通常会更“严格”和“更密集”：
  - `stage1_q1_samples`, `stage1_q2_samples`（常设更大）
  - `stage2_eval_*`, `stage2_*`（常设更大）
  - `stage2_pose_epsilon`（常设更小）
  - `output_dir`（建议单独目录，便于对比）

### 修改建议

- 离线调参成功后，再把关键参数迁移到 `two_stage_system_params.yaml`。
- 若运行很慢，优先降低 `*_samples` 后再迭代。

---

## 3) `two_stage_system.rviz`（可视化配置）

### 作用

- 定义 RViz 的显示布局、视角、显示插件和话题订阅。
- 只影响“看起来是什么样”，不影响规划算法结果。

### 常改项

- `Fixed Frame`（需与系统 frame 一致）
- `Trajectory Topic`（通常是 `/display_planned_path`）
- `Marker Topic`（通常是 `/two_stage_debug_markers`）
- 默认视角参数（`Distance`、`Yaw`、`Pitch`、`Focal Point`）

### 修改建议

- 若“看不到轨迹/标记”，先检查这里的话题名和 frame 是否一致。

---

## 按问题快速定位应该改哪里

- **我想直接输入目标位姿 `T_d`**
  - 改 `two_stage_system_params.yaml`：
    - `use_goal_state_as_target_pose: false`
    - 填 `target_position` 和 `target_orientation`

- **阶段2经常失败 / 阶段切换太严格**
  - 改 `two_stage_system_params.yaml`：
    - 适当增大 `stage2_q12_tolerance`
    - 适当放宽 `stage2_pose_epsilon`
    - 增大 `planning_time` / `planning_attempts`

- **规划太慢**
  - 改 `two_stage_system_params.yaml` 或分析配置：
    - 降低 `stage1_*samples`、`stage2_*samples`

- **轨迹能跑但姿态/位置恢复不理想**
  - 先用 `tools/two_stage_planner_analysis_tool.yaml` 做离线调参：
    - 调 `stage2_pose_wp`、`stage2_pose_wR`
    - 看热力图和 summary 输出后再回写主配置

- **RViz 里看不到轨迹或标记**
  - 改 `two_stage_system.rviz`：
    - 校对 `Trajectory Topic`、`Marker Topic`、`Fixed Frame`
  - 同时确认主配置里的 `display_trajectory_topic`、`marker_topic`

---

## 维护要求

- 修改前建议备份一份当前 YAML。
- 提交配置变更时，建议在提交说明里写明：
  - 改了哪些参数
  - 预期影响是什么
  - 观察到的结果（成功率/耗时/误差）
- 若要做“高精度调参”，先在离线分析配置验证，再迁移到主配置。

