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
  - `use_live_joint_state_as_start`：是否优先使用实时 `/joint_states` 作为规划起点
  - `allow_start_state_fallback_to_config`：若实时状态暂时不可用，是否回退到配置里的 `q_start`
  - `live_start_state_wait_sec`：等待实时起点到来的最长时间
  - `joint_states_topic`：当前关节状态订阅话题
  - `joint_trajectory_topic`：完整轨迹输出 topic
  - `execute_joint_trajectory`：规划成功后是否继续向控制器发送轨迹
  - `follow_joint_trajectory_action`：控制器 action 名称
  - `execute_action_server_wait_sec`：等待 action server 可用的最长时间
  - `execute_result_wait_sec`：等待控制器执行结果的最长时间

- **输出**
  - `output_dir`
  - `export_csv`
  - `marker_topic`, `display_trajectory_topic`

### 修改建议

- 先改一个维度，再看结果；不要同时大改多个参数。
- 先保证“可规划成功”，再追求“更严格阶段分离”。
- 采样数提高会显著增加耗时，先做小步调整。
- 在线运行时，推荐开启 `use_live_joint_state_as_start: true`，让系统从当前真实关节状态起步。
- 若启动初期 `/joint_states` 可能有延迟，可保留 `allow_start_state_fallback_to_config: true` 作为兜底。
- 若你只是想先把轨迹给同事而不立刻执行到底层，可关闭 `execute_joint_trajectory`，只保留标准 `JointTrajectory` 输出。
- 若你要直接和 `ros2_control` 控制器联调，则应确认 `follow_joint_trajectory_action` 与控制器配置一致。

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

- **我要让上层系统通过 service 直接给目标位姿**
  - 启动：
    - `ros2 launch trunk_two_stage_planner two_stage_planner_service.launch.py`
  - 调用接口：
    - `/two_stage_planner/plan_to_pose`
    - 类型：`trunk_two_stage_planner/srv/PlanToPose`
  - 若上层传入：
    - `use_external_target: true`
    - `target_position: [x, y, z]`
    - `target_orientation: [qx, qy, qz, qw]`
    则系统优先使用外部目标位姿。
  - 若上层不传（或 `use_external_target: false`）：
    - 系统回退到本地默认目标逻辑
    - 默认目标仍由 `two_stage_system_params.yaml` 决定

- **我要验证 service 模式是否工作正常**
  - 可使用示例客户端：
    - `ros2 run trunk_two_stage_planner example_plan_to_pose_client --use-default`
    - 或：
      `ros2 run trunk_two_stage_planner example_plan_to_pose_client --position 0.196101 0.0 0.602433 --orientation -0.014919 -0.098712 0.148692 0.983831`
  - 这个脚本模拟“上层系统”发起一次 service 请求。

- **我想直接输入目标位姿 `T_d`**
  - 改 `two_stage_system_params.yaml`：
    - `use_goal_state_as_target_pose: false`
    - 填 `target_position` 和 `target_orientation`

- **我想把完整轨迹发给同事或中间层**
  - 改 `two_stage_system_params.yaml`：
    - `joint_trajectory_topic`
  - 系统会在规划成功后发布：
    - `trajectory_msgs/msg/JointTrajectory`

- **我想规划后直接执行到底层 ros2_control**
  - 改 `two_stage_system_params.yaml`：
    - `execute_joint_trajectory: true`
    - `follow_joint_trajectory_action: "/trunk_group_controller/follow_joint_trajectory"`
    - `execute_action_server_wait_sec`
    - `execute_result_wait_sec`
  - 这会把完整轨迹作为 `FollowJointTrajectory` goal 发给控制器。

- **我只想输出轨迹，不想让系统自动执行**
  - 改 `two_stage_system_params.yaml`：
    - `execute_joint_trajectory: false`
  - 这时仍会发布完整 `JointTrajectory`，但不会发送 action goal。

- **我希望系统从机器人当前状态开始规划，而不是从固定 `q_start` 开始**
  - 改 `two_stage_system_params.yaml`：
    - `use_live_joint_state_as_start: true`
    - `joint_states_topic: "/joint_states"`
  - 如果你希望状态没到就直接失败：
    - `allow_start_state_fallback_to_config: false`
  - 如果你希望先等一会再决定是否回退：
    - 调 `live_start_state_wait_sec`

- **我想保留固定起点模式（例如离线复现或对比实验）**
  - 改 `two_stage_system_params.yaml`：
    - `use_live_joint_state_as_start: false`
  - 此时系统直接使用配置里的 `q_start`

- **service 没收到外部位姿时，系统到底会用什么目标**
  - 改 `two_stage_system_params.yaml`：
    - 若 `use_goal_state_as_target_pose: true`
      - 使用 `goal_joint_target` 经过 FK 得到目标位姿
    - 若 `use_goal_state_as_target_pose: false`
      - 使用 `target_position` + `target_orientation`

- **我要给 service 模式准备默认兜底目标**
  - 改 `two_stage_system_params.yaml`：
    - `goal_joint_target`
    - 或 `target_position` / `target_orientation`
  - service 模式和普通系统模式共用这份默认参数文件。

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
- service 模式下，`two_stage_system_params.yaml` 依然是默认目标与系统参数的来源，不会因为新增 service 而失效。
- 若启用了 `use_live_joint_state_as_start: true`，则本次规划实际使用的 `q_start` 可能与 YAML 里的默认值不同。
- 若需要严格复现实验结果，建议关闭实时起点模式，固定使用配置里的 `q_start`。
- 若启用了 `execute_joint_trajectory: true`，则系统在规划成功后不仅会发布轨迹，还会主动向控制器发送 `FollowJointTrajectory` action goal。
- 若当前只是联调接口，建议先保留 `joint_trajectory_topic` 输出，并根据情况暂时关闭自动执行。
- 如果上层要稳定调用，请约定：
  - `target_position` 固定 3 维
  - `target_orientation` 固定 4 维四元数 `[qx, qy, qz, qw]`
  - 所有数值为有限实数
- 提交配置变更时，建议在提交说明里写明：
  - 改了哪些参数
  - 预期影响是什么
  - 观察到的结果（成功率/耗时/误差）
- 若要做“高精度调参”，先在离线分析配置验证，再迁移到主配置。

