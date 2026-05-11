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
  兼容保留的轻量 RViz 布局，只加载 `RobotModel` 和 debug marker；不再是默认启动配置。
- `two_stage_system_moveit.rviz`  
  默认 MoveIt MotionPlanning RViz 布局，用于查看 MoveIt 面板和轨迹动画。
- `two_stage_system_stable.rviz`  
  备用轻量 RViz 布局，不加载 MoveIt MotionPlanning 插件。

---

## 1) `two_stage_system_params.yaml`（工程运行主配置）

### 作用

- 控制在线两阶段系统的核心行为：
  - 目标位姿来源
  - 两阶段算法参数
  - MoveIt 规划行为
  - 导出与话题

### Namespace 约定

当前系统默认由 launch 放入私有 namespace：

- 默认 namespace：`/trunk_robot`
- joint state：`/trunk_robot/joint_states`
- service：`/trunk_robot/two_stage_planner/plan_to_pose`
- controller action：`/trunk_robot/trunk_group_controller/follow_joint_trajectory`

因此本配置文件里的 topic/action 推荐写**相对名**，不要写全局绝对名：

- 推荐：`joint_states_topic: "joint_states"`
- 推荐：`follow_joint_trajectory_action: "trunk_group_controller/follow_joint_trajectory"`
- 推荐：`display_trajectory_topic: "display_planned_path"`
- 推荐：`marker_topic: "two_stage_debug_markers"`

launch 会把这些相对名解析到 `/trunk_robot/...`。这样可以避免订阅全局 `/joint_states`，防止其他机器人或机构的 joint state 污染 trunk 的 MoveIt 和规划器。

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
  - `use_live_joint_state_as_start`：是否优先使用 namespace 内实时 `joint_states` 作为规划起点
  - `allow_start_state_fallback_to_config`：若实时状态暂时不可用，是否回退到配置里的 `q_start`（仅在不执行控制器时安全）
  - `live_start_state_wait_sec`：旧参数名，保留兼容
  - `joint_state_wait_timeout_sec`：等待实时起点到来的最长时间；执行控制器时必须在该时间内收到有效状态
  - `joint_states_topic`：当前关节状态订阅话题
  - `expected_joint_names`：只接受这些 trunk 关节作为实时起点
  - `strict_joint_states`：是否拒绝包含未知关节的 joint state
  - `warn_unknown_joints`：非严格模式下，遇到未知关节是否报警
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
- 若启用 `execute_joint_trajectory: true`，系统必须先收到有效实时关节状态，不会回退到配置 `q_start` 后继续执行。
- 默认 `joint_state_wait_timeout_sec: 120.0`，用于覆盖 controller manager / joint state broadcaster 启动较慢的情况。
- 若启动初期 `joint_states` 仍可能有延迟，请继续调大 `joint_state_wait_timeout_sec`；`allow_start_state_fallback_to_config` 只适合 planning-only 模式兜底。
- 若你只是想先把轨迹给同事而不立刻执行到底层，可关闭 `execute_joint_trajectory`，只保留标准 `JointTrajectory` 输出。
- 若你要直接和 `ros2_control` 控制器联调，则应确认 `follow_joint_trajectory_action` 与控制器配置一致。
- 多机器人网络里不要把 `joint_states_topic` 改成 `/joint_states`；除非你明确知道全局 topic 只有 trunk 一个发布者。

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

## 3) RViz 可视化配置

### 作用

- 定义 RViz 的显示布局、视角、显示插件和话题订阅。
- 只影响“看起来是什么样”，不影响规划算法结果。
- 默认使用 `two_stage_system_moveit.rviz`，提供 MoveIt 面板和动画。
- 如果本机 RViz/MoveIt 插件不稳定，可通过 launch 参数切换到 `two_stage_system_stable.rviz`。
- 默认启动时序：
  - RViz 延时 `30 s` 启动
  - planner / service 延时 `60 s` 启动
  - live joint state 最长等待 `120 s`
  这样可以让 controller、move_group、MotionPlanning 面板先稳定，再由 planner 发布轨迹。

### 常改项

- `Fixed Frame`（需与系统 frame 一致）
- `RobotModel` 的 `Description Topic`（通常是 `robot_description`，随 namespace 解析）
- `Marker Topic`（通常是 `two_stage_debug_markers`，随 namespace 解析）
- 默认视角参数（`Distance`、`Yaw`、`Pitch`、`Focal Point`）

### 修改建议

- 默认配置加载 `moveit_rviz_plugin/MotionPlanning`，可显示 MoveIt 面板和轨迹动画。
- 备用 `two_stage_system_stable.rviz` 不加载 `moveit_rviz_plugin/MotionPlanning`，用于规避 Humble 上偶发的 InteractiveMarker 插件冲突。
- launch 默认会先延后启动 RViz，再延后启动 planner，使 MotionPlanning 面板在 `DisplayTrajectory` 发布前完成订阅。
- planner 节点存活期间会周期性重发最近一次 `DisplayTrajectory`，方便 RViz 后启动或重连后恢复动画显示。
- RViz 节点会显式放入 `/trunk_robot` namespace，MotionPlanning 应订阅 `/trunk_robot/monitored_planning_scene`。
- 若“看不到模型/标记/动画”，先检查这里的话题名、frame、namespace 是否一致。
- RViz 也在 `/trunk_robot` namespace 下启动，因此这里的 `Description Topic` / `Marker Topic` / `Trajectory Topic` 推荐保持相对名即可。

---

## 按问题快速定位应该改哪里

- **我要让上层系统通过 service 直接给目标位姿**
  - 启动：
    - `ros2 launch trunk_two_stage_planner two_stage_planner_service.launch.py`
  - 调用接口：
    - `/trunk_robot/two_stage_planner/plan_to_pose`
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
    - `follow_joint_trajectory_action: "trunk_group_controller/follow_joint_trajectory"`
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
    - `joint_states_topic: "joint_states"`
  - 如果你希望状态没到就直接失败：
    - `allow_start_state_fallback_to_config: false`
  - 如果你希望先等一会再决定是否回退：
    - 调 `joint_state_wait_timeout_sec`

- **我想确认 namespace 隔离是否生效**
  - 启动后检查：
    - `ros2 topic info /trunk_robot/joint_states -v`
    - `ros2 topic info /joint_states -v`
  - 期望结果：
    - `/trunk_robot/joint_states` 有 trunk 的 publisher/subscriber
    - `/joint_states` 不存在，或至少没有 trunk 系统节点订阅
  - 如果看到 `/two_stage_planner_system` 订阅全局 `/joint_states`，说明启动的是旧进程或旧 install，需要重新编译并 `source install/setup.bash`

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

- **日志停在 `Planning stage1 with MoveIt group 'stage1_group'...`**
  - 这通常不是 joint state 问题，因为此时系统已经拿到了 `q_start`。
  - 重点看后续是否出现：
    - `Connecting stage1 MoveGroupInterface to namespace '/trunk_robot'...`
    - `Stage1 MoveGroupInterface connected.`
    - `Stage1 computing Cartesian path ...`
    - `Stage1 Cartesian fraction=...`
  - 如果卡在连接 MoveGroupInterface，优先检查 planner 和 move_group 是否在同一个 namespace：
    - `ros2 node list | grep trunk_robot`
  - 如果卡在 Cartesian / OMPL 规划，优先调：
    - `planning_time`
    - `planning_attempts`
    - `stage1_use_cartesian`
    - `stage1_min_fraction`
    - `stage1_q4_tolerance`

- **轨迹能跑但姿态/位置恢复不理想**
  - 先用 `tools/two_stage_planner_analysis_tool.yaml` 做离线调参：
    - 调 `stage2_pose_wp`、`stage2_pose_wR`
    - 看热力图和 summary 输出后再回写主配置

- **RViz 里看不到模型、标记或轨迹动画**
  - 默认动画配置是 `two_stage_system_moveit.rviz`：
    - 校对 `Trajectory Topic`、`Planning Scene Topic`、`Robot Description`、`Fixed Frame`
  - 备用轻量配置是 `two_stage_system_stable.rviz`：
    - 校对 `Description Topic`、`Marker Topic`、`Fixed Frame`
  - 同时确认主配置里的 `display_trajectory_topic`、`marker_topic`
  - 规划成功后还可以检查：
    - `ros2 topic info /trunk_robot/robot_description -v`
    - `ros2 topic info /trunk_robot/monitored_planning_scene -v`
    - `ros2 topic info /trunk_robot/display_planned_path -v`
    - `ros2 topic info /trunk_robot/two_stage_debug_markers -v`
    - `ros2 topic info /trunk_robot/two_stage_joint_trajectory -v`
  - 如果 MotionPlanning 面板没有动画，确认 planner 日志中出现：
    - `Published DisplayTrajectory ...`
    - 且 RViz 已在 `/trunk_robot` namespace 下启动。

- **启动日志太多**
  - `load_yaml()`、KDL root inertia、Octomap 等多为 MoveIt/RViz/ros2_control 启动噪声。
  - 调试规划主链路时，优先看 `[trunk_robot.two_stage_planner_system]` 日志。
  - 真正关键的 planner 日志包括：
    - `Using live joint state ...`
    - `Solving two-stage algorithm target...`
    - `Algorithm solved...`
    - `Planning stage1...`
    - `Stage1 planning succeeded.`
    - `Planning stage2...`
    - `Stage2 planning succeeded.`

---

## 维护要求

- 修改前建议备份一份当前 YAML。
- service 模式下，`two_stage_system_params.yaml` 依然是默认目标与系统参数的来源，不会因为新增 service 而失效。
- 若启用了 `use_live_joint_state_as_start: true`，则本次规划实际使用的 `q_start` 可能与 YAML 里的默认值不同。
- 若启用了 `execute_joint_trajectory: true`，实时起点是安全要求；没有有效 `joint_states` 时系统会拒绝执行，而不是回退到配置 `q_start`。
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

