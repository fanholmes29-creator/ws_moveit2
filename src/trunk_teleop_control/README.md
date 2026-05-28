# trunk_teleop_control
[配置手柄环境](https://www.yahboom.com/build.html?id=15676&cid=732)

`trunk_teleop_control` 是 trunk 机器人的关节空间遥操作 / operator command layer。

本 package 负责手柄、键盘以及未来 UI 输入相关的关节空间遥操作逻辑。它有意独立于 `trunk_two_stage_planner`，后者应继续专注于规划算法、规划 service、IK / PlanToPose 等规划相关接口。

## 当前范围

第一阶段迁移实现的是关节空间手柄遥操作：

- 订阅 `/trunk_robot/joy` 的手柄输入
- 订阅 `/trunk_robot/joint_states` 的 trunk 关节状态
- 优先订阅 `/trunk_robot/trunk_group_controller/controller_state` 的 controller actual state 作为当前关节反馈
- 默认向 `/trunk_robot/trunk_group_controller/joint_trajectory` 连续发布 `trajectory_msgs/msg/JointTrajectory`
- 兼容 step 模式下仍可向 `/trunk_robot/trunk_group_controller/follow_joint_trajectory` 发送 `control_msgs/action/FollowJointTrajectory` goal
- 在 RViz 中显示 trunk 模型，用于 FakeSystem/controller 验证

当前 `trunk_joystick_teleop` 的控制目标始终是 `trunk_joint1..4` 的关节位置，不生成笛卡尔 pose / twist 目标，也不调用 MoveIt 做笛卡尔规划。

## 启动

推荐入口：使用 ros2_control-only 统一 bringup（FakeSystem + controller + joy + teleop + RViz）：

```bash

pkill -f ros2 || true
ros2 daemon stop
ros2 daemon start

source ~/ws_moveit2/install/setup.bash
export ROS_DOMAIN_ID=77
export ROS_LOCALHOST_ONLY=1
ros2 launch trunk_teleop_control trunk_teleop_ros2_control_rviz.launch.py \
  namespace:=trunk_robot \
  start_joy_node:=true \
  start_rviz:=true \
  use_sim_time:=false \
  log_level:=info
```

`trunk_teleop_ros2_control_rviz.launch.py` 是新架构下推荐的手柄调试入口。它只启动 ros2_control、手柄遥操作和 RViz，不会启动 `move_group`、`fixed_route_runner`、`two_stage_planner_system` 或自动轨迹发送节点。
该入口默认同时启动 `control_mode_manager`，初始模式为 `manual_teleop`，因此只有手柄 teleop 可以向 controller 输出运动命令。

兼容 / 旧 demo 入口：

```bash
ros2 launch trunk_teleop_control trunk_teleop_fake_system_rviz.launch.py \
  namespace:=trunk_robot \
  start_joy_node:=true \
  start_rviz:=true \
  log_level:=info
```

`trunk_teleop_fake_system_rviz.launch.py` 会 include `trunk_configure/demo.launch.py`，因此可能带起 MoveIt demo 或 `move_group`。它仅用于复用旧 demo 行为，不作为新架构下的推荐入口。

---

终端 1：启动控制栈。

```bash
source ~/ws_moveit2/install/setup.bash
export ROS_DOMAIN_ID=77
export ROS_LOCALHOST_ONLY=1
ros2 launch trunk_two_stage_planner two_stage_planner_service.launch.py
```

控制栈必须在 `/trunk_robot` 命名空间下提供 trunk-only 接口，尤其是：

- `/trunk_robot/joint_states`
- `/trunk_robot/trunk_group_controller/controller_state`
- `/trunk_robot/trunk_group_controller/joint_trajectory`
- `/trunk_robot/trunk_group_controller/follow_joint_trajectory`

终端 2：启动手柄遥操作和 RViz。

```bash
source ~/ws_moveit2/install/setup.bash
export ROS_DOMAIN_ID=77
export ROS_LOCALHOST_ONLY=1
ros2 launch trunk_teleop_control trunk_joystick_teleop_rviz.launch.py start_joy_node:=true log_level:=info
```

如果已有其它 `joy_node` 在发布 `/trunk_robot/joy`，应避免重复发布：

```bash
ros2 launch trunk_teleop_control trunk_joystick_teleop_rviz.launch.py start_joy_node:=false log_level:=info
```

## 健康检查

```bash
ros2 action info /trunk_robot/trunk_group_controller/follow_joint_trajectory
ros2 topic info /trunk_robot/trunk_group_controller/joint_trajectory -v
ros2 topic info /trunk_robot/joint_states -v
ros2 topic echo /trunk_robot/joint_states --once
ros2 topic info /trunk_robot/trunk_group_controller/controller_state -v
ros2 topic echo /trunk_robot/trunk_group_controller/controller_state --once
ros2 topic info /trunk_robot/joy -v
ros2 node list
```

期望结果：

- `Action servers: 1`
- `/trunk_robot/trunk_group_controller/joint_trajectory` 有一个 subscriber，continuous 模式下推摇杆时会收到短 `JointTrajectory` command
- `/trunk_robot/joint_states` 至少有一个 publisher，并且消息中包含 `trunk_joint1..4`
- `/trunk_robot/trunk_group_controller/controller_state` 有一个 publisher，且 `actual.positions` 会随 topic command 或 action goal 更新
- `/trunk_robot/joy` 只有一个 publisher
- 不应出现 `/trunk_robot/move_group`
- `/trunk_robot/control_mode_state` 当前为 `manual_teleop`

在多机器人环境中，正常 trunk 遥操作不要使用全局 `/joint_states`。如果 `/joint_states` 中包含其它机器人的关节，例如 `AR5-*`、`Head_Joint*` 或大写 `Trunk_Joint*`，说明该 topic 已被其它栈污染。此时应使用隔离的 `ROS_DOMAIN_ID`、停止无关 publisher，或让 trunk 控制栈发布 `/trunk_robot/joint_states`。

## Yahboom 手柄映射

当前已验证映射：

- `button_a: 0`
- `button_b: 1`
- `button_x: 3`
- `button_y: 4`
- `button_l1: 6`
- `axis_step: 1`

操作方式：

- 按住 `L1` 作为 deadman
- `L1 + A/B/X/Y` 选择 `trunk_joint1..4`
- 默认 `control_mode: continuous`：左摇杆上/下持续推动时，当前选中关节会按摇杆幅值连续运动；松开摇杆或松开 `L1` 后停止发布运动命令
- `max_velocity_rad_s` 限制最大关节速度，`axis_deadzone` 过滤摇杆零位附近的小抖动，`command_state_max_error_rad` 用于防止内部积分目标相对当前反馈漂移过远
- 兼容 `control_mode: step`：左摇杆上/下跨过 `axis_step_threshold` 时发送一次正向/反向单步 action goal，行为与旧版 rising-edge 模式一致
- continuous 模式通过摇杆幅值积分当前选中关节的目标位置，并发布 `trajectory_msgs/msg/JointTrajectory`
- step 模式通过 `control_msgs/action/FollowJointTrajectory` 做单步关节运动
- 两种模式都属于关节空间控制，不涉及笛卡尔目标设定

## Control Source Boundary / 控制源边界

手柄模式链路：

```text
joy -> trunk_joystick_teleop -> trunk_group_controller/joint_trajectory -> controller
```

自动规划模式链路：

```text
上位机 / PlanToPose -> trunk_two_stage_planner -> FollowJointTrajectory action -> controller
```

`trunk_joystick_teleop` 和 `trunk_two_stage_planner` 是两个不同控制源，不能同时向 `trunk_group_controller` 输出运动命令。真实机器人上必须由 `control_mode_manager` 或上位机做控制源仲裁。

当前模式通过 `/trunk_robot/control_mode_state` 发布，切换服务为 `/trunk_robot/set_control_mode`。支持模式：

- `idle`：两侧都不应输出运动命令
- `manual_teleop`：只允许 `trunk_joystick_teleop` 输出 `JointTrajectory` topic
- `auto_plan_execute`：只允许 `trunk_two_stage_planner` 执行 `FollowJointTrajectory` action
- `estop`：急停/禁用态，两侧都不输出运动命令

切换示例：

```bash
ros2 service call /trunk_robot/set_control_mode trunk_teleop_control/srv/SetControlMode "{mode: manual_teleop}"
ros2 service call /trunk_robot/set_control_mode trunk_teleop_control/srv/SetControlMode "{mode: auto_plan_execute}"
ros2 service call /trunk_robot/set_control_mode trunk_teleop_control/srv/SetControlMode "{mode: idle}"
```

从 `manual_teleop` 切到 `auto_plan_execute` 时，mode manager 会先发布 `idle` 并等待短时间，让手柄 continuous 输出停止，再发布 `auto_plan_execute`。从 `auto_plan_execute` 切到 `manual_teleop` / `idle` / `estop` 时，mode manager 会向 `trunk_group_controller/follow_joint_trajectory` 请求取消当前 action goal，然后再切换模式。

`trunk_joystick_teleop` 只有在当前模式为 `manual_teleop` 时才允许发布 continuous command 或 step action。非手柄模式下它仍可读取 joy 和关节选择状态，但会清除内部连续控制状态，不会输出运动命令。

## Package Boundary / 包边界

`trunk_teleop_control` 负责：

- joystick / keyboard / UI 输入
- 关节空间遥操作
- deadman / timeout / safety 逻辑
- joy 超时、关节状态检查、软限位 clamp
- 向 ros2_control controller 输出 `JointTrajectory` topic 或 `FollowJointTrajectory` action 指令

`trunk_teleop_control` 不负责：

- IK
- 直接生成笛卡尔 pose / twist 目标
- 调用 MoveIt 进行笛卡尔规划
- 发送 `PlanToPose` 或两阶段规划请求

笛卡尔目标、路径规划、两阶段规划、IK 请求应由上位机或 `trunk_two_stage_planner` 相关接口负责。本包只消费输入设备事件，并输出 controller 可执行的关节空间命令。

`trunk_two_stage_planner` / 上位机负责：

- `TwoStagePlanner` 算法
- `PlanToPose` service
- 笛卡尔目标和直观目标设定
- IK / 路径规划 / 两阶段规划请求
- planner demo 和 system launch 文件

`package.xml` 中的 `moveit_configs_utils`、`trunk_configure` 等依赖主要服务于兼容 launch / bringup，不代表 `trunk_joystick_teleop` 节点会执行笛卡尔规划。
