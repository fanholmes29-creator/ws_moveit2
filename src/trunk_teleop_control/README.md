# trunk_teleop_control

`trunk_teleop_control` 是 trunk 机器人的上位机遥操作 / operator command layer。

本 package 负责手柄、键盘以及未来 UI 输入相关的遥操作逻辑。它有意独立于 `trunk_two_stage_planner`，后者应继续专注于规划算法、规划 service 和 planner demo。

## 当前范围

第一阶段迁移实现的是关节空间手柄遥操作：

- 订阅 `/trunk_robot/joy` 的手柄输入
- 订阅 `/trunk_robot/joint_states` 的 trunk 关节状态
- 优先订阅 `/trunk_robot/trunk_group_controller/controller_state` 的 controller actual state 作为当前关节反馈
- 向 `/trunk_robot/trunk_group_controller/follow_joint_trajectory` 发送 `control_msgs/action/FollowJointTrajectory` goal
- 在 RViz 中显示 trunk 模型，用于 FakeSystem/controller 验证

笛卡尔空间遥操作和 mode manager 后续也会放在本 package 中实现。

## 启动

推荐使用 ros2_control-only 统一 bringup（FakeSystem + controller + joy + teleop + RViz）：

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

该入口用于手柄安全调试，只启动 ros2_control、手柄遥操作和 RViz，不会启动 `move_group`、`fixed_route_runner`、`two_stage_planner_system` 或自动轨迹发送节点。

兼容入口：

```bash
ros2 launch trunk_teleop_control trunk_teleop_fake_system_rviz.launch.py \
  namespace:=trunk_robot \
  start_joy_node:=true \
  start_rviz:=true \
  log_level:=info
```

兼容入口会 include `trunk_configure/demo.launch.py`，因此会启动 MoveIt `move_group`。仅在需要复用旧 demo 行为时使用。

---

终端 1：启动控制栈。

```bash
source ~/ws_moveit2/install/setup.bash
export ROS_DOMAIN_ID=77
export ROS_LOCALHOST_ONLY=1
ros2 launch trunk_configure demo.launch.py use_rviz:=false
```

控制栈必须在 `/trunk_robot` 命名空间下提供 trunk-only 接口，尤其是：

- `/trunk_robot/joint_states`
- `/trunk_robot/trunk_group_controller/controller_state`
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
ros2 topic info /trunk_robot/joint_states -v
ros2 topic echo /trunk_robot/joint_states --once
ros2 topic info /trunk_robot/trunk_group_controller/controller_state -v
ros2 topic echo /trunk_robot/trunk_group_controller/controller_state --once
ros2 topic info /trunk_robot/joy -v
ros2 node list
```

期望结果：

- `Action servers: 1`
- `/trunk_robot/joint_states` 至少有一个 publisher，并且消息中包含 `trunk_joint1..4`
- `/trunk_robot/trunk_group_controller/controller_state` 有一个 publisher，且 `actual.positions` 会随 action goal 更新
- `/trunk_robot/joy` 只有一个 publisher
- 不应出现 `/trunk_robot/move_group`

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
- 左摇杆上/下推一次，发送一次正向/反向单步目标
- 摇杆采用 rising-edge 触发，长时间推住不会连续发送 goal

## Package 边界

`trunk_teleop_control` 负责：

- joystick / keyboard / UI 输入
- mode manager
- 关节空间遥操作
- 笛卡尔空间遥操作
- deadman / timeout / safety 逻辑
- 输出 command 到 controller 或 planner

`trunk_two_stage_planner` 负责：

- `TwoStagePlanner` 算法
- `PlanToPose` service
- planner demo 和 system launch 文件
