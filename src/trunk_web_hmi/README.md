# trunk_web_hmi

`trunk_web_hmi` 是 trunk 机器人的 Web 上位机。当前已进入第二版“自动规划增强版”，该包独立放在 `src/trunk_web_hmi`，不修改现有 `trunk_teleop_control`、`trunk_two_stage_planner`、`trunk_hardware_interface` 的核心逻辑。

## 功能范围

- 首页 Dashboard：显示 ROS 连接状态、namespace、当前控制模式、关节角、接口在线状态和最近错误。
- 模式控制：支持 `idle`、`manual_teleop`、`auto_plan_execute`、`estop` 四种模式切换。
- Web UI 不再提供手动点动入口；底层 `manual_teleop` 模式保留给 joystick/其他控制源使用。
- 自动规划增强版：分为关节操作和位置姿态操作；关节操作支持 4 个关节的滑条和数字输入同步，位置姿态操作支持默认目标、四元数位姿、RPY 位姿和点位目标，执行前显示预检查和转换结果。
- 点位管理：支持保存当前关节角点位、四元数位姿点位、RPY 位姿点位、删除点位、按类型回填规划表单。
- 诊断日志：显示最近操作日志、错误日志、topic/service/action 在线状态和最近更新时间。
- 操作台布局：参考工业上位机界面组织顶部状态条、左侧信息面板、中央场景/规划区、右侧分组操作按钮和底部信息栏；未接入 ROS 接口的设备/舱门/液压类按钮保持禁用并标注待接入。
- 轨迹回放：最近一次规划轨迹会缓存，可在 RViz 中展示/暂停展示，执行后仍可重新展示最近轨迹。

## 架构

- 后端：Python FastAPI + `rclpy`
- 前端：React + TypeScript + Vite
- 通信：REST 负责命令和点位管理，WebSocket 负责实时状态
- 默认机器人 namespace：`/trunk_robot`
- 点位存储：本地 JSON 文件 `~/.ros/trunk_web_hmi/waypoints.json`

## 后端接口

- `GET /api/status`
- `POST /api/mode`
- `POST /api/plan_to_pose`
- `POST /api/planning/target_preview`
- `POST /api/planning/preview_target`
- `POST /api/planning/execute_previewed_trajectory`
- `POST /api/planning/trajectory_display`
- `POST /api/planning/plan_to_target`
- `POST /api/manual/jog`（兼容保留，Web UI 不暴露）
- `POST /api/manual/stop`（兼容保留，Web UI 不暴露）
- `GET /api/waypoints`
- `POST /api/waypoints`
- `DELETE /api/waypoints/{id}`
- `GET /api/logs`
- `WebSocket /ws/state`

## 安全约束

后端在接受运动相关请求前会做统一检查：

- `estop` 模式下禁止运动。
- ROS 未连接时禁止运动。
- 手动点动接口仍只允许在 `manual_teleop` 模式下执行，但 Web UI 当前不暴露手动点动入口。
- 自动规划执行只允许在 `auto_plan_execute` 模式下执行。
- planner service 和 trajectory action 不在线时禁止对应动作。
- 位姿输入会拒绝 `NaN`、`Inf` 和范数过小的四元数。
- 自动规划增强 API 会在后端重新执行单位转换、RPY 转四元数、模式检查和 service/action 在线检查，前端禁用按钮不是唯一安全边界。

当前 Web 上位机的运动方式是：设置关节目标或位姿目标 -> 规划预览 -> 确认执行。手动点动后端接口保留兼容，但不在 Web UI 中暴露。

## 自动规划增强版

自动规划页支持以下目标类型：

- 默认目标：`use_external_target=false`，由 planner 使用自身默认目标。
- 笛卡尔位姿 + 四元数：输入 `x y z qx qy qz qw`。
- 笛卡尔位姿 + 欧拉角：输入 `x y z roll pitch yaw`，采用 ROS 常用 RPY 顺序 `roll, pitch, yaw`，后端会转换为 quaternion。
- 关节空间目标：输入 `trunk_joint1..4`，后端通过 `/trunk_robot/two_stage_planner/fk_joint_to_pose` 先做 FK 转换，再把转换后的 target pose 送入两步式规划流程。
- 点位目标：从已保存点位选择并按类型回填到对应目标表单。

单位选择：

- 位置单位：`m` / `mm`，后端统一转换为米。
- 姿态角单位：`rad` / `deg`，后端统一转换为弧度后再做 RPY -> quaternion。
- 关节角单位：`rad` / `deg`，后端统一转换为弧度。

执行前预检查会显示：

- 数值是否合法。
- 当前是否为 `auto_plan_execute`。
- planner service 是否在线。
- `joint_states` 是否新鲜。
- controller/action 是否在线。
- quaternion 是否有效。
- 关节目标是否包含 `trunk_joint1..4`。
- 关节目标是否在内置 trunk 软限位范围内。
- 关节目标的 FK service 是否在线，以及 FK 转换后的 target pose。

`POST /api/planning/target_preview` 只做解析、转换和预检查，不调用 MoveIt 规划。Web 页面实际采用两步式真机流程：

1. 点击“先规划并在 RViz 显示”：调用 `POST /api/planning/preview_target`，后端转发到 `/trunk_robot/two_stage_planner/preview_plan_to_pose`。该 service 只规划、缓存合并后的关节轨迹，并向 RViz 发布一次 `DisplayTrajectory`，不会立即执行真机动作。
2. 点击“执行已规划轨迹”：调用 `POST /api/planning/execute_previewed_trajectory`，后端转发到 `/trunk_robot/two_stage_planner/execute_previewed_trajectory`。执行成功后 planner 会暂停轨迹展示并清除调试 marker，但最近一次规划轨迹仍会保留在后端缓存中。
3. 点击“展示最近规划轨迹”或“暂停轨迹展示”：调用 `POST /api/planning/trajectory_display`，可以在执行前或执行后重新展示/暂停最近一次规划轨迹。

`POST /api/planning/plan_to_target` 和第一版 `/api/plan_to_pose` 仍保留用于兼容旧流程；真机操作建议使用上面的两步式流程。

## 通过 HTTP JSON 控制机器人

除 Web 前端外，也可以直接通过 HTTP 协议向后端发送 JSON 指令。后端默认监听 `http://127.0.0.1:8000`；如果从其他电脑访问，需要把 `127.0.0.1` 替换为运行后端的机器人电脑 IP。

运动执行前需要先切换到自动规划执行模式：

```bash
curl -X POST http://127.0.0.1:8000/api/mode \
  -H "Content-Type: application/json" \
  -d '{"mode":"auto_plan_execute"}'
```

推荐真机调试时使用“两步式”流程：先规划预览，再确认执行。

关节空间目标预览示例：

```bash
curl -X POST http://127.0.0.1:8000/api/planning/preview_target \
  -H "Content-Type: application/json" \
  -d '{
    "target_type": "joint",
    "joint_positions": {
      "trunk_joint1": 0.0,
      "trunk_joint2": 0.0,
      "trunk_joint3": 0.0,
      "trunk_joint4": 0.0
    },
    "units": {
      "position": "m",
      "angle": "rad",
      "joint": "rad"
    }
  }'
```

如果返回 `"success": true`，再执行已规划轨迹：

```bash
curl -X POST http://127.0.0.1:8000/api/planning/execute_previewed_trajectory
```

也可以使用一步式接口直接规划并执行：

```bash
curl -X POST http://127.0.0.1:8000/api/planning/plan_to_target \
  -H "Content-Type: application/json" \
  -d '{
    "target_type": "joint",
    "joint_positions": {
      "trunk_joint1": 0.1,
      "trunk_joint2": 0.2,
      "trunk_joint3": -0.1,
      "trunk_joint4": 0.0
    },
    "units": {
      "position": "m",
      "angle": "rad",
      "joint": "rad"
    }
  }'
```

关节空间目标的含义是发送四个 trunk 关节角，后端会通过 `/trunk_robot/two_stage_planner/fk_joint_to_pose` 将关节目标转换为末端目标位姿，再送入规划流程。关节名必须包含 `trunk_joint1`、`trunk_joint2`、`trunk_joint3`、`trunk_joint4`，单位可使用 `rad` 或 `deg`。

也可以直接发送笛卡尔末端位姿目标：

```bash
curl -X POST http://127.0.0.1:8000/api/planning/preview_target \
  -H "Content-Type: application/json" \
  -d '{
    "target_type": "pose_quaternion",
    "use_external_target": true,
    "position": [0.20, 0.00, 0.30],
    "orientation_quaternion": [0, 0, 0, 1],
    "units": {
      "position": "m",
      "angle": "rad",
      "joint": "rad"
    }
  }'
```

其中 `position` 为 `[x, y, z]`，位置单位由 `units.position` 指定；`orientation_quaternion` 为 `[qx, qy, qz, qw]`。如果接口返回 `IkFailed` 或目标不可达，说明 HTTP 格式已经正确，但该末端位姿在当前机器人状态下无法求解 IK，需要调整位置或姿态。

可通过以下接口查看当前状态、控制模式、关节角、planner service 和 trajectory action 是否在线：

```bash
curl http://127.0.0.1:8000/api/status
```

一次成功的规划响应通常包含：

```json
{
  "success": true,
  "error_code": 0,
  "message": "Planning succeeded."
}
```

若返回 `"success": false`，优先查看 `message` 字段。常见原因包括未处于 `auto_plan_execute` 模式、ROS 未连接、planner service 离线、trajectory action 离线、关节目标超限或末端位姿 IK 不可达。

Planner 侧新增参数 `republish_display_trajectory`，默认 `false`。因此规划轨迹不会每 5 秒反复发布，避免 RViz 动画一直重复。若需要旧的 RViz 重连后自动重播行为，可以显式设为 `true`。

如果 RViz 中轨迹仍然循环播放，优先检查 RViz 配置中的 `MotionPlanning -> Planned Path -> Loop Animation`。当前推荐设为 `false`，否则 RViz 插件会在本地循环播放上一条 `DisplayTrajectory`。

## 启动方式

编译 ROS 工作区：

```bash
cd ~/ws_moveit2
python3 -m pip install -r src/trunk_web_hmi/requirements.txt
colcon build --packages-select trunk_teleop_control trunk_two_stage_planner trunk_web_hmi
source install/setup.bash
```

另开终端启动机器人 bringup / planner，例如：

```bash
source ~/ws_moveit2/install/setup.bash
ros2 launch trunk_two_stage_planner two_stage_planner_service.launch.py
```

启动 Web 上位机后端：

```bash
source ~/ws_moveit2/install/setup.bash
ros2 launch trunk_web_hmi web_hmi_backend.launch.py robot_namespace:=trunk_robot
```

启动 Web 上位机前端：

```bash
cd ~/ws_moveit2/src/trunk_web_hmi/frontend
rm -rf node_modules package-lock.json
npm install
npm run dev
```

浏览器打开 `http://localhost:5173`。

## 前端依赖说明

当前前端依赖固定在 Vite 2 / React 18，兼容 Ubuntu 默认较旧的 Node 12 环境。若后续升级到 Node 20 LTS，可以再升级到新版 Vite。

`npm` 脚本通过 `node ./node_modules/vite/bin/vite.js` 显式启动 Vite，用于规避部分本地 Node/npm 环境中 `node_modules/.bin/vite` shim 出现 `Exec format error` 的问题。

## 当前限制和 TODO

- Web UI 不提供手动点动；如需点动请使用底层 joystick/teleop 工具。
- 自动规划仍采用同步 HTTP 请求等待 planner service 返回；异步 job / 取消规划可作为后续增强。
- 两步式规划缓存最近一次规划轨迹；重新规划会覆盖上一次缓存，执行后仍可重新展示这条最近轨迹。
- 前端样式保持简洁，优先满足 MVP 调试和验收。
- 暂未实现登录认证和多用户权限控制。
- 点位当前使用本地 JSON 存储，后续可替换为 SQLite。
- planner service 调用在 HTTP 请求内同步等待，规划耗时较长时请求会保持打开。
