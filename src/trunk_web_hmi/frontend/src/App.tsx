import { useEffect, useMemo, useState } from "react";
import type { ReactNode } from "react";
import {
  createWaypoint,
  deleteWaypoint,
  getLogs,
  getOperationLogs,
  getRobotConfig,
  getStatus,
  getWaypoints,
  resetRobotError,
  setMode,
  setRobotEnable
} from "./api";
import { PlanningPanel } from "./PlanningPanel";
import type {
  ControlMode,
  LogItem,
  OperationLog,
  RobotConfig,
  Status,
  TargetType,
  Waypoint
} from "./types";

const jointNames = ["trunk_joint1", "trunk_joint2", "trunk_joint3", "trunk_joint4"];
const modes: ControlMode[] = ["idle", "auto_plan_execute", "estop"];
const modeLabels: Record<string, string> = {
  idle: "空闲 idle",
  manual_teleop: "手动 manual_teleop",
  auto_plan_execute: "自动规划 auto_plan_execute",
  estop: "急停 estop",
  unknown: "未知 unknown"
};

function modeText(mode: string) {
  return modeLabels[mode] ?? mode;
}

function waypointKindText(kind: Waypoint["kind"]) {
  if (kind === "joint") return "关节点位";
  if (kind === "pose_rpy") return "RPY 位姿点位";
  return "四元数位姿点位";
}

const initialStatus: Status = {
  ros_connected: false,
  namespace: "/trunk_robot",
  control_mode: "unknown",
  control_mode_last_update_sec: null,
  joint_positions: {},
  joint_states_last_update_sec: null,
  controller_state_last_update_sec: null,
  services: {
    set_control_mode: false,
    planner: false,
    planner_preview: false,
    planner_execute: false,
    fk_joint_to_pose: false,
    trajectory_display: false
  },
  actions: { follow_joint_trajectory: false },
  topics: { control_mode_state: false, joint_states: false, controller_state: false },
  last_error: null
};

function ageText(value: number | null) {
  return value === null ? "从未收到" : `${value.toFixed(1)} 秒前`;
}

function formatRad(value?: number | null) {
  return value === null || value === undefined ? "--" : value.toFixed(4);
}

function enableText(value?: boolean | null) {
  if (value === true) return "已使能";
  if (value === false) return "未使能";
  return "未知/未接入";
}

function App() {
  const [status, setStatus] = useState<Status>(initialStatus);
  const [logs, setLogs] = useState<{ operations: LogItem[]; errors: LogItem[] }>({
    operations: [],
    errors: []
  });
  const [operationLogs, setOperationLogs] = useState<OperationLog[]>([]);
  const [waypoints, setWaypoints] = useState<Waypoint[]>([]);
  const [message, setMessage] = useState("");
  const [waypointName, setWaypointName] = useState("");
  const [waypointNote, setWaypointNote] = useState("");
  const [requestedTargetType, setRequestedTargetType] = useState<TargetType>("pose_quaternion");
  const [robotConfig, setRobotConfig] = useState<RobotConfig | null>(null);

  const commandPhase = status.command_state?.phase ?? "idle";
  const commandBusy = ["manual_jogging", "planning", "executing"].includes(commandPhase);
  const dangerousModeSwitchDisabled = commandPhase === "executing" || commandPhase === "manual_jogging";

  async function refreshSideData() {
    const [logPayload, operationLogPayload, waypointPayload] = await Promise.all([
      getLogs(),
      getOperationLogs(80),
      getWaypoints()
    ]);
    setLogs(logPayload as { operations: LogItem[]; errors: LogItem[] });
    setOperationLogs(operationLogPayload.items);
    setWaypoints(waypointPayload.items);
  }

  useEffect(() => {
    getStatus().then(setStatus).catch((error) => setMessage(error.message));
    getRobotConfig().then(setRobotConfig).catch(() => setRobotConfig(null));
    refreshSideData().catch((error) => setMessage(error.message));
    const protocol = window.location.protocol === "https:" ? "wss" : "ws";
    const socket = new WebSocket(`${protocol}://${window.location.host}/ws/state`);
    socket.onmessage = (event) => setStatus(JSON.parse(event.data) as Status);
    socket.onerror = () => setMessage("WebSocket 状态流未连接。");
    const timer = window.setInterval(() => {
      refreshSideData().catch((error) => setMessage(error.message));
    }, 2000);
    return () => {
      window.clearInterval(timer);
      socket.close();
    };
  }, []);

  async function onSetMode(mode: ControlMode) {
    try {
      const result = await setMode(mode);
      setMessage(result.message || `已发送模式切换请求：${mode}`);
      await refreshSideData();
    } catch (error) {
      setMessage((error as Error).message);
    }
  }

  async function onRobotEnable(enable: boolean) {
    try {
      const result = await setRobotEnable(enable);
      setMessage(result.message);
      await refreshSideData();
    } catch (error) {
      setMessage((error as Error).message);
    }
  }

  async function onResetError() {
    try {
      const result = await resetRobotError();
      setMessage(result.message);
      await refreshSideData();
    } catch (error) {
      setMessage((error as Error).message);
    }
  }

  async function saveJointWaypoint() {
    if (!waypointName.trim()) {
      setMessage("请输入点位名称。");
      return;
    }
    await createWaypoint({
      name: waypointName,
      kind: "joint",
      note: waypointNote,
      joint_positions: status.joint_positions
    });
    setWaypointName("");
    setWaypointNote("");
    await refreshSideData();
  }

  function fillPoseWaypoint(waypoint: Waypoint) {
    if (!waypoint.kind.startsWith("pose") || !waypoint.target_pose) {
      return;
    }
    const pose = waypoint.target_pose as {
      use_external_target?: boolean;
      position?: { x?: number; y?: number; z?: number };
      orientation?: { x?: number; y?: number; z?: number; w?: number };
    };
    setRequestedTargetType("pose_quaternion");
    setMessage(`已载入点位：${waypoint.name}`);
  }

  return (
    <main className="operator-main">
      <header className="operator-header">
        <div className="brand-block">
          <strong className="brand-mark">TRUNK HMI</strong>
          <div>
            <h1>Trunk Web 上位机</h1>
            <p>命名空间：{status.namespace}</p>
          </div>
        </div>
        <div className={`pill ${status.ros_connected ? "ok" : "bad"}`}>
          ROS {status.ros_connected ? "已连接" : "未连接"}
        </div>
      </header>

      <section className="top-status-strip">
        <StatusChip label="ROS Bridge" value={status.connection_state?.ros_bridge_connected ? "已连接" : "未连接"} good={Boolean(status.connection_state?.ros_bridge_connected)} />
        <StatusChip label="Controller 状态" value={status.connection_state?.controller_state_online ? "在线" : "离线"} good={Boolean(status.connection_state?.controller_state_online)} />
        <StatusChip label="机器人使能" value={enableText(status.enable_state?.enabled)} good={status.enable_state?.enabled === true} />
        <StatusChip label="控制模式" value={status.mode_state?.control_mode ?? status.control_mode} good={(status.mode_state?.control_mode ?? status.control_mode) !== "unknown"} />
        <StatusChip label="急停状态" value={status.mode_state?.is_estop ? "急停" : "正常"} good={!status.mode_state?.is_estop} />
        <StatusChip label="规划/FK/轨迹" value={status.services.planner_preview && status.services.fk_joint_to_pose ? "就绪" : "未就绪"} good={Boolean(status.services.planner_preview && status.services.fk_joint_to_pose)} />
        <StatusChip label="命令状态" value={commandPhase} good={!commandBusy} />
      </section>

      {message && <section className="banner info-bar">{message}</section>}

      <section className="operator-layout">
        <aside className="left-console">
          <Card title="信息状态栏">
            <div className="joint-list">
              {jointNames.map((name) => (
                <span key={name}>
                  {name}: {(status.joint_positions[name] ?? 0).toFixed(4)}
                </span>
              ))}
            </div>
            {status.joints_runtime && (
              <div className="joint-runtime-table">
                <div className="joint-runtime-row header">
                  <span>关节</span>
                  <span>目标</span>
                  <span>反馈</span>
                  <span>误差</span>
                  <span>限位</span>
                </div>
                {status.joints_runtime.map((joint) => (
                  <div
                    className={`joint-runtime-row ${joint.within_limit === false ? "bad-row" : ""}`}
                    key={joint.name}
                  >
                    <span>{joint.name}</span>
                    <span>{formatRad(joint.target_rad)}</span>
                    <span>{formatRad(joint.position_rad)}</span>
                    <span className={Math.abs(joint.error_rad ?? 0) > 0.05 ? "warn" : ""}>
                      {formatRad(joint.error_rad)}
                    </span>
                    <span>{joint.within_limit === false ? "超限" : "正常"}</span>
                  </div>
                ))}
              </div>
            )}
            <StatusRow label="joint_states" value={ageText(status.joint_states_last_update_sec)} good={status.topics.joint_states} />
            <StatusRow label="controller_state" value={ageText(status.controller_state_last_update_sec)} good={status.topics.controller_state} />
            <StatusRow label="control_mode_state" value={status.topics.control_mode_state ? "已收到" : "未收到"} good={status.topics.control_mode_state} />
            <StatusRow label="机器人使能" value={status.enable_state?.message ?? "未接入真实硬件使能"} good={status.enable_state?.enabled === true} />
            <StatusRow label="命令状态" value={status.command_state?.message ?? commandPhase} good={!commandBusy} />
            <p>最近错误：{status.last_error?.message ?? "无"}</p>
          </Card>

          <Card title="模式控制">
            <div className="button-grid">
              {modes.map((mode) => (
                <button
                  key={mode}
                  className={mode === "estop" ? "danger" : ""}
                  disabled={!status.services.set_control_mode || (mode !== "estop" && dangerousModeSwitchDisabled)}
                  onClick={() => onSetMode(mode)}
                >
                  {modeText(mode)}
                </button>
              ))}
            </div>
          </Card>
        </aside>

        <section className="center-console">
          <div className="scene-placeholder">
            <div>
              <h2>三维场景 / RViz 视图占位</h2>
              <p>当前 Web 页面负责控制与状态聚合；轨迹和机器人姿态仍在 RViz 中显示。</p>
              <p className="todo">后续可接入 rosbridge / Foxglove / WebGL 机器人模型，把 RViz 视图嵌入此区域。</p>
            </div>
          </div>
          <PlanningPanel
            status={status}
            waypoints={waypoints}
            setMessage={setMessage}
            refreshSideData={refreshSideData}
          requestedTargetType={requestedTargetType}
          robotConfig={robotConfig ?? undefined}
          currentJointPositions={status.joint_positions}
          />
        </section>

        <aside className="right-console">
          <ControlGroup title="连接 / 使能">
            <button disabled>ROS Bridge 随后端启动自动连接</button>
            <button onClick={() => onRobotEnable(true)}>机器人使能（未接入）</button>
            <button onClick={() => onRobotEnable(false)}>机器人下使能（未接入）</button>
            <button onClick={onResetError}>错误复位（未接入）</button>
          </ControlGroup>

          <ControlGroup title="安全 / 模式">
            <button disabled={!status.services.set_control_mode || dangerousModeSwitchDisabled} onClick={() => onSetMode("idle")}>切换 idle</button>
            <button className="danger" disabled={!status.services.set_control_mode} onClick={() => onSetMode("estop")}>急停 estop</button>
          </ControlGroup>

          <ControlGroup title="轨迹回放">
            <p className="todo">最近轨迹展示、暂停和执行在中央规划面板操作。</p>
            <p className="todo">执行流程：先规划预览，再确认执行。</p>
          </ControlGroup>

          <ControlGroup title="机械臂操作">
            <button onClick={() => setRequestedTargetType("joint")}>关节操作</button>
            <button onClick={() => setRequestedTargetType("pose_quaternion")}>位置姿态操作</button>
            <button disabled={!status.services.set_control_mode || dangerousModeSwitchDisabled} onClick={() => onSetMode("auto_plan_execute")}>进入自动规划模式</button>
            <button disabled>运动流程：设置目标 → 规划预览 → 确认执行</button>
          </ControlGroup>
        </aside>
      </section>

      <section className="bottom-console">
        <Card title="点位管理">
          <div className="row">
            <label>
              名称
              <input value={waypointName} onChange={(event) => setWaypointName(event.target.value)} />
            </label>
            <label>
              备注
              <input value={waypointNote} onChange={(event) => setWaypointNote(event.target.value)} />
            </label>
            <button onClick={saveJointWaypoint}>保存当前关节角</button>
          </div>
          <div className="waypoints compact-waypoints">
            {waypoints.map((waypoint) => (
              <div className="waypoint" key={waypoint.id}>
                <strong>{waypoint.name}</strong>
                <span>{waypointKindText(waypoint.kind)}</span>
                <small>{waypoint.note}</small>
                <div className="row">
                  <button disabled={!waypoint.kind.startsWith("pose")} onClick={() => fillPoseWaypoint(waypoint)}>
                    回填
                  </button>
                  <button
                    className="danger"
                    onClick={async () => {
                      await deleteWaypoint(waypoint.id);
                      await refreshSideData();
                    }}
                  >
                    删除
                  </button>
                </div>
              </div>
            ))}
          </div>
        </Card>

        <Card title="信息栏">
          <h3>结构化操作事件</h3>
          <OperationLogList items={operationLogs.slice(-8).reverse()} />
          <h3>操作日志</h3>
          <LogList items={logs.operations.slice(-6).reverse()} />
          <h3>错误日志</h3>
          <LogList items={logs.errors.slice(-4).reverse()} />
        </Card>
      </section>
    </main>
  );
}

function Card({ title, children }: { title: string; children: ReactNode }) {
  return (
    <section className="card">
      <h2>{title}</h2>
      {children}
    </section>
  );
}

function ControlGroup({ title, children }: { title: string; children: ReactNode }) {
  return (
    <section className="control-group">
      <h3>{title}</h3>
      <div className="control-buttons">{children}</div>
    </section>
  );
}

function StatusChip({ label, value, good }: { label: string; value: string; good: boolean }) {
  return (
    <div className="status-chip">
      <span>{label}</span>
      <strong className={good ? "ok-text" : "bad-text"}>{value}</strong>
    </div>
  );
}

function StatusRow({ label, value, good }: { label: string; value: string; good: boolean }) {
  return (
    <div className="status-row">
      <span>{label}</span>
      <strong className={good ? "ok-text" : "bad-text"}>{value}</strong>
    </div>
  );
}

function LogList({ items }: { items: LogItem[] }) {
  if (!items.length) {
    return <p>暂无</p>;
  }
  return (
    <ul className="logs">
      {items.map((item) => (
        <li key={`${item.time}-${item.message}`}>
          <small>{new Date(item.time).toLocaleTimeString()}</small>{" "}
          <strong>{item.level}</strong>{" "}
          {item.event && <span>[{item.event}] </span>}
          {item.message}
        </li>
      ))}
    </ul>
  );
}

function OperationLogList({ items }: { items: OperationLog[] }) {
  if (!items.length) {
    return <p>暂无</p>;
  }
  return (
    <ul className="logs">
      {items.map((item) => (
        <li key={`${item.timestamp}-${item.event}-${item.message}`}>
          <small>{new Date(item.timestamp * 1000).toLocaleTimeString()}</small>{" "}
          <strong>{item.level}</strong>{" "}
          <span>[{item.event}]</span>{" "}
          <span>{item.source}</span>{" "}
          {item.message}
        </li>
      ))}
    </ul>
  );
}

export default App;
