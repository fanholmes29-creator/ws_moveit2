import { useEffect, useMemo, useState } from "react";
import type { ReactNode } from "react";
import {
  createWaypoint,
  executePreviewedTrajectory,
  previewPlanningExecution,
  previewPlanningTarget,
  setTrajectoryDisplay
} from "./api";
import type {
  AngleUnit,
  JointUnit,
  PlanResult,
  PlanningPreview,
  PlanningTargetRequest,
  PositionUnit,
  RobotConfig,
  Status,
  TargetType,
  Waypoint
} from "./types";

const jointNames = ["trunk_joint1", "trunk_joint2", "trunk_joint3", "trunk_joint4"];

type Props = {
  status: Status;
  waypoints: Waypoint[];
  setMessage: (message: string) => void;
  refreshSideData: () => Promise<void>;
  requestedTargetType?: TargetType;
  robotConfig?: RobotConfig;
  currentJointPositions?: Record<string, number>;
};

type FormState = {
  targetType: TargetType;
  positionUnit: PositionUnit;
  angleUnit: AngleUnit;
  jointUnit: JointUnit;
  waypointId: string;
  x: string;
  y: string;
  z: string;
  qx: string;
  qy: string;
  qz: string;
  qw: string;
  roll: string;
  pitch: string;
  yaw: string;
  joints: Record<string, string>;
  waypointName: string;
  waypointNote: string;
};

const initialForm: FormState = {
  targetType: "pose_quaternion",
  positionUnit: "m",
  angleUnit: "rad",
  jointUnit: "rad",
  waypointId: "",
  x: "0.196101",
  y: "0",
  z: "0.602433",
  qx: "-0.014919",
  qy: "-0.098712",
  qz: "0.148692",
  qw: "0.983831",
  roll: "0",
  pitch: "0",
  yaw: "0",
  joints: {
    trunk_joint1: "0",
    trunk_joint2: "0",
    trunk_joint3: "0",
    trunk_joint4: "0"
  },
  waypointName: "",
  waypointNote: ""
};

const jointLimitsRad: Record<string, { min: number; max: number }> = {
  trunk_joint1: { min: -1.39, max: 1.39 },
  trunk_joint2: { min: -1.74, max: 1.74 },
  trunk_joint3: { min: -1.74, max: 1.74 },
  trunk_joint4: { min: -3.14, max: 3.14 }
};

const MAX_TARGET_DELTA_RAD = 0.7;

function finite(value: string) {
  return Number.isFinite(Number(value));
}

function rpyToQuaternion(roll: number, pitch: number, yaw: number) {
  const cy = Math.cos(yaw * 0.5);
  const sy = Math.sin(yaw * 0.5);
  const cp = Math.cos(pitch * 0.5);
  const sp = Math.sin(pitch * 0.5);
  const cr = Math.cos(roll * 0.5);
  const sr = Math.sin(roll * 0.5);
  return [
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
    cr * cp * cy + sr * sp * sy
  ];
}

function angleToRad(value: number, unit: AngleUnit | JointUnit) {
  return unit === "deg" ? (value * Math.PI) / 180.0 : value;
}

function positionToM(value: number, unit: PositionUnit) {
  return unit === "mm" ? value / 1000.0 : value;
}

function localValidation(form: FormState) {
  if (form.targetType === "default" || form.targetType === "waypoint") {
    return "";
  }
  const positionValues = [form.x, form.y, form.z];
  if (!positionValues.every(finite)) {
    return "位置输入必须是有限数字。";
  }
  if (form.targetType === "pose_quaternion") {
    const values = [form.qx, form.qy, form.qz, form.qw];
    if (!values.every(finite)) {
      return "四元数输入必须是有限数字。";
    }
    const norm = Math.sqrt(values.map(Number).reduce((sum, value) => sum + value * value, 0));
    if (norm < 1e-6) {
      return "四元数范数过小。";
    }
  }
  if (form.targetType === "pose_rpy") {
    const values = [form.roll, form.pitch, form.yaw];
    if (!values.every(finite)) {
      return "RPY 输入必须是有限数字。";
    }
  }
  if (form.targetType === "joint") {
    const values = jointNames.map((name) => form.joints[name]);
    if (!values.every(finite)) {
      return "关节目标必须是有限数字。";
    }
  }
  return "";
}

export function PlanningPanel({
  status,
  waypoints,
  setMessage,
  refreshSideData,
  requestedTargetType,
  robotConfig,
  currentJointPositions
}: Props) {
  const [form, setForm] = useState<FormState>(initialForm);
  const [preview, setPreview] = useState<PlanningPreview | null>(null);
  const [planResult, setPlanResult] = useState<PlanResult | null>(null);
  const [submitting, setSubmitting] = useState(false);
  const [executing, setExecuting] = useState(false);
  const [trajectoryReady, setTrajectoryReady] = useState(false);
  const [trajectoryVisible, setTrajectoryVisible] = useState(false);
  const [requestStartedAt, setRequestStartedAt] = useState<string>("");
  const [durationMs, setDurationMs] = useState<number | null>(null);
  const [lastTargetType, setLastTargetType] = useState<TargetType | "">("");
  const backendCommandPhase = status.command_state?.phase ?? "idle";
  const backendCommandBusy = ["manual_jogging", "planning", "executing"].includes(backendCommandPhase);
  const effectiveTrajectoryReady = trajectoryReady || Boolean(status.command_state?.trajectory_ready);
  const effectiveTrajectoryVisible = trajectoryVisible || Boolean(status.command_state?.trajectory_visible);
  const configuredJointNames = useMemo(
    () => robotConfig?.joints.map((joint) => joint.name) ?? jointNames,
    [robotConfig]
  );
  const configuredJointLimits = useMemo(
    () =>
      Object.fromEntries(
        (robotConfig?.joints ?? []).map((joint) => [
          joint.name,
          { min: joint.min_rad, max: joint.max_rad }
        ])
      ),
    [robotConfig]
  );

  const validation = useMemo(() => localValidation(form), [form]);
  const convertedQuaternion = useMemo(() => {
    if (form.targetType !== "pose_rpy" || validation) {
      return null;
    }
    const r = angleToRad(Number(form.roll), form.angleUnit);
    const p = angleToRad(Number(form.pitch), form.angleUnit);
    const y = angleToRad(Number(form.yaw), form.angleUnit);
    return rpyToQuaternion(r, p, y);
  }, [form, validation]);
  const jointJumpWarnings = useMemo(() => {
    if (form.targetType !== "joint") {
      return [];
    }
    return configuredJointNames.flatMap((name) => {
      const feedback = currentJointPositions?.[name];
      const rawTarget = Number(form.joints[name] ?? 0);
      if (feedback === undefined || !Number.isFinite(feedback) || !Number.isFinite(rawTarget)) {
        return [`${name} 缺少有效反馈角或目标角。`];
      }
      const targetRad = form.jointUnit === "deg" ? angleToRad(rawTarget, "deg") : rawTarget;
      const delta = Math.abs(targetRad - feedback);
      if (delta > MAX_TARGET_DELTA_RAD) {
        return [
          `${name} 目标跳变 ${delta.toFixed(4)} rad，超过 ${MAX_TARGET_DELTA_RAD.toFixed(4)} rad。`
        ];
      }
      return [];
    });
  }, [configuredJointNames, currentJointPositions, form.jointUnit, form.joints, form.targetType]);

  const canExecute =
    !submitting &&
    !backendCommandBusy &&
    !validation &&
    status.ros_connected &&
    status.control_mode === "auto_plan_execute" &&
    Boolean(status.services.planner_preview ?? status.services.planner) &&
    (form.targetType !== "joint" || Boolean(status.services.fk_joint_to_pose)) &&
    jointJumpWarnings.length === 0;

  const requestPayload = useMemo(() => buildRequest(form), [form]);

  useEffect(() => {
    if (!requestedTargetType) {
      return;
    }
    setForm((current) => ({ ...current, targetType: requestedTargetType }));
  }, [requestedTargetType]);

  useEffect(() => {
    const timer = window.setTimeout(() => {
      previewPlanningTarget(requestPayload)
        .then(setPreview)
        .catch((error) => setPreview({
          success: false,
          message: error.message,
          target_type: form.targetType,
          use_external_target: form.targetType !== "default",
          target_position: null,
          target_orientation: null,
          checks: [{ name: "后端预检查", ok: false, message: error.message }]
        }));
    }, 250);
    return () => window.clearTimeout(timer);
  }, [requestPayload, form.targetType]);

  function update<K extends keyof FormState>(key: K, value: FormState[K]) {
    setForm((current) => ({ ...current, [key]: value }));
  }

  function updateJoint(name: string, value: string) {
    setForm((current) => ({ ...current, joints: { ...current.joints, [name]: value } }));
  }

  function setOperationMode(targetType: TargetType) {
    setForm((current) => ({ ...current, targetType }));
  }

  function fillFromWaypoint(waypoint: Waypoint) {
    if (waypoint.kind === "joint" && waypoint.joint_positions) {
      const joints = { ...initialForm.joints };
      configuredJointNames.forEach((name) => {
        joints[name] = String(waypoint.joint_positions?.[name] ?? 0);
      });
      setForm((current) => ({ ...current, targetType: "joint", jointUnit: "rad", joints, waypointId: waypoint.id }));
      setMessage(`已回填关节点位：${waypoint.name}`);
      return;
    }
    const pose = waypoint.target_pose as {
      position?: { x?: number; y?: number; z?: number };
      orientation?: { x?: number; y?: number; z?: number; w?: number };
      rpy?: { roll?: number; pitch?: number; yaw?: number };
      angle_unit?: AngleUnit;
      position_unit?: PositionUnit;
    } | undefined;
    if (!pose) {
      setMessage("该点位没有可回填的目标数据。");
      return;
    }
    const base = {
      waypointId: waypoint.id,
      x: String(pose.position?.x ?? 0),
      y: String(pose.position?.y ?? 0),
      z: String(pose.position?.z ?? 0),
      positionUnit: pose.position_unit ?? "m"
    };
    if (waypoint.kind === "pose_rpy" && pose.rpy) {
      setForm((current) => ({
        ...current,
        ...base,
        targetType: "pose_rpy",
        angleUnit: pose.angle_unit ?? "rad",
        roll: String(pose.rpy?.roll ?? 0),
        pitch: String(pose.rpy?.pitch ?? 0),
        yaw: String(pose.rpy?.yaw ?? 0)
      }));
    } else {
      setForm((current) => ({
        ...current,
        ...base,
        targetType: "pose_quaternion",
        qx: String(pose.orientation?.x ?? 0),
        qy: String(pose.orientation?.y ?? 0),
        qz: String(pose.orientation?.z ?? 0),
        qw: String(pose.orientation?.w ?? 1)
      }));
    }
    setMessage(`已回填点位：${waypoint.name}`);
  }

  async function previewPlan() {
    if (status.control_mode === "estop") {
      setMessage("当前为 estop，禁止规划。");
      return;
    }
    if (validation) {
      setMessage(validation);
      return;
    }
    const confirmed = window.confirm(
      [
        "确认开始规划预览？",
        `目标类型：${targetTypeText(form.targetType)}`,
        `位置：${form.x}, ${form.y}, ${form.z} ${form.positionUnit}`,
        `姿态/单位：${orientationSummary(form)}`,
        `当前模式：${status.control_mode}`,
        "规划成功后只在 RViz 显示轨迹，不会立即执行真机动作。"
      ].join("\n")
    );
    if (!confirmed) {
      return;
    }
    const start = Date.now();
    setSubmitting(true);
    setTrajectoryReady(false);
    setTrajectoryVisible(false);
    setRequestStartedAt(new Date(start).toLocaleString());
    setDurationMs(null);
    try {
      const result = await previewPlanningExecution(requestPayload);
      setPlanResult(result);
      setLastTargetType(form.targetType);
      setDurationMs(Date.now() - start);
      setMessage(result.message);
      setTrajectoryReady(Boolean(result.success));
      setTrajectoryVisible(Boolean(result.success));
      await refreshSideData();
    } catch (error) {
      setDurationMs(Date.now() - start);
      setMessage((error as Error).message);
    } finally {
      setSubmitting(false);
    }
  }

  async function executeCachedPlan() {
    if (!effectiveTrajectoryReady) {
      setMessage("请先规划成功，再执行轨迹。");
      return;
    }
    if (status.control_mode === "estop") {
      setMessage("当前为 estop，禁止执行。");
      return;
    }
    const confirmed = window.confirm(
      [
        "确认执行已规划轨迹？",
        "执行后会暂停 RViz 轨迹展示；需要查看时可重新展示最近规划轨迹。",
        `当前模式：${status.control_mode}`
      ].join("\n")
    );
    if (!confirmed) {
      return;
    }
    const start = Date.now();
    setExecuting(true);
    setRequestStartedAt(new Date(start).toLocaleString());
    try {
      const result = await executePreviewedTrajectory();
      setMessage(result.message);
      setDurationMs(Date.now() - start);
      setTrajectoryReady(true);
      setTrajectoryVisible(false);
      await refreshSideData();
    } catch (error) {
      setMessage((error as Error).message);
      setDurationMs(Date.now() - start);
    } finally {
      setExecuting(false);
    }
  }

  async function toggleTrajectoryDisplay(show: boolean) {
    if (!effectiveTrajectoryReady) {
      setMessage("请先完成一次规划。");
      return;
    }
    try {
      const result = await setTrajectoryDisplay(show);
      setMessage(result.message);
      setTrajectoryVisible(show && result.success);
      await refreshSideData();
    } catch (error) {
      setMessage((error as Error).message);
    }
  }

  async function saveCurrentTarget() {
    if (!form.waypointName.trim()) {
      setMessage("请输入点位名称。");
      return;
    }
    if (validation) {
      setMessage(validation);
      return;
    }
    if (form.targetType === "joint") {
      await createWaypoint({
        name: form.waypointName,
        kind: "joint",
        note: form.waypointNote,
        joint_positions: objectMap(form.joints, Number)
      });
    } else if (form.targetType === "pose_rpy") {
      await createWaypoint({
        name: form.waypointName,
        kind: "pose_rpy",
        note: form.waypointNote,
        target_pose: {
          position: { x: Number(form.x), y: Number(form.y), z: Number(form.z) },
          position_unit: form.positionUnit,
          rpy: { roll: Number(form.roll), pitch: Number(form.pitch), yaw: Number(form.yaw) },
          angle_unit: form.angleUnit
        }
      });
    } else {
      await createWaypoint({
        name: form.waypointName,
        kind: "pose_quaternion",
        note: form.waypointNote,
        target_pose: {
          position: { x: Number(form.x), y: Number(form.y), z: Number(form.z) },
          position_unit: form.positionUnit,
          orientation: { x: Number(form.qx), y: Number(form.qy), z: Number(form.qz), w: Number(form.qw) }
        }
      });
    }
    setForm((current) => ({ ...current, waypointName: "", waypointNote: "" }));
    setMessage("点位已保存。");
    await refreshSideData();
  }

  return (
    <Card title="自动规划增强版">
      <div className="mode-tabs">
        <button
          className={form.targetType === "joint" ? "active" : ""}
          onClick={() => setOperationMode("joint")}
        >
          关节操作
        </button>
        <button
          className={form.targetType !== "joint" ? "active" : ""}
          onClick={() => setOperationMode("pose_quaternion")}
        >
          位置姿态操作
        </button>
      </div>

      <div className="row">
        {form.targetType !== "joint" && (
          <>
            <label>
              目标类型
              <select value={form.targetType} onChange={(event) => update("targetType", event.target.value as TargetType)}>
                <option value="default">默认目标</option>
                <option value="pose_quaternion">笛卡尔位姿 + 四元数</option>
                <option value="pose_rpy">笛卡尔位姿 + 欧拉角 RPY</option>
                <option value="waypoint">点位目标</option>
              </select>
            </label>
            <label>
              位置单位
              <select value={form.positionUnit} onChange={(event) => update("positionUnit", event.target.value as PositionUnit)}>
                <option value="m">m</option>
                <option value="mm">mm</option>
              </select>
            </label>
            <label>
              姿态单位
              <select value={form.angleUnit} onChange={(event) => update("angleUnit", event.target.value as AngleUnit)}>
                <option value="rad">rad</option>
                <option value="deg">deg</option>
              </select>
            </label>
          </>
        )}
        <label>
          关节单位
          <select value={form.jointUnit} onChange={(event) => update("jointUnit", event.target.value as JointUnit)}>
            <option value="rad">rad</option>
            <option value="deg">deg</option>
          </select>
        </label>
      </div>

      {form.targetType === "waypoint" && (
        <div className="waypoints">
          {waypoints.map((waypoint) => (
            <button key={waypoint.id} onClick={() => fillFromWaypoint(waypoint)}>
              回填 {waypoint.name}（{waypointKindText(waypoint.kind)}）
            </button>
          ))}
        </div>
      )}

      {form.targetType !== "default" && form.targetType !== "waypoint" && form.targetType !== "joint" && (
        <div className="pose-grid">
          {(["x", "y", "z"] as const).map((field) => (
            <label key={field}>
              {field}
              <input value={form[field]} onChange={(event) => update(field, event.target.value)} />
            </label>
          ))}
        </div>
      )}

      {form.targetType === "pose_quaternion" && (
        <div className="pose-grid">
          {(["qx", "qy", "qz", "qw"] as const).map((field) => (
            <label key={field}>
              {field}
              <input value={form[field]} onChange={(event) => update(field, event.target.value)} />
            </label>
          ))}
        </div>
      )}

      {form.targetType === "pose_rpy" && (
        <>
          <div className="pose-grid">
            {(["roll", "pitch", "yaw"] as const).map((field) => (
              <label key={field}>
                {field}
                <input value={form[field]} onChange={(event) => update(field, event.target.value)} />
              </label>
            ))}
          </div>
          <p>
            前端转换四元数：
            {convertedQuaternion ? convertedQuaternion.map((value) => value.toFixed(6)).join(", ") : "输入无效"}
          </p>
        </>
      )}

      {form.targetType === "joint" && (
        <>
          <div className="joint-target-grid">
            {configuredJointNames.map((name) => (
              <JointTargetControl
                key={name}
                name={name}
                displayName={robotConfig?.joints.find((joint) => joint.name === name)?.display_name}
                unit={form.jointUnit}
                value={form.joints[name] ?? "0"}
                feedbackRad={currentJointPositions?.[name]}
                limitsRad={configuredJointLimits[name]}
                onChange={(value) => updateJoint(name, value)}
              />
            ))}
          </div>
          <p className="todo">滑条和数字框只设置目标值，不会直接控制真机；必须点击“先规划并在 RViz 显示”，再点击“执行已规划轨迹”。</p>
          {jointJumpWarnings.map((warning) => (
            <p className="warn" key={warning}>{warning}</p>
          ))}
        </>
      )}

      <h3>目标预检查</h3>
      {validation && <p className="warn">{validation}</p>}
      <StatusRow label="当前模式" value={status.control_mode} good={status.control_mode === "auto_plan_execute"} />
      <StatusRow label="规划预览 service" value={(status.services.planner_preview ?? status.services.planner) ? "在线" : "离线"} good={Boolean(status.services.planner_preview ?? status.services.planner)} />
      <StatusRow label="轨迹执行 service" value={status.services.planner_execute ? "在线" : "离线"} good={Boolean(status.services.planner_execute)} />
      <StatusRow label="轨迹展示 service" value={status.services.trajectory_display ? "在线" : "离线"} good={Boolean(status.services.trajectory_display)} />
      <StatusRow label="FK service" value={status.services.fk_joint_to_pose ? "在线" : "离线"} good={form.targetType !== "joint" || Boolean(status.services.fk_joint_to_pose)} />
      <StatusRow label="joint_states" value={ageText(status.joint_states_last_update_sec)} good={status.joint_states_last_update_sec !== null && status.joint_states_last_update_sec <= 2.0} />
      <StatusRow label="controller/action" value={status.actions.follow_joint_trajectory ? "在线" : "离线"} good={status.actions.follow_joint_trajectory} />
      {preview?.checks.map((check) => (
        <StatusRow key={`${check.name}-${check.message}`} label={check.name} value={check.message} good={check.ok} />
      ))}
      <StatusRow label="软限位检查" value={form.targetType === "joint" ? "已按内置 trunk 软限位检查" : "非关节目标无需检查"} good />

      <h3>转换结果</h3>
      <pre>{JSON.stringify({
        target_position: preview?.target_position ?? null,
        target_orientation: preview?.target_orientation ?? null,
        message: preview?.message ?? "等待预检查"
      }, null, 2)}</pre>

      <div className="row">
        <button disabled={!canExecute} onClick={previewPlan}>
          {submitting || backendCommandPhase === "planning" ? "规划中..." : "先规划并在 RViz 显示"}
        </button>
        <button
          className="danger"
          disabled={!effectiveTrajectoryReady || executing || backendCommandBusy || status.control_mode !== "auto_plan_execute"}
          onClick={executeCachedPlan}
        >
          {executing || backendCommandPhase === "executing" ? "执行中..." : "执行已规划轨迹"}
        </button>
        <button disabled={!effectiveTrajectoryReady || effectiveTrajectoryVisible || backendCommandPhase === "executing"} onClick={() => toggleTrajectoryDisplay(true)}>
          展示最近规划轨迹
        </button>
        <button disabled={!effectiveTrajectoryVisible || backendCommandPhase === "executing"} onClick={() => toggleTrajectoryDisplay(false)}>
          暂停轨迹展示
        </button>
        <label>
          点位名称
          <input value={form.waypointName} onChange={(event) => update("waypointName", event.target.value)} />
        </label>
        <label>
          备注
          <input value={form.waypointNote} onChange={(event) => update("waypointNote", event.target.value)} />
        </label>
        <button disabled={form.targetType === "default" || form.targetType === "waypoint"} onClick={saveCurrentTarget}>
          保存当前目标为点位
        </button>
      </div>
      {!canExecute && (
        <p className="warn">
          规划需要 auto_plan_execute、非 estop、ROS/planner 预览 service 在线、输入合法；关节目标还需要 FK service 在线。
        </p>
      )}

      <h3>执行状态</h3>
      <StatusRow label="请求状态" value={status.command_state?.message ?? (submitting ? "规划中" : executing ? "执行中" : "空闲")} good={!backendCommandBusy && !submitting && !executing} />
      <StatusRow label="轨迹状态" value={effectiveTrajectoryReady ? "已缓存最近规划轨迹" : "尚无规划轨迹"} good={effectiveTrajectoryReady} />
      <StatusRow label="RViz 展示" value={effectiveTrajectoryVisible ? "正在展示最近规划轨迹" : "轨迹展示已暂停"} good={effectiveTrajectoryVisible} />
      <StatusRow label="请求开始时间" value={requestStartedAt || "无"} good />
      <StatusRow label="请求耗时" value={durationMs === null ? "无" : `${durationMs} ms`} good />
      <StatusRow label="最近目标类型" value={lastTargetType ? targetTypeText(lastTargetType) : "无"} good />
      {planResult && <pre>{JSON.stringify(planResult, null, 2)}</pre>}
    </Card>
  );
}

function buildRequest(form: FormState): PlanningTargetRequest {
  const units = { position: form.positionUnit, angle: form.angleUnit, joint: form.jointUnit };
  if (form.targetType === "default") {
    return { target_type: "default", use_external_target: false, units };
  }
  if (form.targetType === "waypoint") {
    return { target_type: "waypoint", waypoint_id: form.waypointId || undefined, units };
  }
  if (form.targetType === "pose_rpy") {
    return {
      target_type: "pose_rpy",
      use_external_target: true,
      position: [Number(form.x), Number(form.y), Number(form.z)],
      orientation_rpy: [Number(form.roll), Number(form.pitch), Number(form.yaw)],
      units
    };
  }
  if (form.targetType === "joint") {
    return {
      target_type: "joint",
      position: [Number(form.x), Number(form.y), Number(form.z)],
      joint_positions: objectMap(form.joints, Number),
      units
    };
  }
  return {
    target_type: "pose_quaternion",
    use_external_target: true,
    position: [Number(form.x), Number(form.y), Number(form.z)],
    orientation_quaternion: [Number(form.qx), Number(form.qy), Number(form.qz), Number(form.qw)],
    units
  };
}

function objectMap<T>(input: Record<string, string>, mapper: (value: string) => T) {
  return Object.fromEntries(Object.entries(input).map(([key, value]) => [key, mapper(value)]));
}

function targetTypeText(targetType: TargetType) {
  const labels: Record<TargetType, string> = {
    default: "默认目标",
    pose_quaternion: "笛卡尔位姿 + 四元数",
    pose_rpy: "笛卡尔位姿 + 欧拉角",
    joint: "关节空间目标",
    waypoint: "点位目标"
  };
  return labels[targetType];
}

function waypointKindText(kind: Waypoint["kind"]) {
  if (kind === "joint") return "关节点位";
  if (kind === "pose_rpy") return "RPY 位姿点位";
  return "四元数位姿点位";
}

function orientationSummary(form: FormState) {
  if (form.targetType === "pose_quaternion") {
    return `${form.qx}, ${form.qy}, ${form.qz}, ${form.qw}`;
  }
  if (form.targetType === "pose_rpy") {
    return `roll=${form.roll}, pitch=${form.pitch}, yaw=${form.yaw} ${form.angleUnit}`;
  }
  if (form.targetType === "joint") {
    return `关节单位 ${form.jointUnit}`;
  }
  return "使用默认目标或点位目标";
}

function JointTargetControl({
  name,
  displayName,
  unit,
  value,
  feedbackRad,
  limitsRad,
  onChange
}: {
  name: string;
  displayName?: string;
  unit: JointUnit;
  value: string;
  feedbackRad?: number;
  limitsRad?: { min: number; max: number };
  onChange: (value: string) => void;
}) {
  const limit = limitsRad ?? jointLimitsRad[name] ?? { min: -3.14, max: 3.14 };
  const min = unit === "deg" ? (limit.min * 180.0) / Math.PI : limit.min;
  const max = unit === "deg" ? (limit.max * 180.0) / Math.PI : limit.max;
  const step = unit === "deg" ? 0.1 : 0.001;
  const numeric = Number.isFinite(Number(value)) ? Number(value) : 0;
  const sliderValue = Math.min(Math.max(numeric, min), max);
  const feedback = feedbackRad === undefined
    ? null
    : unit === "deg"
      ? (feedbackRad * 180.0) / Math.PI
      : feedbackRad;
  const error = feedback === null ? null : numeric - feedback;
  return (
    <div className="joint-target-control">
      <div className="joint-target-header">
        <strong>{displayName ? `${displayName} (${name})` : name}</strong>
        <span>
          [{min.toFixed(unit === "deg" ? 1 : 3)}, {max.toFixed(unit === "deg" ? 1 : 3)}] {unit}
        </span>
        <span>
          反馈 {feedback === null ? "--" : feedback.toFixed(unit === "deg" ? 2 : 4)}
          {" / "}
          误差 {error === null ? "--" : error.toFixed(unit === "deg" ? 2 : 4)}
        </span>
      </div>
      <input
        type="range"
        min={min}
        max={max}
        step={step}
        value={sliderValue}
        onChange={(event) => onChange(event.target.value)}
      />
      <input
        value={value}
        onChange={(event) => onChange(event.target.value)}
        onBlur={() => {
          if (!Number.isFinite(Number(value))) {
            onChange("0");
            return;
          }
          onChange(String(Math.min(Math.max(Number(value), min), max)));
        }}
      />
    </div>
  );
}

function ageText(value: number | null) {
  return value === null ? "从未收到" : `${value.toFixed(1)} 秒前`;
}

function Card({ title, children }: { title: string; children: ReactNode }) {
  return (
    <section className="card">
      <h2>{title}</h2>
      {children}
    </section>
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
