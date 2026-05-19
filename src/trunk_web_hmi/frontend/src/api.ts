import type {
  ControlMode,
  PlanForm,
  PlanningPreview,
  PlanningTargetRequest,
  PlanResult,
  RobotConfig,
  OperationLog,
  Status,
  Waypoint
} from "./types";

async function requestJson<T>(url: string, options?: RequestInit): Promise<T> {
  const response = await fetch(url, {
    headers: { "Content-Type": "application/json" },
    ...options
  });
  if (!response.ok) {
    const payload = await response.json().catch(() => ({}));
    throw new Error(payload.detail ?? payload.message ?? response.statusText);
  }
  return response.json() as Promise<T>;
}

export function setMode(mode: ControlMode) {
  return requestJson<{ success: boolean; message: string }>("/api/mode", {
    method: "POST",
    body: JSON.stringify({ mode })
  });
}

export function setRobotEnable(enable: boolean) {
  return requestJson<{ ok: boolean; implemented: boolean; message: string }>(
    enable ? "/api/robot/enable" : "/api/robot/disable",
    { method: "POST" }
  );
}

export function resetRobotError() {
  return requestJson<{ ok: boolean; implemented: boolean; message: string }>(
    "/api/robot/reset_error",
    { method: "POST" }
  );
}

export function getStatus() {
  return requestJson<Status>("/api/status");
}

export function getRobotConfig() {
  return requestJson<RobotConfig>("/api/robot/config");
}

export function planToPose(form: PlanForm) {
  return requestJson("/api/plan_to_pose", {
    method: "POST",
    body: JSON.stringify({
      use_external_target: form.use_external_target,
      target_position: [Number(form.x), Number(form.y), Number(form.z)],
      target_orientation: [
        Number(form.qx),
        Number(form.qy),
        Number(form.qz),
        Number(form.qw)
      ]
    })
  });
}

export function previewPlanningTarget(payload: PlanningTargetRequest) {
  return requestJson<PlanningPreview>("/api/planning/target_preview", {
    method: "POST",
    body: JSON.stringify(payload)
  });
}

export function planToTarget(payload: PlanningTargetRequest) {
  return requestJson<PlanResult>("/api/planning/plan_to_target", {
    method: "POST",
    body: JSON.stringify(payload)
  });
}

export function previewPlanningExecution(payload: PlanningTargetRequest) {
  return requestJson<PlanResult>("/api/planning/preview_target", {
    method: "POST",
    body: JSON.stringify(payload)
  });
}

export function executePreviewedTrajectory() {
  return requestJson<{ success: boolean; error_code: number; message: string }>(
    "/api/planning/execute_previewed_trajectory",
    { method: "POST" }
  );
}

export function setTrajectoryDisplay(show: boolean) {
  return requestJson<{ success: boolean; error_code: number; message: string }>(
    "/api/planning/trajectory_display",
    {
      method: "POST",
      body: JSON.stringify({ show })
    }
  );
}

export function jogJoint(
  jointName: string,
  direction: -1 | 1,
  step: number,
  velocityScale: number,
  deadman: boolean
) {
  return requestJson<{ success: boolean; message: string }>("/api/manual/jog", {
    method: "POST",
    body: JSON.stringify({
      joint_name: jointName,
      direction,
      step,
      velocity_scale: velocityScale,
      deadman
    })
  });
}

export function stopManual() {
  return requestJson<{ success: boolean; message: string }>("/api/manual/stop", {
    method: "POST"
  });
}

export function getWaypoints() {
  return requestJson<{ items: Waypoint[] }>("/api/waypoints");
}

export function createWaypoint(payload: Omit<Waypoint, "id" | "created_at">) {
  return requestJson<Waypoint>("/api/waypoints", {
    method: "POST",
    body: JSON.stringify(payload)
  });
}

export function deleteWaypoint(id: string) {
  return requestJson<{ success: boolean }>(`/api/waypoints/${id}`, {
    method: "DELETE"
  });
}

export function getLogs() {
  return requestJson("/api/logs");
}

export function getOperationLogs(limit = 200, level?: string) {
  const params = new URLSearchParams({ limit: String(limit) });
  if (level) {
    params.set("level", level);
  }
  return requestJson<{ items: OperationLog[] }>(`/api/operation_logs?${params.toString()}`);
}
