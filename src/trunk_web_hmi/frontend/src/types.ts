export type ControlMode = "idle" | "manual_teleop" | "auto_plan_execute" | "estop";
export type TargetType = "default" | "pose_quaternion" | "pose_rpy" | "joint" | "waypoint";
export type PositionUnit = "m" | "mm";
export type AngleUnit = "rad" | "deg";
export type JointUnit = "rad" | "deg";

export type Status = {
  ros_connected: boolean;
  namespace: string;
  control_mode: string;
  connection_state?: RobotConnectionState;
  mode_state?: RobotModeState;
  enable_state?: RobotEnableState;
  command_state?: CommandState;
  control_mode_last_update_sec: number | null;
  joint_positions: Record<string, number>;
  joints_runtime?: JointRuntimeState[];
  joint_states_last_update_sec: number | null;
  controller_state_last_update_sec: number | null;
  services: {
    set_control_mode: boolean;
    planner: boolean;
    planner_preview?: boolean;
    planner_execute?: boolean;
    fk_joint_to_pose?: boolean;
    trajectory_display?: boolean;
  };
  actions: {
    follow_joint_trajectory: boolean;
  };
  topics: {
    control_mode_state: boolean;
    joint_states: boolean;
    controller_state: boolean;
  };
  last_error: LogItem | null;
};

export type RobotConnectionState = {
  ros_bridge_connected: boolean;
  joint_states_online: boolean;
  controller_state_online: boolean;
  control_mode_online: boolean;
  planning_service_online: boolean;
  fk_service_online: boolean;
  trajectory_action_online: boolean;
};

export type RobotModeState = {
  control_mode: string;
  is_estop: boolean;
  manual_motion_allowed: boolean;
  auto_motion_allowed: boolean;
};

export type RobotEnableState = {
  enabled: boolean | null;
  source: string;
  message: string;
};

export type CommandState = {
  phase: string;
  active_command?: string | null;
  trajectory_ready: boolean;
  trajectory_visible: boolean;
  started_at?: number | null;
  updated_at?: number | null;
  message?: string | null;
};

export type LogItem = {
  time: string;
  level: string;
  event?: string;
  message: string;
  source?: string;
  details?: Record<string, unknown> | null;
};

export type OperationLog = {
  timestamp: number;
  level: string;
  event: string;
  message: string;
  source: string;
  details?: Record<string, unknown> | null;
};

export type JointConfig = {
  name: string;
  display_name: string;
  group: string;
  index: number;
  min_rad: number;
  max_rad: number;
  default_rad: number;
  unit: string;
};

export type RobotConfig = {
  robot_name: string;
  robot_type: string;
  modules: string[];
  joints: JointConfig[];
};

export type JointRuntimeState = {
  name: string;
  position_rad?: number | null;
  target_rad?: number | null;
  error_rad?: number | null;
  min_rad: number;
  max_rad: number;
  within_limit?: boolean | null;
  last_update_sec?: number | null;
};

export type PlanForm = {
  target_type?: TargetType;
  use_external_target: boolean;
  x: string;
  y: string;
  z: string;
  qx: string;
  qy: string;
  qz: string;
  qw: string;
  roll?: string;
  pitch?: string;
  yaw?: string;
  joint_positions?: Record<string, string>;
  units?: PlanningUnits;
  waypoint_id?: string;
};

export type PlanResult = {
  success: boolean;
  error_code: number;
  message: string;
  used_target_pose: {
    position: { x: number; y: number; z: number };
    orientation: { x: number; y: number; z: number; w: number };
  };
  used_external_target: boolean;
  target_type?: TargetType;
  converted_target_position?: number[];
  converted_target_orientation?: number[];
  preview?: PlanningPreview;
};

export type PlanningUnits = {
  position: PositionUnit;
  angle: AngleUnit;
  joint: JointUnit;
};

export type PlanningTargetRequest = {
  target_type: TargetType;
  use_external_target?: boolean;
  position?: number[];
  orientation_quaternion?: number[];
  orientation_rpy?: number[];
  joint_positions?: Record<string, number>;
  units: PlanningUnits;
  waypoint_id?: string;
};

export type PlanningCheck = {
  name: string;
  ok: boolean;
  message: string;
};

export type PlanningPreview = {
  success: boolean;
  message: string;
  target_type: TargetType;
  use_external_target: boolean;
  target_position: number[] | null;
  target_orientation: number[] | null;
  checks: PlanningCheck[];
};

export type Waypoint = {
  id: string;
  name: string;
  kind: "joint" | "pose" | "pose_quaternion" | "pose_rpy";
  note: string;
  joint_positions?: Record<string, number>;
  target_pose?: Record<string, unknown>;
  created_at: string;
};
