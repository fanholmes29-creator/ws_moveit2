from __future__ import annotations

from typing import Any, Dict, List, Literal, Optional

from pydantic import BaseModel, Field


ControlMode = Literal["idle", "manual_teleop", "auto_plan_execute", "estop"]
TargetType = Literal["default", "pose_quaternion", "pose_rpy", "joint", "waypoint"]
PositionUnit = Literal["m", "mm"]
AngleUnit = Literal["rad", "deg"]
JointUnit = Literal["rad", "deg"]
WaypointKind = Literal["joint", "pose", "pose_quaternion", "pose_rpy"]


class ModeRequest(BaseModel):
    mode: ControlMode


class PlanToPoseRequest(BaseModel):
    use_external_target: bool = True
    target_position: List[float] = Field(default_factory=lambda: [0.0, 0.0, 0.0])
    target_orientation: List[float] = Field(default_factory=lambda: [0.0, 0.0, 0.0, 1.0])


class PlanningUnits(BaseModel):
    position: PositionUnit = "m"
    angle: AngleUnit = "rad"
    joint: JointUnit = "rad"


class PlanningTargetRequest(BaseModel):
    target_type: TargetType
    use_external_target: bool = True
    position: Optional[List[float]] = None
    orientation_quaternion: Optional[List[float]] = None
    orientation_rpy: Optional[List[float]] = None
    joint_positions: Optional[Dict[str, float]] = None
    units: PlanningUnits = Field(default_factory=PlanningUnits)
    waypoint_id: Optional[str] = None


class PlanningCheck(BaseModel):
    name: str
    ok: bool
    message: str


class PlanningPreviewResponse(BaseModel):
    success: bool
    message: str
    target_type: str
    use_external_target: bool
    target_position: Optional[List[float]] = None
    target_orientation: Optional[List[float]] = None
    checks: List[PlanningCheck] = Field(default_factory=list)


class JointConfig(BaseModel):
    name: str
    display_name: str
    group: str
    index: int
    min_rad: float
    max_rad: float
    default_rad: float = 0.0
    unit: str = "rad"


class RobotConfig(BaseModel):
    robot_name: str
    robot_type: str
    modules: List[str]
    joints: List[JointConfig]


class JointRuntimeState(BaseModel):
    # TODO: wire this into TrunkRosBridge.get_status() in the next phase so the
    # frontend can display target/feedback/error in one joint table.
    name: str
    position_rad: Optional[float] = None
    target_rad: Optional[float] = None
    error_rad: Optional[float] = None
    min_rad: float
    max_rad: float
    within_limit: Optional[bool] = None
    last_update_sec: Optional[float] = None


class RobotConnectionState(BaseModel):
    ros_bridge_connected: bool
    joint_states_online: bool
    controller_state_online: bool
    control_mode_online: bool
    planning_service_online: bool
    fk_service_online: bool
    trajectory_action_online: bool


class RobotModeState(BaseModel):
    control_mode: str
    is_estop: bool
    manual_motion_allowed: bool
    auto_motion_allowed: bool


class RobotEnableState(BaseModel):
    enabled: Optional[bool] = None
    source: str
    message: str


class CommandState(BaseModel):
    phase: str = "idle"
    active_command: Optional[str] = None
    trajectory_ready: bool = False
    trajectory_visible: bool = False
    started_at: Optional[float] = None
    updated_at: Optional[float] = None
    message: Optional[str] = None


class OperationLog(BaseModel):
    timestamp: float
    level: str
    event: str
    message: str
    source: str
    details: Optional[Dict[str, Any]] = None


class JogRequest(BaseModel):
    joint_name: str
    direction: Literal[-1, 1]
    step: float = 0.02
    velocity_scale: float = 0.2
    deadman: bool = False


class WaypointCreateRequest(BaseModel):
    name: str
    kind: WaypointKind
    note: str = ""
    joint_positions: Optional[Dict[str, float]] = None
    target_pose: Optional[Dict[str, Any]] = None


class WaypointRecord(BaseModel):
    id: str
    name: str
    kind: WaypointKind
    note: str = ""
    joint_positions: Optional[Dict[str, float]] = None
    target_pose: Optional[Dict[str, Any]] = None
    created_at: str
