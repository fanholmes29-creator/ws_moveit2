from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Tuple

from .models import PlanningCheck, PlanningPreviewResponse, PlanningTargetRequest, WaypointRecord
from .ros_bridge import JOINT_LIMITS, JOINT_NAMES


def finite_list(values: Optional[List[float]], expected_len: int, field_name: str) -> List[float]:
    if values is None:
        raise ValueError(f"{field_name} 不能为空")
    if len(values) != expected_len:
        raise ValueError(f"{field_name} 必须包含 {expected_len} 个值")
    if any(not math.isfinite(value) for value in values):
        raise ValueError(f"{field_name} 不能包含 NaN 或 Inf")
    return [float(value) for value in values]


def position_to_m(values: Optional[List[float]], unit: str) -> List[float]:
    position = finite_list(values, 3, "position")
    if unit == "m":
        return position
    if unit == "mm":
        return [value / 1000.0 for value in position]
    raise ValueError(f"不支持的位置单位：{unit}")


def angle_to_rad(values: List[float], unit: str) -> List[float]:
    if unit == "rad":
        return values
    if unit == "deg":
        return [math.radians(value) for value in values]
    raise ValueError(f"不支持的角度单位：{unit}")


def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> List[float]:
    # ROS 常用 RPY 顺序：roll(X), pitch(Y), yaw(Z)。
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]


def validate_quaternion(quaternion: List[float]) -> List[float]:
    finite_list(quaternion, 4, "orientation_quaternion")
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm < 1.0e-6:
        raise ValueError("四元数范数过小")
    return [value / norm for value in quaternion]


def joint_positions_to_rad(joint_positions: Optional[Dict[str, float]], unit: str) -> Dict[str, float]:
    if joint_positions is None:
        raise ValueError("joint_positions 不能为空")
    missing = [name for name in JOINT_NAMES if name not in joint_positions]
    if missing:
        raise ValueError(f"关节目标缺少：{', '.join(missing)}")
    values = {name: float(joint_positions[name]) for name in JOINT_NAMES}
    if any(not math.isfinite(value) for value in values.values()):
        raise ValueError("关节目标不能包含 NaN 或 Inf")
    if unit == "deg":
        values = {name: math.radians(value) for name, value in values.items()}
    elif unit != "rad":
        raise ValueError(f"不支持的关节单位：{unit}")
    return values


def joint_limit_checks(joint_positions_rad: Optional[Dict[str, float]]) -> List[PlanningCheck]:
    if joint_positions_rad is None:
        return [PlanningCheck(name="关节软限位", ok=True, message="非关节目标无需检查")]
    checks: List[PlanningCheck] = []
    for name in JOINT_NAMES:
        lower, upper = JOINT_LIMITS[name]
        value = joint_positions_rad[name]
        checks.append(
            PlanningCheck(
                name=f"{name} 软限位",
                ok=lower <= value <= upper,
                message=f"{value:.6f} rad，允许范围 [{lower:.6f}, {upper:.6f}]",
            )
        )
    return checks


def waypoint_to_target_request(
    request: PlanningTargetRequest,
    waypoint: WaypointRecord,
) -> PlanningTargetRequest:
    if waypoint.kind == "joint":
        return PlanningTargetRequest(
            target_type="joint",
            joint_positions=waypoint.joint_positions,
            units=request.units.copy(update={"joint": "rad"}),
        )
    if waypoint.kind in ("pose", "pose_quaternion"):
        pose = waypoint.target_pose or {}
        position = _read_position(pose)
        orientation = _read_quaternion(pose)
        position_unit = str(pose.get("position_unit", "m"))
        return PlanningTargetRequest(
            target_type="pose_quaternion",
            use_external_target=True,
            position=position,
            orientation_quaternion=orientation,
            units=request.units.copy(update={"position": position_unit, "angle": "rad"}),
        )
    if waypoint.kind == "pose_rpy":
        pose = waypoint.target_pose or {}
        position = _read_position(pose)
        rpy = _read_rpy(pose)
        angle_unit = str(pose.get("angle_unit", "rad"))
        position_unit = str(pose.get("position_unit", "m"))
        return PlanningTargetRequest(
            target_type="pose_rpy",
            use_external_target=True,
            position=position,
            orientation_rpy=rpy,
            units=request.units.copy(update={"position": position_unit, "angle": angle_unit}),
        )
    raise ValueError(f"不支持的点位类型：{waypoint.kind}")


def resolve_planning_target(
    request: PlanningTargetRequest,
) -> Tuple[Optional[List[float]], Optional[List[float]], bool, Optional[Dict[str, float]]]:
    if request.target_type == "default":
        return None, None, False, None
    if request.target_type == "pose_quaternion":
        position = position_to_m(request.position, request.units.position)
        quaternion = validate_quaternion(
            finite_list(request.orientation_quaternion, 4, "orientation_quaternion")
        )
        return position, quaternion, True, None
    if request.target_type == "pose_rpy":
        position = position_to_m(request.position, request.units.position)
        rpy = angle_to_rad(finite_list(request.orientation_rpy, 3, "orientation_rpy"), request.units.angle)
        quaternion = validate_quaternion(rpy_to_quaternion(rpy[0], rpy[1], rpy[2]))
        return position, quaternion, True, None
    if request.target_type == "joint":
        joints = joint_positions_to_rad(request.joint_positions, request.units.joint)
        raise NotImplementedError("FK backend not implemented for joint target")
    if request.target_type == "waypoint":
        raise ValueError("waypoint 目标必须先解析为具体点位类型")
    raise ValueError(f"不支持的目标类型：{request.target_type}")


def build_preview(
    request: PlanningTargetRequest,
    status: Dict[str, Any],
) -> PlanningPreviewResponse:
    checks: List[PlanningCheck] = [
        PlanningCheck(
            name="ROS 连接",
            ok=bool(status["ros_connected"]),
            message="已连接" if status["ros_connected"] else "ROS 未连接",
        ),
        PlanningCheck(
            name="控制模式",
            ok=status["control_mode"] == "auto_plan_execute",
            message=f"当前模式：{status['control_mode']}",
        ),
        PlanningCheck(
            name="planner service",
            ok=bool(status["services"]["planner"]),
            message="在线" if status["services"]["planner"] else "离线",
        ),
        PlanningCheck(
            name="follow_joint_trajectory action",
            ok=bool(status["actions"]["follow_joint_trajectory"]),
            message="在线" if status["actions"]["follow_joint_trajectory"] else "离线",
        ),
        PlanningCheck(
            name="joint_states 新鲜度",
            ok=status["joint_states_last_update_sec"] is not None
            and status["joint_states_last_update_sec"] <= 2.0,
            message=(
                "从未收到"
                if status["joint_states_last_update_sec"] is None
                else f"{status['joint_states_last_update_sec']:.1f} 秒前"
            ),
        ),
    ]
    target_position: Optional[List[float]] = None
    target_orientation: Optional[List[float]] = None
    joint_positions_rad: Optional[Dict[str, float]] = None
    success = True
    message = "预检查通过"
    try:
        if request.target_type == "default":
            checks.append(PlanningCheck(name="默认目标", ok=True, message="将使用 planner 默认目标"))
        elif request.target_type == "joint":
            joint_positions_rad = joint_positions_to_rad(request.joint_positions, request.units.joint)
            checks.extend(joint_limit_checks(joint_positions_rad))
            checks.append(PlanningCheck(name="FK 转换", ok=True, message="将由后端 FK service 转换为目标位姿"))
        else:
            target_position, target_orientation, _, _ = resolve_planning_target(request)
            checks.append(PlanningCheck(name="目标数值", ok=True, message="输入合法"))
            checks.append(PlanningCheck(name="四元数", ok=True, message="四元数有效并已归一化"))
    except NotImplementedError as exc:
        success = False
        message = str(exc)
        checks.append(PlanningCheck(name="目标转换", ok=False, message=str(exc)))
    except Exception as exc:  # noqa: BLE001
        success = False
        message = str(exc)
        checks.append(PlanningCheck(name="目标数值", ok=False, message=str(exc)))

    if any(not check.ok for check in checks):
        success = False
        if message == "预检查通过":
            message = "预检查存在未通过项"
    return PlanningPreviewResponse(
        success=success,
        message=message,
        target_type=request.target_type,
        use_external_target=request.target_type != "default",
        target_position=target_position,
        target_orientation=target_orientation,
        checks=checks,
    )


def _read_position(pose: Dict[str, Any]) -> List[float]:
    position = pose.get("position", {})
    return [float(position.get("x", 0.0)), float(position.get("y", 0.0)), float(position.get("z", 0.0))]


def _read_quaternion(pose: Dict[str, Any]) -> List[float]:
    orientation = pose.get("orientation", {})
    return [
        float(orientation.get("x", 0.0)),
        float(orientation.get("y", 0.0)),
        float(orientation.get("z", 0.0)),
        float(orientation.get("w", 1.0)),
    ]


def _read_rpy(pose: Dict[str, Any]) -> List[float]:
    rpy = pose.get("rpy", {})
    return [float(rpy.get("roll", 0.0)), float(rpy.get("pitch", 0.0)), float(rpy.get("yaw", 0.0))]
