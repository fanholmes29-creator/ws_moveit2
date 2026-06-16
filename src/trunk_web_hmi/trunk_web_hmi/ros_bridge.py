from __future__ import annotations

import math
import threading
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import Pose
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from trunk_two_stage_planner.srv import ExecutePreviewedTrajectory
from trunk_two_stage_planner.srv import FkJointToPose
from trunk_two_stage_planner.srv import SetTrajectoryDisplay
from trunk_teleop_control.srv import SetControlMode
from trunk_two_stage_planner.srv import PlanToPose

from .models import (
    CommandState,
    JogRequest,
    JointRuntimeState,
    OperationLog,
    PlanToPoseRequest,
    RobotConnectionState,
    RobotEnableState,
    RobotModeState,
)
from .operation_logs import append_operation_log
from .robot_config import MAX_TARGET_DELTA_RAD, get_joint_limits, get_joint_names


JOINT_NAMES = get_joint_names()
CONTROL_MODES = {"idle", "manual_teleop", "auto_plan_execute", "estop"}
JOINT_LIMITS = get_joint_limits()


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _finite_values(values: List[float], expected_len: int, field_name: str) -> None:
    if len(values) != expected_len:
        raise ValueError(f"{field_name} 必须包含 {expected_len} 个值")
    if any(not math.isfinite(value) for value in values):
        raise ValueError(f"{field_name} 不能包含 NaN 或 Inf")


def validate_plan_request(request: PlanToPoseRequest) -> None:
    _finite_values(request.target_position, 3, "target_position")
    _finite_values(request.target_orientation, 4, "target_orientation")
    norm = math.sqrt(sum(value * value for value in request.target_orientation))
    if norm < 1.0e-6:
        raise ValueError("target_orientation 四元数范数过小")


def _pose_to_dict(pose: Pose) -> Dict[str, Dict[str, float]]:
    return {
        "position": {
            "x": pose.position.x,
            "y": pose.position.y,
            "z": pose.position.z,
        },
        "orientation": {
            "x": pose.orientation.x,
            "y": pose.orientation.y,
            "z": pose.orientation.z,
            "w": pose.orientation.w,
        },
    }


def _duration_from_seconds(seconds: float) -> Duration:
    whole_seconds = int(seconds)
    nanoseconds = int((seconds - whole_seconds) * 1.0e9)
    return Duration(sec=whole_seconds, nanosec=nanoseconds)


def _clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


class TrunkRosBridge:
    def __init__(self, namespace: str, waypoint_path: Path) -> None:
        self.namespace = namespace.strip("/") or "trunk_robot"
        self.waypoint_path = waypoint_path
        self._lock = threading.RLock()
        self._logs: Deque[Dict[str, str]] = deque(maxlen=300)
        self._errors: Deque[Dict[str, str]] = deque(maxlen=100)
        self._control_mode = "unknown"
        self._last_control_mode_at: Optional[float] = None
        self._joint_positions: Dict[str, float] = {name: 0.0 for name in JOINT_NAMES}
        self._target_joint_positions: Optional[Dict[str, float]] = None
        self._last_joint_states_at: Optional[float] = None
        self._last_controller_state_at: Optional[float] = None
        self._ros_started = False
        self._node: Optional[Node] = None
        self._executor: Optional[MultiThreadedExecutor] = None
        self._thread: Optional[threading.Thread] = None
        self._service_client: Optional[Any] = None
        self._planner_client: Optional[Any] = None
        self._planner_preview_client: Optional[Any] = None
        self._planner_execute_client: Optional[Any] = None
        self._fk_client: Optional[Any] = None
        self._trajectory_display_client: Optional[Any] = None
        self._trajectory_action_client: Optional[Any] = None
        self._active_goal_handle: Optional[Any] = None
        self._command_state = CommandState(
            phase="idle",
            active_command=None,
            trajectory_ready=False,
            trajectory_visible=False,
            updated_at=time.time(),
            message="空闲",
        )

    def start(self) -> None:
        if self._ros_started:
            return
        if not rclpy.ok():
            rclpy.init(args=None)

        self._node = Node("trunk_web_hmi_backend")
        ns = f"/{self.namespace}"
        qos_state = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._node.create_subscription(
            String,
            f"{ns}/control_mode_state",
            self._on_control_mode,
            qos_state,
        )
        self._node.create_subscription(
            JointState,
            f"{ns}/joint_states",
            self._on_joint_states,
            10,
        )
        self._node.create_subscription(
            JointTrajectoryControllerState,
            f"{ns}/trunk_group_controller/controller_state",
            self._on_controller_state,
            10,
        )
        self._service_client = self._node.create_client(
            SetControlMode,
            f"{ns}/set_control_mode",
        )
        self._planner_client = self._node.create_client(
            PlanToPose,
            f"{ns}/two_stage_planner/plan_to_pose",
        )
        self._planner_preview_client = self._node.create_client(
            PlanToPose,
            f"{ns}/two_stage_planner/preview_plan_to_pose",
        )
        self._planner_execute_client = self._node.create_client(
            ExecutePreviewedTrajectory,
            f"{ns}/two_stage_planner/execute_previewed_trajectory",
        )
        self._fk_client = self._node.create_client(
            FkJointToPose,
            f"{ns}/two_stage_planner/fk_joint_to_pose",
        )
        self._trajectory_display_client = self._node.create_client(
            SetTrajectoryDisplay,
            f"{ns}/two_stage_planner/set_trajectory_display",
        )
        self._trajectory_action_client = ActionClient(
            self._node,
            FollowJointTrajectory,
            f"{ns}/trunk_group_controller/follow_joint_trajectory",
        )

        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()
        self._ros_started = True
        self.add_log("info", "ROS 桥接已启动", event="bridge_start")

    def stop(self) -> None:
        if self._executor is not None:
            self._executor.shutdown()
        if self._node is not None:
            self._node.destroy_node()
        self._ros_started = False
        self.add_log("info", "ROS 桥接已停止", event="bridge_stop")

    def get_status(self) -> Dict[str, Any]:
        now = time.time()
        service_online = self._service_available(self._service_client)
        planner_online = self._service_available(self._planner_client)
        planner_preview_online = self._service_available(self._planner_preview_client)
        planner_execute_online = self._service_available(self._planner_execute_client)
        fk_online = self._service_available(self._fk_client)
        trajectory_display_online = self._service_available(self._trajectory_display_client)
        action_online = self._action_available()
        with self._lock:
            joint_age = self._age_or_none(now, self._last_joint_states_at)
            controller_age = self._age_or_none(now, self._last_controller_state_at)
            ros_connected = self._ros_started and self._node is not None
            joint_states_online = self._is_recent(joint_age, max_age_sec=2.0)
            controller_state_online = self._is_recent(controller_age, max_age_sec=2.0)
            control_mode_online = self._last_control_mode_at is not None
            is_estop = self._control_mode == "estop"
            mode_state = RobotModeState(
                control_mode=self._control_mode,
                is_estop=is_estop,
                manual_motion_allowed=(
                    ros_connected and not is_estop and self._control_mode == "manual_teleop" and action_online
                ),
                auto_motion_allowed=(
                    ros_connected and not is_estop and self._control_mode == "auto_plan_execute" and planner_preview_online
                ),
            )
            connection_state = RobotConnectionState(
                ros_bridge_connected=ros_connected,
                joint_states_online=joint_states_online,
                controller_state_online=controller_state_online,
                control_mode_online=control_mode_online,
                planning_service_online=planner_preview_online or planner_online,
                fk_service_online=fk_online,
                trajectory_action_online=action_online,
            )
            enable_state = RobotEnableState(
                enabled=None,
                source="not_implemented",
                message=(
                    "真实硬件使能状态尚未接入；当前仅使用 control_mode 做 ROS 层运动门控。"
                ),
            )
            return {
                "ros_connected": ros_connected,
                "namespace": f"/{self.namespace}",
                "control_mode": self._control_mode,
                "connection_state": connection_state.dict(),
                "mode_state": mode_state.dict(),
                "enable_state": enable_state.dict(),
                "command_state": self._command_state.dict(),
                "control_mode_last_update_sec": self._age_or_none(
                    now, self._last_control_mode_at
                ),
                "joint_positions": dict(self._joint_positions),
                "joints_runtime": self._build_joints_runtime_locked(now),
                "joint_states_last_update_sec": joint_age,
                "controller_state_last_update_sec": controller_age,
                "services": {
                    "set_control_mode": service_online,
                    "planner": planner_online,
                    "planner_preview": planner_preview_online,
                    "planner_execute": planner_execute_online,
                    "fk_joint_to_pose": fk_online,
                    "trajectory_display": trajectory_display_online,
                },
                "actions": {
                    "follow_joint_trajectory": action_online,
                },
                "topics": {
                    "control_mode_state": control_mode_online,
                    "joint_states": self._last_joint_states_at is not None,
                    "controller_state": self._last_controller_state_at is not None,
                },
                "last_error": self._errors[-1] if self._errors else None,
            }

    def get_logs(self) -> Dict[str, List[Dict[str, str]]]:
        with self._lock:
            return {
                "operations": list(self._logs),
                "errors": list(self._errors),
            }

    def set_mode(self, mode: str) -> Dict[str, Any]:
        normalized = mode.strip()
        if normalized not in CONTROL_MODES:
            raise ValueError(f"不支持的控制模式：{mode}")
        if not self._service_available(self._service_client):
            raise RuntimeError("set_control_mode service 不在线")
        self._validate_mode_switch_allowed(normalized)

        request = SetControlMode.Request()
        request.mode = normalized
        self.add_log(
            "info",
            f"请求切换控制模式：{normalized}",
            event="mode_change_request",
            details={"requested_mode": normalized},
        )
        response = self._call_service(self._service_client, request, timeout_sec=5.0)
        result = {"success": bool(response.success), "message": response.message}
        if response.success:
            self.add_log(
                "safety" if normalized == "estop" else "info",
                f"控制模式已切换为：{normalized}",
                event="mode_change_result",
                details={"mode": normalized, "success": True},
            )
        else:
            self.add_error(
                f"控制模式切换失败：{response.message}",
                event="mode_change_failed",
                details={"mode": normalized, "success": False},
            )
        return result

    def plan_to_pose(self, request: PlanToPoseRequest) -> Dict[str, Any]:
        validate_plan_request(request)
        self._assert_motion_allowed("auto_plan_execute")
        if not self._service_available(self._planner_client):
            raise RuntimeError("plan_to_pose service 不在线")

        ros_request = PlanToPose.Request()
        ros_request.use_external_target = bool(request.use_external_target)
        ros_request.target_position = [float(value) for value in request.target_position]
        ros_request.target_orientation = [
            float(value) for value in request.target_orientation
        ]
        self.add_log("info", "请求执行 PlanToPose", event="plan_to_pose_request")
        response = self._call_service(self._planner_client, ros_request, timeout_sec=120.0)
        result = {
            "success": bool(response.success),
            "error_code": int(response.error_code),
            "message": response.message,
            "used_target_pose": _pose_to_dict(response.used_target_pose),
            "used_external_target": bool(response.used_external_target),
        }
        if response.success:
            self.add_log("info", "PlanToPose 执行成功", event="plan_to_pose_result")
        else:
            self.add_error(f"PlanToPose 执行失败：{response.message}", event="plan_to_pose_failed")
        return result

    def preview_plan_to_pose(self, request: PlanToPoseRequest) -> Dict[str, Any]:
        validate_plan_request(request)
        self._assert_motion_allowed("auto_plan_execute")
        self._reject_if_command_busy("规划预览")
        if not self._service_available(self._planner_preview_client):
            raise RuntimeError("preview_plan_to_pose service 不在线")

        self._set_command_state("planning", "planning_preview", "正在规划预览")
        ros_request = PlanToPose.Request()
        ros_request.use_external_target = bool(request.use_external_target)
        ros_request.target_position = [float(value) for value in request.target_position]
        ros_request.target_orientation = [
            float(value) for value in request.target_orientation
        ]
        self.add_log("info", "请求规划预览 PlanToPose", event="planning_preview_request")
        try:
            response = self._call_service(self._planner_preview_client, ros_request, timeout_sec=120.0)
        except Exception as exc:
            self._set_command_state("failed", "planning_preview", f"规划预览失败：{exc}")
            raise
        result = {
            "success": bool(response.success),
            "error_code": int(response.error_code),
            "message": response.message,
            "used_target_pose": _pose_to_dict(response.used_target_pose),
            "used_external_target": bool(response.used_external_target),
        }
        if response.success:
            self.add_log("info", "规划预览成功，轨迹已发送到 RViz", event="planning_preview_result")
            self._set_command_state(
                "preview_ready",
                "planning_preview",
                "规划预览成功，轨迹可执行",
                trajectory_ready=True,
                trajectory_visible=True,
            )
        else:
            self.add_error(f"规划预览失败：{response.message}", event="planning_preview_failed")
            self._set_command_state(
                "failed",
                "planning_preview",
                f"规划预览失败：{response.message}",
                trajectory_ready=False,
                trajectory_visible=False,
            )
        return result

    def execute_previewed_trajectory(self) -> Dict[str, Any]:
        self._assert_motion_allowed("auto_plan_execute")
        self._validate_execute_allowed()
        if not self._service_available(self._planner_execute_client):
            raise RuntimeError("execute_previewed_trajectory service 不在线")
        request = ExecutePreviewedTrajectory.Request()
        self.add_log("info", "请求执行已规划轨迹", event="trajectory_execute_request")
        self._set_command_state("executing", "execute_previewed_trajectory", "正在执行已规划轨迹")
        try:
            response = self._call_service(self._planner_execute_client, request, timeout_sec=120.0)
        except Exception as exc:
            self._set_command_state("failed", "execute_previewed_trajectory", f"轨迹执行失败：{exc}")
            raise
        result = {
            "success": bool(response.success),
            "error_code": int(response.error_code),
            "message": response.message,
        }
        if response.success:
            self.add_log("info", "已规划轨迹执行成功，RViz 轨迹展示已暂停", event="trajectory_execute_result")
            self._set_command_state(
                "completed",
                "execute_previewed_trajectory",
                "轨迹执行完成",
                trajectory_ready=True,
                trajectory_visible=False,
            )
        else:
            self.add_error(f"已规划轨迹执行失败：{response.message}", event="trajectory_execute_failed")
            self._set_command_state(
                "failed",
                "execute_previewed_trajectory",
                f"轨迹执行失败：{response.message}",
                trajectory_ready=True,
                trajectory_visible=False,
            )
        return result

    def fk_joint_to_pose(self, joint_positions: Dict[str, float]) -> Dict[str, Any]:
        if not self._service_available(self._fk_client):
            raise RuntimeError("fk_joint_to_pose service 不在线")
        request = FkJointToPose.Request()
        request.joint_positions = [float(joint_positions[name]) for name in JOINT_NAMES]
        response = self._call_service(self._fk_client, request, timeout_sec=5.0)
        result = {
            "success": bool(response.success),
            "message": response.message,
            "pose": _pose_to_dict(response.pose),
        }
        if not response.success:
            self.add_error(f"FK 失败：{response.message}", event="fk_failed")
        return result

    def set_trajectory_display(self, show: bool) -> Dict[str, Any]:
        if not self._service_available(self._trajectory_display_client):
            raise RuntimeError("set_trajectory_display service 不在线")
        request = SetTrajectoryDisplay.Request()
        request.show = bool(show)
        response = self._call_service(self._trajectory_display_client, request, timeout_sec=5.0)
        result = {
            "success": bool(response.success),
            "error_code": int(response.error_code),
            "message": response.message,
        }
        if response.success:
            self.add_log(
                "info",
                "已展示最近规划轨迹" if show else "已暂停轨迹展示",
                event="trajectory_display",
                details={"show": show},
            )
            self._set_command_state(
                self._command_state.phase,
                self._command_state.active_command,
                "正在展示最近规划轨迹" if show else "已暂停轨迹展示",
                trajectory_visible=show,
            )
        else:
            self.add_error(f"轨迹显示切换失败：{response.message}", event="trajectory_display_failed")
        return result

    def jog(self, request: JogRequest) -> Dict[str, Any]:
        self._assert_motion_allowed("manual_teleop")
        self._reject_if_command_busy("手动点动")
        if request.joint_name not in JOINT_NAMES:
            raise ValueError(f"不支持的关节：{request.joint_name}")
        if request.step <= 0.0 or not math.isfinite(request.step):
            raise ValueError("步长必须是正的有限数字")
        if request.velocity_scale <= 0.0 or request.velocity_scale > 1.0:
            raise ValueError("速度倍率必须在 (0, 1] 范围内")
        if not request.deadman:
            raise RuntimeError("手动点动前必须启用 deadman")
        if not self._action_available():
            raise RuntimeError("follow_joint_trajectory action 不在线")
        with self._lock:
            if self._active_goal_handle is not None:
                raise RuntimeError("上一个 Web 点动目标仍在执行，请先停止或等待完成")

        q_target = self._make_jog_target(request)
        target_by_name = dict(zip(JOINT_NAMES, q_target))
        self.validate_joint_target_delta(target_by_name)
        duration_sec = _clamp(0.5 / request.velocity_scale, 0.2, 5.0)
        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = list(JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = q_target
        point.time_from_start = _duration_from_seconds(duration_sec)
        trajectory.points = [point]
        if self._node is not None:
            trajectory.header.stamp = self._node.get_clock().now().to_msg()
        goal_msg.trajectory = trajectory

        goal_handle = self._send_trajectory_goal(goal_msg, timeout_sec=3.0)
        self.set_target_joint_positions(target_by_name)
        self._set_command_state("manual_jogging", "manual_jog", "正在手动点动")
        self.add_log(
            "info",
            (
                f"请求手动点动：{request.joint_name} "
                f"方向={request.direction} 步长={request.step} 目标={q_target}"
            ),
            event="manual_jog_request",
            details={
                "joint_name": request.joint_name,
                "direction": request.direction,
                "step": request.step,
                "velocity_scale": request.velocity_scale,
                "target": target_by_name,
            },
        )
        return {
            "success": True,
            "message": f"手动点动目标已发送，goal_id={goal_handle.goal_id.uuid}",
        }

    def stop_manual(self) -> Dict[str, Any]:
        if self._control_mode == "estop":
            return {"success": True, "message": "当前已处于 estop；没有活动点动输出。"}
        goal_handle = None
        with self._lock:
            goal_handle = self._active_goal_handle
        if goal_handle is None:
            self.add_log("info", "请求手动停止：当前没有 Web 点动目标", event="manual_stop")
            self._set_command_state("stopped", None, "当前没有活动的 Web 点动目标")
            return {"success": True, "message": "当前没有活动的 Web 点动目标。"}
        cancel_future = goal_handle.cancel_goal_async()
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            if cancel_future.done():
                self.add_log("info", "已请求取消 Web 点动目标", event="manual_stop")
                with self._lock:
                    self._active_goal_handle = None
                self._set_command_state("stopped", None, "已请求取消当前 Web 点动目标")
                return {"success": True, "message": "已请求取消当前 Web 点动目标。"}
            time.sleep(0.02)
        raise TimeoutError("取消 Web 点动目标超时")

    def set_target_joint_positions(self, target: Dict[str, float]) -> None:
        normalized = {name: float(target[name]) for name in JOINT_NAMES if name in target}
        if len(normalized) != len(JOINT_NAMES):
            missing = [name for name in JOINT_NAMES if name not in normalized]
            raise ValueError(f"目标关节缺少：{', '.join(missing)}")
        with self._lock:
            self._target_joint_positions = normalized

    def validate_joint_target_delta(self, target: Dict[str, float]) -> None:
        with self._lock:
            if self._last_joint_states_at is None:
                raise ValueError("尚未收到 joint_states，禁止发送关节目标")
            missing = [name for name in JOINT_NAMES if name not in target]
            if missing:
                raise ValueError(f"目标关节缺少：{', '.join(missing)}")
            for name in JOINT_NAMES:
                current = self._joint_positions.get(name)
                goal = target[name]
                if current is None or not math.isfinite(current):
                    raise ValueError(f"{name} 当前反馈角无效，禁止发送关节目标")
                if not math.isfinite(goal):
                    raise ValueError(f"{name} 目标角无效，禁止发送关节目标")
                delta = abs(goal - current)
                if delta > MAX_TARGET_DELTA_RAD:
                    message = (
                        f"{name} 目标跳变过大：当前 {current:.6f} rad，"
                        f"目标 {goal:.6f} rad，差值 {delta:.6f} rad，"
                        f"阈值 {MAX_TARGET_DELTA_RAD:.6f} rad"
                    )
                    self.add_log(
                        "safety",
                        message,
                        event="target_delta_rejected",
                        details={
                            "joint_name": name,
                            "current_rad": current,
                            "target_rad": goal,
                            "delta_rad": delta,
                            "threshold_rad": MAX_TARGET_DELTA_RAD,
                        },
                    )
                    raise ValueError(message)

    def _set_command_state(
        self,
        phase: str,
        active_command: Optional[str],
        message: str,
        trajectory_ready: Optional[bool] = None,
        trajectory_visible: Optional[bool] = None,
    ) -> None:
        now = time.time()
        with self._lock:
            previous_phase = self._command_state.phase
            started_at = self._command_state.started_at
            if phase in ("manual_jogging", "planning", "executing") and previous_phase != phase:
                started_at = now
            if phase in ("idle", "completed", "failed", "stopped"):
                started_at = None
            self._command_state = CommandState(
                phase=phase,
                active_command=active_command,
                trajectory_ready=(
                    self._command_state.trajectory_ready
                    if trajectory_ready is None
                    else trajectory_ready
                ),
                trajectory_visible=(
                    self._command_state.trajectory_visible
                    if trajectory_visible is None
                    else trajectory_visible
                ),
                started_at=started_at,
                updated_at=now,
                message=message,
            )
        self.add_log(
            "info" if phase not in ("failed",) else "error",
            message,
            event="command_state_changed",
            details={
                "phase": phase,
                "active_command": active_command,
                "trajectory_ready": self._command_state.trajectory_ready,
                "trajectory_visible": self._command_state.trajectory_visible,
            },
        )

    def _reject_if_command_busy(self, requested_action: str) -> None:
        with self._lock:
            phase = self._command_state.phase
        if phase in ("planning", "executing", "manual_jogging"):
            message = f"当前命令状态为 {phase}，拒绝执行：{requested_action}"
            self.add_log(
                "safety",
                message,
                event="command_rejected_busy",
                details={"phase": phase, "requested_action": requested_action},
            )
            raise RuntimeError(message)

    def _validate_execute_allowed(self) -> None:
        with self._lock:
            phase = self._command_state.phase
            trajectory_ready = self._command_state.trajectory_ready
        if phase in ("planning", "executing", "manual_jogging"):
            message = f"当前命令状态为 {phase}，禁止执行轨迹"
            self.add_log("safety", message, event="command_rejected_busy")
            raise RuntimeError(message)
        if not trajectory_ready:
            message = "当前没有可执行的已规划轨迹，请先规划预览"
            self.add_log("safety", message, event="trajectory_not_ready")
            raise RuntimeError(message)

    def _validate_mode_switch_allowed(self, requested_mode: str) -> None:
        if requested_mode == "estop":
            return
        with self._lock:
            phase = self._command_state.phase
        if phase == "executing":
            message = "轨迹执行中禁止切换普通模式；如需中断请使用 estop"
            self.add_log(
                "safety",
                message,
                event="mode_change_rejected_busy",
                details={"phase": phase, "requested_mode": requested_mode},
            )
            raise RuntimeError(message)
        if phase == "manual_jogging":
            message = "手动点动中禁止直接切换模式，请先停止手动控制或使用 estop"
            self.add_log(
                "safety",
                message,
                event="mode_change_rejected_busy",
                details={"phase": phase, "requested_mode": requested_mode},
            )
            raise RuntimeError(message)

    def add_log(
        self,
        level: str,
        message: str,
        event: str = "operation",
        source: str = "web_hmi",
        details: Optional[Dict[str, Any]] = None,
    ) -> None:
        normalized_level = "info" if level == "operation" else level
        item = {
            "time": _utc_now(),
            "level": normalized_level,
            "event": event,
            "message": message,
            "source": source,
            "details": details,
        }
        with self._lock:
            self._logs.append(item)
        self._persist_operation_log(normalized_level, event, message, source, details)

    def add_error(
        self,
        message: str,
        event: str = "error",
        source: str = "web_hmi",
        details: Optional[Dict[str, Any]] = None,
    ) -> None:
        item = {
            "time": _utc_now(),
            "level": "error",
            "event": event,
            "message": message,
            "source": source,
            "details": details,
        }
        with self._lock:
            self._logs.append(item)
            self._errors.append(item)
        self._persist_operation_log("error", event, message, source, details)

    def _persist_operation_log(
        self,
        level: str,
        event: str,
        message: str,
        source: str,
        details: Optional[Dict[str, Any]],
    ) -> None:
        try:
            append_operation_log(
                OperationLog(
                    timestamp=time.time(),
                    level=level,
                    event=event,
                    message=message,
                    source=source,
                    details=details,
                )
            )
        except Exception as exc:  # noqa: BLE001
            item = {
                "time": _utc_now(),
                "level": "error",
                "event": "operation_log_persist_failed",
                "message": f"操作日志落盘失败：{exc}",
                "source": "web_hmi",
                "details": None,
            }
            with self._lock:
                self._errors.append(item)

    def _assert_motion_allowed(self, required_mode: str) -> None:
        status = self.get_status()
        if not status["ros_connected"]:
            self.add_error("ROS 未连接", event="motion_rejected_offline")
            raise RuntimeError("ROS 未连接")
        if status["control_mode"] == "estop":
            self.add_log("safety", "当前为 estop 模式，禁止运动", event="motion_rejected_estop")
            raise RuntimeError("当前为 estop 模式，禁止运动")
        if status["control_mode"] != required_mode:
            self.add_log(
                "safety",
                f"控制模式必须为 {required_mode}，当前为 {status['control_mode']}",
                event="motion_rejected_mode",
                details={"required_mode": required_mode, "current_mode": status["control_mode"]},
            )
            raise RuntimeError(
                f"控制模式必须为 {required_mode}，当前为 {status['control_mode']}"
            )

    def _build_joints_runtime_locked(self, now: float) -> List[Dict[str, Any]]:
        joint_age = self._age_or_none(now, self._last_joint_states_at)
        items = []
        for name in JOINT_NAMES:
            position = self._joint_positions.get(name)
            target = self._target_joint_positions.get(name) if self._target_joint_positions else None
            error = None
            if position is not None and target is not None:
                error = target - position
            lower, upper = JOINT_LIMITS[name]
            within_limit = None
            if position is not None:
                within_limit = lower <= position <= upper
            items.append(
                JointRuntimeState(
                    name=name,
                    position_rad=position,
                    target_rad=target,
                    error_rad=error,
                    min_rad=lower,
                    max_rad=upper,
                    within_limit=within_limit,
                    last_update_sec=joint_age,
                ).dict()
            )
        return items

    def _make_jog_target(self, request: JogRequest) -> List[float]:
        with self._lock:
            if self._last_joint_states_at is None:
                self.add_log("safety", "尚未收到 joint_states，禁止点动", event="joint_states_timeout")
                raise RuntimeError("尚未收到 joint_states，禁止点动")
            age_sec = time.time() - self._last_joint_states_at
            if age_sec > 2.0:
                message = f"joint_states 已超过 {age_sec:.1f} 秒未更新，禁止点动"
                self.add_log("safety", message, event="joint_states_timeout", details={"age_sec": age_sec})
                raise RuntimeError(message)
            q_target = [self._joint_positions[name] for name in JOINT_NAMES]
        joint_index = JOINT_NAMES.index(request.joint_name)
        lower, upper = JOINT_LIMITS[request.joint_name]
        q_target[joint_index] = _clamp(
            q_target[joint_index] + request.direction * request.step,
            lower,
            upper,
        )
        return q_target

    def _send_trajectory_goal(
        self,
        goal_msg: FollowJointTrajectory.Goal,
        timeout_sec: float,
    ) -> Any:
        if self._trajectory_action_client is None:
            raise RuntimeError("follow_joint_trajectory action client 未初始化")
        future = self._trajectory_action_client.send_goal_async(goal_msg)
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if future.done():
                goal_handle = future.result()
                if goal_handle is None or not goal_handle.accepted:
                    raise RuntimeError("follow_joint_trajectory action 拒绝了点动目标")
                with self._lock:
                    self._active_goal_handle = goal_handle
                goal_handle.get_result_async().add_done_callback(self._on_jog_result)
                return goal_handle
            time.sleep(0.02)
        raise TimeoutError("发送点动 action goal 超时")

    def _on_jog_result(self, future: Any) -> None:
        try:
            result = future.result()
            status_text = self._goal_status_text(result.status)
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self.add_log("operation", f"Web 点动目标完成：{status_text}")
                self._set_command_state("completed", "manual_jog", f"Web 点动目标完成：{status_text}")
            else:
                self.add_error(f"Web 点动目标结束：{status_text}")
                self._set_command_state("failed", "manual_jog", f"Web 点动目标结束：{status_text}")
        except Exception as exc:  # noqa: BLE001
            self.add_error(f"Web 点动目标结果处理失败：{exc}")
            self._set_command_state("failed", "manual_jog", f"Web 点动目标结果处理失败：{exc}")
        finally:
            with self._lock:
                self._active_goal_handle = None

    @staticmethod
    def _goal_status_text(status: int) -> str:
        status_map = {
            GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
            GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
            GoalStatus.STATUS_EXECUTING: "EXECUTING",
            GoalStatus.STATUS_CANCELING: "CANCELING",
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
        }
        return status_map.get(status, f"UNKNOWN_STATUS_{status}")

    def _on_control_mode(self, msg: String) -> None:
        with self._lock:
            self._control_mode = msg.data.strip() or "unknown"
            self._last_control_mode_at = time.time()

    def _on_joint_states(self, msg: JointState) -> None:
        positions = {}
        for joint_name in JOINT_NAMES:
            if joint_name in msg.name:
                index = msg.name.index(joint_name)
                if index < len(msg.position) and math.isfinite(msg.position[index]):
                    positions[joint_name] = float(msg.position[index])
        if not positions:
            return
        with self._lock:
            self._joint_positions.update(positions)
            self._last_joint_states_at = time.time()

    def _on_controller_state(self, _msg: JointTrajectoryControllerState) -> None:
        with self._lock:
            self._last_controller_state_at = time.time()

    def _service_available(self, client: Optional[Any]) -> bool:
        if client is None:
            return False
        try:
            return bool(client.wait_for_service(timeout_sec=0.0))
        except Exception as exc:  # noqa: BLE001
            self.add_error(f"service 检查失败：{exc}", event="service_check_failed")
            return False

    def _action_available(self) -> bool:
        if self._trajectory_action_client is None:
            return False
        try:
            return bool(self._trajectory_action_client.wait_for_server(timeout_sec=0.0))
        except Exception as exc:  # noqa: BLE001
            self.add_error(f"action 检查失败：{exc}", event="action_check_failed")
            return False

    def _call_service(self, client: Any, request: Any, timeout_sec: float) -> Any:
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if future.done():
                result = future.result()
                if result is None:
                    raise RuntimeError("service 未返回响应")
                return result
            time.sleep(0.02)
        raise TimeoutError("service 调用超时")

    @staticmethod
    def _age_or_none(now: float, timestamp: Optional[float]) -> Optional[float]:
        if timestamp is None:
            return None
        return max(0.0, now - timestamp)

    @staticmethod
    def _is_recent(age_sec: Optional[float], max_age_sec: float) -> bool:
        return age_sec is not None and age_sec <= max_age_sec
