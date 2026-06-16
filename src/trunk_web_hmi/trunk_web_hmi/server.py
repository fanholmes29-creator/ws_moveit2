from __future__ import annotations

import asyncio
import os
from pathlib import Path
from typing import Any, Dict

import uvicorn
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse

from .models import (
    JogRequest,
    ModeRequest,
    PlanToPoseRequest,
    PlanningTargetRequest,
    WaypointCreateRequest,
)
from .operation_logs import read_operation_logs
from .planning import (
    build_preview,
    joint_positions_to_rad,
    resolve_planning_target,
    waypoint_to_target_request,
)
from .robot_config import get_robot_config
from .ros_bridge import TrunkRosBridge, validate_plan_request
from .waypoints import WaypointStore


def _default_waypoint_path() -> Path:
    override = os.environ.get("TRUNK_WEB_HMI_WAYPOINTS")
    if override:
        return Path(override).expanduser()
    return Path.home() / ".ros" / "trunk_web_hmi" / "waypoints.json"


def create_app() -> FastAPI:
    namespace = os.environ.get("TRUNK_ROBOT_NAMESPACE", "trunk_robot")
    bridge = TrunkRosBridge(namespace=namespace, waypoint_path=_default_waypoint_path())
    waypoints = WaypointStore(_default_waypoint_path())

    app = FastAPI(title="Trunk Web HMI", version="0.0.1")
    app.add_middleware(
        CORSMiddleware,
        allow_origins=os.environ.get("TRUNK_WEB_HMI_CORS", "*").split(","),
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )
    app.state.bridge = bridge
    app.state.waypoints = waypoints

    @app.on_event("startup")
    def _startup() -> None:
        bridge.start()

    @app.on_event("shutdown")
    def _shutdown() -> None:
        bridge.stop()

    @app.get("/api/status")
    def get_status() -> Dict[str, Any]:
        return bridge.get_status()

    @app.get("/api/robot/config")
    def get_robot_config_api() -> Dict[str, Any]:
        return get_robot_config().dict()

    @app.post("/api/mode")
    def post_mode(request: ModeRequest) -> Dict[str, Any]:
        try:
            return bridge.set_mode(request.mode)
        except ValueError as exc:
            bridge.add_error(str(exc))
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        except Exception as exc:  # noqa: BLE001
            bridge.add_error(str(exc))
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    def _hardware_not_implemented(action_name: str) -> JSONResponse:
        message = (
            f"{action_name} 尚未接入真实硬件接口。"
            "当前请使用 control_mode 做 ROS 层运动门控。"
        )
        bridge.add_log(
            "warning",
            message,
            event="not_implemented",
            details={"action": action_name},
        )
        return JSONResponse(
            status_code=501,
            content={
                "ok": False,
                "implemented": False,
                "message": message,
            },
        )

    @app.post("/api/robot/enable")
    def post_robot_enable() -> JSONResponse:
        return _hardware_not_implemented("机器人使能")

    @app.post("/api/robot/disable")
    def post_robot_disable() -> JSONResponse:
        return _hardware_not_implemented("机器人下使能")

    @app.post("/api/robot/reset_error")
    def post_robot_reset_error() -> JSONResponse:
        return _hardware_not_implemented("错误复位")

    @app.post("/api/plan_to_pose")
    def post_plan_to_pose(request: PlanToPoseRequest) -> Dict[str, Any]:
        try:
            validate_plan_request(request)
            return bridge.plan_to_pose(request)
        except ValueError as exc:
            bridge.add_error(str(exc))
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        except Exception as exc:  # noqa: BLE001
            bridge.add_error(str(exc))
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    def _resolve_waypoint_request(request: PlanningTargetRequest) -> PlanningTargetRequest:
        if request.target_type != "waypoint":
            return request
        if not request.waypoint_id:
            raise ValueError("waypoint_id 不能为空")
        for waypoint in waypoints.list_waypoints():
            if waypoint.id == request.waypoint_id:
                return waypoint_to_target_request(request, waypoint)
        raise ValueError("点位不存在")

    def _pose_dict_to_plan_request(pose: Dict[str, Any]) -> PlanToPoseRequest:
        position = pose["position"]
        orientation = pose["orientation"]
        return PlanToPoseRequest(
            use_external_target=True,
            target_position=[position["x"], position["y"], position["z"]],
            target_orientation=[
                orientation["x"],
                orientation["y"],
                orientation["z"],
                orientation["w"],
            ],
        )

    def _target_to_plan_request(request: PlanningTargetRequest) -> PlanToPoseRequest:
        if request.target_type == "joint":
            joint_positions_rad = joint_positions_to_rad(
                request.joint_positions,
                request.units.joint,
            )
            bridge.validate_joint_target_delta(joint_positions_rad)
            fk_result = bridge.fk_joint_to_pose(joint_positions_rad)
            if not fk_result["success"]:
                raise RuntimeError(f"FK failed for joint target: {fk_result['message']}")
            return _pose_dict_to_plan_request(fk_result["pose"])
        target_position, target_orientation, use_external_target, _ = resolve_planning_target(
            request
        )
        return PlanToPoseRequest(
            use_external_target=use_external_target,
            target_position=target_position or [0.0, 0.0, 0.0],
            target_orientation=target_orientation or [0.0, 0.0, 0.0, 1.0],
        )

    def _enrich_joint_preview(
        request: PlanningTargetRequest,
        preview: Dict[str, Any],
    ) -> Dict[str, Any]:
        if request.target_type != "joint":
            return preview
        joint_positions_rad = joint_positions_to_rad(request.joint_positions, request.units.joint)
        bridge.validate_joint_target_delta(joint_positions_rad)
        fk_result = bridge.fk_joint_to_pose(joint_positions_rad)
        if not fk_result["success"]:
            preview["success"] = False
            preview["message"] = f"FK failed for joint target: {fk_result['message']}"
            preview["checks"].append(
                {"name": "FK 转换", "ok": False, "message": fk_result["message"]}
            )
            return preview
        pose = fk_result["pose"]
        preview["target_position"] = [
            pose["position"]["x"],
            pose["position"]["y"],
            pose["position"]["z"],
        ]
        preview["target_orientation"] = [
            pose["orientation"]["x"],
            pose["orientation"]["y"],
            pose["orientation"]["z"],
            pose["orientation"]["w"],
        ]
        preview["checks"].append({"name": "FK 转换结果", "ok": True, "message": "FK 成功"})
        return preview

    @app.post("/api/planning/target_preview")
    def post_target_preview(request: PlanningTargetRequest) -> Dict[str, Any]:
        try:
            resolved_request = _resolve_waypoint_request(request)
            preview = build_preview(resolved_request, bridge.get_status()).dict()
            return _enrich_joint_preview(resolved_request, preview)
        except ValueError as exc:
            bridge.add_error(str(exc))
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        except Exception as exc:  # noqa: BLE001
            bridge.add_error(str(exc))
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @app.post("/api/planning/plan_to_target")
    def post_plan_to_target(request: PlanningTargetRequest) -> Dict[str, Any]:
        try:
            resolved_request = _resolve_waypoint_request(request)
            preview = build_preview(resolved_request, bridge.get_status())
            if not preview.success:
                return {
                    "success": False,
                    "error_code": -1000,
                    "message": preview.message,
                    "preview": preview.dict(),
                }
            plan_request = _target_to_plan_request(resolved_request)
            result = bridge.plan_to_pose(plan_request)
            result["target_type"] = resolved_request.target_type
            result["converted_target_position"] = plan_request.target_position
            result["converted_target_orientation"] = plan_request.target_orientation
            result["preview"] = preview.dict()
            if result.get("success") and resolved_request.target_type == "joint":
                bridge.set_target_joint_positions(
                    joint_positions_to_rad(resolved_request.joint_positions, resolved_request.units.joint)
                )
            return result
        except NotImplementedError as exc:
            return {"success": False, "error_code": -1001, "message": str(exc)}
        except ValueError as exc:
            bridge.add_error(str(exc))
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        except Exception as exc:  # noqa: BLE001
            bridge.add_error(str(exc))
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @app.post("/api/planning/preview_target")
    def post_preview_target(request: PlanningTargetRequest) -> Dict[str, Any]:
        try:
            resolved_request = _resolve_waypoint_request(request)
            preview = build_preview(resolved_request, bridge.get_status())
            if not preview.success:
                return {
                    "success": False,
                    "error_code": -1000,
                    "message": preview.message,
                    "preview": preview.dict(),
                }
            plan_request = _target_to_plan_request(resolved_request)
            result = bridge.preview_plan_to_pose(plan_request)
            result["target_type"] = resolved_request.target_type
            result["converted_target_position"] = plan_request.target_position
            result["converted_target_orientation"] = plan_request.target_orientation
            result["preview"] = preview.dict()
            if result.get("success") and resolved_request.target_type == "joint":
                bridge.set_target_joint_positions(
                    joint_positions_to_rad(resolved_request.joint_positions, resolved_request.units.joint)
                )
            return result
        except ValueError as exc:
            bridge.add_error(str(exc))
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        except Exception as exc:  # noqa: BLE001
            bridge.add_error(str(exc))
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @app.post("/api/planning/execute_previewed_trajectory")
    def post_execute_previewed_trajectory() -> Dict[str, Any]:
        try:
            return bridge.execute_previewed_trajectory()
        except Exception as exc:  # noqa: BLE001
            bridge.add_error(str(exc))
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @app.post("/api/planning/trajectory_display")
    def post_trajectory_display(payload: Dict[str, Any]) -> Dict[str, Any]:
        try:
            return bridge.set_trajectory_display(bool(payload.get("show", False)))
        except Exception as exc:  # noqa: BLE001
            bridge.add_error(str(exc))
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @app.post("/api/manual/jog")
    def post_manual_jog(request: JogRequest) -> Dict[str, Any]:
        try:
            return bridge.jog(request)
        except ValueError as exc:
            bridge.add_error(str(exc))
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        except Exception as exc:  # noqa: BLE001
            bridge.add_error(str(exc))
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @app.post("/api/manual/stop")
    def post_manual_stop() -> Dict[str, Any]:
        return bridge.stop_manual()

    @app.get("/api/waypoints")
    def get_waypoints() -> Dict[str, Any]:
        return {"items": [item.dict() for item in waypoints.list_waypoints()]}

    @app.post("/api/waypoints")
    def post_waypoint(request: WaypointCreateRequest) -> Dict[str, Any]:
        if not request.name.strip():
            raise HTTPException(status_code=400, detail="请输入点位名称")
        if request.kind == "joint" and not request.joint_positions:
            raise HTTPException(status_code=400, detail="保存关节点位需要 joint_positions")
        if request.kind in ("pose", "pose_quaternion", "pose_rpy") and not request.target_pose:
            raise HTTPException(status_code=400, detail="保存位姿点位需要 target_pose")
        record = waypoints.create_waypoint(request)
        bridge.add_log("operation", f"点位已保存：{record.name}")
        return record.dict()

    @app.delete("/api/waypoints/{waypoint_id}")
    def delete_waypoint(waypoint_id: str) -> Dict[str, Any]:
        if not waypoints.delete_waypoint(waypoint_id):
            raise HTTPException(status_code=404, detail="点位不存在")
        bridge.add_log("operation", f"点位已删除：{waypoint_id}")
        return {"success": True}

    @app.get("/api/logs")
    def get_logs() -> Dict[str, Any]:
        return bridge.get_logs()

    @app.get("/api/operation_logs")
    def get_operation_logs(limit: int = 200, level: str | None = None) -> Dict[str, Any]:
        try:
            return {"items": [item.dict() for item in read_operation_logs(limit=limit, level=level)]}
        except Exception as exc:  # noqa: BLE001
            bridge.add_error(f"读取持久化操作日志失败：{exc}", event="operation_log_read_failed")
            raise HTTPException(status_code=500, detail=str(exc)) from exc

    @app.websocket("/ws/state")
    async def websocket_state(websocket: WebSocket) -> None:
        await websocket.accept()
        try:
            while True:
                await websocket.send_json(bridge.get_status())
                await asyncio.sleep(0.5)
        except WebSocketDisconnect:
            return

    return app


app = create_app()


def main() -> None:
    host = os.environ.get("TRUNK_WEB_HMI_HOST", "0.0.0.0")
    port = int(os.environ.get("TRUNK_WEB_HMI_PORT", "8000"))
    uvicorn.run("trunk_web_hmi.server:app", host=host, port=port, reload=False)


if __name__ == "__main__":
    main()
