from __future__ import annotations

from .models import JointConfig, RobotConfig


DEFAULT_JOINT_CONFIGS = [
    JointConfig(
        name="trunk_joint1",
        display_name="关节 1",
        group="trunk_group",
        index=0,
        min_rad=-1.39,
        max_rad=1.39,
    ),
    JointConfig(
        name="trunk_joint2",
        display_name="关节 2",
        group="trunk_group",
        index=1,
        min_rad=-1.74,
        max_rad=1.74,
    ),
    JointConfig(
        name="trunk_joint3",
        display_name="关节 3",
        group="trunk_group",
        index=2,
        min_rad=-1.74,
        max_rad=1.74,
    ),
    JointConfig(
        name="trunk_joint4",
        display_name="关节 4",
        group="trunk_group",
        index=3,
        min_rad=-3.14,
        max_rad=3.14,
    ),
]

DEFAULT_ROBOT_CONFIG = RobotConfig(
    robot_name="trunk_robot",
    robot_type="four_joint_trunk",
    modules=["trunk"],
    joints=DEFAULT_JOINT_CONFIGS,
)

MAX_TARGET_DELTA_RAD = 0.7


def get_robot_config() -> RobotConfig:
    return DEFAULT_ROBOT_CONFIG


def get_joint_names() -> list[str]:
    return [joint.name for joint in DEFAULT_JOINT_CONFIGS]


def get_joint_limits() -> dict[str, tuple[float, float]]:
    return {joint.name: (joint.min_rad, joint.max_rad) for joint in DEFAULT_JOINT_CONFIGS}
