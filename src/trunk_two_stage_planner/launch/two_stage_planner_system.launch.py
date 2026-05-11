import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # 启动职责：
    # - 启动 trunk_configure 的 demo 栈（关闭其自带 RViz）
    # - 按需启动两阶段系统专用 RViz
    # - 延时启动 manager 节点，等待 move_group 栈就绪
    moveit_config = (
        MoveItConfigsBuilder("trunk_robot", package_name="trunk_configure")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )

    trunk_config_share = get_package_share_directory("trunk_configure")
    system_share = get_package_share_directory("trunk_two_stage_planner")

    # 待办：如需命名完全统一，可将这些文件名改为 planner_* 风格。
    default_rviz_config = os.path.join(system_share, "config", "two_stage_system_moveit.rviz")
    params_yaml = os.path.join(system_share, "config", "two_stage_system_params.yaml")
    robot_namespace = LaunchConfiguration("robot_namespace")
    absolute_robot_namespace = [TextSubstitution(text="/"), robot_namespace]

    rviz_params = [
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("system_use_rviz", default_value="true"),
            DeclareLaunchArgument("system_rviz_config", default_value=default_rviz_config),
            DeclareLaunchArgument("rviz_delay_sec", default_value="30.0"),
            DeclareLaunchArgument("manager_delay_sec", default_value="60.0"),
            DeclareLaunchArgument("joint_state_wait_timeout_sec", default_value="120.0"),
            DeclareLaunchArgument("robot_namespace", default_value="trunk_robot"),
            GroupAction(
                [
                    PushRosNamespace(robot_namespace),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(trunk_config_share, "launch", "demo.launch.py")
                        ),
                        launch_arguments={"use_rviz": "false"}.items(),
                    ),
                    # RViz 启动较重，延后加载可避免干扰 controller spawner 和 planner 起步。
                    TimerAction(
                        period=LaunchConfiguration("rviz_delay_sec"),
                        actions=[
                            Node(
                                package="rviz2",
                                executable="rviz2",
                                namespace=absolute_robot_namespace,
                                arguments=["-d", LaunchConfiguration("system_rviz_config")],
                                output="screen",
                                parameters=rviz_params,
                                condition=IfCondition(LaunchConfiguration("system_use_rviz")),
                            )
                        ],
                    ),
                    TimerAction(
                        # 警告：
                        # 若延时过短，manager 可能在 move_group 完全就绪前发起规划。
                        period=LaunchConfiguration("manager_delay_sec"),
                        actions=[
                            Node(
                                package="trunk_two_stage_planner",
                                executable="two_stage_planner_system",
                                namespace=absolute_robot_namespace,
                                output="screen",
                                arguments=["--ros-args", "--log-level", "info"],
                                parameters=[
                                    moveit_config.to_dict(),
                                    params_yaml,
                                    {
                                        "joint_state_wait_timeout_sec": LaunchConfiguration(
                                            "joint_state_wait_timeout_sec"
                                        )
                                    },
                                ],
                            )
                        ],
                    ),
                ]
            ),
        ]
    )
