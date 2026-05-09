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
    moveit_config = (
        MoveItConfigsBuilder("trunk_robot", package_name="trunk_configure")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )

    trunk_config_share = get_package_share_directory("trunk_configure")
    system_share = get_package_share_directory("trunk_two_stage_planner")

    rviz_config = os.path.join(system_share, "config", "two_stage_system.rviz")
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
            DeclareLaunchArgument("service_use_rviz", default_value="true"),
            DeclareLaunchArgument("service_delay_sec", default_value="5.0"),
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
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        arguments=["-d", rviz_config],
                        output="screen",
                        parameters=rviz_params,
                        condition=IfCondition(LaunchConfiguration("service_use_rviz")),
                    ),
                    TimerAction(
                        period=LaunchConfiguration("service_delay_sec"),
                        actions=[
                            Node(
                                package="trunk_two_stage_planner",
                                executable="two_stage_planner_service",
                                namespace=absolute_robot_namespace,
                                output="screen",
                                arguments=["--ros-args", "--log-level", "info"],
                                parameters=[moveit_config.to_dict(), params_yaml],
                            )
                        ],
                    ),
                ]
            ),
        ]
    )
