"""Bring up the same stack as trunk_configure/demo plus the C++ demo node.

The demo executable must receive MoveIt parameters (especially robot_description_semantic /
SRDF); running `ros2 run trunk_moveit_cpp_demo trunk_moveit_cpp_demo` alone does not load
SRDF, so MoveGroupInterface reports: Group 'trunk_group' was not found.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("trunk_robot", package_name="trunk_configure")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )

    trunk_share = get_package_share_directory("trunk_configure")
    demo_share = get_package_share_directory("trunk_moveit_cpp_demo")
    demo_params = os.path.join(demo_share, "config", "trunk_demo_params.yaml")

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(trunk_share, "launch", "demo.launch.py")
                )
            ),
            # Start the demo node slightly later so move_group action/services
            # are up, and ensure logs flush to the terminal.
            TimerAction(
                # demo.launch.py also brings up ros2_control + controllers; give it time.
                period=8.0,
                actions=[
                    Node(
                        package="trunk_moveit_cpp_demo",
                        executable="trunk_moveit_cpp_demo",
                        output="screen",
                        emulate_tty=True,
                        arguments=["--ros-args", "--log-level", "info"],
                        parameters=[moveit_config.to_dict(), demo_params],
                    ),
                ],
            ),
        ]
    )
