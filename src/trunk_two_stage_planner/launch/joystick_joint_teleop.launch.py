import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("trunk_two_stage_planner")
    default_params_file = os.path.join(
        package_share, "config", "joystick_joint_teleop.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="YAML parameter file for joystick_joint_teleop.",
            ),
            DeclareLaunchArgument("robot_namespace", default_value="trunk_robot"),
            DeclareLaunchArgument("start_joy_node", default_value="true"),
            DeclareLaunchArgument("joy_device_id", default_value="0"),
            DeclareLaunchArgument("joy_deadzone", default_value="0.05"),
            DeclareLaunchArgument("joy_autorepeat_rate", default_value="20.0"),
            DeclareLaunchArgument("teleop_delay_sec", default_value="1.0"),
            DeclareLaunchArgument("log_level", default_value="info"),
            PushRosNamespace(LaunchConfiguration("robot_namespace")),
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_joy_node")),
                parameters=[
                    {
                        "device_id": ParameterValue(
                            LaunchConfiguration("joy_device_id"), value_type=int
                        ),
                        "deadzone": ParameterValue(
                            LaunchConfiguration("joy_deadzone"), value_type=float
                        ),
                        "autorepeat_rate": ParameterValue(
                            LaunchConfiguration("joy_autorepeat_rate"), value_type=float
                        ),
                    }
                ],
            ),
            TimerAction(
                period=LaunchConfiguration("teleop_delay_sec"),
                actions=[
                    Node(
                        package="trunk_two_stage_planner",
                        executable="joystick_joint_teleop",
                        namespace=[TextSubstitution(text="/")],
                        name="joystick_joint_teleop",
                        output="screen",
                        parameters=[LaunchConfiguration("params_file")],
                        arguments=[
                            "--ros-args",
                            "--log-level",
                            LaunchConfiguration("log_level"),
                        ],
                    )
                ],
            ),
        ]
    )
