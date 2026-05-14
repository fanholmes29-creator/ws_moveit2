import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    teleop_share = get_package_share_directory("trunk_teleop_control")
    trunk_configure_share = get_package_share_directory("trunk_configure")

    default_teleop_params_file = os.path.join(
        teleop_share, "config", "trunk_joystick_teleop.yaml"
    )
    default_rviz_config = os.path.join(
        teleop_share, "config", "trunk_joystick_teleop.rviz"
    )
    trunk_demo_launch = os.path.join(trunk_configure_share, "launch", "demo.launch.py")

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="trunk_robot",
                description="Namespace for teleop and ros2_control stack.",
            ),
            DeclareLaunchArgument(
                "start_joy_node",
                default_value="true",
                description="Whether to start joy_node.",
            ),
            DeclareLaunchArgument(
                "start_control_mode_manager",
                default_value="true",
                description="Whether to start control_mode_manager.",
            ),
            DeclareLaunchArgument(
                "initial_control_mode",
                default_value="manual_teleop",
                description="Initial control mode published by control_mode_manager.",
            ),
            DeclareLaunchArgument(
                "start_rviz",
                default_value="true",
                description="Whether to start RViz.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time.",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="Log level for trunk_joystick_teleop.",
            ),
            DeclareLaunchArgument(
                "teleop_params_file",
                default_value=default_teleop_params_file,
                description="Parameter file for trunk_joystick_teleop.",
            ),
            DeclareLaunchArgument(
                "joy_device_id",
                default_value="0",
                description="Joystick device id for joy_node.",
            ),
            DeclareLaunchArgument(
                "joy_deadzone",
                default_value="0.05",
                description="Deadzone for joy_node.",
            ),
            DeclareLaunchArgument(
                "joy_autorepeat_rate",
                default_value="20.0",
                description="Autorepeat rate for joy_node.",
            ),
            GroupAction(
                [
                    PushRosNamespace(namespace),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(trunk_demo_launch),
                        launch_arguments={
                            "use_rviz": "false",
                            "use_sim_time": use_sim_time,
                        }.items(),
                    ),
                    Node(
                        package="joy",
                        executable="joy_node",
                        name="joy_node",
                        output="screen",
                        condition=IfCondition(LaunchConfiguration("start_joy_node")),
                        parameters=[
                            {
                                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
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
                    Node(
                        package="trunk_teleop_control",
                        executable="control_mode_manager",
                        name="control_mode_manager",
                        output="screen",
                        condition=IfCondition(
                            LaunchConfiguration("start_control_mode_manager")
                        ),
                        parameters=[
                            {
                                "use_sim_time": ParameterValue(
                                    use_sim_time, value_type=bool
                                ),
                                "initial_mode": LaunchConfiguration("initial_control_mode"),
                                "state_topic": "control_mode_state",
                                "set_mode_service": "set_control_mode",
                                "follow_joint_trajectory_action": (
                                    "trunk_group_controller/follow_joint_trajectory"
                                ),
                            }
                        ],
                    ),
                    Node(
                        package="trunk_teleop_control",
                        executable="trunk_joystick_teleop",
                        name="trunk_joystick_teleop",
                        output="screen",
                        parameters=[
                            LaunchConfiguration("teleop_params_file"),
                            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                        ],
                        arguments=[
                            "--ros-args",
                            "--log-level",
                            LaunchConfiguration("log_level"),
                        ],
                    ),
                ]
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_rviz")),
                parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
                arguments=["-d", default_rviz_config],
            ),
        ]
    )
