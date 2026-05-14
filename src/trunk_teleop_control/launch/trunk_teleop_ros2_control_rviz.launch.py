import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


class NoAliasDumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True


def launch_setup(context, robot_description, spawn_controllers_launch, *args, **kwargs):
    del args, kwargs

    namespace_value = LaunchConfiguration("namespace").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time")
    ros2_control_params_file = LaunchConfiguration("ros2_control_params_file").perform(context)
    default_rviz_config = LaunchConfiguration("rviz_config").perform(context)
    ros2_control_params_file = expand_ros2_control_params(
        ros2_control_params_file, namespace_value)

    return [
        GroupAction(
            [
                PushRosNamespace(LaunchConfiguration("namespace")),
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name="static_transform_publisher0",
                    output="screen",
                    arguments=[
                        "--x",
                        "0",
                        "--y",
                        "0",
                        "--z",
                        "0",
                        "--roll",
                        "0",
                        "--pitch",
                        "0",
                        "--yaw",
                        "0",
                        "--frame-id",
                        "world",
                        "--child-frame-id",
                        "chassis_base_link",
                    ],
                ),
                Node(
                    package="robot_state_publisher",
                    executable="robot_state_publisher",
                    name="robot_state_publisher",
                    output="screen",
                    parameters=[
                        robot_description,
                        {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                    ],
                ),
                Node(
                    package="controller_manager",
                    executable="ros2_control_node",
                    output="screen",
                    parameters=[
                        robot_description,
                        ros2_control_params_file,
                        {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                    ],
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(spawn_controllers_launch),
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
                            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
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


def expand_ros2_control_params(params_file, namespace_value):
    with open(params_file, "r", encoding="utf-8") as file:
        params = yaml.safe_load(file)

    expanded = dict(params)
    for node_name, node_params in params.items():
        if node_name.endswith("/controller_manager"):
            expanded[f"/{namespace_value}/controller_manager"] = node_params
        elif node_name.endswith("/trunk_group_controller"):
            expanded[f"/{namespace_value}/trunk_group_controller"] = node_params
            expanded[f"/{namespace_value}/controller_manager/trunk_group_controller"] = node_params
            expanded[f"/**/controller_manager/trunk_group_controller"] = node_params

    temp_file = tempfile.NamedTemporaryFile(
        mode="w", prefix="trunk_teleop_ros2_control_", suffix=".yaml", delete=False)
    with temp_file:
        yaml.dump(expanded, temp_file, Dumper=NoAliasDumper, sort_keys=False)
    return temp_file.name


def generate_launch_description():
    teleop_share = get_package_share_directory("trunk_teleop_control")
    trunk_configure_share = get_package_share_directory("trunk_configure")
    moveit_config = MoveItConfigsBuilder(
        "trunk_robot", package_name="trunk_configure"
    ).to_moveit_configs()

    default_teleop_params_file = os.path.join(
        teleop_share, "config", "trunk_joystick_teleop.yaml"
    )
    default_rviz_config = os.path.join(
        teleop_share, "config", "trunk_joystick_teleop.rviz"
    )
    default_ros2_control_params_file = os.path.join(
        trunk_configure_share, "config", "ros2_controllers.yaml"
    )
    spawn_controllers_launch = os.path.join(
        trunk_configure_share, "launch", "spawn_controllers.launch.py"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="trunk_robot",
                description="Namespace for the ros2_control teleop stack.",
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
                "ros2_control_params_file",
                default_value=default_ros2_control_params_file,
                description="ros2_control controller YAML file.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
                description="RViz config file.",
            ),
            DeclareLaunchArgument("joy_device_id", default_value="0"),
            DeclareLaunchArgument("joy_deadzone", default_value="0.05"),
            DeclareLaunchArgument("joy_autorepeat_rate", default_value="20.0"),
            OpaqueFunction(
                function=launch_setup,
                args=[moveit_config.robot_description, spawn_controllers_launch],
            ),
        ]
    )
