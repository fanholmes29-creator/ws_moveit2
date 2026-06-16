import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


class NoAliasDumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True


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
        mode="w", prefix="trunk_real_ros2_control_", suffix=".yaml", delete=False)
    with temp_file:
        yaml.dump(expanded, temp_file, Dumper=NoAliasDumper, sort_keys=False)
    return temp_file.name


def launch_setup(context, *args, **kwargs):
    del args, kwargs

    namespace_value = LaunchConfiguration("namespace").perform(context)
    trunk_configure_share = get_package_share_directory("trunk_configure")
    teleop_share = get_package_share_directory("trunk_teleop_control")

    real_urdf_file = os.path.join(
        trunk_configure_share, "config", "trunk_robot.real.urdf.xacro")
    initial_positions_file = os.path.join(
        trunk_configure_share, "config", "initial_positions.yaml")
    ros2_control_params_file = expand_ros2_control_params(
        LaunchConfiguration("ros2_control_params_file").perform(context),
        namespace_value)
    default_rviz_config = LaunchConfiguration("rviz_config").perform(context)

    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_description = {
        "robot_description": Command(
            [
                "xacro ", real_urdf_file,
                " initial_positions_file:=", initial_positions_file,
                " transport:=", LaunchConfiguration("transport"),
                " port:=", LaunchConfiguration("port"),
                " baudrate:=", LaunchConfiguration("baudrate"),
                " can_interface:=", LaunchConfiguration("can_interface"),
                " ip:=", LaunchConfiguration("ip"),
                " dry_run:=", LaunchConfiguration("dry_run"),
                " command_timeout_ms:=", LaunchConfiguration("command_timeout_ms"),
                " require_homing:=", LaunchConfiguration("require_homing"),
                " require_absolute_encoder:=", LaunchConfiguration("require_absolute_encoder"),
                " max_position_error_rad:=", LaunchConfiguration("max_position_error_rad"),
            ]
        )
    }

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
                        "--x", "0", "--y", "0", "--z", "0",
                        "--roll", "0", "--pitch", "0", "--yaw", "0",
                        "--frame-id", "world", "--child-frame-id", "chassis_base_link",
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
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        "joint_state_broadcaster",
                        "trunk_group_controller",
                        "--controller-manager-timeout",
                        LaunchConfiguration("controller_manager_timeout"),
                        "--service-call-timeout",
                        LaunchConfiguration("controller_service_call_timeout"),
                        "--switch-timeout",
                        LaunchConfiguration("controller_switch_timeout"),
                        "--activate-as-group",
                    ],
                    output="screen",
                ),
                Node(
                    package="trunk_teleop_control",
                    executable="control_mode_manager",
                    name="control_mode_manager",
                    output="screen",
                    condition=IfCondition(LaunchConfiguration("start_control_mode_manager")),
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
                                LaunchConfiguration("joy_device_id"), value_type=int),
                            "deadzone": ParameterValue(
                                LaunchConfiguration("joy_deadzone"), value_type=float),
                            "autorepeat_rate": ParameterValue(
                                LaunchConfiguration("joy_autorepeat_rate"), value_type=float),
                        }
                    ],
                ),
                Node(
                    package="trunk_teleop_control",
                    executable="trunk_joystick_teleop",
                    name="trunk_joystick_teleop",
                    output="screen",
                    parameters=[
                        os.path.join(teleop_share, "config", "trunk_joystick_teleop.yaml"),
                        {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                    ],
                    arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
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


def generate_launch_description():
    trunk_configure_share = get_package_share_directory("trunk_configure")
    teleop_share = get_package_share_directory("trunk_teleop_control")

    default_ros2_control_params_file = os.path.join(
        trunk_configure_share, "config", "ros2_controllers.real.yaml")
    default_rviz_config = os.path.join(
        teleop_share, "config", "trunk_joystick_teleop.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="trunk_robot"),
            DeclareLaunchArgument(
                "ros2_control_params_file",
                default_value=default_ros2_control_params_file),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            DeclareLaunchArgument("start_joy_node", default_value="true"),
            DeclareLaunchArgument("start_control_mode_manager", default_value="true"),
            DeclareLaunchArgument("initial_control_mode", default_value="manual_teleop"),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument("joy_device_id", default_value="0"),
            DeclareLaunchArgument("joy_deadzone", default_value="0.05"),
            DeclareLaunchArgument("joy_autorepeat_rate", default_value="20.0"),
            DeclareLaunchArgument("controller_manager_timeout", default_value="60.0"),
            DeclareLaunchArgument("controller_service_call_timeout", default_value="60.0"),
            DeclareLaunchArgument("controller_switch_timeout", default_value="60.0"),
            DeclareLaunchArgument("transport", default_value="dry_run"),
            DeclareLaunchArgument("port", default_value=""),
            DeclareLaunchArgument("baudrate", default_value="1000000"),
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument("ip", default_value=""),
            DeclareLaunchArgument("dry_run", default_value="true"),
            DeclareLaunchArgument("command_timeout_ms", default_value="100"),
            DeclareLaunchArgument("require_homing", default_value="true"),
            DeclareLaunchArgument("require_absolute_encoder", default_value="true"),
            DeclareLaunchArgument("max_position_error_rad", default_value="0.2"),
            OpaqueFunction(function=launch_setup),
        ]
    )
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


class NoAliasDumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True


def expand_ros2_control_params(params_file, namespace_value):
    with open(params_file, "r", encoding="utf-8") as file:
        params = yaml.safe_load(file)

    expanded = dict(params)
    for node_name, node_params in params.items():
        if node_name.endswith("/controller_manager"):
            expanded[f"/{namespace_value}/controller_manager"] = node_params
        elif node_name.endswith("/trunk_group_controller"):
            expanded[f"/{namespace_value}/trunk_group_controller"] = node_params
            expanded[f"/{namespace_value}/controller_manager/trunk_group_controller"] = (
                node_params
            )
            expanded[f"/**/controller_manager/trunk_group_controller"] = node_params

    temp_file = tempfile.NamedTemporaryFile(
        mode="w", prefix="trunk_real_ros2_control_", suffix=".yaml", delete=False
    )
    with temp_file:
        yaml.dump(expanded, temp_file, Dumper=NoAliasDumper, sort_keys=False)
    return temp_file.name


def launch_setup(context, *args, **kwargs):
    del args, kwargs

    namespace_value = LaunchConfiguration("namespace").perform(context)
    trunk_configure_share = get_package_share_directory("trunk_configure")
    teleop_share = get_package_share_directory("trunk_teleop_control")

    real_urdf_file = os.path.join(
        trunk_configure_share, "config", "trunk_robot.real.urdf.xacro"
    )
    initial_positions_file = os.path.join(
        trunk_configure_share, "config", "initial_positions.yaml"
    )
    ros2_control_params_file = expand_ros2_control_params(
        LaunchConfiguration("ros2_control_params_file").perform(context),
        namespace_value,
    )
    default_rviz_config = LaunchConfiguration("rviz_config").perform(context)

    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_description = {
        "robot_description": Command(
            [
                "xacro ",
                real_urdf_file,
                " initial_positions_file:=",
                initial_positions_file,
                " transport:=",
                LaunchConfiguration("transport"),
                " port:=",
                LaunchConfiguration("port"),
                " baudrate:=",
                LaunchConfiguration("baudrate"),
                " can_interface:=",
                LaunchConfiguration("can_interface"),
                " ip:=",
                LaunchConfiguration("ip"),
                " dry_run:=",
                LaunchConfiguration("dry_run"),
                " command_timeout_ms:=",
                LaunchConfiguration("command_timeout_ms"),
                " require_homing:=",
                LaunchConfiguration("require_homing"),
                " require_absolute_encoder:=",
                LaunchConfiguration("require_absolute_encoder"),
                " max_position_error_rad:=",
                LaunchConfiguration("max_position_error_rad"),
            ]
        )
    }

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
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        "joint_state_broadcaster",
                        "trunk_group_controller",
                        "--controller-manager-timeout",
                        LaunchConfiguration("controller_manager_timeout"),
                        "--service-call-timeout",
                        LaunchConfiguration("controller_service_call_timeout"),
                        "--switch-timeout",
                        LaunchConfiguration("controller_switch_timeout"),
                        "--activate-as-group",
                    ],
                    output="screen",
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
                        os.path.join(
                            teleop_share, "config", "trunk_joystick_teleop.yaml"
                        ),
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


def generate_launch_description():
    trunk_configure_share = get_package_share_directory("trunk_configure")
    teleop_share = get_package_share_directory("trunk_teleop_control")

    default_ros2_control_params_file = os.path.join(
        trunk_configure_share, "config", "ros2_controllers.real.yaml"
    )
    default_rviz_config = os.path.join(
        teleop_share, "config", "trunk_joystick_teleop.rviz"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="trunk_robot",
                description="Namespace for the real trunk teleop stack.",
            ),
            DeclareLaunchArgument(
                "ros2_control_params_file",
                default_value=default_ros2_control_params_file,
                description="Real ros2_control controller YAML file.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
                description="RViz config file.",
            ),
            DeclareLaunchArgument("start_joy_node", default_value="true"),
            DeclareLaunchArgument("start_control_mode_manager", default_value="true"),
            DeclareLaunchArgument("initial_control_mode", default_value="manual_teleop"),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument("joy_device_id", default_value="0"),
            DeclareLaunchArgument("joy_deadzone", default_value="0.05"),
            DeclareLaunchArgument("joy_autorepeat_rate", default_value="20.0"),
            DeclareLaunchArgument("controller_manager_timeout", default_value="60.0"),
            DeclareLaunchArgument("controller_service_call_timeout", default_value="60.0"),
            DeclareLaunchArgument("controller_switch_timeout", default_value="60.0"),
            DeclareLaunchArgument(
                "transport",
                default_value="dry_run",
                description="Motor transport: dry_run, serial, can, ethercat, or ip.",
            ),
            DeclareLaunchArgument("port", default_value=""),
            DeclareLaunchArgument("baudrate", default_value="1000000"),
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument("ip", default_value=""),
            DeclareLaunchArgument(
                "dry_run",
                default_value="true",
                description=(
                    "Keep true until readHardware/writeHardware are connected to real motors."
                ),
            ),
            DeclareLaunchArgument("command_timeout_ms", default_value="100"),
            DeclareLaunchArgument("require_homing", default_value="true"),
            DeclareLaunchArgument("require_absolute_encoder", default_value="true"),
            DeclareLaunchArgument("max_position_error_rad", default_value="0.2"),
            OpaqueFunction(function=launch_setup),
        ]
    )
