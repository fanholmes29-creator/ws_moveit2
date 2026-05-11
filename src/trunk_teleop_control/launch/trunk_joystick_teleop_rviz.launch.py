import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("trunk_teleop_control")
    robot_model_share = get_package_share_directory("robot_model")
    default_params_file = os.path.join(
        package_share, "config", "trunk_joystick_teleop.yaml"
    )
    default_urdf_file = os.path.join(robot_model_share, "urdf", "trunk_robot.urdf")
    default_rviz_config = os.path.join(
        package_share, "config", "trunk_joystick_teleop.rviz"
    )

    with open(default_urdf_file, "r", encoding="utf-8") as urdf_file:
        robot_description_content = urdf_file.read()
    robot_description = {"robot_description": robot_description_content}

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_namespace",
                default_value="trunk_robot",
                description="Namespace for joy_node and robot_state_publisher.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="YAML parameter file for trunk_joystick_teleop.",
            ),
            DeclareLaunchArgument(
                "start_joy_node",
                default_value="true",
                description="Whether to start joy_node in robot namespace.",
            ),
            DeclareLaunchArgument(
                "start_rsp",
                default_value="true",
                description=(
                    "Whether to start robot_state_publisher in robot namespace. "
                    "Disable if your control stack already starts it."
                ),
            ),
            DeclareLaunchArgument(
                "rsp_publish_frequency",
                default_value="15.0",
                description="Publish frequency for robot_state_publisher.",
            ),
            DeclareLaunchArgument("joy_device_id", default_value="0"),
            DeclareLaunchArgument("joy_deadzone", default_value="0.05"),
            DeclareLaunchArgument("joy_autorepeat_rate", default_value="20.0"),
            DeclareLaunchArgument("log_level", default_value="info"),
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                namespace=LaunchConfiguration("robot_namespace"),
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
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                namespace=LaunchConfiguration("robot_namespace"),
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_rsp")),
                parameters=[
                    robot_description,
                    {
                        "publish_frequency": ParameterValue(
                            LaunchConfiguration("rsp_publish_frequency"), value_type=float
                        )
                    },
                ],
            ),
            Node(
                package="trunk_teleop_control",
                executable="trunk_joystick_teleop",
                name="trunk_joystick_teleop",
                namespace=LaunchConfiguration("robot_namespace"),
                output="screen",
                parameters=[LaunchConfiguration("params_file")],
                arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=["-d", default_rviz_config],
            ),
        ]
    )
