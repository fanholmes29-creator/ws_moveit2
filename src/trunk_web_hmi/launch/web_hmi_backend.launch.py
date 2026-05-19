from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value="trunk_robot"),
            DeclareLaunchArgument("host", default_value="0.0.0.0"),
            DeclareLaunchArgument("port", default_value="8000"),
            Node(
                package="trunk_web_hmi",
                executable="web_hmi_backend",
                name="web_hmi_backend",
                output="screen",
                additional_env={
                    "TRUNK_ROBOT_NAMESPACE": LaunchConfiguration("robot_namespace"),
                    "TRUNK_WEB_HMI_HOST": LaunchConfiguration("host"),
                    "TRUNK_WEB_HMI_PORT": LaunchConfiguration("port"),
                },
            ),
        ]
    )
