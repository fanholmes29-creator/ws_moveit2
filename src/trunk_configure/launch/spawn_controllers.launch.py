from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _spawner(controller_names):
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            *controller_names,
            "--controller-manager-timeout",
            LaunchConfiguration("controller_manager_timeout", default="60.0"),
            "--service-call-timeout",
            LaunchConfiguration("controller_service_call_timeout", default="60.0"),
            "--switch-timeout",
            LaunchConfiguration("controller_switch_timeout", default="60.0"),
            "--activate-as-group",
        ],
        output="screen",
    )


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "trunk_robot", package_name="trunk_configure"
    ).to_moveit_configs()
    controller_names = (
        moveit_config.trajectory_execution.get("moveit_simple_controller_manager", {})
        .get("controller_names", [])
    )

    actions = [
        DeclareLaunchArgument("controller_manager_timeout", default_value="60.0"),
        DeclareLaunchArgument("controller_service_call_timeout", default_value="60.0"),
        DeclareLaunchArgument("controller_switch_timeout", default_value="60.0"),
        _spawner(["joint_state_broadcaster"] + controller_names),
    ]
    return LaunchDescription(actions)
