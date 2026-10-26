import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    default_stations_file = os.path.join(
        get_package_share_directory("amr_missions"),
        "config",
        "stations.yaml",
    )

    stations_file = LaunchConfiguration("stations_file")
    navigation_behavior_tree = LaunchConfiguration("navigation_behavior_tree")

    robot_executor = Node(
        package="amr_missions",
        executable="robot_executor",
        name="robot_executor",
        output="screen",
        parameters=[
            {
                "stations_file": stations_file,
                "navigation_behavior_tree": navigation_behavior_tree,
            }
        ],
    )

    mission_manager = Node(
        package='amr_missions',
        executable='mission_manager',
        name='mission_manager',
        output='screen',
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            "stations_file",
            default_value=default_stations_file,
        ),

        # Empty = Nav2 default BT. Absolute path of a NavigateToPose BT to
        # use for stations that do not require precision docking.
        DeclareLaunchArgument(
            "navigation_behavior_tree",
            default_value="",
        ),

        robot_executor,
        mission_manager,
    ])