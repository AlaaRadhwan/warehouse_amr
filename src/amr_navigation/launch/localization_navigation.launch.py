from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    nav2_params = PathJoinSubstitution([
        FindPackageShare("amr_navigation"),
        "config",
        "nav2_params.yaml",
    ])

    map_file = PathJoinSubstitution([
        FindPackageShare("amr_navigation"),
        "maps",
        "warehouse.yaml",
    ])

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("nav2_bringup"),
                "launch",
                "bringup_launch.py",
            ])
        ),
        launch_arguments={
            "slam": "False",

            # Saved-map localization
            "map": map_file,
            "use_localization": "True",
            "serve_static_map": "True",

            "use_sim_time": "True",
            "params_file": nav2_params,
            "autostart": "True",

            "use_composition": "False",
            "use_intra_process_comms": "False",
            "use_respawn": "False",

            "use_keepout_zones": "False",
            "use_speed_zones": "False",
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        parameters=[
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([
        nav2,
        rviz,
    ])