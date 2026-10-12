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

    slam_params = PathJoinSubstitution([
        FindPackageShare("amr_navigation"),
        "config",
        "slam.yaml",
    ])

    # ---------------------------------------------------------
    # SLAM Toolbox
    #
    # Publishes:
    #   /map
    #   map -> odom
    # ---------------------------------------------------------

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("slam_toolbox"),
                "launch",
                "online_async_launch.py",
            ])
        ),
        launch_arguments={
            "slam_params_file": slam_params,
            "use_sim_time": "True",
        }.items(),
    )

    # ---------------------------------------------------------
    # Nav2
    #
    # SLAM Toolbox is our localization / map source here,
    # so AMCL and static map server are disabled.
    # ---------------------------------------------------------

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
            "use_localization": "False",
            "serve_static_map": "False",

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
        slam,
        nav2,
        rviz,
    ])