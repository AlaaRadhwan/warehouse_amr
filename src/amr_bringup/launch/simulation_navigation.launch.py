import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    # ---------------------------------------------------------
    # Package paths
    # ---------------------------------------------------------

    missions_share = get_package_share_directory(
        "amr_missions"
    )

    simulation_share = get_package_share_directory(
        "amr_simulation"
    )

    navigation_share = get_package_share_directory(
        "amr_navigation"
    )

    # ---------------------------------------------------------
    # Load station configuration
    # ---------------------------------------------------------

    stations_file = os.path.join(
        missions_share,
        "config",
        "stations.yaml",
    )

    with open(stations_file, "r") as file:
        station_data = yaml.safe_load(file)

    stations = station_data.get("stations", {})

    if "home" not in stations:
        raise RuntimeError(
            'stations.yaml must contain a "home" station'
        )

    home = stations["home"]

    required_fields = {"x", "y", "yaw"}

    missing = required_fields - home.keys()

    if missing:
        raise RuntimeError(
            f'Home station is missing fields: {missing}'
        )

    home_x = float(home["x"])
    home_y = float(home["y"])
    home_yaw = float(home["yaw"])

    # ---------------------------------------------------------
    # Build runtime Nav2 params
    # ---------------------------------------------------------

    nav2_params_file = os.path.join(
        navigation_share,
        "config",
        "nav2_params.yaml",
    )

    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params_file,
        root_key="",
        param_rewrites={
            "amcl.ros__parameters.set_initial_pose": "true",

            "amcl.ros__parameters.initial_pose.x": str(home_x),
            "amcl.ros__parameters.initial_pose.y": str(home_y),
            "amcl.ros__parameters.initial_pose.z": "0.0",
            "amcl.ros__parameters.initial_pose.yaw": str(home_yaw),
        },
        convert_types=True,
    )

    # ---------------------------------------------------------
    # Simulation
    # ---------------------------------------------------------

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                simulation_share,
                "launch",
                "simulation.launch.py",
            )
        ),
        launch_arguments={
            "spawn_x": str(home_x),
            "spawn_y": str(home_y),
            "spawn_yaw": str(home_yaw),
        }.items(),
    )

    # ---------------------------------------------------------
    # Saved-map localization + Nav2
    # ---------------------------------------------------------

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                navigation_share,
                "launch",
                "localization_navigation.launch.py",
            )
        ),
        launch_arguments={
            "params_file": configured_nav2_params,
        }.items(),
    )

    robot_executor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                missions_share,
                "launch",
                "robot_executor.launch.py",
            )
        ),
    )

    return LaunchDescription([
        simulation,
        navigation,
        robot_executor,
    ])