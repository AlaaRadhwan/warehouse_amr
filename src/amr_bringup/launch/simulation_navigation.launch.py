import os
import yaml
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from nav2_common.launch import RewrittenYaml

def create_dock_database(station_data):

    frame_id = station_data.get(
        "frame_id",
        "map",
    )

    stations = station_data.get(
        "stations",
        {},
    )

    docks = {}

    for station_name, station in stations.items():

        docking = station.get("docking")

        if not docking:
            continue

        if not docking.get("enabled", False):
            continue

        if "type" not in docking:
            raise RuntimeError(
                f'Docking station "{station_name}" '
                f'is missing docking.type'
            )

        required_pose_fields = {
            "x",
            "y",
            "yaw",
        }

        missing = (
            required_pose_fields
            - station.keys()
        )

        if missing:
            raise RuntimeError(
                f'Docking station "{station_name}" '
                f'is missing pose fields: {missing}'
            )

        docks[station_name] = {
            "type": str(
                docking["type"]
            ),

            "frame": str(
                station.get(
                    "frame",
                    frame_id,
                )
            ),

            "pose": [
                float(station["x"]),
                float(station["y"]),
                float(station["yaw"]),
            ],
        }

    if not docks:
        raise RuntimeError(
            "No enabled docking stations "
            "were found in stations.yaml"
        )

    database = {
        "docks": docks,
    }

    database_file = tempfile.NamedTemporaryFile(
        mode="w",
        prefix="warehouse_amr_docks_",
        suffix=".yaml",
        delete=False,
    )

    yaml.safe_dump(
        database,
        database_file,
        sort_keys=False,
    )

    database_file.close()

    return database_file.name

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

    lattice_file = os.path.join(
        navigation_share,
        "config",
        "lattice",
        "warehouse_diff_5cm_r0.5.json",
    )

    if not os.path.isfile(lattice_file):
        raise RuntimeError(
            "State Lattice primitive file not found: "
            f"{lattice_file}"
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

    dock_database_file = create_dock_database(station_data)

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

            # ---------------------------------------------
            # Initial AMCL pose
            # ---------------------------------------------

            "amcl.ros__parameters.set_initial_pose":
                "true",

            "amcl.ros__parameters.initial_pose.x":
                str(home_x),

            "amcl.ros__parameters.initial_pose.y":
                str(home_y),

            "amcl.ros__parameters.initial_pose.z":
                "0.0",

            "amcl.ros__parameters.initial_pose.yaw":
                str(home_yaw),


            # ---------------------------------------------
            # Dock database generated from stations.yaml
            # ---------------------------------------------

            "docking_server.ros__parameters.dock_database":
                dock_database_file,

            "planner_server.ros__parameters.GridBased.lattice_filepath":
                lattice_file,

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