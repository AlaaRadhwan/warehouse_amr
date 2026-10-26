import os
import yaml
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

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

def _launch_setup(context):

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

    route_graph_file = os.path.join(
        navigation_share,
        "route_graph",
        "warehouse_route_graph.geojson",
    )

    if not os.path.isfile(route_graph_file):
        raise RuntimeError(
            "Route graph not found: "
            f"{route_graph_file}"
        )

    route_bt_file = os.path.join(
        navigation_share,
        "behavior_trees",
        "navigate_on_warehouse_route.xml",
    )

    # structured_routing:=true makes the robot executor send its station
    # goals with the route-following behavior tree (default: unchanged).
    structured_routing = LaunchConfiguration("structured_routing").perform(context)

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

    # ---------------------------------------------------------
    # Experimental profiles (defaults reproduce the baseline exactly)
    #   planner:=lattice|navfn
    #   mppi_path_interpretation:=baseline|2d
    # ---------------------------------------------------------
    planner_profile = LaunchConfiguration("planner").perform(context)
    mppi_profile = LaunchConfiguration(
        "mppi_path_interpretation").perform(context)

    overlays = {
        ("planner", "lattice"): None,
        ("planner", "navfn"): "planner_navfn.yaml",
        ("mppi_path_interpretation", "baseline"): None,
        ("mppi_path_interpretation", "2d"): "mppi_2d_paths.yaml",
        ("mppi_path_interpretation", "2d_forward_only"):
            ["mppi_2d_paths.yaml", "mppi_forward_only.yaml"],
    }

    for key in (("planner", planner_profile),
                ("mppi_path_interpretation", mppi_profile)):
        if key not in overlays:
            raise RuntimeError(f"Invalid launch option {key[0]}:={key[1]}")

    with open(nav2_params_file, "r") as file:
        merged_params = yaml.safe_load(file)

    for key in (("planner", planner_profile),
                ("mppi_path_interpretation", mppi_profile)):
        overlay = overlays[key]
        if isinstance(overlay, str):
            overlay = [overlay]
        for overlay_file in overlay or []:
            with open(os.path.join(
                    navigation_share, "config", "profiles",
                    overlay_file)) as f:
                _deep_merge(merged_params, yaml.safe_load(f))

    if planner_profile != "lattice" or mppi_profile != "baseline":
        merged_file = tempfile.NamedTemporaryFile(
            mode="w", prefix="warehouse_amr_nav2_params_",
            suffix=".yaml", delete=False)
        yaml.safe_dump(merged_params, merged_file, sort_keys=False)
        merged_file.close()
        nav2_params_file = merged_file.name
        print(f"[simulation_navigation] EXPERIMENTAL profile: "
              f"planner={planner_profile}, "
              f"mppi_path_interpretation={mppi_profile}, "
              f"params={nav2_params_file}")

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

            "route_server.ros__parameters.graph_filepath":
                route_graph_file,

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
        launch_arguments={
            "navigation_behavior_tree":
                route_bt_file if structured_routing == "true" else "",
        }.items(),
    )

    return [
        simulation,
        navigation,
        robot_executor,
    ]


def _deep_merge(base, overlay):
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _deep_merge(base[key], value)
        else:
            base[key] = value


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "structured_routing",
            default_value="false",
            description="Use the warehouse route graph for station navigation",
        ),
        DeclareLaunchArgument(
            "planner",
            default_value="lattice",
            description="EXPERIMENTAL: global planner profile (lattice|navfn)",
        ),
        DeclareLaunchArgument(
            "mppi_path_interpretation",
            default_value="baseline",
            description="EXPERIMENTAL: MPPI path-orientation settings "
                        "(baseline|2d|2d_forward_only)",
        ),
        OpaqueFunction(function=_launch_setup),
    ])