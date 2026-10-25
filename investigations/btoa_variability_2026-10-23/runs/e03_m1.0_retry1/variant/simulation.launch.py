from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():

    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_yaw = LaunchConfiguration("spawn_yaw")

    # ---------------------------------------------------------
    # Paths
    # ---------------------------------------------------------

    xacro_file = PathJoinSubstitution([
        FindPackageShare("amr_description"),
        "urdf",
        "amr.urdf.xacro",
    ])

    controllers_file = "/home/alaa-radhwan/Projects/warehouse_amr/investigations/btoa_variability_2026-10-23/runs/e03_m1.0_retry1/variant/controllers.yaml"

    ekf_file = PathJoinSubstitution([
        FindPackageShare("amr_localization"),
        "config",
        "ekf.yaml",
    ])

    bridge_file = PathJoinSubstitution([
        FindPackageShare("amr_simulation"),
        "config",
        "bridge.yaml",
    ])

    models_path = PathJoinSubstitution([
        FindPackageShare("amr_simulation"),
        "models",
    ])

    world_file = PathJoinSubstitution([
        FindPackageShare("amr_simulation"),
        "worlds",
        "warehouse_world.sdf",
    ])


    # ---------------------------------------------------------
    # Robot description
    # ---------------------------------------------------------

    robot_description = {
        "robot_description": ParameterValue(
            Command([
                FindExecutable(name="xacro"),
                " ",
                xacro_file,
                " use_sim:=true",
                " controllers_file:=",
                controllers_file,
            ]),
            value_type=str,
        )
    }


    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": True},
        ],
    )


    # ---------------------------------------------------------
    # Gazebo
    # ---------------------------------------------------------

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            ])
        ),
        launch_arguments={
            "gz_args": ["-r -v 3 ", world_file],
        }.items(),
    )


    # ---------------------------------------------------------
    # ROS <-> Gazebo bridge
    # ---------------------------------------------------------

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[
            {"config_file": bridge_file},
        ],
    )


    # ---------------------------------------------------------
    # Spawn robot
    # ---------------------------------------------------------

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-world", "warehouse_world",
            "-topic", "robot_description",
            "-name", "warehouse_amr",

            "-x", spawn_x,
            "-y", spawn_y,
            "-z", "0.02",
            "-Y", spawn_yaw,
        ],
    )


    # ---------------------------------------------------------
    # ros2_control controllers
    # ---------------------------------------------------------

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )


    diff_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "diff_drive_controller",
            "--controller-manager",
            "/controller_manager",

            # Present a clean robot-level command interface
            "--controller-ros-args",
            "-r /diff_drive_controller/cmd_vel:=/cmd_vel",
        ],
    )


    # ---------------------------------------------------------
    # EKF
    # ---------------------------------------------------------

    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            ekf_file,
            {"use_sim_time": True},
        ],
        remappings=[
            ("odometry/filtered", "/odom"),
        ],
    )


    # ---------------------------------------------------------
    # Startup order
    # ---------------------------------------------------------

    start_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster],
        )
    )

    start_diff_drive = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[diff_drive_controller],
        )
    )


    return LaunchDescription([

        DeclareLaunchArgument(
            "spawn_x",
            default_value="0.0",
        ),

        DeclareLaunchArgument(
            "spawn_y",
            default_value="0.0",
        ),

        DeclareLaunchArgument(
            "spawn_yaw",
            default_value="0.0",
        ),

        AppendEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            models_path,
        ),
        
        robot_state_publisher,
        gazebo,
        bridge,
        spawn_robot,

        start_joint_state_broadcaster,
        start_diff_drive,

        ekf,
    ])