import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():

    config_file = os.path.join(
        get_package_share_directory("amr_perception"),
        "config",
        "apriltags.yaml",
    )

    apriltag_node = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag",
        output="screen",

        parameters=[
            config_file,
            {
                "use_sim_time": True,
            },
        ],

        remappings=[
            (
                "image_rect",
                "/camera/image_raw",
            ),
            (
                "camera_info",
                "/camera/camera_info",
            ),
            (
                "detections",
                "/apriltag/detections",
            ),
        ],
    )

    dock_pose_publisher = Node(
        package="amr_perception",
        executable="dock_pose_publisher",
        name="dock_pose_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,

                "tag_id": 1,

                "tag_frame": "tag_station_a",

                "fixed_frame": "odom",

                "dock_offset": 0.75,

                "detection_timeout": 0.5,
            }
        ],
    )

    return LaunchDescription([
        apriltag_node,
        dock_pose_publisher,
    ])