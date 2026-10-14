import math

import rclpy

from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from apriltag_msgs.msg import AprilTagDetectionArray

from tf2_ros import (
    Buffer,
    TransformListener,
    TransformException,
)

class DockPosePublisher(Node):

    def __init__(self):

        super().__init__("dock_pose_publisher")

        self.declare_parameter("tag_id", 1)

        self.declare_parameter(
            "tag_frame",
            "tag_station_a",
        )

        self.declare_parameter(
            "fixed_frame",
            "odom",
        )

        self.declare_parameter(
            "dock_offset",
            0.75,
        )

        self.declare_parameter(
            "detection_timeout",
            0.5,
        )

        self.tag_id = int(
            self.get_parameter("tag_id").value
        )

        self.tag_frame = str(
            self.get_parameter("tag_frame").value
        )

        self.fixed_frame = str(
            self.get_parameter("fixed_frame").value
        )

        self.dock_offset = float(
            self.get_parameter("dock_offset").value
        )

        self.detection_timeout = float(
            self.get_parameter("detection_timeout").value
        )

        # TF

        self.tf_buffer = Buffer(
            cache_time=Duration(seconds=5.0)
        )

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        # Publisher

        self.dock_pose_pub = self.create_publisher(
            PoseStamped,
            "/detected_dock_pose",
            10,
        )

        # AprilTag detections

        self.detection_sub = self.create_subscription(
            AprilTagDetectionArray,
            "/apriltag/detections",
            self.detection_callback,
            qos_profile_sensor_data,
        )

        self.last_detection_time = None

        # A new tag TF may arrive slightly after the detection
        # message, so process it from a timer rather than directly
        # inside the subscription callback.

        self.pending_detection = False

        self.timer = self.create_timer(
            0.05,
            self.update_dock_pose,
        )

        self.get_logger().info(
            f"Dock pose publisher started: "
            f"tag={self.tag_frame}, "
            f"fixed_frame={self.fixed_frame}"
        )

    def detection_callback(self, msg):

        for detection in msg.detections:

            if int(detection.id) == self.tag_id:

                self.last_detection_time = self.get_clock().now()

                self.pending_detection = True

                return

    # Convert tag TF -> final robot docking pose

    def update_dock_pose(self):
        if not self.pending_detection:
            return

        if self.last_detection_time is None:
            return

        now = self.get_clock().now()

        age = (
            now - self.last_detection_time
        ).nanoseconds * 1e-9

        if age > self.detection_timeout:

            self.pending_detection = False

            return

        try:

            transform = self.tf_buffer.lookup_transform(
                self.fixed_frame,
                self.tag_frame,
                Time(),
            )

        except TransformException:
            # TF may arrive just after the detection message.
            # Keep pending_predection true and retry next cycle.
            return

        # Tag pose

        tx = transform.transform.translation.x
        ty = transform.transform.translation.y

        q = transform.transform.rotation

        qx = q.x
        qy = q.y
        qz = q.z
        qw = q.w

        # --------------------------------------------------
        # Extract tag +Y axis in the fixed frame
        #
        # Rotation matrix second column:
        #
        # R01 = 2(xy - zw)
        # R11 = 1 - 2(x² + z²)
        #
        # Because the printed TOP of our floor AprilTag points
        # along station +X:
        #
        # station_forward = -tag_Y
        # --------------------------------------------------

        tag_y_x = 2.0 * (
            qx * qy - qz * qw
        )

        tag_y_y = 1.0 - 2.0 * (
            qx * qx + qz * qz
        )

        forward_x = -tag_y_x
        forward_y = -tag_y_y

        # Project onto the floor plane and normalize
        norm = math.hypot(
            forward_x,
            forward_y,
        )

        if norm < 0.2:

            self.get_logger().warning(
                "Tag orientation projection is invalid"
            )

            return

        forward_x /= norm
        forward_y /= norm

        # --------------------------------------------------
        # Desired robot pose
        #
        # Tag is 0.75 m ahead of the final base_footprint.
        #
        # dock = tag - offset * forward
        # --------------------------------------------------

        dock_x = (
            tx
            - self.dock_offset * forward_x
        )

        dock_y = (
            ty 
            - self.dock_offset * forward_y
        )

        dock_yaw = math.atan2(
            forward_y,
            forward_x,
        )

        # Publish Pose stamped

        pose = PoseStamped()

        # Use the TF measurement timestamp rather than pretending
        # an old observation is new.
        pose.header.stamp = transform.header.stamp

        pose.header.frame_id = self.fixed_frame

        pose.pose.position.x = dock_x
        pose.pose.position.y = dock_y
        pose.pose.position.z = 0.0

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0

        pose.pose.orientation.z = math.sin(dock_yaw/2.0)

        pose.pose.orientation.w = math.cos(dock_yaw/2.0)

        self.dock_pose_pub.publish(pose)

        # Process each camera observation only once.
        self.pending_detection = False


def main(args=None):

    rclpy.init(args=args)

    node = DockPosePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        rclpy.shutdown()

if __name__ == "__main__":
    main()