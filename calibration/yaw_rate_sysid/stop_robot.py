#!/usr/bin/env python3
"""Independent emergency stop: publish zero TwistStamped on /cmd_vel for ~1 s.

Use from a second terminal if run_rotation_test.py is unresponsive:
    python3 calibration/yaw_rate_sysid/stop_robot.py
"""
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.parameter import Parameter


def main():
    rclpy.init()
    node = rclpy.create_node("rotation_sysid_stop",
                             parameter_overrides=[Parameter("use_sim_time", value=True)])
    pub = node.create_publisher(TwistStamped, "/cmd_vel", 10)
    t_end = time.time() + 1.0
    while time.time() < t_end:
        m = TwistStamped()
        m.header.stamp = node.get_clock().now().to_msg()
        m.header.frame_id = "base_footprint"
        pub.publish(m)
        time.sleep(0.02)
    print("zero command sent")
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
