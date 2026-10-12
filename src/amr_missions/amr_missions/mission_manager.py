import os
import yaml
import math

import rclpy
from rclpy.node import Node
from rclpy.action import (
    ActionClient,
    ActionServer,
    GoalResponse,
    CancelResponse,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory

from nav2_msgs.action import NavigateToPose
from amr_interfaces.action import GoToStation

class MissionManager(Node):

    def __init__(self):
        super().__init__('mission_manager')

        self.callback_group = ReentrantCallbackGroup()

        default_file = os.path.join(
            get_package_share_directory('amr_missions'),
            'config',
            'stations.yaml'
        )

        self.declare_parameter('stations_file', default_file)

        stations_file = self.get_parameter(
            'stations_file'
        ).get_parameter_value().string_value

        self.frame_id, self.stations = self.load_stations(stations_file)

        self.get_logger().info(
            f'Loaded {len(self.stations)} stations: '
            f'{", ".join(self.stations.keys())}'
        )

        # Nav2 action client

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.callback_group,
        )

        # High-level AMR action server

        self.action_server = ActionServer(
            self,
            GoToStation,
            'mission/go_to_station',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group,
        )

        self.busy = False
        self.active_nav_goal = None


    # Station Configuration

    def load_stations(self, filename):

        with open(filename, 'r') as file:
            data = yaml.safe_load(file)

        if not data:
            raise RuntimeError(f'stations.yaml is empty')

        frame_id = data.get('frame_id', 'map')
        stations = data.get('stations', {})

        if not stations:
            raise RuntimeError('No stations defined in stations.yaml')

        required_fields = {'x', 'y', 'yaw'}

        for name, station in stations.items():

            missing = required_fields - station.keys()

            if missing:
                raise RuntimeError(
                    f'Station "{name}" is missing required fields: {missing}'
                )

        return frame_id, stations


    # Goal handling

    def goal_callback(self, goal_request):

        station_name = goal_request.station_name

        if station_name not in self.stations:
            self.get_logger().warning(
                f'Rejecting unknown station: {station_name}'
            )
            return GoalResponse.REJECT

        if self.busy:
            self.get_logger().warning(
                f'Rejecting {station_name}: mission already active'
            )
            return GoalResponse.REJECT

        self.busy = True

        self.get_logger().info(
            f'Accepted mission to station: {station_name}'
        )

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):

        self.get_logger().warning(
            f'Cancel requested for '
            f'{goal_handle.request.station_name}'
        )

        if self.active_nav_goal is not None:
            self.active_nav_goal.cancel_goal_async()

        return CancelResponse.ACCEPT

    # Mission Execution

    async def execute_callback(self, goal_handle):

        station_name = goal_handle.request.station_name
        station = self.stations[station_name]
        
        result = GoToStation.Result()

        try:

            # Wait for Nav2

            self.get_logger().info(
                'Waiting for Nav2 NavigateToPose action server...'
            )

            if not self.nav_client.wait_for_server(timeout_sec=5.0):
                result.success = False
                result.message = 'Nav2 NavigateToPose server unavilable'
                result.nav2_error_code = 0

                goal_handle.abort()

                return result

            # Construct Nav2 goal

            nav_goal = NavigateToPose.Goal()

            nav_goal.pose.header.frame_id = self.frame_id
            nav_goal.pose.header.stamp = self.get_clock().now().to_msg()

            nav_goal.pose.pose.position.x = float(station['x'])
            nav_goal.pose.pose.position.y = float(station['y'])
            nav_goal.pose.pose.position.z = 0.0

            yaw = float(station['yaw'])

            nav_goal.pose.pose.orientation.x = 0.0
            nav_goal.pose.pose.orientation.y = 0.0
            nav_goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
            nav_goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

            self.get_logger().info(
                f'Sending Nav2 goal for {station_name}: '
                f'x={station["x"]}, y={station["y"]}, '
            )

            # Send goal

            nav_goal_handle = await self.nav_client.send_goal_async(
                nav_goal,
                feedback_callback=lambda msg:
                    self.nav_feedback_callback(goal_handle, msg)
            )

            self.active_nav_goal = nav_goal_handle

            if not nav_goal_handle.accepted:

                result.success = False
                result.message = 'Nav2 rejected navigation goal'
                result.nav2_error_code = 0

                goal_handle.abort()

                return result

            # In case cancel happened while goal was being accepted
            if goal_handle.is_cancel_requested:
                await nav_goal_handle.cancel_goal_async()

            # Wait for navigation result

            wrapped_result = await nav_goal_handle.get_result_async()

            status = wrapped_result.status
            nav_result = wrapped_result.result

            nav_error = getattr(nav_result, 'error_code', 0)
            nav_error_msg = getattr(nav_result, 'message', '')

            result.nav2_error_code = int(nav_error)

            # Successful navigation

            if status == GoalStatus.STATUS_SUCCEEDED:

                result.success = True
                result.message = f'Arrived at station: {station_name}'

                goal_handle.succeed()

                self.get_logger().info(result.message)

                return result

            # Cancelled

            if status == GoalStatus.STATUS_CANCELED:

                result.success = False
                result.message = f'Navigation to {station_name} cancelled'

                goal_handle.canceled()

                self.get_logger().warning(result.message)

                return result

            # Nav2 Failure

            result.success = False

            result.message = (
                f'Navigation to {station_name} failed'
            )

            if nav_error_msg:
                result.message += f': {nav_error_msg}'

            goal_handle.abort()

            self.get_logger().error(
                f'{result.message} (Nav2 error {nav_error})'
            )

            return result

        except Exception as exc:

            result.success = False
            result.message = f'Navigation to {station_name} failed: {exc}'
            result.nav2_error_code = 0

            goal_handle.abort()

            self.get_logger().error(result.message)

            return result

        finally:

            self.active_nav_goal = None
            self.busy = False

    # Nav2 feedback -> mission feedback

    def nav_feedback_callback(
            self,
            mission_goal_handle,
            nav_feedback_msg,
    ):

        feedback = nav_feedback_msg.feedback

        mission_feedback = GoToStation.Feedback()

        mission_feedback.state = 'NAVIGATING'
        mission_feedback.distance_remaining = float(
            feedback.distance_remaining
        )
        mission_feedback.number_of_recoveries = int(
            feedback.number_of_recoveries
        )

        mission_goal_handle.publish_feedback(mission_feedback)
    
def main(args=None):

    rclpy.init(args=args)

    node = MissionManager()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
