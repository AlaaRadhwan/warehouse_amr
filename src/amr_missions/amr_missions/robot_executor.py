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

from nav2_msgs.action import NavigateToPose, DockRobot
from amr_interfaces.action import GoToStation

class RobotExecutor(Node):

    def __init__(self):
        super().__init__('robot_executor')

        self.callback_group = ReentrantCallbackGroup()

        default_file = os.path.join(
            get_package_share_directory('amr_missions'),
            'config',
            'stations.yaml'
        )

        self.declare_parameter('stations_file', default_file)

        # Optional NavigateToPose behavior tree (absolute path). Empty keeps
        # the Nav2 default tree (free-space planning); set it to
        # navigate_on_warehouse_route.xml for structured warehouse routing.
        self.declare_parameter('navigation_behavior_tree', '')
        self.navigation_behavior_tree = self.get_parameter(
            'navigation_behavior_tree'
        ).get_parameter_value().string_value

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

        self.dock_client = ActionClient(
            self,
            DockRobot,
            'dock_robot',
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
        self.active_downstream_goal = None


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

        if self.active_downstream_goal is not None:
            self.active_downstream_goal.cancel_goal_async()

        return CancelResponse.ACCEPT

    # Mission Execution

    async def execute_callback(self, goal_handle):

        station_name = goal_handle.request.station_name
        station = self.stations[station_name]

        try:

            docking = station.get(
                'docking',
                {}
            )

            if docking.get('enabled', False):

                self.get_logger().info(
                    f'{station_name} requires precision docking'
                )

                return await self.execute_docking(
                    goal_handle,
                    station_name,
                )

            self.get_logger().info(
                f'{station_name} uses normal navigation'
            )

            return await self.execute_navigation(
                goal_handle,
                station_name,
                station,
            )

        except Exception as exc:

            result = GoToStation.Result()

            result.success = False
            result.message = (
                f'GoToStation for {station_name} failed: {exc}'
            )

            result.nav2_error_code = 0

            goal_handle.abort()

            self.get_logger().error(
                result.message
            )

            return result

        finally:

            self.active_downstream_goal = None
            self.busy = False

    async def execute_navigation(
            self,
            goal_handle,
            station_name,
            station,
    ):

        result = GoToStation.Result()

        self.get_logger().info(
            'Waiting for Nav2 NavigateToPose action server...'
        )

        if not self.nav_client.wait_for_server(
            timeout_sec=5.0
        ):

            result.success = False
            result.message = (
                'Nav2 NavigateToPose server unavailable'
            )
            result.nav2_error_code = 0

            goal_handle.abort()

            return result

        # Construct Nav2 goal

        nav_goal = NavigateToPose.Goal()
        nav_goal.behavior_tree = self.navigation_behavior_tree

        nav_goal.pose.header.frame_id = self.frame_id
        nav_goal.pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        nav_goal.pose.pose.position.x = float(
            station['x']
        )

        nav_goal.pose.pose.position.y = float(
            station['y']
        )

        nav_goal.pose.pose.position.z = 0.0

        yaw = float(
            station['yaw']
        )

        nav_goal.pose.pose.orientation.x = 0.0
        nav_goal.pose.pose.orientation.y = 0.0

        nav_goal.pose.pose.orientation.z = (
            math.sin(yaw / 2.0)
        )

        nav_goal.pose.pose.orientation.w = (
            math.cos(yaw / 2.0)
        )

        self.get_logger().info(
            f'Sending NavigateToPose for {station_name}: '
            f'x={station["x"]}, '
            f'y={station["y"]}, '
            f'yaw={station["yaw"]}'
        )

        nav_goal_handle = (
            await self.nav_client.send_goal_async(
                nav_goal,
                feedback_callback=lambda msg:
                    self.nav_feedback_callback(
                        goal_handle,
                        msg,
                    )
            )
        )

        self.active_downstream_goal = (
            nav_goal_handle
        )

        if not nav_goal_handle.accepted:

            result.success = False
            result.message = (
                'Nav2 rejected navigation goal'
            )
            result.nav2_error_code = 0

            goal_handle.abort()

            return result

        # A cancellation may have arrived while
        # Nav2 was accepting the goal.

        if goal_handle.is_cancel_requested:

            await nav_goal_handle.cancel_goal_async()

        wrapped_result = (
            await nav_goal_handle.get_result_async()
        )

        status = wrapped_result.status
        nav_result = wrapped_result.result

        nav_error = int(
            getattr(
                nav_result,
                'error_code',
                0,
            )
        )

        nav_error_msg = getattr(
            nav_result,
            'error_msg',
            '',
        )

        result.nav2_error_code = nav_error

        self.get_logger().info(
            f'NavigateToPose finished: '
            f'status={status}, '
            f'error_code={nav_error}, '
            f'error_msg="{nav_error_msg}"'
        )

        if status == GoalStatus.STATUS_SUCCEEDED:

            result.success = True
            result.message = (
                f'Arrived at station: {station_name}'
            )

            goal_handle.succeed()

            return result

        if status == GoalStatus.STATUS_CANCELED:

            result.success = False
            result.message = (
                f'Navigation to {station_name} cancelled'
            )

            goal_handle.canceled()

            return result

        result.success = False

        result.message = (
            f'Navigation to {station_name} failed'
        )

        if nav_error_msg:
            result.message += (
                f': {nav_error_msg}'
            )

        goal_handle.abort()

        return result

    async def execute_docking(
            self,
            goal_handle,
            station_name,
    ):

        result = GoToStation.Result()

        self.get_logger().info(
            'Waiting for Nav2 DockRobot action server...'
        )

        if not self.dock_client.wait_for_server(
            timeout_sec=5.0
        ):

            result.success = False
            result.message = (
                'Nav2 DockRobot server unavailable'
            )
            result.nav2_error_code = 0

            goal_handle.abort()

            return result

        # ---------------------------------------------
        # Construct docking goal
        # ---------------------------------------------

        dock_goal = DockRobot.Goal()

        dock_goal.use_dock_id = True
        dock_goal.dock_id = station_name

        # Let Docking Server use Nav2 to reach
        # the staging pose first.
        dock_goal.navigate_to_staging_pose = True

        self.get_logger().info(
            f'Sending DockRobot goal for '
            f'{station_name}'
        )

        dock_goal_handle = (
            await self.dock_client.send_goal_async(
                dock_goal,
                feedback_callback=lambda msg:
                    self.dock_feedback_callback(
                        goal_handle,
                        msg,
                    )
            )
        )

        self.active_downstream_goal = (
            dock_goal_handle
        )

        if not dock_goal_handle.accepted:

            result.success = False
            result.message = (
                'Nav2 rejected docking goal'
            )
            result.nav2_error_code = 0

            goal_handle.abort()

            return result

        # Handle cancellation that may have happened
        # while Docking Server was accepting the goal.

        if goal_handle.is_cancel_requested:

            await dock_goal_handle.cancel_goal_async()

        wrapped_result = (
            await dock_goal_handle.get_result_async()
        )

        status = wrapped_result.status
        dock_result = wrapped_result.result

        dock_error = int(
            getattr(
                dock_result,
                'error_code',
                0,
            )
        )

        dock_error_msg = getattr(
            dock_result,
            'error_msg',
            '',
        )

        result.nav2_error_code = dock_error

        self.get_logger().info(
            f'DockRobot finished: '
            f'status={status}, '
            f'error_code={dock_error}, '
            f'error_msg="{dock_error_msg}"'
        )

        # ---------------------------------------------
        # Successful docking
        # ---------------------------------------------

        if (
            status == GoalStatus.STATUS_SUCCEEDED
            and dock_result.success
        ):

            result.success = True
            result.message = (
                f'Docked successfully at '
                f'{station_name}'
            )

            goal_handle.succeed()

            return result

        # ---------------------------------------------
        # Cancelled
        # ---------------------------------------------

        if status == GoalStatus.STATUS_CANCELED:

            result.success = False
            result.message = (
                f'Docking at {station_name} cancelled'
            )

            goal_handle.canceled()

            return result

        # ---------------------------------------------
        # Docking failure
        # ---------------------------------------------

        result.success = False

        result.message = (
            f'Docking at {station_name} failed'
        )

        if dock_error_msg:
            result.message += (
                f': {dock_error_msg}'
            )

        goal_handle.abort()

        return result 

    
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


    def dock_feedback_callback(
            self,
            mission_goal_handle,
            dock_feedback_msg,
    ):

        feedback = dock_feedback_msg.feedback

        mission_feedback = GoToStation.Feedback()

        state_names = {

            DockRobot.Feedback.NONE:
                'DOCKING',

            DockRobot.Feedback.NAV_TO_STAGING_POSE:
                'NAVIGATING_TO_STAGING',

            DockRobot.Feedback.INITIAL_PERCEPTION:
                'INITIAL_PERCEPTION',

            DockRobot.Feedback.CONTROLLING:
                'DOCKING',

            DockRobot.Feedback.WAIT_FOR_CHARGE:
                'WAITING_FOR_CHARGE',

            DockRobot.Feedback.RETRY:
                'DOCKING_RETRY',
        }

        mission_feedback.state = (
            state_names.get(
                feedback.state,
                'DOCKING',
            )
        )

        # DockRobot feedback does not provide
        # distance_remaining.
        mission_feedback.distance_remaining = 0.0

        # Reuse this field temporarily for docking retries.
        mission_feedback.number_of_recoveries = int(
            feedback.num_retries
        )

        mission_goal_handle.publish_feedback(
            mission_feedback
        )

    
def main(args=None):

    rclpy.init(args=args)

    node = RobotExecutor()

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
