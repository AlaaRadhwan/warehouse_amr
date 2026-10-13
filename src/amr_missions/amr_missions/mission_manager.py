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

from amr_interfaces.action import GoToStation
from amr_interfaces.action import ExecuteMission


class MissionManager(Node):

    def __init__(self):
        super().__init__('mission_manager')

        self.callback_group = ReentrantCallbackGroup()

        # Client to the per-robot primitive executor
        self.station_client = ActionClient(
            self,
            GoToStation,
            'mission/go_to_station',
            callback_group=self.callback_group,
        )

        # Higher-level mission interface
        self.mission_server = ActionServer(
            self,
            ExecuteMission,
            'mission/execute_mission',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group,
        )

        self.busy = False
        self.active_station_goal = None

        self.get_logger().info('MissionManager ready')


    # ---------------------------------------------------------
    # Goal handling
    # ---------------------------------------------------------

    def goal_callback(self, goal_request):

        if self.busy:
            self.get_logger().warning(
                'Rejecting mission: another mission is already active'
            )
            return GoalResponse.REJECT

        if not goal_request.station_names:
            self.get_logger().warning(
                'Rejecting mission: station sequence is empty'
            )
            return GoalResponse.REJECT

        self.busy = True

        self.get_logger().info(
            'Accepted mission: '
            + ' -> '.join(goal_request.station_names)
        )

        return GoalResponse.ACCEPT


    def cancel_callback(self, goal_handle):

        self.get_logger().warning(
            'Mission cancellation requested'
        )

        if self.active_station_goal is not None:
            self.active_station_goal.cancel_goal_async()

        return CancelResponse.ACCEPT


    # ---------------------------------------------------------
    # Mission execution
    # ---------------------------------------------------------

    async def execute_callback(self, goal_handle):

        stations = list(goal_handle.request.station_names)
        total_steps = len(stations)

        result = ExecuteMission.Result()

        result.success = False
        result.message = ''
        result.completed_steps = 0
        result.failed_station = ''

        try:

            self.get_logger().info(
                'Waiting for RobotExecutor...'
            )

            if not self.station_client.wait_for_server(
                timeout_sec=5.0
            ):
                result.message = (
                    'RobotExecutor GoToStation server unavailable'
                )

                goal_handle.abort()
                return result

            # -------------------------------------------------
            # Execute station sequence
            # -------------------------------------------------

            for index, station_name in enumerate(stations):

                step = index + 1

                if goal_handle.is_cancel_requested:

                    result.message = 'Mission cancelled'

                    goal_handle.canceled()
                    return result

                self.get_logger().info(
                    f'Mission step {step}/{total_steps}: '
                    f'{station_name}'
                )

                self.publish_feedback(
                    goal_handle,
                    state='STARTING',
                    station_name=station_name,
                    current_step=step,
                    total_steps=total_steps,
                    distance_remaining=0.0,
                    number_of_recoveries=0,
                )

                # ---------------------------------------------
                # Send GoToStation goal
                # ---------------------------------------------

                station_goal = GoToStation.Goal()
                station_goal.station_name = station_name

                station_goal_handle = (
                    await self.station_client.send_goal_async(
                        station_goal,
                        feedback_callback=lambda msg,
                        s=station_name,
                        i=step:
                            self.station_feedback_callback(
                                goal_handle,
                                msg,
                                s,
                                i,
                                total_steps,
                            )
                    )
                )

                if not station_goal_handle.accepted:

                    result.message = (
                        f'RobotExecutor rejected station: '
                        f'{station_name}'
                    )

                    result.failed_station = station_name

                    goal_handle.abort()
                    return result

                self.active_station_goal = station_goal_handle

                # Cancellation may have arrived while the
                # station goal was being accepted.
                if goal_handle.is_cancel_requested:

                    await station_goal_handle.cancel_goal_async()

                # ---------------------------------------------
                # Wait for station result
                # ---------------------------------------------

                wrapped_result = (
                    await station_goal_handle.get_result_async()
                )

                self.active_station_goal = None

                station_status = wrapped_result.status
                station_result = wrapped_result.result

                # ---------------------------------------------
                # Mission canceled
                # ---------------------------------------------

                if (
                    station_status == GoalStatus.STATUS_CANCELED
                    or goal_handle.is_cancel_requested
                ):

                    result.message = (
                        f'Mission cancelled while navigating '
                        f'to {station_name}'
                    )

                    goal_handle.canceled()
                    return result

                # ---------------------------------------------
                # Station failed
                # ---------------------------------------------

                if (
                    station_status != GoalStatus.STATUS_SUCCEEDED
                    or not station_result.success
                ):

                    result.failed_station = station_name

                    result.message = (
                        f'Mission failed at {station_name}: '
                        f'{station_result.message}'
                    )

                    self.get_logger().error(result.message)

                    goal_handle.abort()
                    return result

                # ---------------------------------------------
                # Station reached
                # ---------------------------------------------

                result.completed_steps += 1

                self.publish_feedback(
                    goal_handle,
                    state='STATION_REACHED',
                    station_name=station_name,
                    current_step=step,
                    total_steps=total_steps,
                    distance_remaining=0.0,
                    number_of_recoveries=0,
                )

                self.get_logger().info(
                    f'Reached {station_name} '
                    f'({step}/{total_steps})'
                )

            # -------------------------------------------------
            # Mission completed
            # -------------------------------------------------

            result.success = True
            result.message = (
                f'Mission completed successfully: '
                f'{total_steps} station(s)'
            )
            result.failed_station = ''

            goal_handle.succeed()

            self.get_logger().info(result.message)

            return result

        except Exception as exc:

            result.success = False
            result.message = f'Mission failed: {exc}'

            self.get_logger().error(result.message)

            goal_handle.abort()

            return result

        finally:

            self.active_station_goal = None
            self.busy = False


    # ---------------------------------------------------------
    # Feedback
    # ---------------------------------------------------------

    def station_feedback_callback(
        self,
        mission_goal_handle,
        station_feedback_msg,
        station_name,
        current_step,
        total_steps,
    ):

        station_feedback = station_feedback_msg.feedback

        self.publish_feedback(
            mission_goal_handle,
            state='NAVIGATING',
            station_name=station_name,
            current_step=current_step,
            total_steps=total_steps,
            distance_remaining=float(
                station_feedback.distance_remaining
            ),
            number_of_recoveries=int(
                station_feedback.number_of_recoveries
            ),
        )


    def publish_feedback(
        self,
        goal_handle,
        state,
        station_name,
        current_step,
        total_steps,
        distance_remaining,
        number_of_recoveries,
    ):

        feedback = ExecuteMission.Feedback()

        feedback.state = state
        feedback.current_station = station_name
        feedback.current_step = current_step
        feedback.total_steps = total_steps
        feedback.distance_remaining = distance_remaining
        feedback.number_of_recoveries = number_of_recoveries

        goal_handle.publish_feedback(feedback)


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